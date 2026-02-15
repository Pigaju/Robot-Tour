#include <M5Unified.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <esp_system.h>

#include <time.h>
#include <sys/time.h>

#include <esp_heap_caps.h>

// Optional WiFi + cloud uploader (posts the saved CSV to a webhook)
// Disabled for track testing (keeps firmware focused on control + local logging).
#define RUN_CLOUD_UPLOAD_ENABLE 0

#if RUN_CLOUD_UPLOAD_ENABLE
  #include <WiFi.h>
  #include <WiFiClientSecure.h>
  #include <HTTPClient.h>

  // Keep credentials out of the repo: create include/wifi_secrets.h locally.
  // It should define:
  //   #define WIFI_SSID "..."
  //   #define WIFI_PASSWORD "..."
  //   #define CLOUD_UPLOAD_URL "https://script.google.com/macros/s/.../exec"
  #if __has_include("wifi_secrets.h")
    #include "wifi_secrets.h"
  #endif
  #ifndef WIFI_SSID
    #define WIFI_SSID ""
  #endif
  #ifndef WIFI_PASSWORD
    #define WIFI_PASSWORD ""
  #endif
  #ifndef CLOUD_UPLOAD_URL
    #define CLOUD_UPLOAD_URL ""
  #endif

  #define CLOUD_UPLOAD_CONNECT_TIMEOUT_MS 12000
  #define CLOUD_UPLOAD_HTTP_TIMEOUT_MS 20000
  #define CLOUD_UPLOAD_VERIFY_TLS 0
#endif

// Forward declarations (needed because menu helpers appear before some defs)
void stopAllMotors();
void applyMotorOutputs();
void updateDisplay();
void readWheelEncoders();
static void readWheelEncodersForce();
static void updateMotorDiagnostics();

#if RUN_CLOUD_UPLOAD_ENABLE
static bool cloudUploadLastRunCsv();
#endif

// Firmware ID (update this when behavior changes)
// Robot Tour Event: BFS-based graph navigation system
// v1.0: Initial Robot Tour port with BFS pathfinding
#define FW_VERSION "v1_robot_tour_bfs_2026-02-14"

// ========================================================================
// BFS NAVIGATION SYSTEM FOR ROBOT TOUR
// ========================================================================
// Graph-based pathfinding for navigating waypoints/rooms
// The robot builds a BFS tree from start to goal and follows the path

// Hardware-specific direction fix:
// If the car drives backwards when commanded forward, flip this.
#define INVERT_MOTOR_DIRECTION 1

// Chassis/mounting fix:
// If left/right motors are mirrored, you may need one side reversed so that
// a logical "forward" command makes both wheels propel forward.
#define INVERT_LEFT_MOTOR_DIRECTION 1
// If the *right* motor needs reversing to match "forward".
#define INVERT_RIGHT_MOTOR_DIRECTION 0

// Dial direction: set so turning RIGHT increases values.
// If the dial feels backwards, flip this to -1.
#define DIAL_DIRECTION -1

// Safety/diagnostic toggles
#define RUN_I2C_SCAN_ON_BOOT 0
#define RUN_MOTOR_DIAGNOSTIC_ON_BOOT 0

// Cloud upload behavior
// - ON_STOP: automatically upload /runlog_last.csv after each run ends
// - DEFERRED: do the HTTP work later in loop() (avoids blocking stopRun())
#define RUN_CLOUD_UPLOAD_ON_STOP 1
#define RUN_CLOUD_UPLOAD_DEFERRED 1
#define RUN_CLOUD_UPLOAD_MAX_RETRIES 3
#define RUN_CLOUD_UPLOAD_RETRY_MS 5000

// Hardware pins
#define ENCODER_PIN_A 41
#define ENCODER_PIN_B 40

// I2C addresses
// Canonical mapping (matches prior working encoder test + M5Unit-Hbridge examples)
// - Left Encoder: 0x59
// - Right Encoder: 0x58
// - Left HBridge: 0x21
// - Right HBridge: 0x20
//
// If your physical wiring is swapped, flip these toggles to keep the UI labels sane.
#define SWAP_ENCODERS 1
#define SWAP_MOTORS 0

// Some encoder modules are mounted such that "forward" motion reports negative counts.
// If RUN distance does not increase but encoder counts do change, try flipping one of these.
#define INVERT_LEFT_ENCODER_COUNT 0
#define INVERT_RIGHT_ENCODER_COUNT 1

#if SWAP_ENCODERS
#define ENCODER_L_ADDR 0x58
#define ENCODER_R_ADDR 0x59
#else
#define ENCODER_L_ADDR 0x59
#define ENCODER_R_ADDR 0x58
#endif

#if SWAP_MOTORS
#define MOTOR_L_ADDR 0x20
#define MOTOR_R_ADDR 0x21
#else
#define MOTOR_L_ADDR 0x21
#define MOTOR_R_ADDR 0x20
#endif

// Boot-time motor spin test (to validate I2C write format)
// This runs once in setup(): start motor, wait 5s, stop.
// Set to 1 only when you want to run the boot-only spin test.
#define RUN_BOOT_MOTOR_SPIN_TEST 0
#define BOOT_TEST_MOTOR_ADDR MOTOR_R_ADDR
#define BOOT_TEST_DIRECTION 1 // 1=forward
#define BOOT_TEST_PWM 140
#define BOOT_TEST_DURATION_MS 5000

// Use M5Unified's external I2C pin mapping (Port A) for Wire.
// This matches M5's Unit HBridge docs/examples (device lives on PORT.A).
#define USE_M5_EX_I2C_PINS 1

// Display refresh (ms). Rendering is double-buffered via sprite to reduce flicker.
#define DISPLAY_REFRESH_MS 100

// Master quiet mode: set to 1 to silence all boot/setup Serial output.
// Only run logs will be printed (for post-run analysis).
#define SERIAL_QUIET_MODE 1

// Start-path diagnostics (helps debug "red button does nothing")
// - Prints a few key lines to Serial on start/stop attempts
// - Logs the same lines to SPIFFS so you can retrieve them even if live monitor is flaky
#define RUN_START_DEBUG_SERIAL 1
#define RUN_START_DEBUG_SPIFFS 1

// Set to 1 to periodically print diagnostic status over Serial.
// Keep this OFF during normal runs; it can flood the terminal.
#define SERIAL_STATUS_SPAM 0

// Motor I2C write error prints.
// Keep this OFF during normal runs; errors are already captured in the CSV log.
#define SERIAL_MOTOR_ERR_SPAM 0

// Control-loop update period.
// Fixed control period (simpler + more reproducible).
#define CONTROL_PERIOD_MS 10
static uint16_t control_period_ms = CONTROL_PERIOD_MS;

// Double-buffered drawing surface (sprite)
static M5Canvas ui(&M5.Display);

// I2C speed: higher can reduce loop blocking/phase lag, but only if your wiring/devices are stable.
// We default to 100kHz because 400kHz can cause intermittent I2C failures on some setups (long wires/noisy power),
// which is much worse (no encoder updates => no stop, no steering).
#define I2C_CLOCK_HZ 100000

static const uint32_t i2c_clock_hz = I2C_CLOCK_HZ;

// RUN safety: stop if encoders are missing or not advancing.
// This prevents driving blindly when I2C sensor reads fail.
#define RUN_ENCODER_MISSING_TIMEOUT_MS 400
#define RUN_NO_MOTION_TIMEOUT_MS 1200
#define RUN_NO_MOTION_MIN_PULSES 8

// External buttons on the black digital connector (PORT.B).
// Blue button wired to PORT.B IN, Red button wired to PORT.B OUT.
static int extBlueBtnPin = -1;
static int extRedBtnPin = -1;

// Persistent settings (ESP32 NVS)
static Preferences prefs;
static bool prefs_ok = false;
static void loadPersistedSettings();
static void savePersistedSettings();

// ========================================================================
// BFS GRAPH NAVIGATION CONFIGURATION
// ========================================================================
#define BFS_MAX_NODES 16          // Maximum waypoints/rooms in the graph
#define BFS_MAX_EDGES_PER_NODE 8  // Max adjacent nodes per waypoint
#define BFS_PATH_MAX_LENGTH 16    // Max steps in a path

// Node structure for graph representation
struct BfsNode {
  uint8_t id;                       // Node ID (0-15)
  char name[16];                    // Room/waypoint name
  float x_m;                        // X position (meters)
  float y_m;                        // Y position (meters)
  uint8_t neighbors[BFS_MAX_EDGES_PER_NODE];  // Adjacent node IDs
  uint8_t neighbor_count;           // Number of valid edges
  float distances[BFS_MAX_EDGES_PER_NODE];    // Distance to each neighbor (meters)
};

// BFS state
struct BfsState {
  BfsNode nodes[BFS_MAX_NODES];
  uint8_t node_count;
  uint8_t current_node;             // Current position in graph
  uint8_t goal_node;                // Target position
  uint8_t path[BFS_PATH_MAX_LENGTH];
  uint8_t path_length;
  uint8_t path_index;               // Current step along path
  bool path_valid;
};

static BfsState bfs_state = {0};
static bool bfs_initialized = false;

// Add an edge between two nodes (bidirectional)
static void bfsAddEdge(uint8_t from_id, uint8_t to_id, float distance_m) {
  if (from_id >= BFS_MAX_NODES || to_id >= BFS_MAX_NODES) return;
  
  BfsNode* from = &bfs_state.nodes[from_id];
  if (from->neighbor_count >= BFS_MAX_EDGES_PER_NODE) return;
  
  from->neighbors[from->neighbor_count] = to_id;
  from->distances[from->neighbor_count] = distance_m;
  from->neighbor_count++;
}

// Initialize the BFS graph with waypoints and connections
static void bfsInitializeGraph() {
  // Example graph configuration - customize for your competition venue
  bfs_state.node_count = 4;
  
  // Node 0: Start room
  bfs_state.nodes[0].id = 0;
  strcpy(bfs_state.nodes[0].name, "Start");
  bfs_state.nodes[0].x_m = 0.0f;
  bfs_state.nodes[0].y_m = 0.0f;
  bfs_state.nodes[0].neighbor_count = 0;
  
  // Node 1: Middle room
  bfs_state.nodes[1].id = 1;
  strcpy(bfs_state.nodes[1].name, "Middle");
  bfs_state.nodes[1].x_m = 2.0f;
  bfs_state.nodes[1].y_m = 0.0f;
  bfs_state.nodes[1].neighbor_count = 0;
  
  // Node 2: End room
  bfs_state.nodes[2].id = 2;
  strcpy(bfs_state.nodes[2].name, "End");
  bfs_state.nodes[2].x_m = 4.0f;
  bfs_state.nodes[2].y_m = 0.0f;
  bfs_state.nodes[2].neighbor_count = 0;
  
  // Node 3: Side room
  bfs_state.nodes[3].id = 3;
  strcpy(bfs_state.nodes[3].name, "Side");
  bfs_state.nodes[3].x_m = 2.0f;
  bfs_state.nodes[3].y_m = 2.0f;
  bfs_state.nodes[3].neighbor_count = 0;
  
  // Build edges (connections between rooms)
  bfsAddEdge(0, 1, 2.0f);  // Start -> Middle: 2m
  bfsAddEdge(1, 0, 2.0f);  // Middle -> Start: 2m
  bfsAddEdge(1, 2, 2.0f);  // Middle -> End: 2m
  bfsAddEdge(2, 1, 2.0f);  // End -> Middle: 2m
  bfsAddEdge(1, 3, 2.83f); // Middle -> Side: ~2.83m
  bfsAddEdge(3, 1, 2.83f); // Side -> Middle: ~2.83m
  
  bfs_state.current_node = 0;  // Start at node 0
  bfs_state.goal_node = 2;     // Goal is node 2
  bfs_state.path_valid = false;
  bfs_state.path_length = 0;
  bfs_state.path_index = 0;
  
  bfs_initialized = true;
}

// Breadth-first search to find shortest path from start to goal
static bool bfsComputePath(uint8_t start_node, uint8_t goal_node) {
  if (start_node >= BFS_MAX_NODES || goal_node >= BFS_MAX_NODES) return false;
  
  // Queue for BFS (using circular buffer)
  uint8_t queue[BFS_MAX_NODES * 2];
  uint8_t queue_front = 0, queue_rear = 0;
  
  // Visited tracking
  bool visited[BFS_MAX_NODES];
  uint8_t parent[BFS_MAX_NODES];
  
  memset(visited, false, sizeof(visited));
  memset(parent, 0xFF, sizeof(parent));
  
  // Start BFS
  queue[queue_rear++] = start_node;
  visited[start_node] = true;
  
  while (queue_front < queue_rear) {
    uint8_t current = queue[queue_front++];
    
    if (current == goal_node) {
      // Reconstruct path
      bfs_state.path_length = 0;
      uint8_t node = goal_node;
      
      while (node != 0xFF && bfs_state.path_length < BFS_PATH_MAX_LENGTH) {
        bfs_state.path[bfs_state.path_length++] = node;
        node = parent[node];
      }
      
      // Reverse path (was built backwards)
      for (uint8_t i = 0; i < bfs_state.path_length / 2; i++) {
        uint8_t tmp = bfs_state.path[i];
        bfs_state.path[i] = bfs_state.path[bfs_state.path_length - 1 - i];
        bfs_state.path[bfs_state.path_length - 1 - i] = tmp;
      }
      
      bfs_state.path_index = 0;
      bfs_state.path_valid = true;
      return true;
    }
    
    // Explore neighbors
    BfsNode* node_ptr = &bfs_state.nodes[current];
    for (uint8_t i = 0; i < node_ptr->neighbor_count; i++) {
      uint8_t neighbor = node_ptr->neighbors[i];
      if (!visited[neighbor]) {
        visited[neighbor] = true;
        parent[neighbor] = current;
        if (queue_rear < sizeof(queue)) {
          queue[queue_rear++] = neighbor;
        }
      }
    }
  }
  
  return false;  // No path found
}

// Get next target node along the computed path
static uint8_t bfsGetNextTarget() {
  if (!bfs_state.path_valid || bfs_state.path_index >= bfs_state.path_length) {
    return bfs_state.current_node;
  }
  return bfs_state.path[bfs_state.path_index];
}

// Advance to next waypoint in path
static void bfsAdvancePath() {
  if (bfs_state.path_valid && bfs_state.path_index < bfs_state.path_length - 1) {
    bfs_state.path_index++;
    bfs_state.current_node = bfs_state.path[bfs_state.path_index];
  }
}

// Competition limits (update these when rules are confirmed)
// Allow slight edge adjustments below/above the nominal range.
#define RUN_DISTANCE_MIN_M 6.9f
#define RUN_DISTANCE_MAX_M 10.1f
#define RUN_TIME_MIN_S 10.0f
#define RUN_TIME_MAX_S 20.0f

// Rules-derived parameter granularity:
// - Target Distance interval depends on tournament level.
//   For micro-adjustments, we allow 0.01m (1 cm) steps on the dial.
//   The quantizer will prefer the closer grid; 0.25m is still allowed, but 0.01m
//   lets you dial in per-run calibration without excessive turning.
// - Target Time interval is 0.5s for all tournaments.
#define RUN_DISTANCE_STEP_REGIONAL_M 0.25f
#define RUN_DISTANCE_STEP_STATE_M 0.01f
#define RUN_DISTANCE_DIAL_STEP_M 0.01f
#define RUN_TIME_STEP_S 0.5f

static float quantizeToStep(float v, float step) {
  if (step <= 0.0f) return v;
  return roundf(v / step) * step;
}

static float quantizeToStepDirectional(float v, float step, int direction) {
  if (step <= 0.0f) return v;
  const float x = v / step;
  float q = x;
  if (direction > 0) {
    q = ceilf(x);
  } else if (direction < 0) {
    q = floorf(x);
  } else {
    q = roundf(x);
  }
  return q * step;
}

static float quantizeRunDistanceRules(float v) {
  if (v < RUN_DISTANCE_MIN_M) v = RUN_DISTANCE_MIN_M;
  if (v > RUN_DISTANCE_MAX_M) v = RUN_DISTANCE_MAX_M;

  const float v25 = quantizeToStep(v, RUN_DISTANCE_STEP_REGIONAL_M);
  const float v10 = quantizeToStep(v, RUN_DISTANCE_STEP_STATE_M);

  const float d25 = fabsf(v - v25);
  const float d10 = fabsf(v - v10);

  // Prefer the closer grid; in a tie, prefer the 0.25 grid.
  float out = (d10 < d25) ? v10 : v25;

  if (out < RUN_DISTANCE_MIN_M) out = RUN_DISTANCE_MIN_M;
  if (out > RUN_DISTANCE_MAX_M) out = RUN_DISTANCE_MAX_M;
  return out;
}

static float quantizeRunDistanceRulesDirectional(float v, int direction) {
  if (v < RUN_DISTANCE_MIN_M) v = RUN_DISTANCE_MIN_M;
  if (v > RUN_DISTANCE_MAX_M) v = RUN_DISTANCE_MAX_M;

  const float v25 = quantizeToStepDirectional(v, RUN_DISTANCE_STEP_REGIONAL_M, direction);
  const float v10 = quantizeToStepDirectional(v, RUN_DISTANCE_STEP_STATE_M, direction);

  const float d25 = fabsf(v - v25);
  const float d10 = fabsf(v - v10);

  // Prefer closer grid; in a tie, prefer finer (0.10m) so the dial never gets "stuck".
  float out = (d10 <= d25) ? v10 : v25;

  if (out < RUN_DISTANCE_MIN_M) out = RUN_DISTANCE_MIN_M;
  if (out > RUN_DISTANCE_MAX_M) out = RUN_DISTANCE_MAX_M;
  return out;
}

// Filesystem for run logging (SPIFFS on internal flash)
static bool spiffs_ok = false;

// Record why the last run stopped (helps diagnose early stop conditions).
static const char* lastRunStopReason = "NONE";

// Retain last run stats for display when stopped and for serial 'status'.
static float lastRunElapsedS = 0.0f;
static float lastRunMeters = 0.0f;
static float lastRunForwardM = 0.0f;
static float lastRunLateralM = 0.0f;
static bool lastRunWasCanMode = false;

// Forces control-loop statics to reset even if startRun() blocks long enough that
// (now - runStartMs) is no longer within the short startup window.
static bool runControlResetRequest = false;

#if RUN_CLOUD_UPLOAD_ENABLE
// Deferred cloud-upload state (so we can retry uploads without blocking stopRun())
static bool cloudUploadPending = false;
static uint8_t cloudUploadAttempts = 0;
static unsigned long cloudUploadNextAttemptMs = 0;
static bool cloudUploadLastOk = false;
#endif

// External start debug (counters + last levels)
static uint32_t dbg_redClicks = 0;
static uint32_t dbg_blueClicks = 0;
static int dbg_redLevel = -1;   // raw digitalRead (0/1), or -1 if pin invalid
static int dbg_blueLevel = -1;
static uint32_t dbg_lastStartAttemptMs = 0;
static uint32_t dbg_lastStartOkMs = 0;
static char dbg_lastStartPath[16] = "";  // e.g. "EXT_RED", "BTN_A"

static void startDebugLogLine(const char* msg) {
#if RUN_START_DEBUG_SPIFFS
  if (!spiffs_ok || msg == nullptr) return;
  File f = SPIFFS.open("/start_debug.txt", FILE_APPEND);
  if (!f) return;
  f.printf("%lu %s\n", (unsigned long)millis(), msg);
  f.close();
#else
  (void)msg;
#endif
}

// ========================================================================
// Optional: stream run logs to SPIFFS during the run (survives reset/hit)
// ========================================================================

// When enabled, we write the per-run CSV incrementally during the run. This is
// specifically to support running without USB connected, and to preserve partial
// logs if the car hits something and resets before stopRun() completes.
#define RUNLOG_STREAM_TO_SPIFFS 1

#if RUNLOG_STREAM_TO_SPIFFS
static File runLogStreamFile;
static bool runLogStreamOpen = false;
static char runLogStreamPath[32] = "";
static uint32_t runLogStreamLastFlushMs = 0;
static uint16_t runLogStreamLines = 0;
#endif

// ========================================================================
// Run logging (post-run analysis)
// Logs are stored in RAM during the run, then dumped to Serial and SPIFFS
// after stopping.
// ========================================================================

struct RunLogSample {
  uint32_t t_ms;
  float meters;
  float forwardM;
  float yawRelDeg;
  float yawErrDeg;
  float targetYawDeg;
  float yawIntegral;
  float gz_dps;
  float gz_raw_dps;
  float gz_bias_dps;
  float actualRate;
  float leftRate;
  float rightRate;
  float speedErr;
  float basePWM;
  float corrRaw;
  float corrFiltered;
  float corrOut;
  uint8_t canPhase;
  float turnTargetDeg;
  float turnImuDeg;
  float turnEncDeg;
  float turnFusedDeg;
  uint8_t braking;
  uint8_t lPwm;
  uint8_t rPwm;
  uint8_t mL_err_dir;
  uint8_t mL_err_pwm;
  uint8_t mR_err_dir;
  uint8_t mR_err_pwm;

  // Added for navigation + control-loop analysis (appended at end for compatibility)
  float lateralM;
  float remainingM;
  float nominalTargetRate;
  float effectiveTargetRate;
  float targetRatePps;
  float actualRatePps;
  float speedIntegral;
  float steerIntegral;
  float encTrimPwm;

  // Control-loop timing diagnostics (appended at end for compatibility)
  float loopDtMs;

  // Extra diagnostics for the "right lurch" issue (appended at end for compatibility)
  float corrUsed;
  float motorBiasPwm;
  float satShift;
  float imuReadUs;
  float motorWriteUs;

  // Motor command-path visibility (appended at end for compatibility)
  uint8_t cmdL_dir;
  uint8_t cmdR_dir;
  uint8_t regL_dir;
  uint8_t regL_pwm;
  uint8_t regR_dir;
  uint8_t regR_pwm;
};

// Per-loop I2C timing diagnostics (captured outside the control loop, logged inside it)
static uint32_t runDbg_lastImuReadUs = 0;
static uint32_t runDbg_lastMotorWriteUs = 0;

// Run log storage
// NOTE: keep this buffer out of static RAM (BSS). The UI sprite needs heap,
// and a giant static log buffer can leave the system with too little heap to draw.
// We allocate at runtime, preferring PSRAM when available.
#define RUNLOG_MAX_SAMPLES 2200
static RunLogSample* runLog = nullptr;
static uint16_t runLogCapacity = 0;
static uint16_t runLogCount = 0;
static bool runLogOverflow = false;

// Protect against accidental double-stop (which would overwrite SPIFFS logs).
static unsigned long runLogSavedRunId = 0;
static bool runLogSavedThisRun = false;

// Persisted diagnostics so we can reconcile the UI run number with what was
// actually saved to SPIFFS (useful when a run stops unexpectedly).
static unsigned long runMetaLastAttemptId = 0;
static unsigned long runMetaLastAttemptEpoch = 0;
static unsigned long runMetaLastSavedId = 0;
static unsigned long runMetaLastSavedEpoch = 0;
static unsigned long runMetaLastSavedPerRunBytes = 0;
static unsigned long runMetaLastSavedLastBytes = 0;
static uint8_t runMetaLastSavedPerRunOk = 0;
static uint8_t runMetaLastSavedLastOk = 0;

// Runtime warnings (shown on RUN screen). These should not prevent the run.
static bool runWarnEncMissing = false;
static bool runWarnNoMotion = false;
static bool runWarnLoggingDisabled = false;

// Keep SPIFFS from filling up with old logs.
#ifndef RUNLOG_KEEP_FILES_MAX
#define RUNLOG_KEEP_FILES_MAX 25
#endif
#ifndef RUNLOG_MIN_FREE_FRAC
#define RUNLOG_MIN_FREE_FRAC 0.25f  // keep at least 25% free when possible
#endif

static void runLogPruneOldSpiffsLogsIfNeeded() {
  if (!spiffs_ok) return;

  struct RunLogEntry {
    unsigned long id;
    char name[32];
  };
  RunLogEntry entries[64];
  uint16_t entryCount = 0;

  File root = SPIFFS.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      const char* nm = file.name();
      // Match /runlog_<id>.csv (exclude /runlog_last.csv)
      if (nm && (strncmp(nm, "/runlog_", 8) == 0 || strncmp(nm, "runlog_", 7) == 0)) {
        if (strcmp(nm, "/runlog_last.csv") != 0 && strcmp(nm, "runlog_last.csv") != 0) {
          const char* digits = (strncmp(nm, "/runlog_", 8) == 0) ? (nm + 8) : (nm + 7);
          const char* p = digits;
          bool allDigits = (*p != 0);
          while (*p && *p != '.') {
            if (*p < '0' || *p > '9') {
              allDigits = false;
              break;
            }
            p++;
          }
          if (allDigits && strcmp(p, ".csv") == 0) {
            const unsigned long id = strtoul(digits, nullptr, 10);
            if (entryCount < (sizeof(entries) / sizeof(entries[0]))) {
              entries[entryCount].id = id;
              strncpy(entries[entryCount].name, nm, sizeof(entries[entryCount].name) - 1);
              entries[entryCount].name[sizeof(entries[entryCount].name) - 1] = 0;
              entryCount++;
            }
          }
        }
      }
      file = root.openNextFile();
    }
    root.close();
  }

  // Sort by id ascending (simple insertion sort, entryCount is small).
  for (uint16_t i = 1; i < entryCount; i++) {
    RunLogEntry key = entries[i];
    int j = (int)i - 1;
    while (j >= 0 && entries[j].id > key.id) {
      entries[j + 1] = entries[j];
      j--;
    }
    entries[j + 1] = key;
  }

  const size_t total = SPIFFS.totalBytes();
  size_t used = SPIFFS.usedBytes();
  size_t freeB = (total >= used) ? (total - used) : 0;
  const size_t minFreeB = (size_t)((float)total * (float)RUNLOG_MIN_FREE_FRAC);

  uint16_t idx = 0;
  while (idx < entryCount) {
    const bool tooMany = (entryCount - idx) > (uint16_t)RUNLOG_KEEP_FILES_MAX;
    const bool tooFull = freeB < minFreeB;
    if (!tooMany && !tooFull) break;
    if (!SPIFFS.remove(entries[idx].name)) {
      // Try alternate leading-slash form.
      if (entries[idx].name[0] == '/') {
        SPIFFS.remove(entries[idx].name + 1);
      }
    }
    idx++;
    used = SPIFFS.usedBytes();
    freeB = (total >= used) ? (total - used) : 0;
    delay(0);
  }
}

// Control-loop timing diagnostics (helps debug stalls / missing sample gaps)
static uint32_t runDbg_maxCtrlDtMs = 0;
static uint32_t runDbg_maxCtrlDtAtTms = 0;
static uint32_t runDbg_ctrlOverrunCount = 0;
static uint32_t runDbg_ctrlStall200msCount = 0;

// Run log decimation (computed per-run so long runs don't overflow RAM)
static uint16_t runLogEveryN = 1;
static uint16_t runLogTick = 0;

// Printing a full CSV to Serial at stop can stall/hang when USB is not connected.
// Prefer pulling from SPIFFS after reconnecting.
#ifndef RUNLOG_PRINT_TO_SERIAL_ON_STOP
#define RUNLOG_PRINT_TO_SERIAL_ON_STOP 0
#endif

// If the UI run time is set very large (e.g. to avoid timeouts), using it directly
// makes logs too sparse to analyze. Clamp the expectation used for decimation.
#ifndef RUNLOG_DECIMATION_EXPECTED_S_MAX
#define RUNLOG_DECIMATION_EXPECTED_S_MAX 120.0f
#endif

// Starting a run should be immediate; the IMU already refines bias while stopped.
// A blocking "quick cal" here can cause multi-second perceived latency if I2C is unhappy.
#ifndef RUN_START_IMU_QUICKCAL_ENABLE
#define RUN_START_IMU_QUICKCAL_ENABLE 0
#endif

// ========================================================================
// Test metadata (printed into logs so each run is self-describing)
// ========================================================================

enum ControlMode : uint8_t {
  CTRL_MODE_IMU = 0,
  CTRL_MODE_ENCODER = 1,
  CTRL_MODE_HYBRID = 2,
};

static const char* controlModeName(ControlMode m) {
  switch (m) {
    case CTRL_MODE_IMU:
      return "IMU";
    case CTRL_MODE_ENCODER:
      return "ENC";
    case CTRL_MODE_HYBRID:
      return "HYBRID";
    default:
      return "?";
  }
}

// Default to encoder-primary control for repeatable straight-line behavior.
static ControlMode control_mode = CTRL_MODE_ENCODER;

struct RunMeta {
  uint32_t id;
  uint32_t start_ms;
  float runDistanceM;
  float runTimeS;
  float canDistanceM;
  float pulsesPerMeter;
  float pulsesPerMeterL;
  float pulsesPerMeterR;
  int16_t motor_bias_pwm;
  uint32_t i2c_clock_hz;
  uint16_t control_period_ms;
  uint8_t control_mode;
  uint8_t imu_present;
  uint8_t imu_ok_at_start;
  uint8_t run_lock_pwm_enable;
  float run_lock_pwm_value;
  uint16_t run_softstart_ms;
};

static RunMeta runMeta = {0};
static uint32_t runMetaNextId = 1;
static time_t runStartEpoch = 0;

static bool timeIsValidUtc() {
  // Treat anything before 2024-01-01 as "unset".
  const time_t now = time(nullptr);
  return now > (time_t)1704067200;
}

static void formatUtcIso8601(time_t t, char* out, size_t outSize) {
  if (!out || outSize == 0) return;
  if (t <= 0) {
    out[0] = 0;
    return;
  }
  struct tm tmUtc;
  gmtime_r(&t, &tmUtc);
  strftime(out, outSize, "%Y-%m-%dT%H:%M:%SZ", &tmUtc);
}

static void printSpiffsUsageToSerial(const char* tag) {
  if (!tag) tag = "SPIFFS";
  if (!spiffs_ok) {
    Serial.printf("%s: FAIL\n", tag);
    return;
  }
  const size_t total = SPIFFS.totalBytes();
  const size_t used = SPIFFS.usedBytes();
  const size_t freeB = (total >= used) ? (total - used) : 0;
  Serial.printf("%s: total=%u used=%u free=%u\n", tag, (unsigned)total, (unsigned)used, (unsigned)freeB);
}

static void trySyncTimeWithNtpQuick() {
  // Sync system UTC time via NTP when WiFi is available.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  const unsigned long t0 = millis();
  while (!timeIsValidUtc() && (millis() - t0) < 1500) {
    delay(25);
  }
}

static void printRunMetaToSerial(const char* tag) {
  Serial.printf("%s id=%lu fw=%s mode=%s dist=%.3f time=%.3f can=%.3f ppm=%.1f ppmL=%.1f ppmR=%.1f mtr_bias=%d i2c_hz=%lu ctrl_ms=%u imu_present=%u imu_ok_start=%u lock_pwm=%u lock_pwm_val=%.1f softstart_ms=%u\n",
                tag,
                (unsigned long)runMeta.id,
                FW_VERSION,
                controlModeName((ControlMode)runMeta.control_mode),
                (double)runMeta.runDistanceM,
                (double)runMeta.runTimeS,
                (double)runMeta.canDistanceM,
                (double)runMeta.pulsesPerMeter,
                (double)runMeta.pulsesPerMeterL,
                (double)runMeta.pulsesPerMeterR,
                (int)runMeta.motor_bias_pwm,
                (unsigned long)runMeta.i2c_clock_hz,
                (unsigned)runMeta.control_period_ms,
                (unsigned)runMeta.imu_present,
                (unsigned)runMeta.imu_ok_at_start,
                (unsigned)runMeta.run_lock_pwm_enable,
                (double)runMeta.run_lock_pwm_value,
                (unsigned)runMeta.run_softstart_ms);
}

static void runLogReset() {
  // Lazy-init the log buffer.
  if (runLog == nullptr || runLogCapacity == 0) {
    const size_t bytes = (size_t)RUNLOG_MAX_SAMPLES * sizeof(RunLogSample);
    void* mem = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mem) {
      // Fallback: allocate a smaller buffer in internal RAM.
      const uint16_t fallbackSizes[] = {900, 700, 500, 350, 250, 180, 120};
      for (uint16_t i = 0; i < (sizeof(fallbackSizes) / sizeof(fallbackSizes[0])); i++) {
        const uint16_t n = fallbackSizes[i];
        const size_t bytes2 = (size_t)n * sizeof(RunLogSample);
        mem = heap_caps_malloc(bytes2, MALLOC_CAP_8BIT);
        if (mem) {
          runLog = (RunLogSample*)mem;
          runLogCapacity = n;
          break;
        }
      }
    } else {
      runLog = (RunLogSample*)mem;
      runLogCapacity = RUNLOG_MAX_SAMPLES;
    }

    if (!runLog || runLogCapacity == 0) {
      Serial.println("RunLog: allocation failed (no memory)");
      startDebugLogLine("RunLog: allocation failed (no memory)");
    } else {
      Serial.printf("RunLog: allocated %u samples (%u bytes)\n",
                    (unsigned)runLogCapacity,
                    (unsigned)(runLogCapacity * sizeof(RunLogSample)));
    }
  }

  runLogCount = 0;
  runLogOverflow = false;
}

#if RUNLOG_STREAM_TO_SPIFFS
static void runLogStreamBegin() {
  if (!spiffs_ok) return;
  if (runMeta.id == 0) return;

  // Ensure there is space BEFORE we create/write a new per-run log.
  // If SPIFFS is nearly full, file writes can silently fail and leave 0-byte logs.
  runLogPruneOldSpiffsLogsIfNeeded();

  runLogStreamOpen = false;
  runLogStreamLines = 0;
  runLogStreamLastFlushMs = (uint32_t)millis();

  snprintf(runLogStreamPath, sizeof(runLogStreamPath), "/runlog_%lu.csv", (unsigned long)runMeta.id);
  runLogStreamFile = SPIFFS.open(runLogStreamPath, "w");
  if (!runLogStreamFile) {
    // Some SPIFFS implementations require paths without a leading '/'.
    runLogStreamFile = SPIFFS.open(runLogStreamPath + 1, "w");
  }
  if (!runLogStreamFile) {
    startDebugLogLine("RUNLOG_STREAM open_fail");
    return;
  }

  // Minimal header (compatible with plot tools).
  char nowIso[32];
  nowIso[0] = 0;
  const time_t now = time(nullptr);
  if (timeIsValidUtc()) formatUtcIso8601(now, nowIso, sizeof(nowIso));

  char startIso[32];
  startIso[0] = 0;
  if (runStartEpoch > 0 && timeIsValidUtc()) formatUtcIso8601(runStartEpoch, startIso, sizeof(startIso));

  runLogStreamFile.printf(
      "# TEST_BEGIN id=%lu fw=%s mode=%s dist=%.3f time=%.3f can=%.3f ppm=%.1f ppmL=%.1f ppmR=%.1f mtr_bias=%d i2c_hz=%lu ctrl_ms=%u imu_present=%u imu_ok_start=%u lock_pwm=%u lock_pwm_val=%.1f softstart_ms=%u\n",
      (unsigned long)runMeta.id,
      FW_VERSION,
      controlModeName((ControlMode)runMeta.control_mode),
      (double)runMeta.runDistanceM,
      (double)runMeta.runTimeS,
      (double)runMeta.canDistanceM,
      (double)runMeta.pulsesPerMeter,
      (double)runMeta.pulsesPerMeterL,
      (double)runMeta.pulsesPerMeterR,
      (int)runMeta.motor_bias_pwm,
      (unsigned long)runMeta.i2c_clock_hz,
      (unsigned)runMeta.control_period_ms,
      (unsigned)runMeta.imu_present,
      (unsigned)runMeta.imu_ok_at_start,
      (unsigned)runMeta.run_lock_pwm_enable,
      (double)runMeta.run_lock_pwm_value,
      (unsigned)runMeta.run_softstart_ms);
  if (nowIso[0]) {
    runLogStreamFile.printf("# utc_now=%s epoch_now=%lu\n", nowIso, (unsigned long)now);
  }
  if (startIso[0]) {
    runLogStreamFile.printf("# utc_start=%s epoch_start=%lu\n", startIso, (unsigned long)runStartEpoch);
  }
  {
    const size_t total = SPIFFS.totalBytes();
    const size_t used = SPIFFS.usedBytes();
    const size_t freeB = (total >= used) ? (total - used) : 0;
    runLogStreamFile.printf("# spiffs_total=%u spiffs_used=%u spiffs_free=%u\n",
                            (unsigned)total,
                            (unsigned)used,
                            (unsigned)freeB);
  }
    runLogStreamFile.println(
      "t_ms,meters,forwardM,yawRelDeg,yawErrDeg,targetYawDeg,yawI,gz_dps,gzRaw_dps,gzBias_dps,actualRate,leftRate,rightRate,speedErr,basePWM,corrRaw,corrFiltered,corrOut,canPhase,turnTargetDeg,turnImuDeg,turnEncDeg,turnFusedDeg,braking,lPwm,rPwm,mL_err_dir,mL_err_pwm,mR_err_dir,mR_err_pwm,lateralM,remainingM,nomTargetRate,effTargetRate,targetRatePps,actualRatePps,speedI,steerI,encTrimPwm,loopDtMs,corrUsed,motorBiasPwm,satShift,imuReadUs,motorWriteUs,cmdL_dir,cmdR_dir,regL_dir,regL_pwm,regR_dir,regR_pwm");
  runLogStreamFile.flush();
  runLogStreamOpen = true;

  char line[96];
  snprintf(line, sizeof(line), "RUNLOG_STREAM open_ok id=%lu", (unsigned long)runMeta.id);
  startDebugLogLine(line);
}

static void runLogStreamAppend(const RunLogSample& s) {
  if (!runLogStreamOpen) return;
  if (!runLogStreamFile) return;

  char line[720];
  snprintf(line, sizeof(line),
           "%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u,%u,%u",
           (unsigned long)s.t_ms,
           (double)s.meters,
           (double)s.forwardM,
           (double)s.yawRelDeg,
           (double)s.yawErrDeg,
           (double)s.targetYawDeg,
           (double)s.yawIntegral,
           (double)s.gz_dps,
           (double)s.gz_raw_dps,
           (double)s.gz_bias_dps,
           (double)s.actualRate,
           (double)s.leftRate,
           (double)s.rightRate,
           (double)s.speedErr,
           (double)s.basePWM,
           (double)s.corrRaw,
           (double)s.corrFiltered,
           (double)s.corrOut,
           (unsigned)s.canPhase,
           (double)s.turnTargetDeg,
           (double)s.turnImuDeg,
           (double)s.turnEncDeg,
           (double)s.turnFusedDeg,
           (unsigned)s.braking,
           (unsigned)s.lPwm,
           (unsigned)s.rPwm,
           (unsigned)s.mL_err_dir,
           (unsigned)s.mL_err_pwm,
           (unsigned)s.mR_err_dir,
           (unsigned)s.mR_err_pwm,
           (double)s.lateralM,
           (double)s.remainingM,
           (double)s.nominalTargetRate,
           (double)s.effectiveTargetRate,
           (double)s.targetRatePps,
           (double)s.actualRatePps,
           (double)s.speedIntegral,
           (double)s.steerIntegral,
           (double)s.encTrimPwm,
           (double)s.loopDtMs,
           (double)s.corrUsed,
           (double)s.motorBiasPwm,
           (double)s.satShift,
           (double)s.imuReadUs,
           (double)s.motorWriteUs,
           (unsigned)s.cmdL_dir,
           (unsigned)s.cmdR_dir,
           (unsigned)s.regL_dir,
           (unsigned)s.regL_pwm,
           (unsigned)s.regR_dir,
           (unsigned)s.regR_pwm);

  runLogStreamFile.println(line);
  runLogStreamLines++;

  const uint32_t nowMs = (uint32_t)millis();
  if ((nowMs - runLogStreamLastFlushMs) >= 250 || (runLogStreamLines % 40) == 0) {
    runLogStreamFile.flush();
    runLogStreamLastFlushMs = nowMs;
  }
}

static void runLogStreamEnd() {
  if (!runLogStreamOpen) return;
  if (!runLogStreamFile) {
    runLogStreamOpen = false;
    return;
  }

  runLogStreamFile.printf(
      "# TEST_END id=%lu samples=%u ram_samples=%u overflow=%s stop=%s max_ctrl_dt_ms=%lu max_ctrl_dt_at_ms=%lu ctrl_overruns=%lu ctrl_stalls200ms=%lu\n",
      (unsigned long)runMeta.id,
      (unsigned)runLogStreamLines,
      (unsigned)runLogCount,
      runLogOverflow ? "YES" : "no",
      lastRunStopReason ? lastRunStopReason : "?",
      (unsigned long)runDbg_maxCtrlDtMs,
      (unsigned long)runDbg_maxCtrlDtAtTms,
      (unsigned long)runDbg_ctrlOverrunCount,
      (unsigned long)runDbg_ctrlStall200msCount);
  runLogStreamFile.flush();
  runLogStreamFile.close();
  runLogStreamOpen = false;

  // Best-effort size: if the file isn't found with a leading '/', try without.
  runMetaLastSavedPerRunOk = 0;
  runMetaLastSavedPerRunBytes = 0;
  File f = SPIFFS.open(runLogStreamPath, "r");
  if (!f && runLogStreamPath[0] == '/') f = SPIFFS.open(runLogStreamPath + 1, "r");
  if (f) {
    runMetaLastSavedPerRunBytes = (unsigned long)f.size();
    if (runMetaLastSavedPerRunBytes > 0) runMetaLastSavedPerRunOk = 1;
    f.close();
  }
}
#endif

static void runLogAppend(const RunLogSample& s) {
#if RUNLOG_STREAM_TO_SPIFFS
  // Stream-to-SPIFFS must not depend on RAM buffer availability.
  runLogStreamAppend(s);
#endif
  if (!runLog || runLogCapacity == 0) {
    runLogOverflow = true;
    return;
  }
  if (runLogCount < runLogCapacity) {
    runLog[runLogCount++] = s;
    return;
  }
  runLogOverflow = true;
}

static void runLogPrintCsvToSerial() {
  Serial.println("LOG_BEGIN");
  printRunMetaToSerial("TEST_BEGIN");
    Serial.println(
      "t_ms,meters,forwardM,yawRelDeg,yawErrDeg,targetYawDeg,yawI,gz_dps,gzRaw_dps,gzBias_dps,actualRate,leftRate,rightRate,speedErr,basePWM,corrRaw,corrFiltered,corrOut,canPhase,turnTargetDeg,turnImuDeg,turnEncDeg,turnFusedDeg,braking,lPwm,rPwm,mL_err_dir,mL_err_pwm,mR_err_dir,mR_err_pwm,lateralM,remainingM,nomTargetRate,effTargetRate,targetRatePps,actualRatePps,speedI,steerI,encTrimPwm,loopDtMs,corrUsed,motorBiasPwm,satShift,imuReadUs,motorWriteUs,cmdL_dir,cmdR_dir,regL_dir,regL_pwm,regR_dir,regR_pwm");
  for (uint16_t i = 0; i < runLogCount; i++) {
    const RunLogSample& s = runLog[i];
    Serial.printf(
      "%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u,%u,%u\n",
                  (unsigned long)s.t_ms,
                  (double)s.meters,
                  (double)s.forwardM,
                  (double)s.yawRelDeg,
                  (double)s.yawErrDeg,
                  (double)s.targetYawDeg,
                  (double)s.yawIntegral,
                  (double)s.gz_dps,
                  (double)s.gz_raw_dps,
                  (double)s.gz_bias_dps,
                  (double)s.actualRate,
                  (double)s.leftRate,
                  (double)s.rightRate,
                  (double)s.speedErr,
                  (double)s.basePWM,
                  (double)s.corrRaw,
                  (double)s.corrFiltered,
                  (double)s.corrOut,
                  (unsigned)s.canPhase,
                  (double)s.turnTargetDeg,
                  (double)s.turnImuDeg,
                  (double)s.turnEncDeg,
                  (double)s.turnFusedDeg,
                  (unsigned)s.braking,
                  (unsigned)s.lPwm,
                  (unsigned)s.rPwm,
                  (unsigned)s.mL_err_dir,
                  (unsigned)s.mL_err_pwm,
                  (unsigned)s.mR_err_dir,
                  (unsigned)s.mR_err_pwm,
                  (double)s.lateralM,
                  (double)s.remainingM,
                  (double)s.nominalTargetRate,
                  (double)s.effectiveTargetRate,
                  (double)s.targetRatePps,
                  (double)s.actualRatePps,
                  (double)s.speedIntegral,
                  (double)s.steerIntegral,
                  (double)s.encTrimPwm,
                  (double)s.loopDtMs,
                  (double)s.corrUsed,
                  (double)s.motorBiasPwm,
                  (double)s.satShift,
                  (double)s.imuReadUs,
                  (double)s.motorWriteUs,
                  (unsigned)s.cmdL_dir,
                  (unsigned)s.cmdR_dir,
                  (unsigned)s.regL_dir,
                  (unsigned)s.regL_pwm,
                  (unsigned)s.regR_dir,
                  (unsigned)s.regR_pwm);
  }
  printRunMetaToSerial("TEST_END");
  Serial.printf("LOG_END samples=%u overflow=%s stop=%s\n",
                (unsigned)runLogCount,
                runLogOverflow ? "YES" : "no",
                lastRunStopReason ? lastRunStopReason : "?");

  // Extra diagnostics: helps detect long pauses in the control loop.
  Serial.printf("CTRL_STATS max_dt_ms=%lu max_dt_at_ms=%lu overruns=%lu stalls200ms=%lu\n",
                (unsigned long)runDbg_maxCtrlDtMs,
                (unsigned long)runDbg_maxCtrlDtAtTms,
                (unsigned long)runDbg_ctrlOverrunCount,
                (unsigned long)runDbg_ctrlStall200msCount);
}

static void runLogSaveToSpiffs() {
  if (!spiffs_ok) return;

  // Ensure we have breathing room before attempting any (potentially large) writes.
  runLogPruneOldSpiffsLogsIfNeeded();

  // Clear last-save status for this attempt.
  runMetaLastSavedPerRunOk = 0;
  runMetaLastSavedLastOk = 0;
  runMetaLastSavedPerRunBytes = 0;
  runMetaLastSavedLastBytes = 0;

  auto writeCsv = [&](File& f) {
    char nowIso[32];
    nowIso[0] = 0;
    const time_t now = time(nullptr);
    if (timeIsValidUtc()) formatUtcIso8601(now, nowIso, sizeof(nowIso));

    char startIso[32];
    startIso[0] = 0;
    if (runStartEpoch > 0 && timeIsValidUtc()) formatUtcIso8601(runStartEpoch, startIso, sizeof(startIso));

    // Put run metadata at the top as comment lines (safe for CSV readers).
        f.printf("# TEST_BEGIN id=%lu fw=%s mode=%s dist=%.3f time=%.3f can=%.3f ppm=%.1f ppmL=%.1f ppmR=%.1f mtr_bias=%d i2c_hz=%lu ctrl_ms=%u imu_present=%u imu_ok_start=%u lock_pwm=%u lock_pwm_val=%.1f softstart_ms=%u\n",
             (unsigned long)runMeta.id,
             FW_VERSION,
             controlModeName((ControlMode)runMeta.control_mode),
             (double)runMeta.runDistanceM,
             (double)runMeta.runTimeS,
             (double)runMeta.canDistanceM,
             (double)runMeta.pulsesPerMeter,
          (double)runMeta.pulsesPerMeterL,
          (double)runMeta.pulsesPerMeterR,
             (int)runMeta.motor_bias_pwm,
             (unsigned long)runMeta.i2c_clock_hz,
             (unsigned)runMeta.control_period_ms,
             (unsigned)runMeta.imu_present,
             (unsigned)runMeta.imu_ok_at_start,
             (unsigned)runMeta.run_lock_pwm_enable,
             (double)runMeta.run_lock_pwm_value,
             (unsigned)runMeta.run_softstart_ms);
    if (nowIso[0]) {
      f.printf("# utc_now=%s epoch_now=%lu\n", nowIso, (unsigned long)now);
    }
    if (startIso[0]) {
      f.printf("# utc_start=%s epoch_start=%lu\n", startIso, (unsigned long)runStartEpoch);
    }
    {
      const size_t total = SPIFFS.totalBytes();
      const size_t used = SPIFFS.usedBytes();
      const size_t freeB = (total >= used) ? (total - used) : 0;
      f.printf("# spiffs_total=%u spiffs_used=%u spiffs_free=%u\n", (unsigned)total, (unsigned)used, (unsigned)freeB);
    }
    f.println(
      "t_ms,meters,forwardM,yawRelDeg,yawErrDeg,targetYawDeg,yawI,gz_dps,gzRaw_dps,gzBias_dps,actualRate,leftRate,rightRate,speedErr,basePWM,corrRaw,corrFiltered,corrOut,canPhase,turnTargetDeg,turnImuDeg,turnEncDeg,turnFusedDeg,braking,lPwm,rPwm,mL_err_dir,mL_err_pwm,mR_err_dir,mR_err_pwm,lateralM,remainingM,nomTargetRate,effTargetRate,targetRatePps,actualRatePps,speedI,steerI,encTrimPwm,loopDtMs,corrUsed,motorBiasPwm,satShift,imuReadUs,motorWriteUs,cmdL_dir,cmdR_dir,regL_dir,regL_pwm,regR_dir,regR_pwm");
    for (uint16_t i = 0; i < runLogCount; i++) {
      const RunLogSample& s = runLog[i];
      char line[720];
      snprintf(line, sizeof(line),
           "%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u,%u,%u",
               (unsigned long)s.t_ms,
               (double)s.meters,
               (double)s.forwardM,
               (double)s.yawRelDeg,
               (double)s.yawErrDeg,
               (double)s.targetYawDeg,
               (double)s.yawIntegral,
               (double)s.gz_dps,
               (double)s.gz_raw_dps,
               (double)s.gz_bias_dps,
               (double)s.actualRate,
               (double)s.leftRate,
               (double)s.rightRate,
               (double)s.speedErr,
               (double)s.basePWM,
               (double)s.corrRaw,
               (double)s.corrFiltered,
               (double)s.corrOut,
               (unsigned)s.canPhase,
               (double)s.turnTargetDeg,
               (double)s.turnImuDeg,
               (double)s.turnEncDeg,
               (double)s.turnFusedDeg,
               (unsigned)s.braking,
               (unsigned)s.lPwm,
               (unsigned)s.rPwm,
               (unsigned)s.mL_err_dir,
               (unsigned)s.mL_err_pwm,
               (unsigned)s.mR_err_dir,
               (unsigned)s.mR_err_pwm,
               (double)s.lateralM,
               (double)s.remainingM,
               (double)s.nominalTargetRate,
               (double)s.effectiveTargetRate,
               (double)s.targetRatePps,
               (double)s.actualRatePps,
               (double)s.speedIntegral,
               (double)s.steerIntegral,
               (double)s.encTrimPwm,
               (double)s.loopDtMs,
               (double)s.corrUsed,
               (double)s.motorBiasPwm,
               (double)s.satShift,
               (double)s.imuReadUs,
               (double)s.motorWriteUs,
               (unsigned)s.cmdL_dir,
               (unsigned)s.cmdR_dir,
               (unsigned)s.regL_dir,
               (unsigned)s.regL_pwm,
               (unsigned)s.regR_dir,
               (unsigned)s.regR_pwm);
      f.println(line);
    }
    f.printf("# TEST_END id=%lu samples=%u overflow=%s stop=%s max_ctrl_dt_ms=%lu max_ctrl_dt_at_ms=%lu ctrl_overruns=%lu ctrl_stalls200ms=%lu\n",
             (unsigned long)runMeta.id,
             (unsigned)runLogCount,
             runLogOverflow ? "YES" : "no",
             lastRunStopReason ? lastRunStopReason : "?",
             (unsigned long)runDbg_maxCtrlDtMs,
             (unsigned long)runDbg_maxCtrlDtAtTms,
             (unsigned long)runDbg_ctrlOverrunCount,
             (unsigned long)runDbg_ctrlStall200msCount);
  };

  // Save a unique file per run so data isn't lost if multiple runs happen before a pull.
  // IMPORTANT: When streaming is enabled, the per-run file has already been written
  // incrementally during the run. Do NOT reopen it with "w" here (that would truncate it).
  char perRunPath[32];
  snprintf(perRunPath, sizeof(perRunPath), "/runlog_%lu.csv", (unsigned long)runMeta.id);

#if RUNLOG_STREAM_TO_SPIFFS
  // Prefer the streamed file. If it is missing/empty, fall back to the RAM snapshot.
  {
    File fCheck = SPIFFS.open(perRunPath, "r");
    if (!fCheck) fCheck = SPIFFS.open(perRunPath + 1, "r");
    if (fCheck) {
      runMetaLastSavedPerRunBytes = (unsigned long)fCheck.size();
      runMetaLastSavedPerRunOk = (runMetaLastSavedPerRunBytes > 0) ? 1 : 0;
      fCheck.close();
    }
  }
  if (!runMetaLastSavedPerRunOk) {
    File fRun = SPIFFS.open(perRunPath, "w");
    if (!fRun) fRun = SPIFFS.open(perRunPath + 1, "w");
    if (!fRun) {
    } else {
      writeCsv(fRun);
      runMetaLastSavedPerRunBytes = (unsigned long)fRun.size();
      fRun.close();
      runMetaLastSavedPerRunOk = (runMetaLastSavedPerRunBytes > 0) ? 1 : 0;
      Serial.printf("RunLog: saved fallback to SPIFFS %s (%u samples)\n", perRunPath, (unsigned)runLogCount);
    }
  }
#else
  File fRun = SPIFFS.open(perRunPath, "w");
  if (!fRun) {
    // Some SPIFFS implementations require paths without a leading '/'.
    fRun = SPIFFS.open(perRunPath + 1, "w");
  }
  if (!fRun) {
    Serial.println("RunLog: SPIFFS open failed (per-run)");
  } else {
    writeCsv(fRun);
    runMetaLastSavedPerRunBytes = (unsigned long)fRun.size();
    fRun.close();
    runMetaLastSavedPerRunOk = (runMetaLastSavedPerRunBytes > 0) ? 1 : 0;
    Serial.printf("RunLog: saved to SPIFFS %s (%u samples)\n", perRunPath, (unsigned)runLogCount);
  }
#endif

  // Also keep the legacy location for existing tools.
  // In streaming mode, prefer copying the per-run file so /runlog_last.csv matches it.
  auto copyFile = [&](const char* src, const char* dst) -> bool {
    if (!src || !dst) return false;
    File fin = SPIFFS.open(src, "r");
    if (!fin && src[0] == '/') fin = SPIFFS.open(src + 1, "r");
    if (!fin) return false;
    File fout = SPIFFS.open(dst, "w");
    if (!fout && dst[0] == '/') fout = SPIFFS.open(dst + 1, "w");
    if (!fout) {
      fin.close();
      return false;
    }
    uint8_t buf[512];
    while (fin.available()) {
      const size_t n = fin.read(buf, sizeof(buf));
      if (n == 0) break;
      const size_t w = fout.write(buf, n);
      if (w != n) break;
      // Feed WDT during larger copies.
      delay(0);
    }
    fout.flush();
    runMetaLastSavedLastBytes = (unsigned long)fout.size();
    fin.close();
    fout.close();
    return runMetaLastSavedLastBytes > 0;
  };

#if RUNLOG_STREAM_TO_SPIFFS
  if (runMetaLastSavedPerRunOk && runMetaLastSavedPerRunBytes > 0) {
    if (copyFile(perRunPath, "/runlog_last.csv")) {
      runMetaLastSavedLastOk = 1;
    } else {
      // Fallback if copy failed.
      File fLast = SPIFFS.open("/runlog_last.csv", "w");
      if (!fLast) fLast = SPIFFS.open("runlog_last.csv", "w");
      if (fLast) {
        writeCsv(fLast);
        runMetaLastSavedLastBytes = (unsigned long)fLast.size();
        fLast.close();
        runMetaLastSavedLastOk = (runMetaLastSavedLastBytes > 0) ? 1 : 0;
      } else {
        Serial.println("RunLog: SPIFFS open failed (/runlog_last.csv)");
      }
    }
  } else {
    // No per-run file available; fall back to RAM snapshot.
    File fLast = SPIFFS.open("/runlog_last.csv", "w");
    if (!fLast) fLast = SPIFFS.open("runlog_last.csv", "w");
    if (!fLast) {
      Serial.println("RunLog: SPIFFS open failed (/runlog_last.csv)");
    } else {
      writeCsv(fLast);
      runMetaLastSavedLastBytes = (unsigned long)fLast.size();
      fLast.close();
      runMetaLastSavedLastOk = (runMetaLastSavedLastBytes > 0) ? 1 : 0;
    }
  }
#else
  File fLast = SPIFFS.open("/runlog_last.csv", "w");
  if (!fLast) {
    fLast = SPIFFS.open("runlog_last.csv", "w");
  }
  if (!fLast) {
    Serial.println("RunLog: SPIFFS open failed (/runlog_last.csv)");
  } else {
    writeCsv(fLast);
    runMetaLastSavedLastBytes = (unsigned long)fLast.size();
    fLast.close();
    runMetaLastSavedLastOk = (runMetaLastSavedLastBytes > 0) ? 1 : 0;
  }
#endif

  // Persist last-save diagnostics (best-effort; do not break competition behavior).
  if (prefs_ok) {
    const unsigned long now = (unsigned long)time(nullptr);
    runMetaLastSavedEpoch = now;
    prefs.putULong("rls_ep", runMetaLastSavedEpoch);
    // Only advance the "last saved" id if we successfully wrote at least one file.
    if (runMetaLastSavedPerRunOk || runMetaLastSavedLastOk) {
      runMetaLastSavedId = (unsigned long)runMeta.id;
      prefs.putULong("rls_id", runMetaLastSavedId);
    }
    prefs.putULong("rls_b", runMetaLastSavedPerRunBytes);
    prefs.putULong("rls_lb", runMetaLastSavedLastBytes);
    prefs.putUChar("rls_ok", (uint8_t)runMetaLastSavedPerRunOk);
    prefs.putUChar("rls_lok", (uint8_t)runMetaLastSavedLastOk);
  }

  // Prune again after saving (keeps steady-state bounded).
  runLogPruneOldSpiffsLogsIfNeeded();
}

static void runLogDumpSpiffsToSerial() {
  if (!spiffs_ok) {
    Serial.println("RunLog: SPIFFS not available");
    return;
  }
  File f = SPIFFS.open("/runlog_last.csv", "r");
  if (!f) {
    f = SPIFFS.open("runlog_last.csv", "r");
  }
  if (!f) {
    Serial.println("RunLog: no saved file (/runlog_last.csv)");
    return;
  }
  Serial.println("LOG_BEGIN");
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
  f.close();
  if (!Serial.availableForWrite()) {
    // no-op; just avoid unused warnings on some cores
  }
  Serial.println();
  Serial.println("LOG_END file=/runlog_last.csv");
}

static void runLogDumpSpiffsToSerialFile(const char* path) {
  if (!spiffs_ok) {
    Serial.println("RunLog: SPIFFS not available");
    return;
  }
  if (!path || path[0] == 0) {
    Serial.println("RunLog: invalid path");
    return;
  }
  File f = SPIFFS.open(path, "r");
  if (!f && path[0] == '/') {
    f = SPIFFS.open(path + 1, "r");
  }
  if (!f) {
    Serial.printf("RunLog: no saved file (%s)\n", path);
    return;
  }
  Serial.printf("LOG_BEGIN file=%s\n", path);
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
  f.close();
  Serial.println();
  Serial.printf("LOG_END file=%s\n", path);
}

static void runLogListSpiffsToSerial() {
  if (!spiffs_ok) {
    Serial.println("RUNLOG_LIST_BEGIN spiffs=FAIL");
    Serial.println("RUNLOG_LIST_END");
    return;
  }
  {
    const size_t total = SPIFFS.totalBytes();
    const size_t used = SPIFFS.usedBytes();
    const size_t freeB = (total >= used) ? (total - used) : 0;
    Serial.printf("RUNLOG_LIST_BEGIN total=%u used=%u free=%u\n", (unsigned)total, (unsigned)used, (unsigned)freeB);
  }

  File root = SPIFFS.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      const char* nm = file.name();
      if (nm) {
        Serial.printf("RUNLOG name=%s bytes=%u\n", nm, (unsigned)file.size());
      }
      file = root.openNextFile();
    }
    root.close();
  }

  Serial.println("RUNLOG_LIST_END");
}

static bool runLogDeleteById(unsigned long id) {
  if (!spiffs_ok || id == 0) return false;
  char path[32];
  snprintf(path, sizeof(path), "/runlog_%lu.csv", id);
  if (SPIFFS.exists(path)) {
    return SPIFFS.remove(path);
  }
  // Try without leading slash.
  if (SPIFFS.exists(path + 1)) {
    return SPIFFS.remove(path + 1);
  }
  return false;
}

static void startDebugDumpSpiffsToSerial() {
  if (!spiffs_ok) {
    Serial.println("StartDebug: SPIFFS not available");
    return;
  }
  File f = SPIFFS.open("/start_debug.txt", "r");
  if (!f) {
    Serial.println("StartDebug: no saved file (/start_debug.txt)");
    return;
  }
  Serial.println("STARTDBG_BEGIN file=/start_debug.txt");
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
  f.close();
  Serial.println();
  Serial.println("STARTDBG_END");
}

static void startDebugDumpSpiffsTailToSerial(uint16_t lastLines) {
  if (!spiffs_ok) {
    Serial.println("StartDebug: SPIFFS not available");
    return;
  }
  if (lastLines == 0) lastLines = 1;
  if (lastLines > 2000) lastLines = 2000;

  File f = SPIFFS.open("/start_debug.txt", "r");
  if (!f) {
    Serial.println("StartDebug: no saved file (/start_debug.txt)");
    return;
  }

  // Tail by scanning newline offsets (same approach as runlog tail).
  const size_t cap = (size_t)lastLines + 1;
  size_t* nlOff = (size_t*)malloc(sizeof(size_t) * cap);
  if (!nlOff) {
    Serial.println("StartDebug: tail OOM");
    f.close();
    return;
  }
  size_t nlCount = 0;
  size_t pos = 0;
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    pos++;
    if (c == '\n') {
      nlOff[nlCount % cap] = pos;
      nlCount++;
    }
  }

  size_t start = 0;
  if (nlCount > lastLines) {
    start = nlOff[(nlCount - lastLines - 1) % cap];
  } else {
    start = 0;
  }
  free(nlOff);

  f.seek(start, SeekSet);
  Serial.printf("STARTDBG_TAIL_BEGIN lines=%u offset=%u file=/start_debug.txt\n",
                (unsigned)lastLines,
                (unsigned)start);
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
  f.close();
  Serial.println();
  Serial.println("STARTDBG_TAIL_END");
}

static void runLogDumpSpiffsTailToSerial(uint16_t lastLines) {
  if (!spiffs_ok) {
    Serial.println("RunLog: SPIFFS not available");
    return;
  }
  if (lastLines == 0) lastLines = 1;
  if (lastLines > 5000) lastLines = 5000;

  File f = SPIFFS.open("/runlog_last.csv", "r");
  if (!f) {
    Serial.println("RunLog: no saved file (/runlog_last.csv)");
    return;
  }

  // Find the starting offset of the last N lines by scanning once.
  // Keep a ring buffer of newline offsets.
  const size_t cap = (size_t)lastLines + 1;
  size_t* nlOff = (size_t*)malloc(sizeof(size_t) * cap);
  if (!nlOff) {
    Serial.println("RunLog: tail OOM");
    f.close();
    return;
  }
  size_t nlCount = 0;
  size_t pos = 0;
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    pos++;
    if (c == '\n') {
      nlOff[nlCount % cap] = pos;
      nlCount++;
    }
  }

  size_t start = 0;
  if (nlCount > lastLines) {
    start = nlOff[(nlCount - lastLines - 1) % cap];
  } else {
    start = 0;
  }
  free(nlOff);

  f.seek(start, SeekSet);
  Serial.printf("LOG_TAIL_BEGIN lines=%u offset=%u file=/runlog_last.csv\n",
                (unsigned)lastLines,
                (unsigned)start);
  while (f.available()) {
    const int c = f.read();
    if (c < 0) break;
    Serial.write((uint8_t)c);
  }
  f.close();
  Serial.println();
  Serial.println("LOG_TAIL_END");
}

#if RUN_CLOUD_UPLOAD_ENABLE
static bool cloudUploadLastRunCsv() {
  if (!spiffs_ok) {
    Serial.println("CloudUpload: SPIFFS not available");
    return false;
  }
  if (strlen(WIFI_SSID) == 0 || strlen(WIFI_PASSWORD) == 0) {
    Serial.println("CloudUpload: WiFi credentials not configured (see include/wifi_secrets.h)");
    return false;
  }
  if (strlen(CLOUD_UPLOAD_URL) == 0) {
    Serial.println("CloudUpload: CLOUD_UPLOAD_URL not configured (see include/wifi_secrets.h)");
    return false;
  }

  // Apps Script web apps commonly respond to POST with a 302 redirect to a
  // script.googleusercontent.com URL. That redirected URL is typically GET/HEAD
  // only (POST returns 405). The correct flow is:
  //   1) POST to /exec
  //   2) If 302 + Location, GET the Location to read the script response.

  File f = SPIFFS.open("/runlog_last.csv", "r");
  if (!f) {
    Serial.println("CloudUpload: no saved file (/runlog_last.csv)");
    return false;
  }

  const String url = String(CLOUD_UPLOAD_URL);

  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    const unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < CLOUD_UPLOAD_CONNECT_TIMEOUT_MS) {
      delay(50);
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("CloudUpload: WiFi connect failed");
    f.close();
    return false;
  }

  // If time isn't set yet, try a quick NTP sync before uploading.
  if (!timeIsValidUtc()) {
    trySyncTimeWithNtpQuick();
  }

  WiFiClientSecure client;
#if CLOUD_UPLOAD_VERIFY_TLS
  // If you want verification, set a root CA cert here.
#else
  client.setInsecure();
#endif

  HTTPClient http;
  http.setTimeout(CLOUD_UPLOAD_HTTP_TIMEOUT_MS);

  const char* hdrs[] = {"Location"};
  http.collectHeaders(hdrs, 1);

  if (!http.begin(client, url)) {
    Serial.println("CloudUpload: http.begin failed");
    http.end();
    f.close();
    return false;
  }

  http.addHeader("Content-Type", "text/csv");
  const int code = http.sendRequest("POST", &f, (size_t)f.size());
  const String location = http.header("Location");
  Serial.printf("CloudUpload: HTTP %d\n", code);
  const String resp = http.getString();
  if (resp.length() > 0) {
    Serial.print("CloudUpload resp: ");
    Serial.println(resp);
  }
  http.end();
  f.close();

  if (code >= 200 && code < 300) return true;

  if ((code == 301 || code == 302 || code == 303 || code == 307 || code == 308) && location.length() > 0) {
    HTTPClient http2;
    http2.setTimeout(CLOUD_UPLOAD_HTTP_TIMEOUT_MS);
    if (!http2.begin(client, location)) {
      Serial.println("CloudUpload: http.begin (redirect) failed");
      http2.end();
      return false;
    }
    const int code2 = http2.GET();
    Serial.printf("CloudUpload: HTTP %d\n", code2);
    const String resp2 = http2.getString();
    if (resp2.length() > 0) {
      Serial.print("CloudUpload resp: ");
      Serial.println(resp2);
    }
    http2.end();
    return (code2 >= 200 && code2 < 300);
  }

  return false;
}
#endif

static void handleSerialCommands() {
  // Non-blocking line reader for simple commands.
  // Commands:
  // - dump     : dump last saved SPIFFS log to Serial
  // - dumpid N : dump a specific per-run SPIFFS log (e.g., /runlog_12.csv)
  // - tail [N] : dump last N lines of the saved SPIFFS log
  // - listlogs : list runlog files present on SPIFFS
  // - dump_ram : dump current RAM buffer to Serial
  // - startdump     : dump /start_debug.txt
  // - starttail [N] : tail last N lines of /start_debug.txt
  // - status   : log status
  // (I2C speed is fixed in firmware)
  // - format_spiffs : FORMAT SPIFFS (erases files)
  // - upload   : upload last saved SPIFFS log to cloud (requires WiFi config)
  // - help     : list commands
  static char buf[64];
  static uint8_t n = 0;

  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) break;

    if (ch == '\r') continue;
    if (ch == '\n') {
      buf[n] = 0;
      n = 0;

      // trim leading spaces
      char* p = buf;
      while (*p == ' ' || *p == '\t') p++;
      if (*p == 0) return;

      // lowercase in-place
      for (char* q = p; *q; q++) {
        if (*q >= 'A' && *q <= 'Z') *q = (char)(*q - 'A' + 'a');
      }

      if (strcmp(p, "dump") == 0 || strcmp(p, "log") == 0) {
        runLogDumpSpiffsToSerial();
      } else if (strncmp(p, "dumpid", 6) == 0) {
        const char* a = p + 6;
        while (*a == ' ' || *a == '\t') a++;
        unsigned long id = 0;
        if (*a) {
          char* endp = nullptr;
          const unsigned long v = strtoul(a, &endp, 10);
          if (endp != a) id = v;
        }
        if (id == 0) {
          Serial.println("Usage: dumpid <run_id>");
        } else {
          char path[32];
          snprintf(path, sizeof(path), "/runlog_%lu.csv", id);
          runLogDumpSpiffsToSerialFile(path);
        }
      } else if (strncmp(p, "tail", 4) == 0) {
        const char* a = p + 4;
        while (*a == ' ' || *a == '\t') a++;
        uint16_t nLines = 80;
        if (*a) {
          char* endp = nullptr;
          const unsigned long v = strtoul(a, &endp, 10);
          if (endp != a) nLines = (uint16_t)v;
        }
        runLogDumpSpiffsTailToSerial(nLines);
      } else if (strcmp(p, "startdump") == 0) {
        startDebugDumpSpiffsToSerial();
      } else if (strncmp(p, "starttail", 9) == 0) {
        const char* a = p + 9;
        while (*a == ' ' || *a == '\t') a++;
        uint16_t nLines = 120;
        if (*a) {
          char* endp = nullptr;
          const unsigned long v = strtoul(a, &endp, 10);
          if (endp != a) nLines = (uint16_t)v;
        }
        startDebugDumpSpiffsTailToSerial(nLines);
      } else if (strcmp(p, "dump_ram") == 0) {
        runLogPrintCsvToSerial();
      } else if (strcmp(p, "listlogs") == 0) {
        runLogListSpiffsToSerial();
      } else if (strncmp(p, "dellog", 6) == 0) {
        const char* a = p + 6;
        while (*a == ' ' || *a == '\t') a++;
        unsigned long id = 0;
        if (*a) {
          char* endp = nullptr;
          const unsigned long v = strtoul(a, &endp, 10);
          if (endp != a) id = v;
        }
        if (id == 0) {
          Serial.println("Usage: dellog <run_id>");
        } else {
          const bool ok = runLogDeleteById(id);
          Serial.printf("DELLOG %s id=%lu\n", ok ? "OK" : "FAIL", id);
        }
      } else if (strncmp(p, "setepoch", 8) == 0) {
        const char* a = p + 8;
        while (*a == ' ' || *a == '\t') a++;
        unsigned long epoch = 0;
        if (*a) {
          char* endp = nullptr;
          const unsigned long v = strtoul(a, &endp, 10);
          if (endp != a) epoch = v;
        }
        if (epoch < 1000000000UL) {
          Serial.println("Usage: setepoch <unix_seconds> (example: 1735689600)");
        } else {
          struct timeval tv;
          tv.tv_sec = (time_t)epoch;
          tv.tv_usec = 0;
          settimeofday(&tv, nullptr);
          char iso[32];
          iso[0] = 0;
          formatUtcIso8601((time_t)epoch, iso, sizeof(iso));
          Serial.printf("TIME_SET epoch=%lu utc=%s\n", epoch, iso);
        }
#if RUN_CLOUD_UPLOAD_ENABLE
      } else if (strcmp(p, "upload") == 0) {
        cloudUploadLastRunCsv();
#endif
      } else if (strcmp(p, "status") == 0) {
        Serial.printf("FW: %s\n", FW_VERSION);
        Serial.printf("RunLog: ram_samples=%u overflow=%s spiffs=%s\n",
                      (unsigned)runLogCount,
                      runLogOverflow ? "YES" : "no",
                      spiffs_ok ? "OK" : "FAIL");
        Serial.printf("RunId: next=%lu last_attempt=%lu last_saved=%lu\n",
                      (unsigned long)runMetaNextId,
                      (unsigned long)runMetaLastAttemptId,
                      (unsigned long)runMetaLastSavedId);
        if (timeIsValidUtc()) {
          char isoA[32];
          isoA[0] = 0;
          char isoS[32];
          isoS[0] = 0;
          if (runMetaLastAttemptEpoch > 0) formatUtcIso8601((time_t)runMetaLastAttemptEpoch, isoA, sizeof(isoA));
          if (runMetaLastSavedEpoch > 0) formatUtcIso8601((time_t)runMetaLastSavedEpoch, isoS, sizeof(isoS));
          Serial.printf("RunLogSave: attempt_utc=%s saved_utc=%s\n",
                        isoA[0] ? isoA : "(unset)",
                        isoS[0] ? isoS : "(unset)");
        }
        Serial.printf("RunLogSave: per_ok=%u per_bytes=%lu last_ok=%u last_bytes=%lu\n",
                      (unsigned)runMetaLastSavedPerRunOk,
                      (unsigned long)runMetaLastSavedPerRunBytes,
                      (unsigned)runMetaLastSavedLastOk,
                      (unsigned long)runMetaLastSavedLastBytes);
        printSpiffsUsageToSerial("SPIFFS");
        {
          const time_t now = time(nullptr);
          char iso[32];
          iso[0] = 0;
          if (timeIsValidUtc()) formatUtcIso8601(now, iso, sizeof(iso));
          Serial.printf("TIME: epoch=%lu utc=%s valid=%u\n", (unsigned long)now, iso[0] ? iso : "(unset)", timeIsValidUtc() ? 1u : 0u);
        }
        Serial.printf("LastStop: reason=%s elapsed=%.2fs\n",
                      lastRunStopReason ? lastRunStopReason : "?",
                      (double)lastRunElapsedS);
        Serial.printf("I2C: clock_hz=%u\n", (unsigned)i2c_clock_hz);
#if RUN_CLOUD_UPLOAD_ENABLE
        Serial.printf("CloudUpload: pending=%u attempts=%u last_ok=%u\n",
                      cloudUploadPending ? 1u : 0u,
                      (unsigned)cloudUploadAttempts,
                      cloudUploadLastOk ? 1u : 0u);
#endif
      } else if (strcmp(p, "format_spiffs") == 0) {
        Serial.println("FORMAT SPIFFS: erasing filesystem...");
        const bool ok = SPIFFS.format();
        spiffs_ok = SPIFFS.begin(false);
        Serial.printf("FORMAT SPIFFS: %s  mount=%s\n", ok ? "OK" : "FAIL", spiffs_ok ? "OK" : "FAIL");
      } else if (strcmp(p, "help") == 0 || strcmp(p, "?") == 0) {
#if RUN_CLOUD_UPLOAD_ENABLE
        Serial.println("Commands: help | status | dump | dumpid <id> | listlogs | dellog <id> | setepoch <unix_s> | tail [N] | dump_ram | startdump | starttail [N] | format_spiffs | upload");
#else
        Serial.println("Commands: help | status | dump | dumpid <id> | listlogs | dellog <id> | setepoch <unix_s> | tail [N] | dump_ram | startdump | starttail [N] | format_spiffs");
#endif
      } else {
        Serial.printf("Unknown command: '%s' (type 'help')\n", p);
      }
      return;
    }

    if (n < (sizeof(buf) - 1)) {
      buf[n++] = (char)ch;
    }
  }
}

struct ExtButtonState {
  bool stableLevel = true;  // HIGH = released (with pullup)
  bool lastReadLevel = true;
  unsigned long lastChangeMs = 0;
};

static ExtButtonState extBlueState;
static ExtButtonState extRedState;

static bool extButtonWasClicked(int pin, ExtButtonState* st) {
  if (pin < 0 || st == nullptr) return false;
  const unsigned long now = millis();
  const bool level = (digitalRead(pin) != 0);

  if (level != st->lastReadLevel) {
    st->lastReadLevel = level;
    st->lastChangeMs = now;
  }

  // Debounce
  const unsigned long debounceMs = 25;
  if ((now - st->lastChangeMs) < debounceMs) return false;

  if (level != st->stableLevel) {
    const bool prevStable = st->stableLevel;
    st->stableLevel = level;
    // Click = released->pressed transition (active-low, so HIGH->LOW)
    if (prevStable == true && st->stableLevel == false) {
      return true;
    }
  }

  return false;
}

// Status tracking
int encoderA_state = 0;
int encoderB_state = 0;
int encoderA_prev = 0;
int encoderB_prev = 0;
int32_t encoderL_count = 0;
int32_t encoderR_count = 0;
bool encoderL_found = false;
bool encoderR_found = false;

// Motor control
uint8_t selected_motor = 0;  // 0=left, 1=right
// Per-motor direction (enables true pivot turns)
// 0=reverse, 1=forward
uint8_t motor_l_direction = 1;
uint8_t motor_r_direction = 1;
int16_t motor_command = 0;   // -255..255 (HW test screen; sign = direction, 0 = stop)
uint8_t motor_enabled = 0;   // 0=stopped, 1=running (derived from motor_command)
uint8_t motor_l_speed = 0;
uint8_t motor_r_speed = 0;

// RUN screen fixed PWM
#define RUN_PWM 255

// IMU attitude (degrees)
static bool imu_ok = false;
static float imu_pitch = 0.0f;
static float imu_roll = 0.0f;
static float imu_yaw = 0.0f;
static float imu_gz_dps = 0.0f;
static float imu_gz_bias_dps = 0.0f;
static float imu_gz_raw_dps = 0.0f;
static float imu_gz_still_err_dps = 0.0f;
// UI/diagnostic: true while we're actively refining gyro bias while stopped.
static bool imu_bias_refining = false;

// Persist a last-known-good gyro Z bias so a "bad staged calibration" doesn't ruin a run.
static unsigned long imu_bias_last_save_ms = 0;
static float imu_bias_last_saved_dps = NAN;
#define IMU_BIAS_PERSIST_KEY "gz_bias"
#define IMU_BIAS_PERSIST_MIN_INTERVAL_MS 8000
#define IMU_BIAS_PERSIST_MIN_DELTA_DPS 0.25f

static void persistImuGyroBiasIfNeeded() {
  if (!prefs_ok) return;
  if (!isfinite(imu_gz_bias_dps)) return;

  const unsigned long now = millis();
  if (imu_bias_last_save_ms != 0 && (now - imu_bias_last_save_ms) < IMU_BIAS_PERSIST_MIN_INTERVAL_MS) return;
  if (isfinite(imu_bias_last_saved_dps) && fabsf(imu_gz_bias_dps - imu_bias_last_saved_dps) < IMU_BIAS_PERSIST_MIN_DELTA_DPS) return;

  prefs.putFloat(IMU_BIAS_PERSIST_KEY, imu_gz_bias_dps);
  imu_bias_last_saved_dps = imu_gz_bias_dps;
  imu_bias_last_save_ms = now;
}

// Enable IMU influence on driving control (straight heading-hold + CAN IMU path).
// Set false only when testing encoder-only behavior.
static bool imu_control_enabled = true;

// Motor bias (feedforward): constant PWM offset applied to left motor to compensate for
// mechanical differences (e.g., left wheel has more drag). Positive = boost left motor.
// This is applied as feedforward BEFORE the feedback loop, reducing how hard the PID must work.
static int16_t motor_bias_pwm = 0;
#define MOTOR_BIAS_STEP_PWM 1
#define MOTOR_BIAS_MAX_PWM 40

// Apply motor_bias_pwm during normal RUN control (in addition to BIAS TEST mode).
// This is OFF by default because earlier testing requested a bias-free RUN path.
// Set to 1 to use motor_bias_pwm as a feedforward trim that reduces long-term drift from mechanical asymmetry.
#define RUN_MOTOR_BIAS_ENABLE 0

// Bias test mode: when true, feedback loop is disabled and only motor_bias_pwm is applied.
// This lets you tune the bias until the car goes approximately straight without feedback.
static bool bias_test_mode = false;

// External Unit IMU presence (MPU6886)
static bool imu_present = false;
static uint8_t imu_whoami = 0;

// Forward declaration (used by boot-time gyro calibration helpers)
static bool imu6886Read(float* ax_g, float* ay_g, float* az_g,
                        float* gx_dps, float* gy_dps, float* gz_dps,
                        float* temp_c);

// Boot-time gyro bias calibration (keep the car still during this window)
#define IMU_GYRO_CAL_MS 5000
#define IMU_GYRO_CAL_SAMPLE_MS 10

// Boot splash + countdown before calibration
#define BOOT_SPLASH_MS 3000
#define IMU_CAL_COUNTDOWN_S 3

static void showBVSplashScreen() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_BLUE);
  ui.setTextSize(3);
  ui.drawString("BLUE VALLEY", 120, 80);
  ui.drawString("NORTH", 120, 110);
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(2);
  ui.drawString("Science Olympiad", 120, 150);
  ui.drawString("E.V. 2025-26", 120, 170);
  ui.setTextSize(1);
  ui.pushSprite(0, 0);
}

static void showImuCountdownScreen(int secondsLeft) {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("IMU CAL", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Keep car still", 120, 55);
  ui.drawString("Calibration starts in", 120, 80);

  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(5);
  ui.drawString(String(secondsLeft), 120, 140);

  ui.setTextSize(1);
  ui.pushSprite(0, 0);
}

static void showImuReadyScreen(float biasZ) {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(2);
  ui.drawString("READY TO RACE", 120, 60);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Gyro Z bias (dps)", 120, 105);
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(2);
  ui.drawString(String(biasZ, 2), 120, 130);

  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(1);
  ui.drawString("OK to move car", 120, 200);

  ui.setTextSize(1);
  ui.pushSprite(0, 0);
}

static void showImuCalScreen(int pct, float biasZ) {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("IMU CAL", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Keep car still", 120, 55);

  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(2);
  ui.drawString(String(pct) + "%", 120, 90);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Gyro Z bias (dps)", 120, 130);
  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(2);
  ui.drawString(String(biasZ, 2), 120, 155);

  ui.pushSprite(0, 0);
}

// Gyro Z bias sanity clamp (deg/s).
// A wildly wrong bias (even if "stable") will destroy yaw integration and heading hold.
// NOTE: This clamp was previously too tight (5 dps) and could leave a large residual
// bias uncorrected, causing steady yaw drift and steering off the line.
#define IMU_GZ_BIAS_CLAMP_DPS 12.0f

static void calibrateImuGyroBias() {
  const float prevBias = imu_gz_bias_dps;
  if (!imu_present) {
    showImuCalScreen(0, 0.0f);
    delay(200);
    return;
  }

  const unsigned long start = millis();
  unsigned long lastSample = 0;
  int samples = 0;
  double sumGz = 0.0;
  double sumSq = 0.0;
  float minGz = 1e9f;
  float maxGz = -1e9f;

  while (millis() - start < (unsigned long)IMU_GYRO_CAL_MS) {
    const unsigned long now = millis();
    if (now - lastSample < (unsigned long)IMU_GYRO_CAL_SAMPLE_MS) {
      delay(1);
      continue;
    }
    lastSample = now;

    float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
    float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps_local = 0.0f;
    float temp_c = 0.0f;
    const bool ok = imu6886Read(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps_local, &temp_c);
    if (ok) {
      sumGz += (double)gz_dps_local;
      sumSq += (double)gz_dps_local * (double)gz_dps_local;
      samples += 1;
      if (gz_dps_local < minGz) minGz = gz_dps_local;
      if (gz_dps_local > maxGz) maxGz = gz_dps_local;
    }

    const int pct = (int)((100.0f * (float)(now - start)) / (float)IMU_GYRO_CAL_MS);
    const float estBias = (samples > 0) ? (float)(sumGz / (double)samples) : 0.0f;
    showImuCalScreen(pct < 100 ? pct : 100, estBias);
  }

  // Only accept the estimated bias if readings were stable.
  // If the car is touched/moved during calibration, keep the previous (persisted) bias.
  if (samples >= 80) {
    const double mean = sumGz / (double)samples;
    const double var = (sumSq / (double)samples) - (mean * mean);
    const float stddev = (var > 0.0) ? (float)sqrt(var) : 0.0f;
    const float span = (maxGz > minGz) ? (maxGz - minGz) : 0.0f;

    // Large-but-stable offsets are valid; don't require small magnitude.
    const bool stable = (stddev <= 0.80f) && (span <= 6.0f);
    if (stable) {
      imu_gz_bias_dps = (float)mean;
    } else {
      imu_gz_bias_dps = prevBias;
    }
  } else {
    imu_gz_bias_dps = prevBias;
  }

  // Sanity: if the estimated bias is wildly large, something was moving or scaling is wrong.
  // Use 0 so we don't destroy heading hold.
  if (fabsf(imu_gz_bias_dps) > 20.0f) {
    Serial.printf("WARN: IMU gyro bias %.2f dps too large; forcing 0\n", (double)imu_gz_bias_dps);
    imu_gz_bias_dps = 0.0f;
  }

  // Normal clamp: keeps yaw integration from going unstable even if calibration was imperfect.
  if (imu_gz_bias_dps > IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = IMU_GZ_BIAS_CLAMP_DPS;
  if (imu_gz_bias_dps < -IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = -IMU_GZ_BIAS_CLAMP_DPS;
  showImuCalScreen(100, imu_gz_bias_dps);
  delay(250);

  persistImuGyroBiasIfNeeded();
}

static void quickCalibrateImuGyroBiasForRunStart() {
  if (!imu_present) return;

  const float prevBias = imu_gz_bias_dps;

  // Quick (blocking) bias estimate right before enabling motion.
  // Accept based on *stability* (low variance), not absolute magnitude.
  // This avoids rejecting a real large sensor offset (which would leave residual drift).
  const unsigned long t0 = millis();
  double sumGz = 0.0;
  double sumSq = 0.0;
  int samples = 0;

  float minGz = 1e9f;
  float maxGz = -1e9f;

  float maxAbsGx = 0.0f;
  float maxAbsGy = 0.0f;

  while (millis() - t0 < 360) {
    float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
    float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps_local = 0.0f;
    float temp_c = 0.0f;
    const bool ok = imu6886Read(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps_local, &temp_c);
    if (ok) {
      sumGz += (double)gz_dps_local;
      sumSq += (double)gz_dps_local * (double)gz_dps_local;
      samples += 1;
      if (gz_dps_local < minGz) minGz = gz_dps_local;
      if (gz_dps_local > maxGz) maxGz = gz_dps_local;
      const float agx = fabsf(gx_dps);
      const float agy = fabsf(gy_dps);
      if (agx > maxAbsGx) maxAbsGx = agx;
      if (agy > maxAbsGy) maxAbsGy = agy;
    }
    delay(2);
  }

  // If we were moving while starting, don't overwrite bias with garbage.
  // Use stddev as the stillness test (more robust than maxAbs).
  if (samples >= 60) {
    const double mean = sumGz / (double)samples;
    const double var = (sumSq / (double)samples) - (mean * mean);
    const float stddev = (var > 0.0) ? (float)sqrt(var) : 0.0f;
    const float span = (maxGz > minGz) ? (maxGz - minGz) : 0.0f;
    const bool stable = (stddev <= 0.60f) && (span <= 4.0f) && (maxAbsGx <= 8.0f) && (maxAbsGy <= 8.0f);
    if (stable) {
      imu_gz_bias_dps = (float)mean;
    } else {
      imu_gz_bias_dps = prevBias;
    }
  } else {
    imu_gz_bias_dps = prevBias;
  }

  // Always clamp at run start.
  if (imu_gz_bias_dps > IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = IMU_GZ_BIAS_CLAMP_DPS;
  if (imu_gz_bias_dps < -IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = -IMU_GZ_BIAS_CLAMP_DPS;

  // Keep derived values consistent until the next IMU tick.
  imu_gz_dps = imu_gz_raw_dps - imu_gz_bias_dps;

  persistImuGyroBiasIfNeeded();
}

// ========================================================================
// Unit IMU (MPU6886) on I2C @ 0x68
// Ref: https://docs.m5stack.com/en/unit/imu
// ========================================================================

#define IMU_6886_ADDR 0x68
#define IMU_6886_WHOAMI 0x75
#define IMU_6886_SMPLRT_DIV 0x19
#define IMU_6886_CONFIG 0x1A
#define IMU_6886_GYRO_CONFIG 0x1B
#define IMU_6886_ACCEL_CONFIG 0x1C
#define IMU_6886_ACCEL_CONFIG2 0x1D
#define IMU_6886_INT_PIN_CFG 0x37
#define IMU_6886_INT_ENABLE 0x38
#define IMU_6886_ACCEL_XOUT_H 0x3B
#define IMU_6886_TEMP_OUT_H 0x41
#define IMU_6886_GYRO_XOUT_H 0x43
#define IMU_6886_USER_CTRL 0x6A
#define IMU_6886_PWR_MGMT_1 0x6B
#define IMU_6886_FIFO_EN 0x23

// Scale factors (computed from configured full-scale ranges).
// Defaults match our init writes (accel +-8g, gyro +-2000 dps), but we also read back registers
// so the conversion stays correct even if the device boots with different ranges.
static float imu6886_aRes_g_per_lsb = 8.0f / 32768.0f;
static float imu6886_gRes_dps_per_lsb = 2000.0f / 32768.0f;

static bool imu6886WriteByte(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_6886_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

static bool imu6886ReadBytes(uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(IMU_6886_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const size_t got = Wire.requestFrom((uint8_t)IMU_6886_ADDR, (size_t)len);
  if (got != len) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    buf[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool imu6886Init() {
  uint8_t who = 0;
  if (!imu6886ReadBytes(IMU_6886_WHOAMI, &who, 1)) return false;
  imu_whoami = who;

  // Init sequence based on M5Stack Unit IMU example (MPU6886)
  // Wake
  if (!imu6886WriteByte(IMU_6886_PWR_MGMT_1, 0x00)) return false;
  delay(10);
  // Reset
  if (!imu6886WriteByte(IMU_6886_PWR_MGMT_1, 0x80)) return false;
  delay(10);
  // Auto-select best clock
  if (!imu6886WriteByte(IMU_6886_PWR_MGMT_1, 0x01)) return false;
  delay(10);

  // Accel: +-8g
  if (!imu6886WriteByte(IMU_6886_ACCEL_CONFIG, 0x10)) return false;
  delay(1);
  // Gyro: +-2000 dps
  if (!imu6886WriteByte(IMU_6886_GYRO_CONFIG, 0x18)) return false;
  delay(1);
  // DLPF
  if (!imu6886WriteByte(IMU_6886_CONFIG, 0x01)) return false;
  delay(1);
  // Sample rate divider
  if (!imu6886WriteByte(IMU_6886_SMPLRT_DIV, 0x01)) return false;
  delay(1);

  (void)imu6886WriteByte(IMU_6886_INT_ENABLE, 0x00);
  delay(1);
  (void)imu6886WriteByte(IMU_6886_ACCEL_CONFIG2, 0x00);
  delay(1);
  (void)imu6886WriteByte(IMU_6886_USER_CTRL, 0x00);
  delay(1);
  (void)imu6886WriteByte(IMU_6886_FIFO_EN, 0x00);
  delay(1);
  (void)imu6886WriteByte(IMU_6886_INT_PIN_CFG, 0x22);
  delay(1);
  (void)imu6886WriteByte(IMU_6886_INT_ENABLE, 0x01);
  delay(10);

  // Read back full-scale ranges and compute scale factors.
  // This keeps our dps/g conversion correct even if the IMU doesn't take our writes
  // or if a different default range is active.
  uint8_t accel_cfg = 0;
  uint8_t gyro_cfg = 0;
  if (imu6886ReadBytes(IMU_6886_ACCEL_CONFIG, &accel_cfg, 1)) {
    const uint8_t fs = (accel_cfg >> 3) & 0x03;
    float aFS_g = 8.0f;
    if (fs == 0) aFS_g = 2.0f;
    else if (fs == 1) aFS_g = 4.0f;
    else if (fs == 2) aFS_g = 8.0f;
    else aFS_g = 16.0f;
    imu6886_aRes_g_per_lsb = aFS_g / 32768.0f;
  }
  if (imu6886ReadBytes(IMU_6886_GYRO_CONFIG, &gyro_cfg, 1)) {
    const uint8_t fs = (gyro_cfg >> 3) & 0x03;
    float gFS_dps = 2000.0f;
    if (fs == 0) gFS_dps = 250.0f;
    else if (fs == 1) gFS_dps = 500.0f;
    else if (fs == 2) gFS_dps = 1000.0f;
    else gFS_dps = 2000.0f;
    imu6886_gRes_dps_per_lsb = gFS_dps / 32768.0f;
  }

  return true;
}

static inline int16_t be16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] << 8 | (uint16_t)p[1]);
}

static bool imu6886Read(float* ax_g, float* ay_g, float* az_g,
                        float* gx_dps, float* gy_dps, float* gz_dps,
                        float* temp_c) {
  uint8_t buf[14];
  if (!imu6886ReadBytes(IMU_6886_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

  const int16_t ax = be16(&buf[0]);
  const int16_t ay = be16(&buf[2]);
  const int16_t az = be16(&buf[4]);
  const int16_t t = be16(&buf[6]);
  const int16_t gx = be16(&buf[8]);
  const int16_t gy = be16(&buf[10]);
  const int16_t gz = be16(&buf[12]);

  if (ax_g) *ax_g = (float)ax * imu6886_aRes_g_per_lsb;
  if (ay_g) *ay_g = (float)ay * imu6886_aRes_g_per_lsb;
  if (az_g) *az_g = (float)az * imu6886_aRes_g_per_lsb;
  if (gx_dps) *gx_dps = (float)gx * imu6886_gRes_dps_per_lsb;
  if (gy_dps) *gy_dps = (float)gy * imu6886_gRes_dps_per_lsb;
  if (gz_dps) *gz_dps = (float)gz * imu6886_gRes_dps_per_lsb;
  if (temp_c) *temp_c = (float)t / 326.8f + 25.0f;

  return true;
}

// ========================================================================
// Screen / menu state (pulled from backup_jan4, simplified)
// ========================================================================

enum ScreenState {
  SCREEN_MAIN_MENU,
  SCREEN_RUN,
  SCREEN_SET_DISTANCE,
  SCREEN_SET_TIME,
  SCREEN_SET_CAN_DISTANCE,
  SCREEN_BIAS_TEST,
  SCREEN_SET_ENC_CAL,
  SCREEN_SET_CTRL_MODE,
  SCREEN_HW_TEST,
  SCREEN_BFS_NAV,
};

static ScreenState currentScreen = SCREEN_MAIN_MENU;

static const int NUM_MENU_ITEMS = 9;
static const char* menuNames[NUM_MENU_ITEMS] = {
  "CAR RUN",
  "DISTANCE",
  "TIME",
  "CAN DIST",
  "BIAS TEST",
  "ENC CAL",
  "CTRL MODE",
  "BFS NAV",
  "HW TEST",
};

static int selectedMenuItem = 0;

// BFS Navigation state
static uint8_t bfs_start_node = 0;     // Starting waypoint
static uint8_t bfs_goal_node = 2;      // Goal waypoint
static bool bfs_run_active = false;    // Currently executing BFS path
static uint8_t bfs_current_target = 0; // Current target waypoint in path
static float bfs_target_distance = 0.0f;
static float bfs_target_angle = 0.0f;

static float runDistanceM = 7.0f;
static float runTimeS = 15.0f;
// Can bonus setting (OFF if < CAN_ENABLE_MIN_M):
// Official "Inside Can Distance" (meters) = distance along Bonus Line between the INSIDE EDGES of the 2 cans.
// This is measured by the Event Supervisor.
static float canDistanceM = 0.0f;
// Treat very small/near-zero values as OFF (also protects against tiny persisted float remnants).
static constexpr float CAN_ENABLE_MIN_M = 0.05f;

// Encoder distance calibration (pulses per meter). Adjustable from the dial.
// Note: if the car overshoots the target distance, you usually want a LOWER value
// (so computed meters increases faster and we stop sooner). If it stops short,
// you usually want a HIGHER value.
// 2026-01-21 new motors: 10.0m roll test -> L=18988, R=-18990 (with INVERT_RIGHT_ENCODER_COUNT=1)
// => avg path pulses ~= 18989 over 10m => ~1899 pulses/m.
// Per-wheel calibration for wheel size/gearbox differences.
static float pulsesPerMeterL = 1899.0f;
static float pulsesPerMeterR = 1899.0f;
// Legacy/summary value (used by existing plots/tools): average of L/R.
static float pulsesPerMeter = 1899.0f;

static bool encCalEditingLeft = true;

static inline float clampPpm(float ppm) {
  if (ppm < 1500.0f) ppm = 1500.0f;
  if (ppm > 4000.0f) ppm = 4000.0f;
  return ppm;
}

static inline void syncPulsesPerMeterDerived() {
  pulsesPerMeterL = clampPpm(pulsesPerMeterL);
  pulsesPerMeterR = clampPpm(pulsesPerMeterR);
  pulsesPerMeter = clampPpm(0.5f * (pulsesPerMeterL + pulsesPerMeterR));
}

// Auto-save persisted settings after dial stops moving.
static bool settings_dirty = false;
static unsigned long settings_dirty_ms = 0;
#define SETTINGS_AUTOSAVE_IDLE_MS 1000

static void loadPersistedSettings() {
  if (!prefs_ok) return;
  // Legacy single-ppm support: if ppmL/ppmR are unset, fall back to stored "ppm".
  const float legacyPpm = prefs.getFloat("ppm", pulsesPerMeter);
  pulsesPerMeterL = prefs.getFloat("ppmL", legacyPpm);
  pulsesPerMeterR = prefs.getFloat("ppmR", legacyPpm);
  pulsesPerMeter = legacyPpm;
  syncPulsesPerMeterDerived();
  motor_bias_pwm = (int16_t)prefs.getShort("mtr_bias", (int16_t)motor_bias_pwm);

  // Last-known-good gyro bias (used as a safe fallback if staged calibration is disturbed).
  imu_gz_bias_dps = prefs.getFloat(IMU_BIAS_PERSIST_KEY, imu_gz_bias_dps);
  if (imu_gz_bias_dps > IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = IMU_GZ_BIAS_CLAMP_DPS;
  if (imu_gz_bias_dps < -IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = -IMU_GZ_BIAS_CLAMP_DPS;
  imu_bias_last_saved_dps = imu_gz_bias_dps;

  // Run ID counter (persisted so each run has a unique id across reboots).
  runMetaNextId = prefs.getULong("run_id_next", runMetaNextId);
  if (runMetaNextId == 0) runMetaNextId = 1;

  // Run log save diagnostics.
  // NOTE: ESP32 NVS key length limit is 15 chars.
  runMetaLastAttemptId = prefs.getULong("rla_id", runMetaLastAttemptId);
  runMetaLastAttemptEpoch = prefs.getULong("rla_ep", runMetaLastAttemptEpoch);
  runMetaLastSavedId = prefs.getULong("rls_id", runMetaLastSavedId);
  runMetaLastSavedEpoch = prefs.getULong("rls_ep", runMetaLastSavedEpoch);
  runMetaLastSavedPerRunBytes = prefs.getULong("rls_b", runMetaLastSavedPerRunBytes);
  runMetaLastSavedLastBytes = prefs.getULong("rls_lb", runMetaLastSavedLastBytes);
  runMetaLastSavedPerRunOk = (uint8_t)prefs.getUChar("rls_ok", runMetaLastSavedPerRunOk);
  runMetaLastSavedLastOk = (uint8_t)prefs.getUChar("rls_lok", runMetaLastSavedLastOk);

  // Control mode (0=IMU, 1=ENC, 2=HYBRID)
  {
    const uint8_t m = (uint8_t)prefs.getUChar("ctrl_mode", (uint8_t)control_mode);
    if (m <= (uint8_t)CTRL_MODE_HYBRID) control_mode = (ControlMode)m;
  }

  // Run parameters
  runDistanceM = prefs.getFloat("run_dist", runDistanceM);
  runTimeS = prefs.getFloat("run_time", runTimeS);
  canDistanceM = prefs.getFloat("can_dist", canDistanceM);
  if (canDistanceM < CAN_ENABLE_MIN_M) canDistanceM = 0.0f;

  // BFS Navigation settings
  bfs_start_node = (uint8_t)prefs.getUChar("bfs_start", bfs_start_node);
  bfs_goal_node = (uint8_t)prefs.getUChar("bfs_goal", bfs_goal_node);
  if (bfs_start_node >= BFS_MAX_NODES) bfs_start_node = 0;
  if (bfs_goal_node >= BFS_MAX_NODES) bfs_goal_node = 2;

  // Enforce competition ranges
  if (runDistanceM < RUN_DISTANCE_MIN_M) runDistanceM = RUN_DISTANCE_MIN_M;
  if (runDistanceM > RUN_DISTANCE_MAX_M) runDistanceM = RUN_DISTANCE_MAX_M;
  if (runTimeS < RUN_TIME_MIN_S) runTimeS = RUN_TIME_MIN_S;
  if (runTimeS > RUN_TIME_MAX_S) runTimeS = RUN_TIME_MAX_S;

  // Enforce competition granularity
  runDistanceM = quantizeRunDistanceRules(runDistanceM);
  runTimeS = quantizeToStep(runTimeS, RUN_TIME_STEP_S);
  if (runDistanceM < RUN_DISTANCE_MIN_M) runDistanceM = RUN_DISTANCE_MIN_M;
  if (runDistanceM > RUN_DISTANCE_MAX_M) runDistanceM = RUN_DISTANCE_MAX_M;
  if (runTimeS < RUN_TIME_MIN_S) runTimeS = RUN_TIME_MIN_S;
  if (runTimeS > RUN_TIME_MAX_S) runTimeS = RUN_TIME_MAX_S;

}

static void savePersistedSettings() {
  if (!prefs_ok) return;

  // Normalize to legal values before persisting.
  runDistanceM = quantizeRunDistanceRules(runDistanceM);
  runTimeS = quantizeToStep(runTimeS, RUN_TIME_STEP_S);

  syncPulsesPerMeterDerived();
  // Write both the legacy key and the per-wheel keys.
  prefs.putFloat("ppm", pulsesPerMeter);
  prefs.putFloat("ppmL", pulsesPerMeterL);
  prefs.putFloat("ppmR", pulsesPerMeterR);
  prefs.putShort("mtr_bias", (int16_t)motor_bias_pwm);

  // Control mode
  prefs.putUChar("ctrl_mode", (uint8_t)control_mode);

  // Run parameters
  prefs.putFloat("run_dist", runDistanceM);
  prefs.putFloat("run_time", runTimeS);
  prefs.putFloat("can_dist", canDistanceM);

  // BFS Navigation settings
  prefs.putUChar("bfs_start", bfs_start_node);
  prefs.putUChar("bfs_goal", bfs_goal_node);
}

static bool runActive = false;
static unsigned long runStartMs = 0;
static int32_t runStartEncL = 0;
static int32_t runStartEncR = 0;

// Live debug (shown on RUN screen)
static uint8_t runDebug_canPhase = 255;
static float runDebug_forwardM = 0.0f;
static float runDebug_lateralM = 0.0f;
static float runDebug_targetYawDeg = 0.0f;
static float runDebug_basePwm = 0.0f;
static float runDebug_turnRemainingDeg = NAN;

// Updated 2026-01-05 initial starting point (was derived from shortfall tests): 2450 pulses/m

// Control loop tuning
// PI speed control: tries to match avg pulse rate so we reach target distance at target time.
// Units:
// - targetRate, actualRate: pulses/sec
// - basePWM: 0..255
// Start with conservative gains; we will tune on your track.
#define SPEED_INITIAL_PWM 220.0f
#define SPEED_KP 0.06f
#define SPEED_KI 0.02f

// Stall avoidance during RUN: if we are commanding motion, keep PWM above this.
// Tune based on testing.
#define RUN_MIN_PWM 90.0f

// Test mode: lock RUN PWM (bypasses speed PI). Useful for heading tuning and diagnosing power/brownout.
// NOTE: v73 phased navigation uses a distance-based S-curve profile instead.
#define RUN_LOCK_PWM_ENABLE 0
#define RUN_LOCK_PWM_VALUE 200.0f

// Distance-based speed profile (S-curve) to reduce wheel slip and improve repeatability.
// Applied per drive phase using down-course distance (forwardM).
#define RUN_SPEED_PROFILE_ENABLE 0
#define RUN_SPEED_PROFILE_MAX_PWM 235.0f
#define RUN_SPEED_PROFILE_MIN_PWM 110.0f
#define RUN_SPEED_PROFILE_ACCEL_M 0.35f
#define RUN_SPEED_PROFILE_DECEL_M 1.00f

// Extra kick at the beginning of each run to break static friction.
// Prevents early NO_MOTION stop if the car doesn't start rolling.
#define RUN_SPEED_PROFILE_START_BOOST_MS 400
#define RUN_SPEED_PROFILE_START_BOOST_PWM 90.0f

// If the car still hasn't moved (encoder pulses) shortly after start,
// keep applying an extra kick for a bit longer.
#define RUN_START_ASSIST_ENABLE 1
#define RUN_START_ASSIST_MS 1200
#define RUN_START_ASSIST_PWM 90.0f
#define RUN_START_ASSIST_MAX_PULSES 60

// Phased CAN navigation (stop/turn/drive segments)
// Active only when CAN DIST is enabled and IMU control is OK.
#define CAN_PHASED_NAV_ENABLE 1
// Target lateral offset is derived from Inside Can Distance (see CAN mode notes below).
// Phase plan relative to the Bonus Line (halfway point):
// - Be parallel before the Bonus Line
// - Stay parallel past the Bonus Line
// For a 7m run, Bonus Line is at 3.5m; with defaults below, we are parallel by 2.5m and return at 5.0m.
#define CAN_PHASED_ENTRY_BEFORE_HALF_M 1.00f
#define CAN_PHASED_EXIT_AFTER_HALF_M 1.50f
// Diagonal angle (degrees) for the out-and-back CAN shift.
// The firmware computes the minimum angle required to reach the target lateral offset by the
// "parallel entry" point, while trying to keep turns shallow for repeatability.
#define CAN_PHASED_NOMINAL_ANGLE_DEG 25.0f
#define CAN_PHASED_MAX_ANGLE_DEG 45.0f
// Dampen the second turn (TURN_TO_PARALLEL) by capping its angle below the global max.
#define CAN_PHASED_TURN_PARALLEL_MAX_ANGLE_DEG 35.0f
// Prefer to drive at least this far straight before the first pivot/diagonal (helps stability).
#define CAN_PHASED_MIN_STRAIGHT0_M 0.30f
// Refuse to enter phased CAN if the total run distance is too short for sane geometry.
// Very short/zero distances can collapse the phase plan and cause rapid left/right pivots.
#define CAN_PHASED_MIN_RUN_DISTANCE_M 2.25f
// Pivot turn tuning
// After drivetrain changes, full-scale pivot PWM can be too violent. Use a ramp and allow
// a modest slowdown near target, while still keeping enough torque to avoid stalling.
#define CAN_PHASED_TURN_PWM_START 205.0f
#define CAN_PHASED_TURN_PWM_MAX 230.0f
// Softer PWM ramp for the second turn (TURN_TO_PARALLEL) to avoid aggressive right pivots.
#define CAN_PHASED_TURN_PARALLEL_PWM_START 185.0f
#define CAN_PHASED_TURN_PARALLEL_PWM_MAX 210.0f
// Ramp up quickly to reduce the initial jerk.
#define CAN_PHASED_TURN_RAMP_MS 180
// Never drop below this during a pivot.
#define CAN_PHASED_TURN_MIN_PWM 165.0f
// Near the target, allow a lower minimum so we don't overshoot and ring.
#define CAN_PHASED_TURN_NEAR_DEG 6.0f
#define CAN_PHASED_TURN_NEAR_MIN_PWM 125.0f
#define CAN_PHASED_TURN_SLOW_DEG 10.0f
#define CAN_PHASED_TURN_TOL_DEG 1.5f
// Damping: back off PWM when yaw rate is already high.
#define CAN_PHASED_TURN_RATE_DAMP_PWM_PER_DPS 0.35f
// Slew limit: prevents abrupt PWM jumps that excite ringing.
#define CAN_PHASED_TURN_PWM_SLEW_PER_S 2000.0f
// If we cross the target (sign flip) but we're very close, stop/hold instead of reversing hard.
#define CAN_PHASED_TURN_CROSS_STOP_DEG 2.5f

// If we stall near the target (yaw not changing), bump PWM back up to avoid hanging.
#define CAN_PHASED_TURN_NEAR_STALL_WINDOW_MS 180
#define CAN_PHASED_TURN_NEAR_STALL_MIN_DYAW_DEG 0.30f
#define CAN_PHASED_TURN_NEAR_STALL_BOOST_PWM 175.0f
// When we enter the tolerance band, stop pivoting and require the IMU to remain
// in-tolerance for this long before declaring the turn complete. Prevents "hunt/jerk".
#define CAN_PHASED_TURN_IN_TOL_HOLD_MS 120
// If a single pivot phase runs too long, abort and save logs instead of sitting forever.
#define CAN_PHASED_TURN_TIMEOUT_MS 3500
// If the turn barely moves for a bit, assume we're stalled and immediately boost to max.
#define CAN_PHASED_TURN_STALL_BOOST_AFTER_MS 260
#define CAN_PHASED_TURN_STALL_MIN_DELTA_DEG 2.0f
// If your yaw sign is inverted vs the pivot command, flip this to -1.
#define CAN_PHASED_YAW_SIGN 1
// If your pivot direction is inverted (turns the wrong way), flip this to -1.
#define CAN_PHASED_PIVOT_SIGN 1

// During the first moments of a phased CAN DRIVE segment, target yaw can step.
// For option A we use a unified startup damp (see below), so keep this disabled.
#define CAN_PHASED_DRIVE_STEER_RAMP_MS 0

// In phased CAN DRIVE segments, use lateral error to bias the yaw setpoint.
// Without this, we only hold yaw and can drift significantly off the target lateral offset.
#define CAN_PHASED_LAT_YAW_ENABLE 1
#define CAN_PHASED_LAT_TO_YAW_KP_DEG_PER_M 30.0f
#define CAN_PHASED_LAT_TO_YAW_MAX_DEG 15.0f

// When base PWM is near saturation, a constant motor bias contributes to large L/R differentials.
// Fade bias out as we approach 255 to avoid the characteristic one-sided lurch.
#define MOTOR_BIAS_FADE_NEAR_SAT_ENABLE 1
#define MOTOR_BIAS_FADE_WINDOW_PWM 35.0f

// Track width for encoder-based turn measurement (meters)
// Measured wheel-to-wheel spacing: 148 mm.
#define TURN_TRACK_WIDTH_M 0.148f
// Encoder/IMU agreement thresholds (degrees)
#define TURN_ENC_TOL_DEG 2.5f
#define TURN_IMU_TOL_DEG CAN_PHASED_TURN_TOL_DEG
#define TURN_IMU_ENC_DISAGREE_DEG 8.0f

// Soft-start ramp to reduce current spikes when starting a run (helps prevent brownouts).
#define RUN_SOFTSTART_MS 300

// Steering authority ramp at the beginning of a run. Prevents an initial "kick"
// when the controller state transitions out of the reset window.
#define RUN_STEER_RAMP_MS 300
// Unified startup dampening (applies to straight runs and CAN drive phase starts).
#define STARTUP_STEER_RAMP_MS 1800
// Left motor startup ramp: slow left-side rise so it doesn't overpower right authority at launch.
#define LEFT_STARTUP_RAMP_MS 1800

// Start-of-motion guard: whenever the motors restart after being stopped (including
// phased CAN segment transitions), hold steering correction at 0 briefly so the gyro
// derivative term / quantized first encoder tick can't produce a one-sided "jerk".
// DISABLED: was causing stuttering during CAN phase transitions.
#define RUN_RESTART_STEER_ZERO_MS 0

// IMU read can occasionally miss a tick due to I2C timing. Allow brief dropouts without
// disabling CAN mode mid-run (which would snap the car back to centerline yaw=0).
#define IMU_OK_GRACE_MS 250

// When CAN phased nav is enabled, start the run by pivoting to the diagonal heading (no initial straight segment).
#define CAN_PHASED_START_WITH_PIVOT 1

// After a pivot/settle, the first drive tick can be stall-prone (static friction + fresh controller state).
// Apply a short minimum PWM kick at the start of each phased CAN DRIVE segment.
#define CAN_PHASED_DRIVE_START_ASSIST_MS 650
#define CAN_PHASED_DRIVE_START_ASSIST_PWM 120.0f

// Start debugging: trace motor commands briefly after run start.
// This helps confirm whether we are commanding a turn (different L/R PWM or dirs)
// or if one motor isn't receiving writes.
#define RUN_START_TRACE_ENABLE 1
#define RUN_START_TRACE_MS 1500
#define RUN_START_TRACE_PERIOD_MS 50

// Distance-stop braking pulse: briefly command reverse to reduce coast distance.
// This is only applied when we stop due to reaching the target distance ("DIST").
#define DIST_STOP_BRAKE_ENABLE 1
#define DIST_STOP_BRAKE_MS 180
#define DIST_STOP_BRAKE_PWM 120

// Braking / approach tuning near the stop distance.
// As we get within BRAKE_ZONE_M of the target distance, reduce the target speed
// and cap PWM, but still enforce a minimum PWM so we don't stall.
#define BRAKE_ZONE_M 0.25f
#define BRAKE_MIN_SPEED_FACTOR 0.70f
// For time-optimized runs (e.g. 10m/10s), capping PWM here can prevent hitting the time.
// Leave uncapped by default; distance stop handles the final cutoff.
#define BRAKE_MAX_PWM 255.0f
#define BRAKE_MIN_PWM 105.0f

// PID steering trim: keeps left/right pulse rates equal to avoid veering.
// Error is (leftRate - rightRate) in pulses/sec.
#define STEER_KP 0.35f
#define STEER_KI 0.020f
#define STEER_KD 0.00f

// Clamp steering integral contribution (PWM units) to avoid windup/oscillation.
#define STEER_I_MAX_CORR 120.0f
// Leak the steering integral so it can't build a long-period sinusoid.
// Larger value = slower leak (more persistence). Smaller = more damping.
#define STEER_I_LEAK_TAU_S 2.0f

// Deadband for steering I integration (pulses/sec). Prevents integrating quantization noise.
#define STEER_I_DEADBAND_PPS 6.0f

// IMU yaw assist: helps the car correct left/right faster than encoder-rate-only steering.
// Also used for CAN turning control (with gyro derivative).
#define STRAIGHT_YAW_KP 1.60f
#define STRAIGHT_YAW_MAX_CORR 180.0f
#define YAW_KD 0.35f
#define STRAIGHT_YAW_KD 0.25f

// Proportional deadband for yaw hold (deg): prevents rapid left/right toggling near target.
#define STRAIGHT_YAW_P_DEADBAND_DEG 0.25f

// Straight heading-hold integral (helps overcome deadband and steady biases)
#define STRAIGHT_YAW_KI 0.12f
#define STRAIGHT_YAW_I_MAX_CORR 80.0f
#define STRAIGHT_YAW_I_LEAK_TAU_S 2.0f

// Deadband for yaw integral: don't integrate when error is smaller than this (degrees).
// Prevents slow drift when car is already nearly straight.
#define STRAIGHT_YAW_I_DEADBAND_DEG 0.20f

// Encoder-primary mode: optional IMU assist (P+D only, no I). Keep small so encoders remain primary.
#define ENC_PRIMARY_YAW_KP 0.35f
#define ENC_PRIMARY_YAW_KD 0.12f
#define ENC_PRIMARY_YAW_MAX_CORR 25.0f

// Encoder-primary mode: add a small IMU yaw I term to cancel long-term mechanical bias
// (wheel diameter mismatch, drag, floor differences) that encoder equal-rate steering cannot eliminate.
#define ENC_PRIMARY_YAW_KI 0.04f
#define ENC_PRIMARY_YAW_I_MAX_CORR 18.0f
#define ENC_PRIMARY_YAW_I_LEAK_TAU_S 2.5f
#define ENC_PRIMARY_YAW_I_DEADBAND_DEG 0.20f

// Straight-run "bow" compensation:
// Use dead-reckoned lateral drift (meters) as a slow corrective steering term.
// This is intended to cancel systematic curvature that yaw-hold alone isn't eliminating.
#define STRAIGHT_LATERAL_COMP_ENABLE 1
#define STRAIGHT_LATERAL_MIN_FORWARD_M 0.50f
#define STRAIGHT_LATERAL_KP_PWM_PER_M 120.0f
#define STRAIGHT_LATERAL_MAX_CORR 60.0f

// Lateral position hold (outer loop): convert lateral error into a small yaw setpoint offset.
// This tends to behave better than injecting lateral correction directly in PWM units because
// it preserves the yaw controller's dynamics/limits.
#define STRAIGHT_LATERAL_HOLD_ENABLE 0
#define STRAIGHT_LATERAL_HOLD_MIN_FORWARD_M 0.50f
#define STRAIGHT_LATERAL_HOLD_DEADBAND_M 0.02f
#define STRAIGHT_LATERAL_HOLD_KP_DEG_PER_M 8.0f
#define STRAIGHT_LATERAL_HOLD_MAX_YAW_DEG 6.0f
#define STRAIGHT_LATERAL_HOLD_TAU_S 0.60f
#define STRAIGHT_LATERAL_HOLD_SLEW_DPS 25.0f

// Option 2: slow learned trim to cancel constant steering bias.
// Idea: during steady straight driving, the controller often needs a nonzero mean corr to keep
// yaw near zero (mechanical asymmetry). Learn a small additive corr bias (PWM units) so the
// mean controller output trends toward 0.
// Disabled: in run 109 this learned ~-37 PWM and directly reduced steering authority
// (corrUsed = corrOut + encTrimPwm), preventing correction of systematic drift.
#define RUN_TRIM_LEARN_ENABLE 0
#define RUN_TRIM_LEARN_START_DELAY_MS 1400
#define RUN_TRIM_LEARN_MIN_SPEED_MPS 0.18f
#define RUN_TRIM_LEARN_TAU_S 2.5f
#define RUN_TRIM_LEARN_MAX_PWM 70.0f
#define RUN_TRIM_LEARN_MAX_YAW_ERR_DEG 2.5f
#define RUN_TRIM_LEARN_MAX_YAW_RATE_DPS 12.0f

// Trim should only act as a *bias canceler* when we are actually trying to hold a near-zero yaw setpoint.
// If we are intentionally commanding a nonzero yaw (e.g., lateral-hold outer loop), applying trim can
// reduce effective steering authority (corrUsed < corrOut).
#define RUN_TRIM_APPLY_MAX_TARGET_YAW_DEG 0.35f
#define RUN_TRIM_APPLY_MAX_YAW_ERR_DEG 6.0f

// Safety: if yaw error stays large, abort the run instead of drifting into obstacles.
#define RUN_YAW_ABORT_ENABLE 1
#define RUN_YAW_ABORT_DEG 45.0f
#define RUN_YAW_ABORT_HOLD_MS 1500

// Hybrid mode mixing (ENC + IMU). Keep conservative; this is for A/B testing.
// Strategy: IMU does fast heading hold; encoders only learn a *slow trim* so the
// average IMU correction trends toward 0 over a long run (cancels motor bias).
#define HYBRID_MAX_CORR 180.0f
#define HYBRID_ENC_TRIM_TAU_S 6.0f
#define HYBRID_ENC_TRIM_KI 0.0008f
#define HYBRID_ENC_TRIM_MAX 25.0f
#define HYBRID_ENC_TRIM_DEADBAND_PPS 6.0f

// Optional in-run gyro bias learning.
// Disabled by default: if the car is *actually* yawing but wheel rates match (slip / drift),
// this can incorrectly learn real yaw rate as "bias", breaking heading hold.
#define RUN_GYRO_BIAS_LEARN 0
#define RUN_GYRO_BIAS_LEARN_TAU_S 8.0f

#define RUN_GYRO_BIAS_LEARN_MAX_STEP_DPS_PER_S 1.0f
#define RUN_GYRO_BIAS_LEARN_MIN_RATE_PPS 120.0f
#define RUN_GYRO_BIAS_LEARN_MAX_RATE_DIFF_PPS 10.0f
#define RUN_GYRO_BIAS_LEARN_MAX_YAW_ERR_DEG 3.0f
#define RUN_GYRO_BIAS_LEARN_MAX_CORR_PWM 10.0f

// Startup bias disabled (yaw assist replaces it)
#define STARTUP_BIAS_TICKS 0
#define STARTUP_BIAS_MAX 0.0f

// CAN mode (drive around cans) tuning
// Course is 2.0m wide, so LEFT boundary (outside can inside edge) is +1.0m from the centerline.
// User enters Inside Can Distance (meters) between INSIDE EDGES of the 2 cans.
// The midpoint between inside edges is our nominal target lateral:
//   targetLateralM = 1.0 - canDistanceM/2
#define CAN_BOUNDARY_M 1.0f
// Safety clamp: never allow CAN logic to command beyond this lateral distance from centerline.
// This is an override to prevent hitting the 1.0m boundary can even if gap math/settings drift.
#define CAN_MAX_LATERAL_M 0.90f
// Additional safety inset away from the boundary line (meters).
// This compensates for tracking/overshoot so we don't physically hit the 1.0m can even when the
// computed midpoint is close to the boundary (e.g., gap=0.2..0.3).
#define CAN_GAP_MIDLINE_INSET_M 0.00f
// Keep at least this much clearance from the inside can edge.
#define CAN_INNER_CLEARANCE_M 0.02f
#define CAN_TURN_DEG 90.0f
// Lateral tracking / phase thresholds (not a safety margin; just avoids requiring exact 0.000m)
#define CAN_LATERAL_TOL_M 0.03f
// While driving "forward" on a line, apply a small heading command proportional to lateral error.
// This helps re-center after the cans instead of staying parallel-offset.
#define CAN_LINE_K_DEG_PER_M 35.0f
#define CAN_LINE_MAX_DEG 25.0f
// During shift phases, enforce a minimum heading command while error is still significant.
#define CAN_SHIFT_MIN_DEG 8.0f
// Start returning to centerline this far AFTER the halfway point (down-course meters).
#define CAN_RETURN_AFTER_HALF_M 0.25f
// Optional small initial forward before first left shift.
#define CAN_START_FORWARD_M 0.0f
// Heading hold proportional gain (PWM per degree of yaw error)
#define YAW_KP 1.6f
#define YAW_MAX_CORR 110.0f
// When turning (|cos(yaw)| small), steer based on path-speed to avoid PI windup.
#define SPEED_TURN_COS_THRESHOLD 0.70f

static inline float wrapDeg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

// Smooth S-curve from 0->1 with zero slope at ends.
static inline float smoothstep01(float x) {
  x = clamp01(x);
  return x * x * (3.0f - 2.0f * x);
}

// Returns 0..1 gate based on distance into a segment and distance remaining.
static inline float sCurveGate(float distFromStartM, float distToEndM, float accelM, float decelM) {
  float up = 1.0f;
  float down = 1.0f;
  if (accelM > 0.01f) up = smoothstep01(distFromStartM / accelM);
  if (decelM > 0.01f) down = smoothstep01(distToEndM / decelM);
  return (up < down) ? up : down;
}

static inline void setMotorsStop() {
  motor_l_speed = 0;
  motor_r_speed = 0;
}

static inline void setMotorsForwardPwm(uint8_t leftPwm, uint8_t rightPwm) {
  motor_l_direction = 1;
  motor_r_direction = 1;
  motor_l_speed = leftPwm;
  motor_r_speed = rightPwm;
}

static inline void setMotorsReversePwm(uint8_t leftPwm, uint8_t rightPwm) {
  motor_l_direction = 0;
  motor_r_direction = 0;
  motor_l_speed = leftPwm;
  motor_r_speed = rightPwm;
}

// sign>0 = turn left, sign<0 = turn right (in vehicle frame).
static inline void setMotorsPivotPwm(int sign, uint8_t pwm) {
  if (sign >= 0) {
    motor_l_direction = 0;  // left backward
    motor_r_direction = 1;  // right forward
  } else {
    motor_l_direction = 1;  // left forward
    motor_r_direction = 0;  // right backward
  }
  motor_l_speed = pwm;
  motor_r_speed = pwm;
}

// HBridge diagnostics (fixed addresses 0x20/0x21 as you configured)
static bool motorL_present = false;
static bool motorR_present = false;
static uint8_t motorL_dir_rb = 0xFF;
static uint8_t motorR_dir_rb = 0xFF;
static uint8_t motorL_spd_rb = 0xFF;
static uint8_t motorR_spd_rb = 0xFF;
static uint8_t motorL_err_dir = 0xFF;
static uint8_t motorL_err_pwm = 0xFF;
static uint8_t motorR_err_dir = 0xFF;
static uint8_t motorR_err_pwm = 0xFF;

// Last values written to the HBridge registers (post inversion + min PWM clamp)
static uint8_t motorL_last_reg_dir = 0xFF;
static uint8_t motorL_last_reg_pwm = 0xFF;
static uint8_t motorR_last_reg_dir = 0xFF;
static uint8_t motorR_last_reg_pwm = 0xFF;

#if RUN_START_TRACE_ENABLE
static uint32_t dbg_startTraceUntilMs = 0;
static uint32_t dbg_lastStartTraceMs = 0;
#endif

static float runStartYawDeg = 0.0f;
// Gyro-integrated yaw relative to run start (deg). This is the heading estimate used for control.
static float runYawHoldDeg = 0.0f;

// Safety: stop if we exceed target time by this much.
#define RUN_MAX_OVERTIME_S 5.0f

static void enterScreen(ScreenState next) {
  if (currentScreen == next) return;
  runActive = false;
  motor_command = 0;
  stopAllMotors();

  // IMU calibration happens when entering the RUN screen (not on Start).
  // This keeps START latency low and makes calibration explicit.
  if (next == SCREEN_RUN) {
    if (imu_present && imu_control_enabled) {
      // Give the car a moment to settle (users often enter RUN while still handling it).
      delay(500);
      calibrateImuGyroBias();
      showImuReadyScreen(imu_gz_bias_dps);
      delay(250);
    }
  }
  currentScreen = next;
}

static void startRun() {
  dbg_lastStartOkMs = (uint32_t)millis();
#if RUN_START_DEBUG_SERIAL
  Serial.printf("RUN_START_ATTEMPT path=%s screen=%d redPin=%d bluePin=%d redLvl=%d blueLvl=%d ENC=%c%c M=%c%c\n",
                dbg_lastStartPath,
                (int)currentScreen,
                extRedBtnPin,
                extBlueBtnPin,
                dbg_redLevel,
                dbg_blueLevel,
                encoderL_found ? 'L' : '-',
                encoderR_found ? 'R' : '-',
                motorL_present ? 'L' : '-',
                motorR_present ? 'R' : '-');
#endif
  {
    char line[160];
    snprintf(line, sizeof(line),
             "RUN_START_ATTEMPT path=%s scr=%d redPin=%d bluePin=%d redLvl=%d blueLvl=%d ENC=%c%c M=%c%c",
             dbg_lastStartPath,
             (int)currentScreen,
             extRedBtnPin,
             extBlueBtnPin,
             dbg_redLevel,
             dbg_blueLevel,
             encoderL_found ? 'L' : '-',
             encoderR_found ? 'R' : '-',
             motorL_present ? 'L' : '-',
             motorR_present ? 'R' : '-');
    startDebugLogLine(line);
  }

  // Capture run/test metadata early so a log file exists even if we fail to start motion.
  runMeta.id = runMetaNextId++;
  if (prefs_ok) {
    prefs.putULong("run_id_next", runMetaNextId);
  }

  // Persist the attempt id immediately so we can see what the UI thought the run ID was,
  // even if we reset/brownout before saving.
  if (prefs_ok) {
    runMetaLastAttemptId = (unsigned long)runMeta.id;
    runMetaLastAttemptEpoch = (unsigned long)time(nullptr);
    prefs.putULong("rla_id", runMetaLastAttemptId);
    prefs.putULong("rla_ep", runMetaLastAttemptEpoch);
  }

  runActive = true;
  runStartMs = millis();
  runStartEpoch = time(nullptr);

  // Ensure the control loop fully re-initializes for this run even if quick-cal blocks.
  runControlResetRequest = true;

#if RUN_START_IMU_QUICKCAL_ENABLE
  // Optional quick cal; may block briefly on I2C.
  quickCalibrateImuGyroBiasForRunStart();
#endif

#if RUN_START_TRACE_ENABLE
  dbg_startTraceUntilMs = (uint32_t)runStartMs + (uint32_t)RUN_START_TRACE_MS;
  dbg_lastStartTraceMs = 0;
#endif

  runStartEncL = encoderL_count;
  runStartEncR = encoderR_count;
  runStartYawDeg = imu_yaw;
  runYawHoldDeg = 0.0f;

  lastRunStopReason = "RUNNING";
  lastRunElapsedS = 0.0f;
  lastRunMeters = 0.0f;
  lastRunForwardM = 0.0f;
  lastRunLateralM = 0.0f;
  lastRunWasCanMode = (canDistanceM >= CAN_ENABLE_MIN_M);

  runDebug_canPhase = 255;
  runDebug_forwardM = 0.0f;
  runDebug_lateralM = 0.0f;
  runDebug_targetYawDeg = 0.0f;
  runDebug_basePwm = 0.0f;
  runDebug_turnRemainingDeg = NAN;

  {
    char line[160];
    snprintf(line, sizeof(line), "RUN_ID id=%lu next=%lu", (unsigned long)runMeta.id, (unsigned long)runMetaNextId);
    startDebugLogLine(line);
  }
  runMeta.start_ms = runStartMs;
  runMeta.runDistanceM = runDistanceM;
  runMeta.runTimeS = runTimeS;
  runMeta.canDistanceM = canDistanceM;
  syncPulsesPerMeterDerived();
  runMeta.pulsesPerMeter = pulsesPerMeter;
  runMeta.pulsesPerMeterL = pulsesPerMeterL;
  runMeta.pulsesPerMeterR = pulsesPerMeterR;
  runMeta.motor_bias_pwm = motor_bias_pwm;
  runMeta.i2c_clock_hz = i2c_clock_hz;
  runMeta.control_period_ms = control_period_ms;
  runMeta.control_mode = (uint8_t)control_mode;
  runMeta.imu_present = imu_present ? 1 : 0;
  runMeta.imu_ok_at_start = imu_ok ? 1 : 0;
#if RUN_LOCK_PWM_ENABLE
  runMeta.run_lock_pwm_enable = 1;
  runMeta.run_lock_pwm_value = RUN_LOCK_PWM_VALUE;
#else
  runMeta.run_lock_pwm_enable = 0;
  runMeta.run_lock_pwm_value = 0.0f;
#endif
  runMeta.run_softstart_ms = (uint16_t)RUN_SOFTSTART_MS;
  printRunMetaToSerial("RUN_START");

  // New run: clear the "saved" latch so this run can be written exactly once.
  runLogSavedRunId = 0;
  runLogSavedThisRun = false;

  // Clear warnings.
  runWarnEncMissing = false;
  runWarnNoMotion = false;
  runWarnLoggingDisabled = false;

  // Reset control-loop timing diagnostics.
  runDbg_maxCtrlDtMs = 0;
  runDbg_maxCtrlDtAtTms = 0;
  runDbg_ctrlOverrunCount = 0;
  runDbg_ctrlStall200msCount = 0;

  // Start a fresh log for this run
  runLogReset();

#if RUNLOG_STREAM_TO_SPIFFS
  runLogStreamBegin();
#endif

  // If we cannot allocate RAM for logging, continue the run anyway.
  // Competition behavior must not depend on logging.
  // The resulting saved CSV will contain headers/metadata but no samples.
  if (!runLog || runLogCapacity == 0) {
    runLogOverflow = true;
    runWarnLoggingDisabled = true;
    startDebugLogLine("RUN_WARN logging_disabled reason=LOG_OOM");
  }

  // Choose a log decimation factor so we don't overflow RUNLOG_MAX_SAMPLES.
  // For short runs we log every control tick; for long runs we log less often.
  runLogTick = 0;
  runLogEveryN = 1;
  float expectedS = (runTimeS > 0.0f) ? runTimeS : 15.0f;
  if (expectedS > RUNLOG_DECIMATION_EXPECTED_S_MAX) expectedS = RUNLOG_DECIMATION_EXPECTED_S_MAX;
  const uint16_t cap = (runLogCapacity > 0) ? runLogCapacity : 1;
  const float usableSamples = (float)(cap - 50);
  if (usableSamples > 100.0f) {
    const float minPeriodMs = (expectedS * 1000.0f) / usableSamples;
    const float n = ceilf(minPeriodMs / (float)control_period_ms);
    if (n > 1.0f) runLogEveryN = (uint16_t)n;
  }

  // Reset controllers
  // (static locals in loop will use these reset flags)
}

static void stopRun() {
  // Idempotency: if we've already saved this run's log, don't overwrite it.
  if (!runActive && runLogSavedThisRun && runLogSavedRunId == (unsigned long)runMeta.id && runMeta.id != 0) {
    stopAllMotors();
    return;
  }

  // Optional: active braking to reduce coast distance when we stopped due to reaching the target distance.
  if (DIST_STOP_BRAKE_ENABLE && lastRunStopReason && strcmp(lastRunStopReason, "DIST") == 0) {
    setMotorsReversePwm((uint8_t)DIST_STOP_BRAKE_PWM, (uint8_t)DIST_STOP_BRAKE_PWM);
    applyMotorOutputs();
    delay((unsigned long)DIST_STOP_BRAKE_MS);
  }

  if (runStartMs != 0) {
    lastRunElapsedS = (millis() - runStartMs) / 1000.0f;
    if (lastRunElapsedS < 0.0f) lastRunElapsedS = 0.0f;
  }

  // Snapshot distance at stop so the STOPPED screen stays stable even if encoders change later.
  {
    const int32_t dL = encoderL_count - runStartEncL;
    const int32_t dR = encoderR_count - runStartEncR;
    const float dLcorr = (INVERT_LEFT_ENCODER_COUNT ? -(float)dL : (float)dL);
    const float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR);
    const float sL = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
    const float sR = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
    const float pathM = 0.5f * (sL + sR);
    lastRunMeters = fabsf(pathM);
  }

  // Snapshot CAN dead-reckoning state at stop (used for UI in CAN mode).
  lastRunForwardM = runDebug_forwardM;
  lastRunLateralM = runDebug_lateralM;

  runActive = false;
  stopAllMotors();

#if RUNLOG_STREAM_TO_SPIFFS
  runLogStreamEnd();
#endif

#if RUN_START_DEBUG_SERIAL
  Serial.printf("RUN_STOP reason=%s elapsed=%.2fs ENC=%c%c M=%c%c eL=%u/%u eR=%u/%u\n",
                lastRunStopReason ? lastRunStopReason : "?",
                (double)lastRunElapsedS,
                encoderL_found ? 'L' : '-',
                encoderR_found ? 'R' : '-',
                motorL_present ? 'L' : '-',
                motorR_present ? 'R' : '-',
                (unsigned)motorL_err_dir,
                (unsigned)motorL_err_pwm,
                (unsigned)motorR_err_dir,
                (unsigned)motorR_err_pwm);
#endif
  {
    char line[160];
    snprintf(line, sizeof(line),
             "RUN_STOP reason=%s elapsed=%.2fs ENC=%c%c M=%c%c eL=%u/%u eR=%u/%u",
             lastRunStopReason ? lastRunStopReason : "?",
             (double)lastRunElapsedS,
             encoderL_found ? 'L' : '-',
             encoderR_found ? 'R' : '-',
             motorL_present ? 'L' : '-',
             motorR_present ? 'R' : '-',
             (unsigned)motorL_err_dir,
             (unsigned)motorL_err_pwm,
             (unsigned)motorR_err_dir,
             (unsigned)motorR_err_pwm);
    startDebugLogLine(line);
  }

  printRunMetaToSerial("RUN_STOP");

  // Dump and save log for analysis
  runLogSaveToSpiffs();
  runLogSavedRunId = (unsigned long)runMeta.id;
  runLogSavedThisRun = true;

  {
    char line[200];
    snprintf(line, sizeof(line),
             "RUNLOG_SAVE id=%lu per_ok=%u per_bytes=%lu last_ok=%u last_bytes=%lu",
             (unsigned long)runMeta.id,
             (unsigned)runMetaLastSavedPerRunOk,
             (unsigned long)runMetaLastSavedPerRunBytes,
             (unsigned)runMetaLastSavedLastOk,
             (unsigned long)runMetaLastSavedLastBytes);
    startDebugLogLine(line);
  }

#if RUN_CLOUD_UPLOAD_ENABLE
  // Upload the saved CSV to a webhook for plotting.
  // Do this deferred by default so stopRun() stays responsive.
#if RUN_CLOUD_UPLOAD_ON_STOP
#if RUN_CLOUD_UPLOAD_DEFERRED
  cloudUploadPending = true;
  cloudUploadAttempts = 0;
  cloudUploadNextAttemptMs = millis();
  cloudUploadLastOk = false;
  Serial.println("CloudUpload: queued");
#else
  cloudUploadLastOk = cloudUploadLastRunCsv();
#endif
#endif
#endif

#if RUNLOG_PRINT_TO_SERIAL_ON_STOP
  runLogPrintCsvToSerial();
#endif

  // Prevent later UI actions (like exiting the RUN screen) from recomputing huge elapsed times.
  runStartMs = 0;
}

static int readDialDetentSteps() {
  encoderA_state = digitalRead(ENCODER_PIN_A);
  encoderB_state = digitalRead(ENCODER_PIN_B);

  uint8_t curr_state = (encoderA_state << 1) | encoderB_state;
  uint8_t prev_state = (encoderA_prev << 1) | encoderB_prev;

  if (curr_state != prev_state) {
    const bool cw =
      (prev_state == 0 && curr_state == 2) ||
      (prev_state == 2 && curr_state == 3) ||
      (prev_state == 3 && curr_state == 1) ||
      (prev_state == 1 && curr_state == 0);

    const bool ccw =
      (prev_state == 0 && curr_state == 1) ||
      (prev_state == 1 && curr_state == 3) ||
      (prev_state == 3 && curr_state == 2) ||
      (prev_state == 2 && curr_state == 0);

    static int8_t transition_accum = 0;
    if (cw) transition_accum += 1;
    else if (ccw) transition_accum -= 1;

    const int detent_transitions = 4;
    int steps = 0;
    while (transition_accum >= detent_transitions) {
      transition_accum -= detent_transitions;
      steps += 1;
    }
    while (transition_accum <= -detent_transitions) {
      transition_accum += detent_transitions;
      steps -= 1;
    }

    encoderA_prev = encoderA_state;
    encoderB_prev = encoderB_state;
    return steps * (int)DIAL_DIRECTION;
  }

  encoderA_prev = encoderA_state;
  encoderB_prev = encoderB_state;
  return 0;
}

// UI/input timing
unsigned long boot_ms = 0;
unsigned long last_button_ms = 0;

// When we exit a screen via long-press, ignore the subsequent release-click.
static bool suppressNextClick = false;

// Forward declarations
static bool i2cPing(uint8_t addr);
void stopAllMotors();
void applyMotorOutputs();
static void updateMotorDiagnostics();
void readWheelEncoders();
void updateDisplay();

static void readWheelEncodersInternal(bool force);
static void readWheelEncodersForce() { readWheelEncodersInternal(true); }

void readWheelEncoders() {
  readWheelEncodersInternal(false);
}

static void readWheelEncodersInternal(bool force) {
  // Throttle reads to keep I2C stable and avoid spamming errors.
  static unsigned long last_read_ms = 0;
  const unsigned long now = millis();
  if (!force && (now - last_read_ms < (unsigned long)control_period_ms)) return;
  last_read_ms = now;

  encoderL_found = false;
  encoderR_found = false;

  // Only attempt requestFrom() if the device ACKs at all.
  if (i2cPing(ENCODER_L_ADDR)) {
    Wire.beginTransmission(ENCODER_L_ADDR);
    Wire.write((uint8_t)0x00);
    if (Wire.endTransmission(true) == 0) {
      delayMicroseconds(200);
      if (Wire.requestFrom((uint8_t)ENCODER_L_ADDR, (size_t)4) == 4 && Wire.available() >= 4) {
        int32_t value = 0;
        value |= (int32_t)Wire.read() << 0;
        value |= (int32_t)Wire.read() << 8;
        value |= (int32_t)Wire.read() << 16;
        value |= (int32_t)Wire.read() << 24;
        encoderL_count = value;
        encoderL_found = true;
      }
    }
  }

  if (i2cPing(ENCODER_R_ADDR)) {
    Wire.beginTransmission(ENCODER_R_ADDR);
    Wire.write((uint8_t)0x00);
    if (Wire.endTransmission(true) == 0) {
      delayMicroseconds(200);
      if (Wire.requestFrom((uint8_t)ENCODER_R_ADDR, (size_t)4) == 4 && Wire.available() >= 4) {
        int32_t value = 0;
        value |= (int32_t)Wire.read() << 0;
        value |= (int32_t)Wire.read() << 8;
        value |= (int32_t)Wire.read() << 16;
        value |= (int32_t)Wire.read() << 24;
        encoderR_count = value;
        encoderR_found = true;
      }
    }
  }
}

void driveMotor(uint8_t motor_addr, uint8_t direction, uint8_t speed) {
  // H-Bridge v1.1 protocol (matches the previously working backup code):
  // Register 0x00: Direction (0=STOP, 1=FORWARD, 2=BACKWARD)
  // Register 0x01: Speed PWM (0-255)
  // Empirically, PWM below ~51 may stall the motor.

  uint8_t reg_dir = 0;
  uint8_t pwm = 0;

  if (speed == 0) {
    reg_dir = 0;
    pwm = 0;
  } else {
    uint8_t logical_dir = (direction == 1) ? 1 : 0; // 1=fwd, 0=rev
  #if INVERT_MOTOR_DIRECTION
    logical_dir = logical_dir ? 0 : 1;
  #endif

  #if INVERT_LEFT_MOTOR_DIRECTION
    if (motor_addr == MOTOR_L_ADDR) logical_dir = logical_dir ? 0 : 1;
  #endif

  #if INVERT_RIGHT_MOTOR_DIRECTION
    if (motor_addr == MOTOR_R_ADDR) logical_dir = logical_dir ? 0 : 1;
  #endif
    reg_dir = logical_dir ? 1 : 2;
    pwm = (speed < 51) ? 51 : speed;
  }

  uint8_t err_dir = 0;
  uint8_t err_pwm = 0;

  Wire.beginTransmission(motor_addr);
  Wire.write(0x00);
  Wire.write(reg_dir);
  err_dir = Wire.endTransmission(true);

  Wire.beginTransmission(motor_addr);
  Wire.write(0x01);
  Wire.write(pwm);
  err_pwm = Wire.endTransmission(true);

  // Store last I2C status so we can see whether 0x20/0x21 are ACKing.
  if (motor_addr == MOTOR_L_ADDR) {
    motorL_err_dir = err_dir;
    motorL_err_pwm = err_pwm;
    motorL_last_reg_dir = reg_dir;
    motorL_last_reg_pwm = pwm;
  } else if (motor_addr == MOTOR_R_ADDR) {
    motorR_err_dir = err_dir;
    motorR_err_pwm = err_pwm;
    motorR_last_reg_dir = reg_dir;
    motorR_last_reg_pwm = pwm;
  }

  // Debug log on errors
  static unsigned long lastLog = 0;
#if SERIAL_MOTOR_ERR_SPAM
  if (millis() - lastLog > 2000) {
    lastLog = millis();
    if (err_dir != 0 || err_pwm != 0) {
      Serial.printf("Motor 0x%02X ERR: dir=%d pwm=%d err_dir=%d err_pwm=%d\n",
                    motor_addr, reg_dir, pwm, err_dir, err_pwm);
    }
  }
#endif
}

static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true) == 0;
}

static bool readReg8(uint8_t addr, uint8_t reg, uint8_t* out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return false;
  delayMicroseconds(200);
  if (Wire.requestFrom((uint8_t)addr, (size_t)1) != 1) return false;
  if (!Wire.available()) return false;
  *out = Wire.read();
  return true;
}

static void updateMotorDiagnostics() {
  motorL_present = i2cPing(MOTOR_L_ADDR);
  motorR_present = i2cPing(MOTOR_R_ADDR);

  if (motorL_present) {
    uint8_t v;
    if (readReg8(MOTOR_L_ADDR, 0x00, &v)) motorL_dir_rb = v;
    if (readReg8(MOTOR_L_ADDR, 0x01, &v)) motorL_spd_rb = v;
  } else {
    motorL_dir_rb = 0xFF;
    motorL_spd_rb = 0xFF;
  }

  if (motorR_present) {
    uint8_t v;
    if (readReg8(MOTOR_R_ADDR, 0x00, &v)) motorR_dir_rb = v;
    if (readReg8(MOTOR_R_ADDR, 0x01, &v)) motorR_spd_rb = v;
  } else {
    motorR_dir_rb = 0xFF;
    motorR_spd_rb = 0xFF;
  }
}

static void hbridgeWriteDirectionSpeed8(uint8_t motor_addr, uint8_t direction, uint8_t speed, uint8_t* err_dir, uint8_t* err_spd) {
  // Official Unit HBridge v1.1 protocol (per M5Unit-Hbridge library):
  // reg 0x00: direction (0=STOP, 1=FORWARD, 2=BACKWARD)
  // reg 0x01: speed (8-bit)
  uint8_t e0 = 0;
  uint8_t e1 = 0;

  Wire.beginTransmission(motor_addr);
  Wire.write((uint8_t)0x00);
  Wire.write(direction);
  e0 = Wire.endTransmission(true);

  Wire.beginTransmission(motor_addr);
  Wire.write((uint8_t)0x01);
  Wire.write(speed);
  e1 = Wire.endTransmission(true);

  if (err_dir) *err_dir = e0;
  if (err_spd) *err_spd = e1;
}

void stopAllMotors() {
  motor_l_speed = 0;
  motor_r_speed = 0;
  applyMotorOutputs();
}

void applyMotorOutputs() {
  // Avoid spamming I2C; only send when values change or at a low refresh rate.
  // Note: keeping this interval > control_period_ms helps reduce I2C load and jitter
  // when menus call stopAllMotors() frequently.
  const unsigned long keepalive_ms = 250;
  static unsigned long last_send_ms = 0;
  static uint8_t last_ldir = 255;
  static uint8_t last_rdir = 255;
  static uint8_t last_l = 255;
  static uint8_t last_r = 255;

  const unsigned long now = millis();
  const bool values_changed = (motor_l_direction != last_ldir) || (motor_r_direction != last_rdir) || (motor_l_speed != last_l) || (motor_r_speed != last_r);
  if (!values_changed && (now - last_send_ms) < keepalive_ms) {
    return;
  }

  last_send_ms = now;
  last_ldir = motor_l_direction;
  last_rdir = motor_r_direction;
  last_l = motor_l_speed;
  last_r = motor_r_speed;

  const uint32_t t0us = (uint32_t)micros();

  // Write order can matter slightly if one motor effectively gets commanded earlier on each I2C update.
  // Alternate which motor is written first to avoid introducing a consistent steer bias.
  static bool writeRightFirst = false;
  writeRightFirst = !writeRightFirst;
  if (writeRightFirst) {
    driveMotor(MOTOR_R_ADDR, motor_r_direction, motor_r_speed);
    driveMotor(MOTOR_L_ADDR, motor_l_direction, motor_l_speed);
  } else {
    driveMotor(MOTOR_L_ADDR, motor_l_direction, motor_l_speed);
    driveMotor(MOTOR_R_ADDR, motor_r_direction, motor_r_speed);
  }

  const uint32_t t1us = (uint32_t)micros();
  runDbg_lastMotorWriteUs = (t1us >= t0us) ? (t1us - t0us) : 0;

#if RUN_START_TRACE_ENABLE
  if (runActive) {
    const uint32_t ms = (uint32_t)millis();
    if (dbg_startTraceUntilMs != 0 && ms <= dbg_startTraceUntilMs) {
      if (dbg_lastStartTraceMs == 0 || (ms - dbg_lastStartTraceMs) >= (uint32_t)RUN_START_TRACE_PERIOD_MS) {
        dbg_lastStartTraceMs = ms;
        const int32_t dL = encoderL_count - runStartEncL;
        const int32_t dR = encoderR_count - runStartEncR;
        char line[220];
        snprintf(line, sizeof(line),
                 "START_TRACE t=%lums reqL(d%u,p%u) reqR(d%u,p%u) wrL(reg%u,p%u,e%u/%u) wrR(reg%u,p%u,e%u/%u) enc(dL=%ld dR=%ld)",
                 (unsigned long)(ms - runStartMs),
                 (unsigned)motor_l_direction,
                 (unsigned)motor_l_speed,
                 (unsigned)motor_r_direction,
                 (unsigned)motor_r_speed,
                 (unsigned)motorL_last_reg_dir,
                 (unsigned)motorL_last_reg_pwm,
                 (unsigned)motorL_err_dir,
                 (unsigned)motorL_err_pwm,
                 (unsigned)motorR_last_reg_dir,
                 (unsigned)motorR_last_reg_pwm,
                 (unsigned)motorR_err_dir,
                 (unsigned)motorR_err_pwm,
                 (long)dL,
                 (long)dR);
        startDebugLogLine(line);
#if RUN_START_DEBUG_SERIAL
        Serial.println(line);
#endif
      }
    }
  }
#endif
}

static void applyHardwareTestDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  const int step_per_detent = 5;
  int next_cmd = (int)motor_command + detentSteps * step_per_detent;
  if (next_cmd < -255) next_cmd = -255;
  if (next_cmd > 255) next_cmd = 255;
  if (abs(next_cmd) < step_per_detent) next_cmd = 0;
  motor_command = (int16_t)next_cmd;
}

static void applyMenuDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  int next = selectedMenuItem + detentSteps;
  while (next < 0) next += NUM_MENU_ITEMS;
  while (next >= NUM_MENU_ITEMS) next -= NUM_MENU_ITEMS;
  selectedMenuItem = next;
}

static void applyDistanceDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  // Move in 0.05m increments, then snap to the nearest valid rules grid value
  // (either 0.10m or 0.25m), so values like 8.10 and 8.25 are both reachable.
  runDistanceM += detentSteps * RUN_DISTANCE_DIAL_STEP_M;
  runDistanceM = quantizeRunDistanceRulesDirectional(runDistanceM, detentSteps);

  settings_dirty = true;
  settings_dirty_ms = millis();
}

static void applyTimeDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  runTimeS += detentSteps * RUN_TIME_STEP_S;
  if (runTimeS < RUN_TIME_MIN_S) runTimeS = RUN_TIME_MIN_S;
  if (runTimeS > RUN_TIME_MAX_S) runTimeS = RUN_TIME_MAX_S;
  runTimeS = quantizeToStep(runTimeS, RUN_TIME_STEP_S);

  settings_dirty = true;
  settings_dirty_ms = millis();
}

static void applyCanDistanceDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  // Inside Can Distance is measured in cm; allow 1cm resolution.
  canDistanceM += detentSteps * 0.01f;
  if (canDistanceM < 0.0f) canDistanceM = 0.0f;
  // Upper bound: rules allow up to ~110cm between inside edges.
  if (canDistanceM > 1.10f) canDistanceM = 1.10f;

  settings_dirty = true;
  settings_dirty_ms = millis();
}

static void applyEncoderCalDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  const float step = 5.0f;  // pulses per detent
  float* target = encCalEditingLeft ? &pulsesPerMeterL : &pulsesPerMeterR;
  float next = *target + (float)detentSteps * step;
  next = clampPpm(next);
  if (fabsf(*target - next) > 0.001f) {
    *target = next;
    syncPulsesPerMeterDerived();
    settings_dirty = true;
    settings_dirty_ms = millis();
  }
}

static void applyControlModeDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  int next = (int)control_mode + detentSteps;
  const int maxMode = (int)CTRL_MODE_HYBRID;
  while (next < 0) next += (maxMode + 1);
  while (next > maxMode) next -= (maxMode + 1);
  if ((int)control_mode != next) {
    control_mode = (ControlMode)next;
    settings_dirty = true;
    settings_dirty_ms = millis();
  }
}

static void applyBfsDialSteps(int detentSteps, int& bfs_setup_step) {
  // bfs_setup_step 0 = selecting start node
  // bfs_setup_step 1 = selecting goal node
  if (detentSteps == 0) return;
  
  if (bfs_setup_step == 0) {
    int32_t next = (int32_t)bfs_start_node + detentSteps;
    if (next < 0) next = 0;
    if (next >= (int32_t)bfs_state.node_count) next = bfs_state.node_count - 1;
    bfs_start_node = (uint8_t)next;
  } else {
    int32_t next = (int32_t)bfs_goal_node + detentSteps;
    if (next < 0) next = 0;
    if (next >= (int32_t)bfs_state.node_count) next = bfs_state.node_count - 1;
    bfs_goal_node = (uint8_t)next;
  }
  settings_dirty = true;
  settings_dirty_ms = millis();
}

static void drawMainMenu() {
  ui.fillScreen(TFT_BLACK);

  // Force a fresh I2C snapshot so the user can trust the RUN screen diagnostics.
  // If the critical devices aren't present, do not start the run (safer than open-loop).
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(1);
  ui.drawString("MENU", 120, 15);

  float radius = 70.0f;
  float centerX = 120.0f;
  float centerY = 120.0f;

  for (int i = 0; i < NUM_MENU_ITEMS; i++) {
    float angle = (i * (360.0f / NUM_MENU_ITEMS) - 90.0f) * (float)M_PI / 180.0f;
    float itemX = centerX + radius * cosf(angle);
    float itemY = centerY + radius * sinf(angle);

    if (i == selectedMenuItem) {
      ui.fillCircle(itemX, itemY, 30, TFT_DARKGREEN);
      ui.drawCircle(itemX, itemY, 30, TFT_GREEN);
      ui.setTextColor(TFT_WHITE);
      ui.setTextSize(2);
    } else {
      ui.drawCircle(itemX, itemY, 25, TFT_DARKGREY);
      ui.setTextColor(TFT_DARKGREY);
      ui.setTextSize(1);
    }

    ui.drawString(String(i + 1), itemX, itemY);
  }

  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(2);
  ui.drawString(menuNames[selectedMenuItem], centerX, centerY);

  // Status lines (keep readable; don't cram everything onto one line)
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(2);
  const String line1 = String(runDistanceM, 2) + "m / " + String(runTimeS, 1) + "s";
  ui.drawString(line1, 120, 195);

  ui.setTextColor(TFT_WHITE);
  const String line2 = (canDistanceM >= CAN_ENABLE_MIN_M) ? (String("CAN DIST: ") + String(canDistanceM, 2) + "m") : String("CAN DIST: OFF");
  ui.drawString(line2, 120, 220);

  ui.setTextSize(1);
}

static void drawSetDistance() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("SET DISTANCE", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Turn dial to adjust", 120, 55);

  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(4);
  ui.drawString(String(runDistanceM, 2), 120, 115);
  ui.setTextSize(2);
  ui.drawString("meters", 120, 155);

  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(2);
  ui.drawString("Press to save", 120, 210);
  ui.setTextSize(1);
}

static void drawSetControlMode() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("CTRL MODE", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Turn dial to adjust", 120, 55);
  ui.drawString("Click to save", 120, 70);

  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(3);
  ui.drawString(controlModeName(control_mode), 120, 125);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("IMU = gyro heading hold", 120, 170);
  ui.drawString("ENC = rate match only", 120, 185);
  ui.drawString("HYBRID = ENC + IMU", 120, 200);
}

static void drawSetTime() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("SET TIME", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Turn dial to adjust", 120, 55);

  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(4);
  ui.drawString(String(runTimeS, 1), 120, 115);
  ui.setTextSize(2);
  ui.drawString("seconds", 120, 155);

  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(2);
  ui.drawString("Press to save", 120, 210);
  ui.setTextSize(1);
}

static void drawSetCanDistance() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("CAN DIST", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Turn dial to adjust", 120, 55);
  ui.drawString("Distance between", 120, 70);
  ui.drawString("inside can edges", 120, 85);
  ui.drawString("0 = straight run", 120, 100);

  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(4);
  ui.drawString(String(canDistanceM, 2), 120, 115);
  ui.setTextSize(2);
  ui.drawString("meters", 120, 155);

  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(2);
  ui.drawString("Press to save", 120, 210);
  ui.setTextSize(1);
}

static void drawBiasTestScreen() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_ORANGE);
  ui.setTextSize(2);
  ui.drawString("BIAS TEST", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Feedback DISABLED", 120, 50);
  ui.drawString("Turn dial: adjust bias", 120, 65);
  ui.drawString("+ = boost LEFT motor", 120, 80);

  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(4);
  ui.drawString(String((int)motor_bias_pwm), 120, 125);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("bias pwm", 120, 165);

  if (bias_test_mode) {
    ui.setTextColor(TFT_RED);
    ui.setTextSize(2);
    ui.drawString("RUNNING", 120, 190);
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(1);
    ui.drawString("Press to STOP", 120, 210);
    ui.drawString("Hold: SAVE + EXIT", 120, 225);
  } else {
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(2);
    ui.drawString("STOPPED", 120, 190);
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(1);
    ui.drawString("Press to START", 120, 210);
    ui.drawString("Hold: SAVE + EXIT", 120, 225);
  }
  ui.setTextSize(1);
}

static void drawSetEncoderCal() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("ENC CAL", 120, 25);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  ui.drawString("Dial: adjust selected", 120, 55);
  ui.drawString("Click: toggle L/R", 120, 70);
  ui.drawString("Hold: save + exit", 120, 85);

  ui.setTextSize(2);
  ui.setTextColor(encCalEditingLeft ? TFT_GREEN : TFT_DARKGREY);
  ui.drawString(String("L ") + String(pulsesPerMeterL, 1), 120, 125);
  ui.setTextColor(encCalEditingLeft ? TFT_DARKGREY : TFT_GREEN);
  ui.drawString(String("R ") + String(pulsesPerMeterR, 1), 120, 155);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(String("avg ") + String(pulsesPerMeter, 1), 120, 185);
  ui.setTextSize(1);
}

static void drawRunScreen() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 120, 2);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("CAR RUN", 120, 15);

  // Run tracking + date/time
  ui.setTextSize(2);
  ui.setTextColor(TFT_WHITE);
  if (runActive) ui.drawString(String("Run ID ") + String((unsigned long)runMeta.id), 120, 38);
  else ui.drawString(String("Next Run ") + String((unsigned long)runMetaNextId), 120, 38);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  if (runActive) {
    ui.drawString("BtnA: STOP", 120, 66);
  } else {
    ui.drawString("BtnA: EXIT", 120, 66);
  }
  ui.drawString("RED: START (RUN screen)", 120, 78);
  {
    String warn = "";
    if (runWarnLoggingDisabled) warn += "LOG OFF ";
    if (runWarnEncMissing) warn += "ENC MISS ";
    if (runWarnNoMotion) warn += "NO MOTION ";
    if (warn.length() > 0) {
      ui.setTextColor(TFT_RED);
      ui.drawString("WARN: " + warn, 120, 90);
      ui.setTextColor(TFT_WHITE);
    }
  }

  float elapsed = 0.0f;
  if (runActive) {
    elapsed = (millis() - runStartMs) / 1000.0f;
    if (elapsed < 0.0f) elapsed = 0.0f;
  } else {
    elapsed = lastRunElapsedS;
  }

  float meters = 0.0f;
  if (runActive) {
    const int32_t dL = encoderL_count - runStartEncL;
    const int32_t dR = encoderR_count - runStartEncR;
    // Distance traveled along the vehicle's forward path is proportional to the *average* wheel travel.
    // Using fabs(L)+fabs(R) over-counts whenever one wheel slows/reverses during steering/wobble.
    const float dLcorr = (INVERT_LEFT_ENCODER_COUNT ? -(float)dL : (float)dL);
    const float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR);
    const float sL = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
    const float sR = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
    const float pathM = 0.5f * (sL + sR);
    meters = fabsf(pathM);
  } else {
    meters = lastRunMeters;
  }

  const bool showForwardM = runActive ? (canDistanceM >= CAN_ENABLE_MIN_M) : lastRunWasCanMode;
  const float shownM = showForwardM ? (runActive ? runDebug_forwardM : lastRunForwardM) : meters;

  ui.setTextColor(runActive ? TFT_GREEN : TFT_YELLOW);
  ui.setTextSize(2);
  ui.drawString(runActive ? "RUNNING" : "STOPPED", 120, 98);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(4);
  ui.drawString(String(shownM, 2) + " m", 120, 145);

  // Keep the elapsed timer visible for recording results.
  ui.setTextSize(2);
  ui.drawString(String(elapsed, 1) + " s", 120, 170);

  // Show active setpoints so it's clear what the run will use.
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(String("Target: ") + String(runDistanceM, 2) + "m / " + String(runTimeS, 1) + "s", 120, 185);

  ui.setTextSize(1);
  if (canDistanceM >= CAN_ENABLE_MIN_M) {
    ui.setTextColor(TFT_ORANGE);
    ui.drawString("CAN DIST: " + String(canDistanceM, 2) + "m", 120, 198);
  } else {
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("CAN DIST: OFF", 120, 198);
  }

  float pct_time = runTimeS > 0.0f ? (elapsed / runTimeS) : 0.0f;
  if (pct_time < 0.0f) pct_time = 0.0f;
  if (pct_time > 1.0f) pct_time = 1.0f;

  const float progressM = showForwardM ? shownM : meters;
  float pct_dist = runDistanceM > 0.0f ? (progressM / runDistanceM) : 0.0f;
  if (pct_dist < 0.0f) pct_dist = 0.0f;
  if (pct_dist > 1.0f) pct_dist = 1.0f;

  const float pct = (pct_time > pct_dist) ? pct_time : pct_dist;
  int bar_w = (int)(200.0f * pct);
  ui.drawRect(20, 210, 200, 12, TFT_DARKGREY);
  ui.fillRect(20, 210, bar_w, 12, TFT_BLUE);
}

// ========================================================================
// BFS NAV DISPLAY SCREEN
// ========================================================================
static void drawBfsNavScreen(int bfs_setup_step) {
  ui.fillScreen(TFT_BLACK);
  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("BFS PATHFINDING", 120, 30);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  
  // Show current setup step
  ui.setTextColor(bfs_setup_step == 0 ? TFT_YELLOW : TFT_DARKGREY);
  ui.drawString("Start Node:", 30, 80);
  ui.drawString(bfs_state.nodes[bfs_start_node].name, 160, 80);
  
  ui.setTextColor(bfs_setup_step == 1 ? TFT_YELLOW : TFT_DARKGREY);
  ui.drawString("Goal Node:", 30, 110);
  ui.drawString(bfs_state.nodes[bfs_goal_node].name, 160, 110);
  
  // Show all available nodes as reference
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Nodes:", 30, 150);
  
  char nodeList[64] = "";
  for (uint8_t i = 0; i < bfs_state.node_count; i++) {
    if (i > 0) strcat(nodeList, " ");
    strcat(nodeList, bfs_state.nodes[i].name);
  }
  ui.drawString(nodeList, 30, 165);
  
  // Instructions
  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(1);
  ui.drawString("Turn dial to select, click to toggle", 30, 190);
  ui.drawString("Press & hold to start navigation", 30, 205);
}

static void renderCurrentScreen() {
  // BFS Nav screen setup state (persistent across re-renders)
  static int bfs_setup_step = 0;
  
  switch (currentScreen) {
    case SCREEN_MAIN_MENU:
      drawMainMenu();
      break;
    case SCREEN_RUN:
      drawRunScreen();
      break;
    case SCREEN_SET_DISTANCE:
      drawSetDistance();
      break;
    case SCREEN_SET_TIME:
      drawSetTime();
      break;
    case SCREEN_SET_CAN_DISTANCE:
      drawSetCanDistance();
      break;
    case SCREEN_BIAS_TEST:
      drawBiasTestScreen();
      break;
    case SCREEN_SET_ENC_CAL:
      drawSetEncoderCal();
      break;
    case SCREEN_SET_CTRL_MODE:
      drawSetControlMode();
      break;
    case SCREEN_BFS_NAV:
      drawBfsNavScreen(bfs_setup_step);
      break;
    case SCREEN_HW_TEST:
      updateDisplay();
      break;
  }

  // Present the full frame at once to avoid visible redraw flicker.
  ui.pushSprite(0, 0);
}

void testMotorFormats() {
  // Try different motor command formats
  Serial.println("\n=== MOTOR DIAGNOSTIC TEST ===");
  Serial.println("Testing motor controller at 0x20 (RIGHT motor)...\n");
 
  // Format 1: [direction, speed]
  Serial.println("Format 1: [direction=1, speed=100]");
  Wire.beginTransmission(MOTOR_R_ADDR);
  Wire.write(1);    // Forward
  Wire.write(100);  // Speed
  uint8_t err1 = Wire.endTransmission();
  Serial.printf("Result: error=%d\n\n", err1);
  delay(500);
 
  // Format 2: [speed, direction]
  Serial.println("Format 2: [speed=100, direction=1]");
  Wire.beginTransmission(MOTOR_R_ADDR);
  Wire.write(100);  // Speed
  Wire.write(1);    // Forward
  uint8_t err2 = Wire.endTransmission();
  Serial.printf("Result: error=%d\n\n", err2);
  delay(500);
 
  // Format 3: Just speed
  Serial.println("Format 3: [speed=150]");
  Wire.beginTransmission(MOTOR_R_ADDR);
  Wire.write(150);
  uint8_t err3 = Wire.endTransmission();
  Serial.printf("Result: error=%d\n\n", err3);
  delay(500);
 
  // Format 4: Register address + command
  Serial.println("Format 4: [register=0, direction=1, speed=100]");
  Wire.beginTransmission(MOTOR_R_ADDR);
  Wire.write(0);    // Register
  Wire.write(1);    // Forward
  Wire.write(100);  // Speed
  uint8_t err4 = Wire.endTransmission();
  Serial.printf("Result: error=%d\n\n", err4);
  delay(500);
 
  // Format 5: Read back what's stored on motor controller
  Serial.println("Format 5: Attempting to READ from motor controller register 0");
  Wire.beginTransmission(MOTOR_R_ADDR);
  Wire.write(0);
  if (Wire.endTransmission(false) == 0) {
    int avail = (int)Wire.requestFrom((uint8_t)MOTOR_R_ADDR, (size_t)2);
    if (avail > 0) {
      uint8_t val1 = Wire.read();
      Serial.printf("Got %d bytes. Values: 0x%02X", avail, val1);
      if (avail > 1) Serial.printf(" 0x%02X", Wire.read());
      Serial.println();
    } else {
      Serial.println("No data returned");
    }
  } else {
    Serial.println("Transmission failed");
  }
 
  Serial.println("\n=== END DIAGNOSTIC ===\n");
}

void scanI2CBus() {
  Serial.println("\n=== I2C BUS SCAN ===");
  Serial.println("Scanning I2C addresses 0x00-0x7F...\n");
 
  byte devices_found = 0;
 
  for(byte i = 0; i < 128; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
   
    if (error == 0) {
      Serial.printf("FOUND: 0x%02X", i);
     
      // Identify known devices
      if (i == ENCODER_L_ADDR) Serial.print(" <- Left Wheel Encoder");
      else if (i == ENCODER_R_ADDR) Serial.print(" <- Right Wheel Encoder");
      else if (i == MOTOR_L_ADDR) Serial.print(" <- Left Motor Driver");
      else if (i == MOTOR_R_ADDR) Serial.print(" <- Right Motor Driver");
      else if (i == 0x68) Serial.print(" <- IMU (MPU6050)");
     
      Serial.println();
      devices_found++;
    }
  }
 
  Serial.printf("\nTotal devices found: %d\n", devices_found);
  Serial.println("===========================\n");
}

void updateDisplay() {
  ui.fillScreen(TFT_BLACK);

  // Title
  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("HW TEST", 120, 15);

  // I2C clock status
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(String("I2C: ") + String((unsigned)(i2c_clock_hz / 1000u)) + " kHz", 120, 30);
  ui.drawString("Hold BtnA: exit", 120, 45);
 
  // Selected motor
  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(1);
  ui.drawString("Motor:", 120, 65);
 
  ui.setTextColor(selected_motor == 0 ? TFT_GREEN : TFT_WHITE);
  ui.setTextSize(2);
  ui.drawString(selected_motor == 0 ? "LEFT" : "RIGHT", 120, 85);
 
  // Speed display
  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(1);
  ui.drawString("Speed (+/-):", 120, 110);
 
  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(2);
  char spd_str[16];
  snprintf(spd_str, sizeof(spd_str), "%d", (int)motor_command);
  ui.drawString(spd_str, 120, 135);

  // Legend (use sign for direction)
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
#if INVERT_MOTOR_DIRECTION
  ui.drawString("+ = REV   - = FWD", 120, 160);
#else
  ui.drawString("+ = FWD   - = REV", 120, 160);
#endif

  // Exit hint
  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(1);
  ui.drawString("Hold BtnA: MENU", 120, 172);

  // IMU attitude (degrees)
  ui.setTextSize(1);
  ui.setTextColor(TFT_WHITE);
  if (imu_ok) {
    char imu1[48];
    char imu2[48];
    // Display as X/Y/Z degrees for your test harness
    snprintf(imu1, sizeof(imu1), "IMU deg  X:% .1f  Y:% .1f", (double)imu_roll, (double)imu_pitch);
    snprintf(imu2, sizeof(imu2), "         Z:% .1f", (double)imu_yaw);
    ui.drawString(imu1, 120, 182);
    ui.drawString(imu2, 120, 194);
  } else {
    ui.drawString("IMU deg  X:--  Y:--  Z:--", 120, 188);
  }


  // Wheel encoder values (larger for readability)
  ui.setTextColor(TFT_BLUE);
  ui.setTextSize(2);
  char lbuf[32];
  char rbuf[32];
  if (encoderL_found) snprintf(lbuf, sizeof(lbuf), "L:%ld", encoderL_count);
  else snprintf(lbuf, sizeof(lbuf), "L:--");
  if (encoderR_found) snprintf(rbuf, sizeof(rbuf), "R:%ld", encoderR_count);
  else snprintf(rbuf, sizeof(rbuf), "R:--");
  ui.drawString(lbuf, 120, 210);
  ui.drawString(rbuf, 120, 230);
}

static void showBootMotorTestScreen(const char* line1, const char* line2, const char* line3) {
  ui.fillScreen(TFT_BLACK);
  ui.setTextDatum(middle_center);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("BOOT MOTOR TEST", 120, 30);

  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  if (line1) ui.drawString(line1, 120, 80);
  if (line2) ui.drawString(line2, 120, 100);
  if (line3) ui.drawString(line3, 120, 120);

  ui.setTextColor(TFT_BLUE);
  ui.setTextSize(1);
  char lbuf[32];
  char rbuf[32];
  if (encoderL_found) snprintf(lbuf, sizeof(lbuf), "L:%ld", encoderL_count);
  else snprintf(lbuf, sizeof(lbuf), "L:--");
  if (encoderR_found) snprintf(rbuf, sizeof(rbuf), "R:%ld", encoderR_count);
  else snprintf(rbuf, sizeof(rbuf), "R:--");
  ui.drawString(lbuf, 60, 210);
  ui.drawString(rbuf, 180, 210);

  ui.pushSprite(0, 0);
}

static void runBootMotorSpinTest() {
  const uint8_t addr = (uint8_t)BOOT_TEST_MOTOR_ADDR;
  const uint8_t dir = (uint8_t)BOOT_TEST_DIRECTION;
  const uint8_t pwm = (uint8_t)BOOT_TEST_PWM;

  Serial.println("\n=== BOOT MOTOR SPIN TEST ===");
  Serial.printf("Motor addr: 0x%02X\n", addr);
  Serial.printf("Write format: reg0x00=dir, reg0x01=speed8\n");
  Serial.printf("Command: dir=%u speed8=%u for %u ms\n", dir, pwm, (unsigned)BOOT_TEST_DURATION_MS);

  char l1[32];
  char l2[32];
  char l3[48];
  snprintf(l1, sizeof(l1), "ADDR 0x%02X", addr);
  snprintf(l2, sizeof(l2), "WRITE: REG00/REG01");
  snprintf(l3, sizeof(l3), "DIR=%u  SPD8=%u  (5s)", dir, pwm);
  showBootMotorTestScreen(l1, l2, l3);

  // Start motor
  uint8_t err_dir = 0;
  uint8_t err_spd = 0;
  hbridgeWriteDirectionSpeed8(addr, dir, pwm, &err_dir, &err_spd);
  Serial.printf("Start write err_dir=%u err_spd=%u\n", err_dir, err_spd);

  const unsigned long start_ms = millis();
  while (millis() - start_ms < (unsigned long)BOOT_TEST_DURATION_MS) {
    readWheelEncoders();
    showBootMotorTestScreen(l1, l2, l3);
    delay(100);
  }

  // Stop motor using the same format
  hbridgeWriteDirectionSpeed8(addr, 0, 0, &err_dir, &err_spd);
  Serial.printf("Stop write err_dir=%u err_spd=%u\n", err_dir, err_spd);
  Serial.println("=== BOOT MOTOR SPIN TEST DONE ===\n");

  showBootMotorTestScreen("STOPPED", "Boot test complete", "(interactive control disabled)");
}

void setup() {
  // Initialize serial immediately
  Serial.begin(115200);
  delay(100);

  // If the device is rebooting unexpectedly, this helps identify the cause.
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("RESET reason=%d\n", (int)rr);
  }

  Serial.printf("FW: %s\n", FW_VERSION);

  // Filesystem for post-run logging
  // Non-destructive mount: do NOT auto-format on failure (would erase logs).
  spiffs_ok = SPIFFS.begin(false);
#if !SERIAL_QUIET_MODE
  Serial.printf("SPIFFS: %s\n", spiffs_ok ? "OK" : "FAIL");
  printSpiffsUsageToSerial("SPIFFS");
  Serial.println("TEST 1: Serial working");
#endif
 
  // Initialize M5Dial
  M5.begin();
  M5.Display.setRotation(1);
  M5.Display.setTextDatum(middle_center);

  // External buttons on black digital connector (PORT.B)
  extBlueBtnPin = M5.getPin(m5::pin_name_t::port_b_in);
  extRedBtnPin = M5.getPin(m5::pin_name_t::port_b_out);
  if (extBlueBtnPin >= 0) pinMode(extBlueBtnPin, INPUT_PULLUP);
  if (extRedBtnPin >= 0) pinMode(extRedBtnPin, INPUT_PULLUP);
#if !SERIAL_QUIET_MODE
  Serial.printf("EXT buttons: blue(PORT.B IN) GPIO=%d  red(PORT.B OUT) GPIO=%d\n", extBlueBtnPin, extRedBtnPin);
#endif

  // Persistent settings
  prefs_ok = prefs.begin("bv_ev", false);
  if (prefs_ok) {
    loadPersistedSettings();
#if !SERIAL_QUIET_MODE
    Serial.printf("Loaded settings: ppm=%.1f ppmL=%.1f ppmR=%.1f mtr_bias=%d\n",
                  (double)pulsesPerMeter,
                  (double)pulsesPerMeterL,
                  (double)pulsesPerMeterR,
                  (int)motor_bias_pwm);
#endif
  } else {
#if !SERIAL_QUIET_MODE
    Serial.println("Preferences begin failed; settings will not persist");
#endif
  }

  // Create a full-screen sprite for flicker-free UI updates.
  ui.setColorDepth(16);
  ui.createSprite(M5.Display.width(), M5.Display.height());
  ui.setTextDatum(middle_center);

  // Splash screen to give time to set the car down before calibration.
  showBVSplashScreen();
  delay(BOOT_SPLASH_MS);

#if RUN_CLOUD_UPLOAD_ENABLE
  // Best-effort time sync (fast timeout). If WiFi isn't configured/available,
  // the RUN screen will show UTC as (unset) and logs will omit timestamps.
  if (strlen(WIFI_SSID) > 0 && strlen(WIFI_PASSWORD) > 0) {
    WiFi.mode(WIFI_STA);
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      const unsigned long t0 = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 1500) {
        delay(25);
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      trySyncTimeWithNtpQuick();
    }
  }
#endif
 
#if !SERIAL_QUIET_MODE
  Serial.println("TEST 2: M5.begin() done");
#endif
 
  // Setup I2C (use external Port A pins via M5Unified mapping)
#if USE_M5_EX_I2C_PINS
  int ex_sda = M5.getPin(m5::pin_name_t::ex_i2c_sda);
  int ex_scl = M5.getPin(m5::pin_name_t::ex_i2c_scl);
  if (ex_sda >= 0 && ex_scl >= 0) {
    Wire.begin(ex_sda, ex_scl);
  } else {
    // Defensive fallback: if EX pin mapping isn't available on this board/firmware,
    // use the default Wire pins so we don't lose the I2C bus entirely.
    Wire.begin();
  }
  Wire.setTimeOut(20);
  Wire.setClock(I2C_CLOCK_HZ);
#if !SERIAL_QUIET_MODE
  if (ex_sda >= 0 && ex_scl >= 0) {
    Serial.printf("TEST 3: I2C initialized (EX) SDA=%d SCL=%d @%uHz\n", ex_sda, ex_scl, (unsigned)I2C_CLOCK_HZ);
  } else {
    Serial.printf("TEST 3: I2C initialized (fallback default pins) ex_sda=%d ex_scl=%d @%uHz\n", ex_sda, ex_scl, (unsigned)I2C_CLOCK_HZ);
  }
#endif
#else
  Wire.begin();
  Wire.setTimeOut(20);
  Wire.setClock(I2C_CLOCK_HZ);
#if !SERIAL_QUIET_MODE
  Serial.printf("TEST 3: I2C initialized (default pins) @%uHz\n", (unsigned)I2C_CLOCK_HZ);
#endif
#endif

  // Init external Unit IMU (MPU6886 @0x68)
  imu_present = imu6886Init();
#if !SERIAL_QUIET_MODE
  Serial.printf("IMU (MPU6886) init: %s WHOAMI=0x%02X\n", imu_present ? "OK" : "FAIL", (unsigned)imu_whoami);
#endif
 
  // Setup encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
 
#if !SERIAL_QUIET_MODE
  Serial.println("TEST 4: Screen dial encoder pins configured");
  Serial.printf("  Pin A: GPIO %d\n", ENCODER_PIN_A);
  Serial.printf("  Pin B: GPIO %d\n", ENCODER_PIN_B);
 
  Serial.println("TEST 5: I2C addresses");
  Serial.printf("  Left Wheel Encoder: 0x%02X\n", ENCODER_L_ADDR);
  Serial.printf("  Right Wheel Encoder: 0x%02X\n", ENCODER_R_ADDR);
  Serial.printf("  Left Motor Driver: 0x%02X\n", MOTOR_L_ADDR);
  Serial.printf("  Right Motor Driver: 0x%02X\n", MOTOR_R_ADDR);
 
  Serial.println("Setup complete\\n");
  Serial.println("Use screen dial encoder to adjust speed");
  Serial.println("Speed range: 0-255 (0 = stopped)\\n");
#endif
 
  boot_ms = millis();

  // Safety: ensure motors are stopped on boot.
  motor_enabled = 0;
  motor_command = 0;
  stopAllMotors();

  // Initial motor presence + readback.
  updateMotorDiagnostics();
#if !SERIAL_QUIET_MODE
  Serial.printf("HBridge presence: 0x%02X=%s  0x%02X=%s\n", MOTOR_L_ADDR, motorL_present ? "OK" : "--", MOTOR_R_ADDR, motorR_present ? "OK" : "--");
#endif

#if RUN_BOOT_MOTOR_SPIN_TEST
  // Run a single known-format motor command at boot, then stop.
  // This intentionally blocks for ~5 seconds.
  runBootMotorSpinTest();
#endif

#if RUN_I2C_SCAN_ON_BOOT
  // Scan I2C bus to see what devices are present
  scanI2CBus();
#endif

#if RUN_MOTOR_DIAGNOSTIC_ON_BOOT
  // WARNING: this sends non-zero commands.
  testMotorFormats();
#endif
 
  renderCurrentScreen();
}

void loop() {
  M5.update();

  // Allow post-run interaction over USB serial (e.g., dump saved log).
  handleSerialCommands();

  const unsigned long now = millis();

#if RUN_CLOUD_UPLOAD_ENABLE
#if RUN_CLOUD_UPLOAD_ON_STOP
#if RUN_CLOUD_UPLOAD_DEFERRED
  // Deferred upload worker: retries a few times after a run ends.
  if (!runActive && cloudUploadPending && (long)(now - cloudUploadNextAttemptMs) >= 0) {
    cloudUploadAttempts++;
    Serial.printf("CloudUpload: attempt %u/%u\n", (unsigned)cloudUploadAttempts, (unsigned)RUN_CLOUD_UPLOAD_MAX_RETRIES);
    cloudUploadLastOk = cloudUploadLastRunCsv();
    if (cloudUploadLastOk) {
      cloudUploadPending = false;
      Serial.println("CloudUpload: OK");
    } else if (cloudUploadAttempts >= RUN_CLOUD_UPLOAD_MAX_RETRIES) {
      cloudUploadPending = false;
      Serial.println("CloudUpload: FAILED (giving up)");
    } else {
      cloudUploadNextAttemptMs = now + (unsigned long)RUN_CLOUD_UPLOAD_RETRY_MS;
      Serial.printf("CloudUpload: retry in %u ms\n", (unsigned)RUN_CLOUD_UPLOAD_RETRY_MS);
    }
  }
#endif
#endif
#endif

  // Auto-save calibration settings after the dial stops.
  if (settings_dirty && (now - settings_dirty_ms) >= (unsigned long)SETTINGS_AUTOSAVE_IDLE_MS) {
    if (currentScreen == SCREEN_SET_ENC_CAL || currentScreen == SCREEN_SET_CTRL_MODE) {
      savePersistedSettings();
      settings_dirty = false;
    }
  }

  // External buttons (PORT.B): red starts RUN.
  const bool blueClicked = extButtonWasClicked(extBlueBtnPin, &extBlueState);
  const bool redClicked = extButtonWasClicked(extRedBtnPin, &extRedState);

  // Track raw levels and click counts for debugging wiring/port mapping issues.
  dbg_blueLevel = (extBlueBtnPin >= 0) ? (digitalRead(extBlueBtnPin) ? 1 : 0) : -1;
  dbg_redLevel = (extRedBtnPin >= 0) ? (digitalRead(extRedBtnPin) ? 1 : 0) : -1;
  if (blueClicked) dbg_blueClicks++;
  if (redClicked) dbg_redClicks++;

  // Competition rule: ONLY external RED starts the car, and only from the RUN screen.
  if (redClicked && currentScreen == SCREEN_RUN && !runActive) {
    dbg_lastStartAttemptMs = (uint32_t)millis();
    strncpy(dbg_lastStartPath, "EXT_RED", sizeof(dbg_lastStartPath));
    dbg_lastStartPath[sizeof(dbg_lastStartPath) - 1] = 0;
#if RUN_START_DEBUG_SERIAL
    Serial.printf("START_CLICK path=%s screen=%d runActive=%u\n", dbg_lastStartPath, (int)currentScreen, runActive ? 1u : 0u);
#endif
    {
      char line[96];
      snprintf(line, sizeof(line), "START_CLICK path=%s scr=%d runActive=%u", dbg_lastStartPath, (int)currentScreen, runActive ? 1u : 0u);
      startDebugLogLine(line);
    }
    startRun();
    // startRun() blocks briefly (IMU quick-cal). Don't continue this loop()
    // iteration with a stale 'now' value; let the next iteration recompute time.
    return;
  }

  // Optional: external BLUE stops the run (but never starts it).
  if (blueClicked && currentScreen == SCREEN_RUN && runActive) {
    lastRunStopReason = "USER_STOP";
    stopRun();
    return;
  }

  // Always keep sensor polling active (I2C)
  readWheelEncoders();

  static unsigned long lastMotorDiag = 0;
  if (now - lastMotorDiag > 500) {
    lastMotorDiag = now;
    updateMotorDiagnostics();
  }

  // Read IMU attitude periodically (degrees)
  static unsigned long lastImuMs = 0;
  static unsigned long lastYawMs = 0;
  if (now - lastImuMs >= (unsigned long)control_period_ms) {
    lastImuMs = now;

    float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
    float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;
    float temp_c = 0.0f;
    const uint32_t t0us = (uint32_t)micros();
    const bool ok = imu_present && imu6886Read(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps, &temp_c);
    const uint32_t t1us = (uint32_t)micros();
    runDbg_lastImuReadUs = (t1us >= t0us) ? (t1us - t0us) : 0;
    imu_ok = ok;
    imu_gz_raw_dps = ok ? gz_dps : 0.0f;
    imu_gz_dps = ok ? (gz_dps - imu_gz_bias_dps) : 0.0f;

    // While stopped (NOT during a run), refine gyro bias using a stability-gated window.
    // This is a "start line" improvement: you can sit staged for seconds/minutes and let
    // the bias converge, without risking in-run learning.
    static unsigned long stillWinStartMs = 0;
    static double stillSum = 0.0;
    static double stillSumSq = 0.0;
    static float stillMin = 1e9f;
    static float stillMax = -1e9f;
    static int stillN = 0;

    const bool motorsStopped = (motor_command == 0) && (motor_enabled == 0);
    const bool accelStill = ok && fabsf(gx_dps) < 5.0f && fabsf(gy_dps) < 5.0f;
    if (!runActive && motorsStopped && accelStill && ok) {
      imu_bias_refining = true;
      if (stillWinStartMs == 0) {
        stillWinStartMs = now;
        stillSum = 0.0;
        stillSumSq = 0.0;
        stillMin = 1e9f;
        stillMax = -1e9f;
        stillN = 0;
      }

      // Accumulate raw gz so the mean is the bias.
      stillSum += (double)gz_dps;
      stillSumSq += (double)gz_dps * (double)gz_dps;
      stillN += 1;
      if (gz_dps < stillMin) stillMin = gz_dps;
      if (gz_dps > stillMax) stillMax = gz_dps;

      const unsigned long winMs = now - stillWinStartMs;
      if (winMs >= 1200 && stillN >= 40) {
        const double mean = stillSum / (double)stillN;
        const double var = (stillSumSq / (double)stillN) - (mean * mean);
        const float stddev = (var > 0.0) ? (float)sqrt(var) : 0.0f;
        const float span = (stillMax > stillMin) ? (stillMax - stillMin) : 0.0f;

        // Only update if stable. Do NOT require small magnitude; large-but-stable offsets are valid.
        if (stddev <= 0.80f && span <= 6.0f) {
          const float tau_s = 6.0f;
          const float dt_s = (float)winMs / 1000.0f;
          const float alpha = (dt_s > 0.0f) ? (dt_s / (tau_s + dt_s)) : 0.0f;
          imu_gz_bias_dps = (1.0f - alpha) * imu_gz_bias_dps + alpha * (float)mean;

          if (imu_gz_bias_dps > IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = IMU_GZ_BIAS_CLAMP_DPS;
          if (imu_gz_bias_dps < -IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = -IMU_GZ_BIAS_CLAMP_DPS;

          persistImuGyroBiasIfNeeded();
        }

        // Reset window for the next update.
        stillWinStartMs = 0;
      }

      // Low-pass the residual corrected error for visibility/debug.
      const float gz_err = gz_dps - imu_gz_bias_dps;
      imu_gz_still_err_dps = (0.98f * imu_gz_still_err_dps) + (0.02f * gz_err);
      imu_gz_dps = gz_dps - imu_gz_bias_dps;
    } else {
      imu_bias_refining = false;
      stillWinStartMs = 0;
    }
  // Start-line IMU bias calibration indicator.
  // This runs only while stopped (never learns during a run).
  if (!runActive && imu_present) {
    ui.setTextSize(1);
    if (imu_bias_refining) {
      ui.setTextColor(TFT_YELLOW);
      ui.drawString("IMU BIAS CAL...", 120, 64);
    } else {
      ui.setTextColor(TFT_DARKGREY);
      ui.drawString("IMU bias ready", 120, 64);
    }
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString(String("gzBias ") + String(imu_gz_bias_dps, 2) + " dps", 120, 74);
  }

    if (ok) {
      // Tilt estimation from accelerometer (good for a quick hardware test)
      const float roll_deg = atan2f(ay_g, az_g) * (180.0f / (float)M_PI);
      const float pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * (180.0f / (float)M_PI);
      imu_roll = roll_deg;
      imu_pitch = pitch_deg;

      // Integrate gyro Z (deg/s) into a yaw-ish angle (will drift, but shows motion)
      if (lastYawMs == 0) lastYawMs = now;
      const float dt_s = (now - lastYawMs) / 1000.0f;
      lastYawMs = now;
      if (dt_s > 0.0f && dt_s < 0.5f) {
        imu_yaw += imu_gz_dps * dt_s;
        // Wrap to [-180, 180]
        while (imu_yaw > 180.0f) imu_yaw -= 360.0f;
        while (imu_yaw < -180.0f) imu_yaw += 360.0f;
      }
    }
  }

  // Read dial steps once per loop and route by screen
  const int dialSteps = readDialDetentSteps();

  // Run screen no longer requires a long-press to exit.

  // Screen-specific input + motor behavior
  if (currentScreen == SCREEN_MAIN_MENU) {
    applyMenuDialSteps(dialSteps);
    const bool clicked = ((now - boot_ms) > 500) && M5.BtnA.wasClicked();
    if (clicked && suppressNextClick) {
      suppressNextClick = false;
    } else if (clicked) {
      switch (selectedMenuItem) {
        case 0:
          // Enter Run screen; require explicit start (red button / BtnA) to begin motion.
          enterScreen(SCREEN_RUN);
          break;
        case 1:
          enterScreen(SCREEN_SET_DISTANCE);
          break;
        case 2:
          enterScreen(SCREEN_SET_TIME);
          break;
        case 3:
          enterScreen(SCREEN_SET_CAN_DISTANCE);
          break;
        case 4:
          enterScreen(SCREEN_BIAS_TEST);
          break;
        case 5:
          enterScreen(SCREEN_SET_ENC_CAL);
          break;
        case 6:
          enterScreen(SCREEN_SET_CTRL_MODE);
          break;
        case 7:
          enterScreen(SCREEN_BFS_NAV);
          break;
        case 8:
          enterScreen(SCREEN_HW_TEST);
          break;
      }
    }

    // In menu: motors always off
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_BFS_NAV) {
    // BFS Navigation setup screen
    // Dial selects: 0=start node, 1=goal node
    static int bfs_setup_step = 0;
    
    // Use centralized dial handler for node selection
    applyBfsDialSteps(dialSteps, bfs_setup_step);
    
    // Button toggles between start and goal selection, long-press to begin navigation
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      bfs_setup_step = 1 - bfs_setup_step;  // Toggle between 0 and 1
    }
    
    static bool bfsNavLongPressHandled = false;
    if (!M5.BtnA.isPressed()) bfsNavLongPressHandled = false;
    if (!bfsNavLongPressHandled && M5.BtnA.pressedFor(800)) {
      bfsNavLongPressHandled = true;
      // Compute path and start navigation
      if (bfsComputePath(bfs_start_node, bfs_goal_node)) {
        bfs_run_active = true;
        bfs_current_target = bfsGetNextTarget();
        savePersistedSettings();
        enterScreen(SCREEN_RUN);
      }
    }
    
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_SET_DISTANCE) {
    applyDistanceDialSteps(dialSteps);
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      savePersistedSettings();
      settings_dirty = false;
      enterScreen(SCREEN_MAIN_MENU);
    }
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_SET_TIME) {
    applyTimeDialSteps(dialSteps);
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      savePersistedSettings();
      settings_dirty = false;
      enterScreen(SCREEN_MAIN_MENU);
    }
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_SET_CAN_DISTANCE) {
    applyCanDistanceDialSteps(dialSteps);
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      savePersistedSettings();
      settings_dirty = false;
      enterScreen(SCREEN_MAIN_MENU);
    }
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_BIAS_TEST) {
    // Adjust motor bias with dial (even while running)
    if (dialSteps != 0) {
      int32_t next = (int32_t)motor_bias_pwm + (int32_t)dialSteps * (int32_t)MOTOR_BIAS_STEP_PWM;
      if (next > MOTOR_BIAS_MAX_PWM) next = MOTOR_BIAS_MAX_PWM;
      if (next < -MOTOR_BIAS_MAX_PWM) next = -MOTOR_BIAS_MAX_PWM;
      motor_bias_pwm = (int16_t)next;
    }
    // Button toggles running state
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      bias_test_mode = !bias_test_mode;
      if (!bias_test_mode) {
        stopAllMotors();
      }
    }
    // Hold button to exit back to menu
    static bool biasTestLongPressHandled = false;
    if (!M5.BtnA.isPressed()) biasTestLongPressHandled = false;
    if (!biasTestLongPressHandled && M5.BtnA.pressedFor(800)) {
      biasTestLongPressHandled = true;
      bias_test_mode = false;
      stopAllMotors();
      savePersistedSettings();
      settings_dirty = false;
      enterScreen(SCREEN_MAIN_MENU);
    }
    // Apply motor outputs when in test mode
    if (bias_test_mode) {
      // Run at fixed PWM with only motor_bias_pwm applied (no feedback)
      const float basePWM = 180.0f;  // Moderate speed for testing
      float leftPWMf = basePWM + (float)motor_bias_pwm;
      float rightPWMf = basePWM - (float)motor_bias_pwm;
      if (leftPWMf < 0.0f) leftPWMf = 0.0f;
      if (leftPWMf > 255.0f) leftPWMf = 255.0f;
      if (rightPWMf < 0.0f) rightPWMf = 0.0f;
      if (rightPWMf > 255.0f) rightPWMf = 255.0f;
      motor_l_direction = 1;
      motor_r_direction = 1;
      motor_l_speed = (uint8_t)(leftPWMf + 0.5f);
      motor_r_speed = (uint8_t)(rightPWMf + 0.5f);
      applyMotorOutputs();
    } else {
      stopAllMotors();
    }
  }

  else if (currentScreen == SCREEN_SET_ENC_CAL) {
    applyEncoderCalDialSteps(dialSteps);
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      encCalEditingLeft = !encCalEditingLeft;
    }
    static bool encCalLongPressHandled = false;
    if (!M5.BtnA.isPressed()) encCalLongPressHandled = false;
    if (!encCalLongPressHandled && M5.BtnA.pressedFor(800)) {
      encCalLongPressHandled = true;
      savePersistedSettings();
      settings_dirty = false;
      enterScreen(SCREEN_MAIN_MENU);
    }
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_SET_CTRL_MODE) {
    applyControlModeDialSteps(dialSteps);
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      savePersistedSettings();
      settings_dirty = false;
      enterScreen(SCREEN_MAIN_MENU);
    }
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_RUN) {
    // BtnA behavior on RUN screen:
    // - If running: STOP
    // - If stopped: EXIT to menu
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      if (runActive) {
        lastRunStopReason = "USER_STOP";
        stopRun();
        return;
      } else {
        suppressNextClick = true;
        enterScreen(SCREEN_MAIN_MENU);
      }
    }

    if (runActive) {
      const float elapsed = (now - runStartMs) / 1000.0f;
      const int32_t dL = encoderL_count - runStartEncL;
      const int32_t dR = encoderR_count - runStartEncR;
      // For distance: use signed average (prevents over-count during steering/wobble).
      // For motion detection: keep a magnitude-based measure so in-place turning still counts as motion.
      const float avgPulsesAbs = (fabsf((float)dL) + fabsf((float)dR)) * 0.5f;
      const float dLcorr = (INVERT_LEFT_ENCODER_COUNT ? -(float)dL : (float)dL);
      const float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR);
      const float sL = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
      const float sR = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
      const float pathM = 0.5f * (sL + sR);
      const float meters = fabsf(pathM);

      // Fail-safe: if encoders disappear on I2C, do not continue driving open-loop.
      static unsigned long lastEncOkMs = 0;
      static int32_t lastMotionEncL = 0;
      static int32_t lastMotionEncR = 0;
      static unsigned long lastMotionMs = 0;
      if (encoderL_found && encoderR_found) {
        lastEncOkMs = now;
      }
      // Initialize timers at run start
      if (runControlResetRequest || (now - runStartMs) < 250) {
        lastEncOkMs = now;
        lastMotionEncL = encoderL_count;
        lastMotionEncR = encoderR_count;
        lastMotionMs = now;
      }

      // Motion detection based on incremental encoder activity (works for pivots where forward distance ~= 0).
      {
        const int32_t dML = encoderL_count - lastMotionEncL;
        const int32_t dMR = encoderR_count - lastMotionEncR;
        const float motionAbs = (fabsf((float)dML) + fabsf((float)dMR)) * 0.5f;
        if (motionAbs >= (float)RUN_NO_MOTION_MIN_PULSES) {
          lastMotionEncL = encoderL_count;
          lastMotionEncR = encoderR_count;
          lastMotionMs = now;
        }
      }

      if ((now - lastEncOkMs) > (unsigned long)RUN_ENCODER_MISSING_TIMEOUT_MS) {
        // Competition preference: warn but keep running.
        // Distance stop and closed-loop behavior will degrade, but a run should still proceed.
        runWarnEncMissing = true;
      }
      if ((now - lastMotionMs) > (unsigned long)RUN_NO_MOTION_TIMEOUT_MS) {
        // Warn but keep running (user can still stop).
        runWarnNoMotion = true;
      }

      // Use the per-run latched mode (runMeta.control_mode) so the controller behavior matches
      // what was recorded in the run CSV header.
      const ControlMode cm = (ControlMode)runMeta.control_mode;

      // CAN mode latch + IMU health tracking.
      // If CAN is enabled for this run, do not silently fall back to straight-line yaw=0 mid-run
      // (that looks like a "jerk back to center" and prevents lateral deviation).
      static bool canModeLatched = false;
      static unsigned long imuLastOkMs = 0;

      // Track last successful IMU read time (guards against occasional 1-tick I2C hiccups).
      if (imuLastOkMs == 0 || runControlResetRequest || (now - runStartMs) < 250) imuLastOkMs = now;
      if (imu_ok) imuLastOkMs = now;

      const bool imuControlOk = imu_control_enabled && imu_present && (cm != CTRL_MODE_ENCODER) &&
                                ((now - imuLastOkMs) <= (unsigned long)IMU_OK_GRACE_MS);

      // Latch CAN mode once per run.
      // IMPORTANT: do NOT gate this on imu_ok; brief I2C/IMU read hiccups should not kick us back
      // to straight-line centerline control.
      if (!canModeLatched) {
        if ((canDistanceM >= CAN_ENABLE_MIN_M) && imu_control_enabled && imu_present && (cm != CTRL_MODE_ENCODER)) {
          canModeLatched = true;
        }
      }

      const bool canMode = canModeLatched;

      // Target forward rate (m/s). For time runs, bias toward staying on-schedule:
      // as we approach the end, command speed = remainingM / remainingTime.
      const float nominalTargetForwardRate = (runTimeS > 0.0f) ? (runDistanceM / runTimeS) : 0.0f;

      // Safety timeout: distance is the primary stop condition.
      // Keep a hard upper bound so we can't run away if encoders fail.
      // (Do NOT stop early just because we missed the requested time.)
      // Competition times are 10..20s; if we're still running well beyond that, something is wrong.
      // When RUN_LOCK_PWM_ENABLE is set, we are intentionally *not* trying to hit the time schedule,
      // so allow a much larger margin to ensure we can still reach the target distance.
    #if RUN_LOCK_PWM_ENABLE
      const float hardTimeoutS = (runTimeS > 0.0f) ? (runTimeS + 30.0f) : 60.0f;
    #else
      const float hardTimeoutS = (runTimeS > 0.0f) ? (runTimeS + 12.0f) : 32.0f;
    #endif
      if (elapsed > hardTimeoutS) {
        lastRunStopReason = "TIMEOUT";
        stopRun();
        return;
      }

      // Stop primarily on distance (distance dominates scoring). Time is handled by speed control.
      else {
        // Control update (every control_period_ms)
        static unsigned long lastControlMs = 0;
        static float speedIntegral = 0.0f;
        static float steerIntegral = 0.0f;
        static float yawIntegral = 0.0f;
        static float prevSteerErr = 0.0f;
        static float prevAvgPulses = 0.0f;
        static int32_t prevEncL = 0;
        static int32_t prevEncR = 0;

        // Steering correction shaping (helps reduce limit-cycle wobble)
        static float corrFiltered = 0.0f;
        static float corrOut = 0.0f;

        // HYBRID slow trim learned from encoder rate mismatch
        static float encTrimPwm = 0.0f;

        // Dead reckoning for CAN mode (and also used to stop accurately when turning)
        static float prevPathM = 0.0f;
        static float forwardM = 0.0f;
        static float lateralM = 0.0f;

        // Straight-run lateral hold target yaw offset (deg).
        static float straightTargetYawDeg = 0.0f;

        // One-shot warning latch for cases where CAN mode is requested but phased CAN is disabled.
        static bool canPhasedWarnedThisRun = false;

        enum CanPhase {
          CANP_INIT = 0,
          CANP_DRIVE_STRAIGHT0,
          CANP_TURN_TO_OFFSET,
          CANP_DRIVE_DIAG_OUT,
          CANP_TURN_TO_PARALLEL,
          CANP_DRIVE_PARALLEL,
          CANP_TURN_BACK_IN,
          CANP_DRIVE_DIAG_IN,
          CANP_TURN_TO_CENTER,
          CANP_DRIVE_FINISH,
          CANP_TURN_SETTLE,
        };
        static CanPhase canPhase = CANP_INIT;

        // Phase bookkeeping (distances are down-course, i.e., forwardM)
        static float phaseStartForwardM = 0.0f;
        static float phaseTargetForwardM = 0.0f;
        static float phaseTargetLateralM = 0.0f;
        static float phaseTargetYawDeg = 0.0f;
        static uint32_t drivePhaseStartMs = 0;

        // Turn bookkeeping
        static float turnYawStartDeg = 0.0f;
        static float turnTargetDeltaDeg = 0.0f;
        static int32_t turnEncStartL = 0;
        static int32_t turnEncStartR = 0;
        static uint32_t turnPhaseStartMs = 0;
        static uint32_t turnInTolStartMs = 0;
        static float turnPwmCmdF = 0.0f;
        static int turnLastSign = 0;
        static float turnLastDeltaFusedDeg = 0.0f;
        static uint32_t turnLastMoveMs = 0;

        // Post-turn settle bookkeeping: a short hold/pivot to remove residual yaw motion and
        // avoid a control transient when switching to the next drive phase.
        static uint32_t settlePhaseStartMs = 0;
        static uint32_t settleInTolStartMs = 0;
        static CanPhase settleNextDrivePhase = CANP_DRIVE_FINISH;
        static float settleTargetYawDeg = 0.0f;
        static float settleTargetForwardM = 0.0f;
        static float settleTargetLatM = 0.0f;

        static int startupBiasCount = 0;
        static float startupBiasSum = 0.0f;
        static float steerBias = 0.0f;

        // Reset controller state at start of a run
        if (runControlResetRequest || lastControlMs == 0 || (now - runStartMs) < 250) {
          lastControlMs = now;
          speedIntegral = 0.0f;
          steerIntegral = 0.0f;
          yawIntegral = 0.0f;
          prevSteerErr = 0.0f;
          prevAvgPulses = avgPulsesAbs;
          prevEncL = encoderL_count;
          prevEncR = encoderR_count;

          corrFiltered = 0.0f;
          corrOut = 0.0f;
          encTrimPwm = 0.0f;

          prevPathM = meters;
          forwardM = 0.0f;
          lateralM = 0.0f;
          straightTargetYawDeg = 0.0f;
          canPhase = CANP_INIT;
          phaseStartForwardM = 0.0f;
          phaseTargetForwardM = 0.0f;
          phaseTargetLateralM = 0.0f;
          phaseTargetYawDeg = 0.0f;
          drivePhaseStartMs = (uint32_t)now;
          turnYawStartDeg = 0.0f;
          turnTargetDeltaDeg = 0.0f;
          turnPwmCmdF = 0.0f;
          turnLastSign = 0;
          turnLastDeltaFusedDeg = 0.0f;
          turnLastMoveMs = 0;

          startupBiasCount = 0;
          startupBiasSum = 0.0f;
          steerBias = 0.0f;

          // Reset CAN/IMU latches for this run.
          canModeLatched = false;
          imuLastOkMs = 0;

          canPhasedWarnedThisRun = false;

          runControlResetRequest = false;
        }

        if ((now - lastControlMs) >= (unsigned long)control_period_ms) {
          const uint32_t dtMs = (uint32_t)(now - lastControlMs);
          const float dt = (float)dtMs / 1000.0f;
          lastControlMs = now;

          // Timing diagnostics: detect long control-loop stalls.
          if (dtMs > runDbg_maxCtrlDtMs) {
            runDbg_maxCtrlDtMs = dtMs;
            runDbg_maxCtrlDtAtTms = (uint32_t)(now - runStartMs);
          }
          if (dtMs > (uint32_t)control_period_ms * 2u) {
            runDbg_ctrlOverrunCount++;
          }
          if (dtMs >= 200u) {
            runDbg_ctrlStall200msCount++;
          }

          // Abort guard: sustained large yaw error implies we're veering badly.
          static unsigned long yawBadSinceMs = 0;

          // Encoder deltas and per-wheel pulse rates for this control tick (used for logging and fallback control).
          // Rates can be quantized at 100Hz (e.g., ~1300/1400 pps jumps). Smooth over a short
          // window so feedback sees a steadier estimate without adding much lag.
          const int32_t dEncL = encoderL_count - prevEncL;
          const int32_t dEncR = encoderR_count - prevEncR;
          prevEncL = encoderL_count;
          prevEncR = encoderR_count;

          // 3-sample moving window in pulse-domain.
          static int32_t dEncL_hist[3] = {0, 0, 0};
          static int32_t dEncR_hist[3] = {0, 0, 0};
          static float dt_hist[3] = {0.0f, 0.0f, 0.0f};
          static uint8_t encHistIdx = 0;
          dEncL_hist[encHistIdx] = dEncL;
          dEncR_hist[encHistIdx] = dEncR;
          dt_hist[encHistIdx] = dt;
          encHistIdx = (uint8_t)((encHistIdx + 1u) % 3u);

          float sumDt = 0.0f;
          float sumAbsL = 0.0f;
          float sumAbsR = 0.0f;
          float sumAbsPathPulses = 0.0f;
          for (int i = 0; i < 3; i++) {
            const float dti = dt_hist[i];
            if (dti > 0.0f && dti < 0.5f) {
              sumDt += dti;
              sumAbsL += fabsf((float)dEncL_hist[i]);
              sumAbsR += fabsf((float)dEncR_hist[i]);

              // Smoothed path pulses for speed estimation.
              // Use the same encoder sign corrections as distance math so "forward" is consistent,
              // then take magnitude so this works for low-speed quantized pulses.
              const float dLcorr = (INVERT_LEFT_ENCODER_COUNT ? -(float)dEncL_hist[i] : (float)dEncL_hist[i]);
              const float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dEncR_hist[i] : (float)dEncR_hist[i]);
              const float pathPulses = (dLcorr + dRcorr) * 0.5f;
              sumAbsPathPulses += fabsf(pathPulses);
            }
          }
          const float leftRate = (sumDt > 0.0f) ? (sumAbsL / sumDt) : 0.0f;
          const float rightRate = (sumDt > 0.0f) ? (sumAbsR / sumDt) : 0.0f;
          const float pathRatePps = (sumDt > 0.0f) ? (sumAbsPathPulses / sumDt) : 0.0f;

          // Gyro-integrated yaw hold (relative to run start). This is our primary straight-line steering signal.
          if (imuControlOk && dt > 0.0f && dt < 0.5f) {
            runYawHoldDeg = wrapDeg(runYawHoldDeg + (imu_gz_dps * dt));
          }

          // Update dead-reckoned progress
          const float pathM = meters;
          const float dPathM = pathM - prevPathM;
          prevPathM = pathM;
          // IMPORTANT: never snap yaw to 0 just because we missed an IMU read.
          // If yaw-hold is enabled, keep using the integrated heading estimate.
          const float yawRelDeg = (imu_control_enabled && imu_present) ? runYawHoldDeg : 0.0f;
          const float yawRelRad = yawRelDeg * ((float)M_PI / 180.0f);
          forwardM += imuControlOk ? (dPathM * cosf(yawRelRad)) : dPathM;
          lateralM += imuControlOk ? (dPathM * sinf(yawRelRad)) : 0.0f;

          // Phased CAN is only safe/useful when the IMU is online and the run distance is sane.
          const bool canPhasedActive = canMode && CAN_PHASED_NAV_ENABLE && imuControlOk && (runDistanceM >= CAN_PHASED_MIN_RUN_DISTANCE_M);
          if (canMode && CAN_PHASED_NAV_ENABLE && !canPhasedActive && !canPhasedWarnedThisRun) {
            canPhasedWarnedThisRun = true;
            char line[180];
            snprintf(line, sizeof(line),
                     "CAN_PHASED_DISABLED imuOk=%u runDist=%.2f(min=%.2f)",
                     (unsigned)(imuControlOk ? 1u : 0u),
                     (double)runDistanceM,
                     (double)CAN_PHASED_MIN_RUN_DISTANCE_M);
            startDebugLogLine(line);
          }

          // Distance stop check:
          // - In straight mode (CAN off): stop on encoder path distance (meters) so stop matches the UI.
          // - In CAN mode (with turns): stop on dead-reckoned forward progress so turns don't break distance.
          const float stopProgressM = canPhasedActive ? forwardM : meters;
          if (runDistanceM > 0.0f && stopProgressM >= runDistanceM) {
            lastRunStopReason = "DIST";
            stopRun();
            // Skip motor update when stopping
            return;
          }

          // Update live debug (shown on RUN screen)
          runDebug_forwardM = forwardM;
          runDebug_lateralM = lateralM;

          // Braking/scheduling distance metric must match the stop metric.
          // In straight mode we stop on encoder path distance (meters).
          // In CAN phased mode we stop on dead-reckoned down-course progress (forwardM).
          const float remainingM = runDistanceM - stopProgressM;
          const bool brakingActive = (runDistanceM > 0.0f) && (remainingM > 0.0f) && (remainingM < BRAKE_ZONE_M);
          float effectiveTargetForwardRate = nominalTargetForwardRate;

          // Schedule-based speed near the end: prevents the brake zone from making
          // 10m/10s (1.0 m/s) impossible.
          if (runTimeS > 0.0f) {
            const float remainingTimeS = runTimeS - elapsed;
            if (remainingTimeS > 0.20f && remainingM > 0.0f) {
              float sched = remainingM / remainingTimeS;
              if (sched < 0.0f) sched = 0.0f;
              // Keep this bounded to avoid requesting absurd speeds if we're very late.
              // High schedule boost pegs basePWM at 255 and reduces effective steering control.
              const float maxSched = (nominalTargetForwardRate > 0.0f) ? (1.20f * nominalTargetForwardRate) : 1.0f;
              if (sched > maxSched) sched = maxSched;
              effectiveTargetForwardRate = sched;
            }
          }
          float minPwmForMotion = RUN_MIN_PWM;
          if (brakingActive) {
            float factor = remainingM / BRAKE_ZONE_M;
            if (factor < BRAKE_MIN_SPEED_FACTOR) factor = BRAKE_MIN_SPEED_FACTOR;
            if (factor > 1.0f) factor = 1.0f;

            // Only slow down in the brake zone when we are on/ahead of schedule.
            // If we're behind schedule (effective target rate > nominal), preserve the schedule-based
            // speed so we don't unnecessarily add time error.
            if (runTimeS <= 0.0f || effectiveTargetForwardRate <= nominalTargetForwardRate) {
              effectiveTargetForwardRate = effectiveTargetForwardRate * factor;
              minPwmForMotion = BRAKE_MIN_PWM;
            }
          }

          // Start-slow softstart:
          // The early "kick" is real vehicle motion. To reduce it, ramp the requested
          // forward target rate *and* the minimum PWM floor so the controller doesn't
          // immediately jump to a nonzero asymmetric command.
          // Start at minimum effective PWM (~80/255 = 0.31) since motors don't move below that.
          float runStartScale = 1.0f;
          if (RUN_SOFTSTART_MS > 0 && runStartMs != 0) {
            const uint32_t tSinceStartMs = (uint32_t)(now - runStartMs);
            if (tSinceStartMs < (uint32_t)RUN_SOFTSTART_MS) {
              float a = (float)tSinceStartMs / (float)RUN_SOFTSTART_MS;
              if (a < 0.0f) a = 0.0f;
              if (a > 1.0f) a = 1.0f;
              // Smoothstep (C1 continuous) to avoid a step in acceleration/authority.
              float smoothstep = a * a * (3.0f - 2.0f * a);
              // Scale from minimum effective PWM (0.31) to full (1.0)
              const float minEffectivePwmScale = 80.0f / 255.0f;  // ~0.31
              runStartScale = minEffectivePwmScale + smoothstep * (1.0f - minEffectivePwmScale);
            }
          }
          effectiveTargetForwardRate *= runStartScale;
          minPwmForMotion *= runStartScale;

          // Actual rates
          // Use a short moving-window encoder-based estimate to reduce quantization at low speed.
          // This avoids log/controller artifacts where dPathM is 0 in a tick while the wheel-rate
          // estimate is nonzero (which creates huge speedErr spikes).
          const float cosYaw = cosf(yawRelRad);
          const float actualPathRate = (pulsesPerMeterL > 1.0f && pulsesPerMeterR > 1.0f)
                                          ? (0.5f * ((leftRate / pulsesPerMeterL) + (rightRate / pulsesPerMeterR)))
                                          : ((pulsesPerMeter > 1.0f) ? (pathRatePps / pulsesPerMeter) : 0.0f);
          const float actualForwardRate = actualPathRate * cosYaw;

          // Speed control
          // - Normal: PI in pulse-rate units
          // - Test: locked PWM to isolate heading control and reduce startup current surge
          const float actualRateForControl_mps = (fabsf(cosYaw) >= SPEED_TURN_COS_THRESHOLD) ? actualForwardRate : actualPathRate;
          float targetRatePpsForLog = 0.0f;
          float actualRatePpsForLog = 0.0f;
          float speedErrPps = 0.0f;
          float basePWMf = 0.0f;

#if RUN_SPEED_PROFILE_ENABLE
          // Distance-based S-curve profile (uses down-course distance).
          // We still compute speedErrPps for logging, but do not use it for control.
          {
            targetRatePpsForLog = (pulsesPerMeter > 1.0f) ? (effectiveTargetForwardRate * pulsesPerMeter) : 0.0f;
            actualRatePpsForLog = (pulsesPerMeter > 1.0f) ? (actualRateForControl_mps * pulsesPerMeter) : 0.0f;
            speedErrPps = targetRatePpsForLog - actualRatePpsForLog;
          }

          // Determine per-phase distance window for the S-curve.
          float distFromStartM = forwardM;
          float distToEndM = runDistanceM - forwardM;

          bool inTurnPhase = false;
          if (canPhasedActive) {
            inTurnPhase = (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                           canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER || canPhase == CANP_TURN_SETTLE);

            // For drive phases, ramp within the current phase window.
            if (!inTurnPhase) {
              distFromStartM = forwardM - phaseStartForwardM;
              distToEndM = phaseTargetForwardM - forwardM;
              if (distFromStartM < 0.0f) distFromStartM = 0.0f;
              if (distToEndM < 0.0f) distToEndM = 0.0f;
            }
          }

          if (inTurnPhase) {
            basePWMf = 0.0f;
          } else {
            const float gate = sCurveGate(distFromStartM, distToEndM, RUN_SPEED_PROFILE_ACCEL_M, RUN_SPEED_PROFILE_DECEL_M);

            // Make the profile schedule-aware so 7m/10s actually targets the requested speed.
            // effectiveTargetForwardRate already includes end-of-run scheduling/brake-zone scaling.
            float minP = RUN_SPEED_PROFILE_MIN_PWM;
            float maxP = RUN_SPEED_PROFILE_MAX_PWM;
            if (runTimeS > 0.0f && nominalTargetForwardRate > 0.05f) {
              float s = effectiveTargetForwardRate / nominalTargetForwardRate;
              // Keep conservative bounds to avoid runaway if we're late.
              if (s < 0.80f) s = 0.80f;
              if (s > 1.30f) s = 1.30f;
              maxP = minP + (maxP - minP) * s;
              if (maxP > 255.0f) maxP = 255.0f;
              if (maxP < minP) maxP = minP;
            }
            basePWMf = minP + (maxP - minP) * gate;
          }

          // Override the min PWM used later by the motor clamp logic.
          minPwmForMotion = RUN_SPEED_PROFILE_MIN_PWM;
#else

#if RUN_LOCK_PWM_ENABLE
          (void)speedIntegral;
          // Keep a meaningful error in the log for post-run analysis (still uses scheduled target rate).
          targetRatePpsForLog = (pulsesPerMeter > 1.0f) ? (effectiveTargetForwardRate * pulsesPerMeter) : 0.0f;
          actualRatePpsForLog = (pulsesPerMeter > 1.0f) ? (actualRateForControl_mps * pulsesPerMeter) : 0.0f;
          speedErrPps = targetRatePpsForLog - actualRatePpsForLog;
          basePWMf = RUN_LOCK_PWM_VALUE;
#else
          targetRatePpsForLog = (pulsesPerMeter > 1.0f) ? (effectiveTargetForwardRate * pulsesPerMeter) : 0.0f;
          actualRatePpsForLog = (pulsesPerMeter > 1.0f) ? (actualRateForControl_mps * pulsesPerMeter) : 0.0f;
          speedErrPps = targetRatePpsForLog - actualRatePpsForLog;

          // Saturation-aware anti-windup: if we'd be pegged at 0/255 in the direction of the error,
          // don't keep integrating (it just makes recovery slower).
          float nextIntegral = speedIntegral + speedErrPps * dt;
          if (nextIntegral > 20000.0f) nextIntegral = 20000.0f;
          if (nextIntegral < -20000.0f) nextIntegral = -20000.0f;
          basePWMf = SPEED_INITIAL_PWM + (SPEED_KP * speedErrPps) + (SPEED_KI * nextIntegral);
          if ((basePWMf >= 255.0f && speedErrPps > 0.0f) || (basePWMf <= 0.0f && speedErrPps < 0.0f)) {
            // hold integral
          } else {
            speedIntegral = nextIntegral;
            basePWMf = SPEED_INITIAL_PWM + (SPEED_KP * speedErrPps) + (SPEED_KI * speedIntegral);
          }
#endif

#endif  // RUN_SPEED_PROFILE_ENABLE

          // Soft-start ramp: apply the same scale to the final base PWM as well.
          // This ramps SPEED_INITIAL_PWM / speed-profile floor from 0 upward.
          if (runStartScale < 1.0f) {
            basePWMf *= runStartScale;
          }

          // Optional start boost / assist (disabled in the proven v72 behavior).
#if RUN_SPEED_PROFILE_START_BOOST_MS > 0
          // Apply AFTER softstart so the boost isn't scaled down to near-zero.
          if ((now - runStartMs) < (unsigned long)RUN_SPEED_PROFILE_START_BOOST_MS) {
            if (basePWMf < RUN_SPEED_PROFILE_START_BOOST_PWM) basePWMf = RUN_SPEED_PROFILE_START_BOOST_PWM;
          }
#endif

#if RUN_START_ASSIST_ENABLE
          if ((now - runStartMs) < (unsigned long)RUN_START_ASSIST_MS && avgPulsesAbs < (float)RUN_START_ASSIST_MAX_PULSES) {
            if (basePWMf < RUN_START_ASSIST_PWM) basePWMf = RUN_START_ASSIST_PWM;
          }
#endif

          // Phased CAN: kick at the beginning of each DRIVE segment to avoid post-turn stalling.
          // (This is independent of run-start assist, which only applies at the very beginning of the run.)
          if (canPhasedActive && CAN_PHASED_DRIVE_START_ASSIST_MS > 0) {
            const bool canTurnPhaseNow = (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                                          canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER ||
                                          canPhase == CANP_TURN_SETTLE);
            if (!canTurnPhaseNow && drivePhaseStartMs != 0) {
              const uint32_t tMs = (uint32_t)now - drivePhaseStartMs;
              if (tMs < (uint32_t)CAN_PHASED_DRIVE_START_ASSIST_MS) {
                // Avoid an abrupt PWM step at phase transitions (this can excite a one-sided yaw "kick"
                // and reintroduce the same gyro ringing we fixed in straight runs).
                const float a = (float)tMs / (float)CAN_PHASED_DRIVE_START_ASSIST_MS;
                const float assistPwm = CAN_PHASED_DRIVE_START_ASSIST_PWM * smoothstep01(a);
                if (basePWMf < assistPwm) basePWMf = assistPwm;
              }
            }
          }

          // Clamp base PWM to driver limits before steering is applied.
          if (basePWMf < 0.0f) basePWMf = 0.0f;
          if (basePWMf > 255.0f) basePWMf = 255.0f;

          if (brakingActive && basePWMf > BRAKE_MAX_PWM) basePWMf = BRAKE_MAX_PWM;

          // If we're trying to move, avoid stall-prone low PWM.
          if (effectiveTargetForwardRate > 0.0f) {
            if (basePWMf > 0.0f && basePWMf < minPwmForMotion) basePWMf = minPwmForMotion;
          }

          float corrRaw = 0.0f;
          float targetYawDeg = 0.0f;
          float yawErrDegForLog = 0.0f;

#if STRAIGHT_LATERAL_HOLD_ENABLE
          // Straight runs: lateral position hold outer loop.
          // Generates a bounded target yaw offset from lateral error and smoothly updates it.
          if (!canPhasedActive) {
            float desiredYawDeg = 0.0f;
            if (imuControlOk && forwardM >= STRAIGHT_LATERAL_HOLD_MIN_FORWARD_M) {
              // Target centerline: target lateral = 0.
              float latErrM = -lateralM;
              if (fabsf(latErrM) < STRAIGHT_LATERAL_HOLD_DEADBAND_M) latErrM = 0.0f;
              desiredYawDeg = STRAIGHT_LATERAL_HOLD_KP_DEG_PER_M * latErrM;
              if (desiredYawDeg > STRAIGHT_LATERAL_HOLD_MAX_YAW_DEG) desiredYawDeg = STRAIGHT_LATERAL_HOLD_MAX_YAW_DEG;
              if (desiredYawDeg < -STRAIGHT_LATERAL_HOLD_MAX_YAW_DEG) desiredYawDeg = -STRAIGHT_LATERAL_HOLD_MAX_YAW_DEG;
            }

            const float prevTgt = straightTargetYawDeg;
            float nextTgt = desiredYawDeg;
            if (dt > 0.0f && STRAIGHT_LATERAL_HOLD_TAU_S > 0.0f) {
              const float alpha = dt / (STRAIGHT_LATERAL_HOLD_TAU_S + dt);
              nextTgt = prevTgt + alpha * (desiredYawDeg - prevTgt);
            }
            if (dt > 0.0f && STRAIGHT_LATERAL_HOLD_SLEW_DPS > 0.0f) {
              const float maxStep = STRAIGHT_LATERAL_HOLD_SLEW_DPS * dt;
              float delta = nextTgt - prevTgt;
              if (delta > maxStep) delta = maxStep;
              if (delta < -maxStep) delta = -maxStep;
              nextTgt = prevTgt + delta;
            }
            straightTargetYawDeg = nextTgt;

            targetYawDeg = straightTargetYawDeg;
            runDebug_targetYawDeg = targetYawDeg;
          }
#endif

          // Extra debug fields for post-run analysis (especially for phased CAN turns).
          uint8_t canPhaseForLog = 255;
          float turnTargetDegForLog = NAN;
          float turnImuDegForLog = NAN;
          float turnEncDegForLog = NAN;
          float turnFusedDegForLog = NAN;

          // Phased CAN navigation (stop/turn/drive) using fixed geometry.
          // Uses down-course distance (forwardM) and lateral drift (lateralM) from dead-reckoning.
          // Only active when CAN DIST is enabled (canDistanceM > 0) and IMU control is OK.
          if (canPhasedActive) {
            canPhaseForLog = (uint8_t)canPhase;
            runDebug_canPhase = (uint8_t)canPhase;

            // Convert Inside Can Distance (between inside edges) to a target lateral offset from centerline.
            // Outside can inside edge is fixed at +1.0m. If Inside Can Distance = D, midpoint is 1.0 - D/2.
            const float canDistM_raw = canDistanceM;
            float canDistM = canDistM_raw;
            if (canDistM < 0.0f) canDistM = 0.0f;
            if (canDistM > 1.10f) canDistM = 1.10f;
            float offsetM = CAN_BOUNDARY_M - 0.5f * canDistM;
            offsetM -= CAN_GAP_MIDLINE_INSET_M;
            if (offsetM < 0.05f) offsetM = 0.05f;
            if (offsetM > CAN_MAX_LATERAL_M) offsetM = CAN_MAX_LATERAL_M;

            // Bonus Line is halfway to the target distance.
            const float halfM = 0.5f * runDistanceM;
            const float parallelEntryM = (halfM > CAN_PHASED_ENTRY_BEFORE_HALF_M) ? (halfM - CAN_PHASED_ENTRY_BEFORE_HALF_M) : 0.0f;
            float parallelExitM = halfM + CAN_PHASED_EXIT_AFTER_HALF_M;
            if (parallelExitM > runDistanceM) parallelExitM = runDistanceM;

            // Compute the diagonal angle needed to reach offsetM by parallelEntryM.
            // We try to keep the diagonal shallow, but if we need more angle (short entry), we increase it.
            float thetaDeg = CAN_PHASED_NOMINAL_ANGLE_DEG;
            if (thetaDeg < 1.0f) thetaDeg = 1.0f;
            if (thetaDeg > CAN_PHASED_MAX_ANGLE_DEG) thetaDeg = CAN_PHASED_MAX_ANGLE_DEG;

            // If we reserve some straight distance at the start, what's the remaining down-course length for the diagonal?
            float availableDiagDownM = parallelEntryM - CAN_PHASED_MIN_STRAIGHT0_M;
            if (availableDiagDownM < 0.01f) availableDiagDownM = parallelEntryM;
            if (availableDiagDownM < 0.01f) availableDiagDownM = 0.01f;

            // Minimum angle needed so that offset = diagDown * tan(theta) by the time we hit parallelEntryM.
            const float thetaMinNeededDeg = atan2f(offsetM, availableDiagDownM) * (180.0f / (float)M_PI);
            if (thetaDeg < thetaMinNeededDeg) thetaDeg = thetaMinNeededDeg;
            if (thetaDeg > CAN_PHASED_MAX_ANGLE_DEG) thetaDeg = CAN_PHASED_MAX_ANGLE_DEG;

            const float thetaRad = thetaDeg * ((float)M_PI / 180.0f);
            const float sinT = sinf(thetaRad);
            const float cosT = cosf(thetaRad);
            const float cotT = (sinT > 0.01f) ? (cosT / sinT) : 100.0f;
            const float diagDownM = offsetM * cotT;

            // Drive straight until the diagonal begins.
            const float straight0M = (parallelEntryM > diagDownM) ? (parallelEntryM - diagDownM) : 0.0f;

            // Helpers to begin phases
            auto beginDrivePhase = [&](CanPhase next, float targetYaw, float targetForwardAbs, float targetLatAbs) {
              const uint8_t prev = (uint8_t)canPhase;
              canPhase = next;
              phaseStartForwardM = forwardM;
              phaseTargetForwardM = targetForwardAbs;
              phaseTargetLateralM = targetLatAbs;
              phaseTargetYawDeg = targetYaw;
              drivePhaseStartMs = (uint32_t)now;

              // Reset control state so we don't carry turn transients into the drive controller.
              speedIntegral = 0.0f;
              steerIntegral = 0.0f;
              yawIntegral = 0.0f;
              prevSteerErr = 0.0f;
              corrFiltered = 0.0f;
              corrOut = 0.0f;

              char line[220];
              snprintf(line, sizeof(line),
                       "CAN_DRIVE %u->%u tgtYaw=%.1f fwd=%.2f->%.2f lat=%.2f->%.2f th=%.1f off=%.2f diag=%.2f",
                       (unsigned)prev,
                       (unsigned)next,
                       (double)targetYaw,
                       (double)forwardM,
                       (double)targetForwardAbs,
                       (double)lateralM,
                       (double)targetLatAbs,
                       (double)thetaDeg,
                       (double)offsetM,
                       (double)diagDownM);
              startDebugLogLine(line);
            };
            auto beginTurnPhase = [&](CanPhase next, float targetYawAbsDeg) {
              const uint8_t prev = (uint8_t)canPhase;
              canPhase = next;
              turnYawStartDeg = runYawHoldDeg;
              const float deltaYawDeg = wrapDeg(targetYawAbsDeg - turnYawStartDeg) * (float)CAN_PHASED_YAW_SIGN;
              turnTargetDeltaDeg = deltaYawDeg;
              turnEncStartL = encoderL_count;
              turnEncStartR = encoderR_count;
              turnPhaseStartMs = (uint32_t)now;
              turnInTolStartMs = 0;
              turnPwmCmdF = CAN_PHASED_TURN_PWM_START;
              turnLastSign = 0;
              turnLastDeltaFusedDeg = 0.0f;
              turnLastMoveMs = (uint32_t)now;
              setMotorsStop();

              // Reset control state at the beginning of a pivot.
              speedIntegral = 0.0f;
              steerIntegral = 0.0f;
              yawIntegral = 0.0f;
              prevSteerErr = 0.0f;
              corrFiltered = 0.0f;
              corrOut = 0.0f;

              char line[220];
              snprintf(line, sizeof(line),
                       "CAN_TURN %u->%u tgtYaw=%.1f dYaw=%.1f yaw=%.1f fwd=%.2f lat=%.2f th=%.1f off=%.2f",
                       (unsigned)prev,
                       (unsigned)next,
                       (double)targetYawAbsDeg,
                       (double)deltaYawDeg,
                       (double)runYawHoldDeg,
                       (double)forwardM,
                       (double)lateralM,
                       (double)thetaDeg,
                       (double)offsetM);
              startDebugLogLine(line);
            };

            if (canPhase == CANP_INIT) {
#if CAN_PHASED_START_WITH_PIVOT
              // Start by pivoting immediately to the diagonal heading (saves time vs drive+stop+turn).
              beginTurnPhase(CANP_TURN_TO_OFFSET, +thetaDeg);
#else
              // Start by driving straight a short distance so we have good momentum before the first stop/turn.
              beginDrivePhase(CANP_DRIVE_STRAIGHT0, 0.0f, straight0M, 0.0f);
#endif
            }

            const bool canTurnPhase = (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                                       canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER || canPhase == CANP_TURN_SETTLE);

            // Default: during drive phases, hold a fixed yaw setpoint.
            // Additionally, bias yaw based on lateral error so we actively converge to the desired
            // lateral offset (and return to center cleanly).
            targetYawDeg = phaseTargetYawDeg;
            if (!canTurnPhase) {
#if CAN_PHASED_LAT_YAW_ENABLE
              {
                const float latErrM = phaseTargetLateralM - lateralM;
                float yawBiasDeg = CAN_PHASED_LAT_TO_YAW_KP_DEG_PER_M * latErrM;
                if (yawBiasDeg > CAN_PHASED_LAT_TO_YAW_MAX_DEG) yawBiasDeg = CAN_PHASED_LAT_TO_YAW_MAX_DEG;
                if (yawBiasDeg < -CAN_PHASED_LAT_TO_YAW_MAX_DEG) yawBiasDeg = -CAN_PHASED_LAT_TO_YAW_MAX_DEG;

                // Key anti-ring fix:
                // When a new DRIVE segment begins, latErr can jump, which would step targetYawDeg
                // (especially when yawBias saturates). Ramp the yaw bias in briefly so the gyro
                // loop doesn't do a one-sided "kick" and ring.
                if (CAN_PHASED_DRIVE_STEER_RAMP_MS > 0 && drivePhaseStartMs != 0) {
                  const uint32_t tMs = (uint32_t)now - drivePhaseStartMs;
                  if (tMs < (uint32_t)CAN_PHASED_DRIVE_STEER_RAMP_MS) {
                    float a = (float)tMs / (float)CAN_PHASED_DRIVE_STEER_RAMP_MS;
                    if (a < 0.0f) a = 0.0f;
                    if (a > 1.0f) a = 1.0f;
                    const float biasScale = a * a * (3.0f - 2.0f * a);
                    yawBiasDeg *= biasScale;
                  }
                }

                targetYawDeg = wrapDeg(targetYawDeg + yawBiasDeg);
              }
#endif
            }
            runDebug_targetYawDeg = targetYawDeg;

            // Determine completion of current phase
            const float fwdInPhase = forwardM - phaseStartForwardM;
            const float fwdToTarget = phaseTargetForwardM - forwardM;
            const float latErr = phaseTargetLateralM - lateralM;

            // TURN phases: pivot in place until IMU reports the delta angle.
            if (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL || canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER) {
              // IMU-based delta yaw (deg)
              const float deltaImuDeg = wrapDeg(runYawHoldDeg - turnYawStartDeg) * (float)CAN_PHASED_YAW_SIGN;

              // Encoder-based delta yaw (deg): theta = (sR - sL) / W
              // Use the same encoder sign corrections as distance math so forward is positive.
              float deltaEncDeg = deltaImuDeg;
              bool encOk = encoderL_found && encoderR_found && (pulsesPerMeterL > 1.0f) && (pulsesPerMeterR > 1.0f);
              if (encOk) {
                const int32_t dL = encoderL_count - turnEncStartL;
                const int32_t dR = encoderR_count - turnEncStartR;
                const float dLcorr = (INVERT_LEFT_ENCODER_COUNT ? -(float)dL : (float)dL);
                const float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR);
                const float sL = dLcorr / pulsesPerMeterL;
                const float sR = dRcorr / pulsesPerMeterR;
                const float yawEncRad = (TURN_TRACK_WIDTH_M > 0.01f) ? ((sR - sL) / TURN_TRACK_WIDTH_M) : 0.0f;
                deltaEncDeg = yawEncRad * (180.0f / (float)M_PI) * (float)CAN_PHASED_YAW_SIGN;
              }

              // Fuse when they agree; otherwise prefer IMU for control but still validate with encoders.
              const float diffDeg = fabsf(deltaImuDeg - deltaEncDeg);
              float deltaFusedDeg = deltaImuDeg;
              if (encOk && diffDeg <= TURN_IMU_ENC_DISAGREE_DEG) {
                deltaFusedDeg = 0.5f * (deltaImuDeg + deltaEncDeg);
              }

              // Detect whether we're still making progress in the turn.
              if (fabsf(deltaFusedDeg - turnLastDeltaFusedDeg) >= CAN_PHASED_TURN_NEAR_STALL_MIN_DYAW_DEG) {
                turnLastDeltaFusedDeg = deltaFusedDeg;
                turnLastMoveMs = (uint32_t)now;
              }

              turnTargetDegForLog = turnTargetDeltaDeg;
              turnImuDegForLog = deltaImuDeg;
              turnEncDegForLog = deltaEncDeg;
              turnFusedDegForLog = deltaFusedDeg;

              const float remainingImu = turnTargetDeltaDeg - deltaImuDeg;
              const float remainingEnc = turnTargetDeltaDeg - deltaEncDeg;
              const float remainingFused = turnTargetDeltaDeg - deltaFusedDeg;
              yawErrDegForLog = remainingFused;
              runDebug_turnRemainingDeg = remainingFused;

              // Track turn direction so we can avoid hard reversals right at the setpoint.
              const int currTurnSign = (remainingFused >= 0.0f) ? +1 : -1;
              if (turnLastSign == 0) turnLastSign = currTurnSign;
              const bool crossedNearTarget = (currTurnSign != turnLastSign) && (fabsf(remainingFused) <= CAN_PHASED_TURN_CROSS_STOP_DEG);

              // If we stop moving during a pivot (common when we hit a wall), abort promptly.
              // In run 75 this presented as meters/forwardM frozen, gz~0, and motors still commanded.
              {
                const uint32_t sinceYawMoveMs = (uint32_t)now - turnLastMoveMs;
                const uint32_t sinceEncMoveMs = (lastMotionMs != 0) ? ((uint32_t)now - (uint32_t)lastMotionMs) : 0u;
                const bool yawStalled = (fabsf(imu_gz_dps) <= 1.5f) && (sinceYawMoveMs >= 700u);
                const bool encStalled = (sinceEncMoveMs >= 700u);
                if (yawStalled && encStalled && fabsf(remainingFused) > TURN_IMU_TOL_DEG) {
                  setMotorsStop();
                  lastRunStopReason = "CAN_TURN_STUCK";
                  stopRun();
                  return;
                }
              }

              // If a turn phase runs too long, abort and save logs rather than hunting forever.
              if (CAN_PHASED_TURN_TIMEOUT_MS > 0) {
                const uint32_t tMs = (uint32_t)now - turnPhaseStartMs;
                if (tMs >= (uint32_t)CAN_PHASED_TURN_TIMEOUT_MS) {
                  setMotorsStop();
                  lastRunStopReason = "CAN_TURN_TIMEOUT";
                  stopRun();
                  return;
                }
              }

              // In-tolerance settle: once the *fused* estimate is close (or we just crossed the target very near it),
              // stop pivoting and wait a short hold.
              // Rationale: IMU-only remaining can lag by a couple degrees near the end; encoders can confirm
              // we are effectively at target, and fused remaining captures that.
              const bool inTol = (fabsf(remainingFused) <= TURN_IMU_TOL_DEG) || crossedNearTarget;
              if (inTol) {
                if (turnInTolStartMs == 0) turnInTolStartMs = (uint32_t)now;
                setMotorsStop();
              } else {
                turnInTolStartMs = 0;
              }

              const bool holdOk = (turnInTolStartMs != 0) &&
                                  (((uint32_t)now - turnInTolStartMs) >= (uint32_t)CAN_PHASED_TURN_IN_TOL_HOLD_MS);

              bool turnDone = false;
              if (encOk) {
                // Prefer both when they agree, but allow IMU-only completion if we settle in tolerance.
                turnDone = ((fabsf(remainingFused) <= TURN_IMU_TOL_DEG) && (fabsf(remainingEnc) <= TURN_ENC_TOL_DEG)) || holdOk;
              } else {
                // IMU-only if encoders aren't valid.
                turnDone = holdOk;
              }

              if (turnDone) {
                setMotorsStop();
                runDebug_turnRemainingDeg = NAN;

                // After a turn, enter a short settle phase to avoid a transient "jump" when switching
                // back to a drive/yaw-hold controller.
                {
                  const uint8_t prev = (uint8_t)canPhase;
                  settlePhaseStartMs = (uint32_t)now;
                  settleInTolStartMs = 0;

                  if (canPhase == CANP_TURN_TO_OFFSET) {
                    settleNextDrivePhase = CANP_DRIVE_DIAG_OUT;
                    settleTargetYawDeg = +thetaDeg;
                    settleTargetForwardM = parallelEntryM;
                    settleTargetLatM = offsetM;
                  } else if (canPhase == CANP_TURN_TO_PARALLEL) {
                    settleNextDrivePhase = CANP_DRIVE_PARALLEL;
                    settleTargetYawDeg = 0.0f;
                    settleTargetForwardM = parallelExitM;
                    settleTargetLatM = offsetM;
                  } else if (canPhase == CANP_TURN_BACK_IN) {
                    settleNextDrivePhase = CANP_DRIVE_DIAG_IN;
                    settleTargetYawDeg = -thetaDeg;
                    settleTargetForwardM = forwardM + diagDownM;
                    settleTargetLatM = 0.0f;
                  } else {
                    // turn to center complete
                    settleNextDrivePhase = CANP_DRIVE_FINISH;
                    settleTargetYawDeg = 0.0f;
                    settleTargetForwardM = runDistanceM;
                    settleTargetLatM = 0.0f;
                  }

                  canPhase = CANP_TURN_SETTLE;

                  // Reset control/filter state before resuming motion.
                  speedIntegral = 0.0f;
                  steerIntegral = 0.0f;
                  yawIntegral = 0.0f;
                  prevSteerErr = 0.0f;
                  corrFiltered = 0.0f;
                  corrOut = 0.0f;

                  char line[220];
                  snprintf(line, sizeof(line),
                           "CAN_SETTLE %u->%u next=%u tgtYaw=%.1f fwd=%.2f lat=%.2f",
                           (unsigned)prev,
                           (unsigned)CANP_TURN_SETTLE,
                           (unsigned)settleNextDrivePhase,
                           (double)settleTargetYawDeg,
                           (double)forwardM,
                           (double)lateralM);
                  startDebugLogLine(line);
                }
              } else if (!inTol) {
                // Pivot with a strong PWM (to avoid stalling) and ramp up quickly.
                float pwmStart = CAN_PHASED_TURN_PWM_START;
                float pwmMax = CAN_PHASED_TURN_PWM_MAX;
                if (canPhase == CANP_TURN_TO_PARALLEL) {
                  pwmStart = CAN_PHASED_TURN_PARALLEL_PWM_START;
                  pwmMax = CAN_PHASED_TURN_PARALLEL_PWM_MAX;
                }
                float pwmBase = pwmStart;
                if (CAN_PHASED_TURN_RAMP_MS > 0) {
                  const uint32_t tMs = (uint32_t)now - turnPhaseStartMs;
                  float a = (float)tMs / (float)CAN_PHASED_TURN_RAMP_MS;
                  if (a < 0.0f) a = 0.0f;
                  if (a > 1.0f) a = 1.0f;
                  pwmBase = pwmStart + (pwmMax - pwmStart) * a;
                }

                // Slow down near the target to reduce overshoot, but never drop below a safe minimum.
                float pwmF = pwmBase;
                const float remForSlew = fabsf(remainingFused);
                if (remForSlew < CAN_PHASED_TURN_SLOW_DEG) {
                  pwmF = pwmBase * (remForSlew / CAN_PHASED_TURN_SLOW_DEG);
                }

                // Rate damping: if we're already spinning fast, back off to avoid overshoot.
                const float signedYawRateDps = imu_gz_dps * (float)CAN_PHASED_YAW_SIGN;
                pwmF -= CAN_PHASED_TURN_RATE_DAMP_PWM_PER_DPS * fabsf(signedYawRateDps);

                const float minPwm = (remForSlew < CAN_PHASED_TURN_NEAR_DEG) ? CAN_PHASED_TURN_NEAR_MIN_PWM : CAN_PHASED_TURN_MIN_PWM;
                if (pwmF < minPwm) pwmF = minPwm;

                // Near-target stall rescue: if yaw stops changing but we're not in tolerance yet,
                // bump PWM back up so we actually reach the target instead of hanging.
                if (remForSlew < CAN_PHASED_TURN_NEAR_DEG && fabsf(remainingFused) > TURN_IMU_TOL_DEG) {
                  const uint32_t sinceMoveMs = (uint32_t)now - turnLastMoveMs;
                  if (sinceMoveMs >= (uint32_t)CAN_PHASED_TURN_NEAR_STALL_WINDOW_MS) {
                    if (pwmF < CAN_PHASED_TURN_NEAR_STALL_BOOST_PWM) pwmF = CAN_PHASED_TURN_NEAR_STALL_BOOST_PWM;
                  }
                }

                // Stall boost: if we've been turning a while but yaw barely changed, punch to max.
                if (CAN_PHASED_TURN_STALL_BOOST_AFTER_MS > 0) {
                  const uint32_t tMs = (uint32_t)now - turnPhaseStartMs;
                  if (tMs >= (uint32_t)CAN_PHASED_TURN_STALL_BOOST_AFTER_MS && fabsf(deltaFusedDeg) < CAN_PHASED_TURN_STALL_MIN_DELTA_DEG) {
                    pwmF = pwmMax;
                  }
                }

                if (pwmF > 255.0f) pwmF = 255.0f;

                // Slew limit the PWM so we don't excite ringing (slow + steady).
                if (dt > 0.0f && CAN_PHASED_TURN_PWM_SLEW_PER_S > 0.0f) {
                  const float maxStep = CAN_PHASED_TURN_PWM_SLEW_PER_S * dt;
                  if (turnPwmCmdF < pwmF - maxStep) {
                    turnPwmCmdF += maxStep;
                  } else if (turnPwmCmdF > pwmF + maxStep) {
                    turnPwmCmdF -= maxStep;
                  } else {
                    turnPwmCmdF = pwmF;
                  }
                  pwmF = turnPwmCmdF;
                }

                // Update last sign only when we're actively turning (prevents jitter near 0).
                turnLastSign = currTurnSign;

                const int turnSign = currTurnSign;
                setMotorsPivotPwm(turnSign * CAN_PHASED_PIVOT_SIGN, (uint8_t)(pwmF + 0.5f));
              }
            }

            // SETTLE phase: small pivot/hold to ensure yaw is stable before driving.
            else if (canPhase == CANP_TURN_SETTLE) {
              // Hold the upcoming drive's yaw.
              targetYawDeg = settleTargetYawDeg;
              runDebug_targetYawDeg = targetYawDeg;

              const uint32_t tMs = (uint32_t)now - settlePhaseStartMs;
              const float yawErr = wrapDeg(runYawHoldDeg - targetYawDeg);
              yawErrDegForLog = yawErr;
              runDebug_turnRemainingDeg = yawErr;

              const float tolDeg = 2.5f;
              const uint32_t holdMs = 200;
              const uint32_t maxMs = 650;

              const bool inTol = (fabsf(yawErr) <= tolDeg);
              if (inTol) {
                if (settleInTolStartMs == 0) settleInTolStartMs = (uint32_t)now;
                setMotorsStop();
              } else {
                settleInTolStartMs = 0;
                // Gentle corrective pivot proportional to error, capped.
                float pwmF = 70.0f + 6.0f * fabsf(yawErr);
                if (pwmF < 70.0f) pwmF = 70.0f;
                if (pwmF > 140.0f) pwmF = 140.0f;
                const int turnSign = (yawErr <= 0.0f) ? +1 : -1;
                setMotorsPivotPwm(turnSign * CAN_PHASED_PIVOT_SIGN, (uint8_t)(pwmF + 0.5f));
              }

              const bool holdOk = (settleInTolStartMs != 0) && (((uint32_t)now - settleInTolStartMs) >= holdMs);
              if (holdOk || tMs >= maxMs) {
                runDebug_turnRemainingDeg = NAN;
                beginDrivePhase(settleNextDrivePhase, settleTargetYawDeg, settleTargetForwardM, settleTargetLatM);
              }
            }

            // DRIVE phases: forward motion under yaw hold, stop when reaching the phase target.
            else {
              // Completion criteria
              bool done = false;
              if (canPhase == CANP_DRIVE_STRAIGHT0) {
                done = (forwardM >= straight0M);
              } else if (canPhase == CANP_DRIVE_DIAG_OUT) {
                done = (lateralM >= (offsetM - CAN_LATERAL_TOL_M)) || (forwardM >= phaseTargetForwardM);
              } else if (canPhase == CANP_DRIVE_PARALLEL) {
                done = (forwardM >= phaseTargetForwardM);
              } else if (canPhase == CANP_DRIVE_DIAG_IN) {
                done = (fabsf(lateralM) <= CAN_LATERAL_TOL_M) || (forwardM >= phaseTargetForwardM);
              } else if (canPhase == CANP_DRIVE_FINISH) {
                done = (forwardM >= runDistanceM);
              }

              if (done) {
                setMotorsStop();
                // Next turn
                if (canPhase == CANP_DRIVE_STRAIGHT0) {
                  beginTurnPhase(CANP_TURN_TO_OFFSET, +thetaDeg);
                } else if (canPhase == CANP_DRIVE_DIAG_OUT) {
                  beginTurnPhase(CANP_TURN_TO_PARALLEL, -thetaDeg);
                } else if (canPhase == CANP_DRIVE_PARALLEL) {
                  beginTurnPhase(CANP_TURN_BACK_IN, -thetaDeg);
                } else if (canPhase == CANP_DRIVE_DIAG_IN) {
                  beginTurnPhase(CANP_TURN_TO_CENTER, +thetaDeg);
                } else {
                  // finished
                  lastRunStopReason = "DIST";
                  stopRun();
                  return;
                }
              }
            }

            // While in phased CAN mode, we don't use the continuous CAN steering math.
            // We set targetYawDeg above and let the straight-line IMU controller generate corrRaw below.
          }

          // In phased CAN mode, we still need heading-hold steering during DRIVE phases.
          // (Without this, phases run open-loop and drift.)
          if (canPhasedActive) {
            const bool canTurnPhase = (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                                       canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER ||
                                       canPhase == CANP_TURN_SETTLE);
            if (!canTurnPhase && imuControlOk) {
              // Heading-hold steering for phased CAN DRIVE segments.
              // Differential-drive sign convention: positive yaw (left/CCW) occurs when the RIGHT wheel
              // runs faster than the LEFT wheel. Our motor mixer implements this as corr>0 => rightPWM higher.
              // Therefore we use yawErr = target - yaw (positive error => need more positive yaw => corr>0).
              const float yawErrDegRaw = wrapDeg(targetYawDeg - runYawHoldDeg);
              yawErrDegForLog = yawErrDegRaw;
              const float yawErrDeg = (fabsf(yawErrDegRaw) < STRAIGHT_YAW_P_DEADBAND_DEG) ? 0.0f : yawErrDegRaw;

              // Heading integral with leak + clamp + deadband
              if (dt > 0.0f && STRAIGHT_YAW_I_LEAK_TAU_S > 0.0f) {
                yawIntegral -= yawIntegral * (dt / STRAIGHT_YAW_I_LEAK_TAU_S);
              }
              const float errForYawI = (fabsf(yawErrDeg) >= STRAIGHT_YAW_I_DEADBAND_DEG) ? yawErrDeg : 0.0f;
              yawIntegral += errForYawI * dt;
              if (STRAIGHT_YAW_KI > 0.0f) {
                const float iLim = STRAIGHT_YAW_I_MAX_CORR / STRAIGHT_YAW_KI;
                if (yawIntegral > iLim) yawIntegral = iLim;
                if (yawIntegral < -iLim) yawIntegral = -iLim;
              } else {
                yawIntegral = 0.0f;
              }

              // With yawErr defined as (target - yaw), derivative is -KD*rate.
              corrRaw = (STRAIGHT_YAW_KP * yawErrDeg) + (STRAIGHT_YAW_KI * yawIntegral) - (STRAIGHT_YAW_KD * imu_gz_dps);
              if (corrRaw > STRAIGHT_YAW_MAX_CORR) corrRaw = STRAIGHT_YAW_MAX_CORR;
              if (corrRaw < -STRAIGHT_YAW_MAX_CORR) corrRaw = -STRAIGHT_YAW_MAX_CORR;

              // Keep the encoder-based integrator from accumulating while IMU is active.
              steerIntegral = 0.0f;
              prevSteerErr = 0.0f;
            } else {
              // Don't wind up heading integral while pivoting.
              yawIntegral = 0.0f;
            }
          }

          // Base PWM debug
          runDebug_basePwm = basePWMf;

          if (!canPhasedActive) {
            // Straight-line control
            if (cm == CTRL_MODE_IMU && imuControlOk) {
              // Primary: gyro heading-hold (target heading = 0 deg relative to run start).
              // This avoids encoder quantization creating a long-period weave.
              const float yawErrDegRaw = wrapDeg(targetYawDeg - runYawHoldDeg);
              yawErrDegForLog = yawErrDegRaw;
              const float yawErrDeg = (fabsf(yawErrDegRaw) < STRAIGHT_YAW_P_DEADBAND_DEG) ? 0.0f : yawErrDegRaw;

              // Heading integral with leak + clamp + deadband
              if (dt > 0.0f && STRAIGHT_YAW_I_LEAK_TAU_S > 0.0f) {
                yawIntegral -= yawIntegral * (dt / STRAIGHT_YAW_I_LEAK_TAU_S);
              }
              // Only integrate when error exceeds deadband (prevents slow drift when nearly straight)
              const float errForYawI = (fabsf(yawErrDeg) >= STRAIGHT_YAW_I_DEADBAND_DEG) ? yawErrDeg : 0.0f;
              yawIntegral += errForYawI * dt;
              if (STRAIGHT_YAW_KI > 0.0f) {
                const float iLim = STRAIGHT_YAW_I_MAX_CORR / STRAIGHT_YAW_KI;
                if (yawIntegral > iLim) yawIntegral = iLim;
                if (yawIntegral < -iLim) yawIntegral = -iLim;
              } else {
                yawIntegral = 0.0f;
              }

              // With yawErr defined as (target - yaw), derivative is -KD*rate.
              corrRaw = (STRAIGHT_YAW_KP * yawErrDeg) + (STRAIGHT_YAW_KI * yawIntegral) - (STRAIGHT_YAW_KD * imu_gz_dps);
              if (corrRaw > STRAIGHT_YAW_MAX_CORR) corrRaw = STRAIGHT_YAW_MAX_CORR;
              if (corrRaw < -STRAIGHT_YAW_MAX_CORR) corrRaw = -STRAIGHT_YAW_MAX_CORR;

#if STRAIGHT_LATERAL_COMP_ENABLE && !STRAIGHT_LATERAL_HOLD_ENABLE
              if (forwardM >= STRAIGHT_LATERAL_MIN_FORWARD_M) {
                // Lateral compensation should steer back toward the centerline.
                // With our sign convention, lateralM>0 means drift left; lateralM<0 means drift right.
                // Therefore corrLat must be proportional to -lateralM (drift right => corrLat>0 => steer left).
                float corrLat = -STRAIGHT_LATERAL_KP_PWM_PER_M * lateralM;
                if (corrLat > STRAIGHT_LATERAL_MAX_CORR) corrLat = STRAIGHT_LATERAL_MAX_CORR;
                if (corrLat < -STRAIGHT_LATERAL_MAX_CORR) corrLat = -STRAIGHT_LATERAL_MAX_CORR;
                corrRaw += corrLat;
                if (corrRaw > STRAIGHT_YAW_MAX_CORR) corrRaw = STRAIGHT_YAW_MAX_CORR;
                if (corrRaw < -STRAIGHT_YAW_MAX_CORR) corrRaw = -STRAIGHT_YAW_MAX_CORR;
              }
#endif

#if RUN_GYRO_BIAS_LEARN
              // Very conservative: if wheel rates match and we're not actively steering, assume yaw rate should be ~0.
              if (dt > 0.0f && dt < 0.5f) {
                const float ppmRef = (pulsesPerMeter > 1.0f) ? pulsesPerMeter : 1.0f;
                const float leftRateEq = (pulsesPerMeterL > 1.0f && ppmRef > 1.0f) ? (leftRate * (ppmRef / pulsesPerMeterL)) : leftRate;
                const float rightRateEq = (pulsesPerMeterR > 1.0f && ppmRef > 1.0f) ? (rightRate * (ppmRef / pulsesPerMeterR)) : rightRate;
                const float rateAbs = (leftRateEq + rightRateEq) * 0.5f;
                const float rateDiff = fabsf(leftRateEq - rightRateEq);
                const float yawErrAbs = fabsf(yawErrDeg);
                const float corrAbs = fabsf(corrOut);
                if (rateAbs >= RUN_GYRO_BIAS_LEARN_MIN_RATE_PPS &&
                    rateDiff <= RUN_GYRO_BIAS_LEARN_MAX_RATE_DIFF_PPS &&
                    yawErrAbs <= RUN_GYRO_BIAS_LEARN_MAX_YAW_ERR_DEG &&
                    corrAbs <= RUN_GYRO_BIAS_LEARN_MAX_CORR_PWM) {
                  const float alpha = dt / (RUN_GYRO_BIAS_LEARN_TAU_S + dt);
                  float step = alpha * (imu_gz_raw_dps - imu_gz_bias_dps);
                  const float maxStep = RUN_GYRO_BIAS_LEARN_MAX_STEP_DPS_PER_S * dt;
                  if (step > maxStep) step = maxStep;
                  if (step < -maxStep) step = -maxStep;
                  imu_gz_bias_dps += step;

                  if (imu_gz_bias_dps > IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = IMU_GZ_BIAS_CLAMP_DPS;
                  if (imu_gz_bias_dps < -IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = -IMU_GZ_BIAS_CLAMP_DPS;
                }
              }
#endif

              // Keep the old encoder-based integrator from accumulating while IMU is active.
              steerIntegral = 0.0f;
              prevSteerErr = 0.0f;
            } else if (cm == CTRL_MODE_HYBRID && imuControlOk) {
              // HYBRID (encoder-primary): encoder wheel-rate steering PID (fast) + small IMU assist (P+D).
              // IMPORTANT: No in-run learning here (no encTrimPwm learning; no gyro bias learning during run).

              // Primary: encoder steering PID using per-wheel pulse rates (magnitudes).
              const float ppmRef = (pulsesPerMeter > 1.0f) ? pulsesPerMeter : 1.0f;
              const float leftRateEq = (pulsesPerMeterL > 1.0f && ppmRef > 1.0f) ? (leftRate * (ppmRef / pulsesPerMeterL)) : leftRate;
              const float rightRateEq = (pulsesPerMeterR > 1.0f && ppmRef > 1.0f) ? (rightRate * (ppmRef / pulsesPerMeterR)) : rightRate;
              const float steerErr = leftRateEq - rightRateEq;
              const float steerErrBiased = steerErr;

              if (dt > 0.0f && STEER_I_LEAK_TAU_S > 0.0f) {
                steerIntegral -= steerIntegral * (dt / STEER_I_LEAK_TAU_S);
              }
              const float errForI = (fabsf(steerErrBiased) >= STEER_I_DEADBAND_PPS) ? steerErrBiased : 0.0f;
              steerIntegral += errForI * dt;
              if (STEER_KI > 0.0f) {
                const float iLim = STEER_I_MAX_CORR / STEER_KI;
                if (steerIntegral > iLim) steerIntegral = iLim;
                if (steerIntegral < -iLim) steerIntegral = -iLim;
              } else {
                steerIntegral = 0.0f;
              }
              const float steerDeriv = (dt > 0.0f) ? ((steerErrBiased - prevSteerErr) / dt) : 0.0f;
              prevSteerErr = steerErrBiased;
              const float corrEnc = (STEER_KP * steerErrBiased) + (STEER_KI * steerIntegral) + (STEER_KD * steerDeriv);

              // Secondary: IMU assist (P+D) to fight yaw drift/slip without becoming the main loop.
              const float yawErrDegRaw = wrapDeg(targetYawDeg - runYawHoldDeg);
              yawErrDegForLog = yawErrDegRaw;
              const float yawErrDeg = (fabsf(yawErrDegRaw) < STRAIGHT_YAW_P_DEADBAND_DEG) ? 0.0f : yawErrDegRaw;

              // Allow a small yaw integral term here so HYBRID can cancel steady mechanical bias.
              // Without this, HYBRID tends to settle with a nonzero yaw error because keeping
              // left/right pulse rates equal does not guarantee straight travel if per-wheel
              // pulses-per-meter differ.
              if (dt > 0.0f && ENC_PRIMARY_YAW_I_LEAK_TAU_S > 0.0f) {
                yawIntegral -= yawIntegral * (dt / ENC_PRIMARY_YAW_I_LEAK_TAU_S);
              }
              const float errForYawI = (fabsf(yawErrDeg) >= ENC_PRIMARY_YAW_I_DEADBAND_DEG) ? yawErrDeg : 0.0f;
              yawIntegral += errForYawI * dt;
              if (ENC_PRIMARY_YAW_KI > 0.0f) {
                const float iLim = ENC_PRIMARY_YAW_I_MAX_CORR / ENC_PRIMARY_YAW_KI;
                if (yawIntegral > iLim) yawIntegral = iLim;
                if (yawIntegral < -iLim) yawIntegral = -iLim;
              } else {
                yawIntegral = 0.0f;
              }

              float corrYaw = (ENC_PRIMARY_YAW_KP * yawErrDeg) + (ENC_PRIMARY_YAW_KI * yawIntegral) - (ENC_PRIMARY_YAW_KD * imu_gz_dps);
              if (corrYaw > ENC_PRIMARY_YAW_MAX_CORR) corrYaw = ENC_PRIMARY_YAW_MAX_CORR;
              if (corrYaw < -ENC_PRIMARY_YAW_MAX_CORR) corrYaw = -ENC_PRIMARY_YAW_MAX_CORR;

              // Option 2: allow a slow learned trim to cancel steady bias.
              // (Learned later in the loop from the post-shaped corr signal.)

              corrRaw = corrEnc + corrYaw;
              if (corrRaw > HYBRID_MAX_CORR) corrRaw = HYBRID_MAX_CORR;
              if (corrRaw < -HYBRID_MAX_CORR) corrRaw = -HYBRID_MAX_CORR;

#if STRAIGHT_LATERAL_COMP_ENABLE && !STRAIGHT_LATERAL_HOLD_ENABLE
              if (forwardM >= STRAIGHT_LATERAL_MIN_FORWARD_M) {
                // See straight IMU mode comment: compensate opposite to lateral drift.
                float corrLat = -STRAIGHT_LATERAL_KP_PWM_PER_M * lateralM;
                if (corrLat > STRAIGHT_LATERAL_MAX_CORR) corrLat = STRAIGHT_LATERAL_MAX_CORR;
                if (corrLat < -STRAIGHT_LATERAL_MAX_CORR) corrLat = -STRAIGHT_LATERAL_MAX_CORR;
                corrRaw += corrLat;
                if (corrRaw > HYBRID_MAX_CORR) corrRaw = HYBRID_MAX_CORR;
                if (corrRaw < -HYBRID_MAX_CORR) corrRaw = -HYBRID_MAX_CORR;
              }
#endif
            } else {
              // Fallback: encoder-rate steering PID using per-wheel pulse rates (magnitudes)
              const float ppmRef = (pulsesPerMeter > 1.0f) ? pulsesPerMeter : 1.0f;
              const float leftRateEq = (pulsesPerMeterL > 1.0f && ppmRef > 1.0f) ? (leftRate * (ppmRef / pulsesPerMeterL)) : leftRate;
              const float rightRateEq = (pulsesPerMeterR > 1.0f && ppmRef > 1.0f) ? (rightRate * (ppmRef / pulsesPerMeterR)) : rightRate;
              const float steerErr = leftRateEq - rightRateEq;

              // Startup bias disabled; keep steerBias at 0.
              (void)startupBiasCount;
              (void)startupBiasSum;
              steerBias = 0.0f;
              const float steerErrBiased = steerErr;

              // Leaky integrator + deadband to reduce long-period sinusoidal wobble.
              if (dt > 0.0f && STEER_I_LEAK_TAU_S > 0.0f) {
                steerIntegral -= steerIntegral * (dt / STEER_I_LEAK_TAU_S);
              }
              const float errForI = (fabsf(steerErrBiased) >= STEER_I_DEADBAND_PPS) ? steerErrBiased : 0.0f;
              steerIntegral += errForI * dt;

              // Anti-windup: cap the integral so its PWM contribution stays bounded.
              if (STEER_KI > 0.0f) {
                const float iLim = STEER_I_MAX_CORR / STEER_KI;
                if (steerIntegral > iLim) steerIntegral = iLim;
                if (steerIntegral < -iLim) steerIntegral = -iLim;
              } else {
                steerIntegral = 0.0f;
              }
              const float steerDeriv = (dt > 0.0f) ? ((steerErrBiased - prevSteerErr) / dt) : 0.0f;
              prevSteerErr = steerErrBiased;

              corrRaw = (STEER_KP * steerErrBiased) + (STEER_KI * steerIntegral) + (STEER_KD * steerDeriv);

              // Encoder-only mode: do not accumulate yaw integral.
              yawIntegral = 0.0f;
            }
          }

#if RUN_YAW_ABORT_ENABLE
          // Only enforce during straight driving (not during CAN pivot turns).
          // In straight runs, this prevents drifting into obstacles if gyro bias or wheel slip causes sustained error.
          {
            const bool inCanTurn = canPhasedActive &&
                                   (runDebug_canPhase == (uint8_t)CANP_TURN_TO_OFFSET || runDebug_canPhase == (uint8_t)CANP_TURN_TO_PARALLEL ||
                                    runDebug_canPhase == (uint8_t)CANP_TURN_BACK_IN || runDebug_canPhase == (uint8_t)CANP_TURN_TO_CENTER);

            if (!inCanTurn && imuControlOk && fabsf(yawErrDegForLog) >= RUN_YAW_ABORT_DEG) {
              if (yawBadSinceMs == 0) yawBadSinceMs = now;
              if ((now - yawBadSinceMs) >= (unsigned long)RUN_YAW_ABORT_HOLD_MS) {
                Serial.printf("RUN FAULT: yaw error %.2f deg >= %.2f for %lums; stopping\n",
                              (double)yawErrDegForLog,
                              (double)RUN_YAW_ABORT_DEG,
                              (unsigned long)(now - yawBadSinceMs));
                lastRunStopReason = "YAW_ABORT";
                stopRun();
                return;
              }
            } else {
              yawBadSinceMs = 0;
            }
          }
#endif

          // Speed-based gain scheduling: reduce steering authority at low speed
          // (encoder quantization is worse and tends to create wobble).
          if (!canMode) {
            const float v = fabsf(actualRateForControl_mps);
            const float v0 = 0.10f; // m/s -> minimum steering authority
            const float v1 = 0.40f; // m/s -> full steering authority (your ~7m/15s is ~0.47 m/s)
            float speedScale = 1.0f;
            // With IMU heading-hold, keep more authority at low speeds.
            // In braking/slowdown, we *especially* want heading authority so we don't finish with a few-degree yaw error.
            const float minScale = imuControlOk ? (brakingActive ? 1.00f : 0.65f) : 0.20f;
            if (v <= v0) speedScale = minScale;
            else if (v >= v1) speedScale = 1.0f;
            else speedScale = minScale + (v - v0) * ((1.0f - minScale) / (v1 - v0));
            corrRaw *= speedScale;
          }

          // Low-pass + slew-rate limit on steering correction to prevent limit-cycle oscillation.
          // IMU heading-hold needs less lag than encoder-only steering.
          float corrTauS = 0.25f;
          float corrSlewPwmPerS = 240.0f;
          if (canPhasedActive) {
            corrTauS = 0.06f;
            corrSlewPwmPerS = 900.0f;
          } else if (imuControlOk) {
            // Reduce phase lag in straight runs; too much lag can create a slow sinusoidal weave.
            corrTauS = 0.03f;
            corrSlewPwmPerS = 800.0f;
           
            // During startup ramp, use faster filter to prevent lurch but maintain stability
            if (RUN_STEER_RAMP_MS > 0 && runStartMs != 0) {
              const uint32_t tSinceStartMs = (uint32_t)(now - runStartMs);
              if (tSinceStartMs < (uint32_t)RUN_STEER_RAMP_MS) {
                corrTauS = 0.012f;  // Fast but not too aggressive during shorter startup
                corrSlewPwmPerS = 1000.0f;
              }
            }
          }

          float corr = corrRaw;
          const bool bypassSteerFilter = false;
          if (bypassSteerFilter) {
            corrFiltered = corrRaw;
            corrOut = corrRaw;
            corr = corrRaw;
          } else if (dt > 0.0f) {
            const float alpha = dt / (corrTauS + dt);
            corrFiltered = corrFiltered + alpha * (corrRaw - corrFiltered);

            const float desired = corrFiltered;
            const float maxDelta = corrSlewPwmPerS * dt;
            float delta = desired - corrOut;
            if (delta > maxDelta) delta = maxDelta;
            if (delta < -maxDelta) delta = -maxDelta;
            corrOut += delta;
            corr = corrOut;
          }

          // At the start of a run, ramp steering authority briefly to avoid a step transient.
          // This applies to all control modes (including non-CAN straight runs).
          if (RUN_STEER_RAMP_MS > 0 && runStartMs != 0) {
            const uint32_t tSinceStartMs = (uint32_t)(now - runStartMs);
            if (tSinceStartMs < (uint32_t)RUN_STEER_RAMP_MS) {
              float a = (float)tSinceStartMs / (float)RUN_STEER_RAMP_MS;
              if (a < 0.0f) a = 0.0f;
              if (a > 1.0f) a = 1.0f;
              const float steerScale = a * a * (3.0f - 2.0f * a);
              corr *= steerScale;
            }
          }

          // Unified startup damp: apply to run start and CAN drive-phase starts.
          if (STARTUP_STEER_RAMP_MS > 0) {
            float steerScale = 1.0f;
            if (runStartMs != 0) {
              const uint32_t tSinceStartMs = (uint32_t)(now - runStartMs);
              if (tSinceStartMs < (uint32_t)STARTUP_STEER_RAMP_MS) {
                float a = (float)tSinceStartMs / (float)STARTUP_STEER_RAMP_MS;
                if (a < 0.0f) a = 0.0f;
                if (a > 1.0f) a = 1.0f;
                const float scaleRun = a * a * (3.0f - 2.0f * a);
                if (scaleRun < steerScale) steerScale = scaleRun;
              }
            }
            if (canPhasedActive && drivePhaseStartMs != 0) {
              const uint32_t tSincePhaseMs = (uint32_t)now - drivePhaseStartMs;
              if (tSincePhaseMs < (uint32_t)STARTUP_STEER_RAMP_MS) {
                float a = (float)tSincePhaseMs / (float)STARTUP_STEER_RAMP_MS;
                if (a < 0.0f) a = 0.0f;
                if (a > 1.0f) a = 1.0f;
                const float scalePhase = a * a * (3.0f - 2.0f * a);
                if (scalePhase < steerScale) steerScale = scalePhase;
              }
            }
            if (steerScale < 1.0f) {
              corr *= steerScale;
            }
          }

          // Restart guard: detect stop->forward transitions and suppress steering briefly.
          // This targets the recurring ~10deg yaw "kick" when resuming motion.
          static uint32_t restartSteerZeroUntilMs = 0;
          const bool prevCmdStopped = (motor_l_speed == 0 && motor_r_speed == 0);
          const bool canTurnPhaseNow = (canPhasedActive && (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                                                           canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER ||
                                                           canPhase == CANP_TURN_SETTLE));
          const bool willCommandForward = (effectiveTargetForwardRate > 0.0f) && (basePWMf > 0.0f) && !canTurnPhaseNow;
          if (RUN_RESTART_STEER_ZERO_MS > 0 && prevCmdStopped && willCommandForward) {
            restartSteerZeroUntilMs = (uint32_t)now + (uint32_t)RUN_RESTART_STEER_ZERO_MS;
          }
          if (restartSteerZeroUntilMs != 0 && (uint32_t)now < restartSteerZeroUntilMs && willCommandForward) {
            corrFiltered = 0.0f;
            corrOut = 0.0f;
            corr = 0.0f;
          }

          // In phased CAN DRIVE segments, prevent the steering correction from driving one wheel to 0.
          // If one side hits 0, the car behaves like it's "pivoting" and can flip-flop left/right.
          if (canPhasedActive) {
            const bool canTurnPhaseNow = (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                                          canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER ||
                                          canPhase == CANP_TURN_SETTLE);
            if (!canTurnPhaseNow && effectiveTargetForwardRate > 0.0f) {
              float maxCorrByThrottle = basePWMf - minPwmForMotion;
              if (maxCorrByThrottle < 0.0f) maxCorrByThrottle = 0.0f;
              if (corr > maxCorrByThrottle) corr = maxCorrByThrottle;
              if (corr < -maxCorrByThrottle) corr = -maxCorrByThrottle;
            }
          }

          // At the start of a phased CAN DRIVE segment, ramp steering authority briefly.
          // This prevents abrupt correction spikes when targetYaw changes between segments.
          // Use the same smoothstep curve as the run-start ramp for consistent dampening.
          float phaseSteerScale = 1.0f;
          {
            if (canPhasedActive && CAN_PHASED_DRIVE_STEER_RAMP_MS > 0) {
              const bool canTurnPhaseNow = (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                                            canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER ||
                                            canPhase == CANP_TURN_SETTLE);
              if (!canTurnPhaseNow && drivePhaseStartMs != 0) {
                const uint32_t tMs = (uint32_t)now - drivePhaseStartMs;
                if (tMs < (uint32_t)CAN_PHASED_DRIVE_STEER_RAMP_MS) {
                  float a = (float)tMs / (float)CAN_PHASED_DRIVE_STEER_RAMP_MS;
                  if (a < 0.0f) a = 0.0f;
                  if (a > 1.0f) a = 1.0f;
                  // Smoothstep: stays near 0 longer, then accelerates smoothly
                  phaseSteerScale = a * a * (3.0f - 2.0f * a);
                  corr *= phaseSteerScale;
                }
              }
            }
          }

          // Option 2: learn a slow trim that cancels constant steering bias.
          // Only enable for straight runs (CAN off) so it doesn't interfere with pivot/turn logic.
          // Learn from the post-shaped controller output (PWM units) so the trim directly offsets
          // whatever constant correction the controller needs.
          float corrNoTrim = corr;
          if (RUN_TRIM_LEARN_ENABLE && !canPhasedActive && cm == CTRL_MODE_HYBRID) {
            const bool okToApplyTrim =
              (fabsf(targetYawDeg) <= RUN_TRIM_APPLY_MAX_TARGET_YAW_DEG) &&
              (fabsf(yawErrDegForLog) <= RUN_TRIM_APPLY_MAX_YAW_ERR_DEG);

            // Apply trim to the final correction only when it is acting like a bias canceler.
            if (okToApplyTrim) {
              corr += encTrimPwm;
            }

            // Update trim slowly when we're up to speed and past the start transient.
            const uint32_t tSinceStartMs = (uint32_t)(now - runStartMs);
            const float vAbs = fabsf(actualRateForControl_mps);
            const bool okToLearn =
              okToApplyTrim &&
              (tSinceStartMs >= (uint32_t)RUN_TRIM_LEARN_START_DELAY_MS) &&
              (effectiveTargetForwardRate > 0.0f) &&
              (!brakingActive) &&
              (vAbs >= RUN_TRIM_LEARN_MIN_SPEED_MPS) &&
              (fabsf(yawErrDegForLog) <= RUN_TRIM_LEARN_MAX_YAW_ERR_DEG) &&
              (fabsf(imu_gz_dps) <= RUN_TRIM_LEARN_MAX_YAW_RATE_DPS) &&
              (dt > 0.0f && dt < 0.5f) &&
              (basePWMf > 0.0f && basePWMf < 250.0f);

            if (okToLearn && RUN_TRIM_LEARN_TAU_S > 0.0f) {
              // Drive encTrimPwm toward -corrNoTrim with a first-order time constant.
              const float alpha = dt / (RUN_TRIM_LEARN_TAU_S + dt);
              encTrimPwm += alpha * ((-corrNoTrim) - encTrimPwm);

              if (encTrimPwm > RUN_TRIM_LEARN_MAX_PWM) encTrimPwm = RUN_TRIM_LEARN_MAX_PWM;
              if (encTrimPwm < -RUN_TRIM_LEARN_MAX_PWM) encTrimPwm = -RUN_TRIM_LEARN_MAX_PWM;
            }
          }

          // Compute motor PWMs
          // Apply motor_bias_pwm as feedforward to compensate for mechanical differences
          // (e.g., left wheel has more drag). This reduces how hard the PID must work.
          // Bias has been removed from RUN control per testing request.
          // Keep motor_bias_pwm available for BIAS TEST mode only.
          float biasPwmF = 0.0f;
#if RUN_MOTOR_BIAS_ENABLE
          biasPwmF = (float)motor_bias_pwm;
          if (biasPwmF > (float)MOTOR_BIAS_MAX_PWM) biasPwmF = (float)MOTOR_BIAS_MAX_PWM;
          if (biasPwmF < -(float)MOTOR_BIAS_MAX_PWM) biasPwmF = -(float)MOTOR_BIAS_MAX_PWM;
#endif

          // Motor mixer: corr>0 should create positive yaw (left/CCW) by speeding up the RIGHT wheel.
          float leftPWMf = basePWMf - corr + biasPwmF;
          float rightPWMf = basePWMf + corr - biasPwmF;

          // Left motor startup ramp (reduces left-side rise for the first moments of a run).
          if (LEFT_STARTUP_RAMP_MS > 0 && runStartMs != 0 && effectiveTargetForwardRate > 0.0f) {
            const uint32_t tSinceStartMs = (uint32_t)(now - runStartMs);
            if (tSinceStartMs < (uint32_t)LEFT_STARTUP_RAMP_MS) {
              float a = (float)tSinceStartMs / (float)LEFT_STARTUP_RAMP_MS;
              if (a < 0.0f) a = 0.0f;
              if (a > 1.0f) a = 1.0f;
              const float scale = a * a * (3.0f - 2.0f * a);
              leftPWMf *= scale;
            }
          }

          // Track saturation shifting so we can detect when bias/corr are being amplified by lack of headroom.
          float satShift = 0.0f;

          // Preserve steering differential under saturation by shifting both sides together
          // (keeps (right-left) the same, but avoids one side clipping at 255/0).
          {
            float hi = (leftPWMf > rightPWMf) ? leftPWMf : rightPWMf;
            float lo = (leftPWMf < rightPWMf) ? leftPWMf : rightPWMf;
            if (hi > 255.0f) {
              const float over = hi - 255.0f;
              satShift += over;
              leftPWMf -= over;
              rightPWMf -= over;
            }
            if (lo < 0.0f) {
              const float under = 0.0f - lo;
              satShift -= under;
              leftPWMf += under;
              rightPWMf += under;
            }
          }

          // Bias-free RUN: do not apply persistent/user trim.

          // Clamp to valid 8-bit range (required by motor driver protocol)
          if (leftPWMf < 0.0f) leftPWMf = 0.0f;
          if (leftPWMf > 255.0f) leftPWMf = 255.0f;
          if (rightPWMf < 0.0f) rightPWMf = 0.0f;
          if (rightPWMf > 255.0f) rightPWMf = 255.0f;

          // If we're trying to move, enforce per-side minimum too.
          if (effectiveTargetForwardRate > 0.0f) {
            if (canPhasedActive) {
              // In CAN phased drive segments, keep both wheels above the minimum to avoid flip-flop pivoting.
              if (leftPWMf > 0.0f && leftPWMf < minPwmForMotion) leftPWMf = minPwmForMotion;
              if (rightPWMf > 0.0f && rightPWMf < minPwmForMotion) rightPWMf = minPwmForMotion;
            } else {
              // Straight runs: allow one wheel to go below the per-wheel minimum if needed for heading authority.
              // If BOTH wheels are small nonzero values, lift both together to avoid stalling.
              const bool lSmall = (leftPWMf > 0.0f && leftPWMf < minPwmForMotion);
              const bool rSmall = (rightPWMf > 0.0f && rightPWMf < minPwmForMotion);
              if (lSmall && rSmall) {
                leftPWMf = minPwmForMotion;
                rightPWMf = minPwmForMotion;
              }
            }
          }

          // If phased CAN is active, motor commands may already be set by turn/settle logic above.
          if (!(canPhasedActive && (canPhase == CANP_TURN_TO_OFFSET || canPhase == CANP_TURN_TO_PARALLEL ||
                                    canPhase == CANP_TURN_BACK_IN || canPhase == CANP_TURN_TO_CENTER ||
                                    canPhase == CANP_TURN_SETTLE))) {
            setMotorsForwardPwm((uint8_t)(leftPWMf + 0.5f), (uint8_t)(rightPWMf + 0.5f));
          }

          // Apply motor outputs now so command-path register fields match this tick's command.
          applyMotorOutputs();

          // Append log sample (use best-available left/right rates)
          RunLogSample s;
          s.t_ms = now - runStartMs;
          s.meters = meters;
          s.forwardM = forwardM;
          s.yawRelDeg = yawRelDeg;
          s.yawErrDeg = yawErrDegForLog;
          s.targetYawDeg = targetYawDeg;
          s.yawIntegral = yawIntegral;
          s.gz_dps = imu_gz_dps;
          s.gz_raw_dps = imu_gz_raw_dps;
          s.gz_bias_dps = imu_gz_bias_dps;
          s.actualRate = actualRateForControl_mps;
          s.leftRate = leftRate;
          s.rightRate = rightRate;
          s.speedErr = speedErrPps;
          s.basePWM = basePWMf;
          s.corrRaw = corrRaw;
          s.corrFiltered = corrFiltered;
          s.corrOut = corrOut;
          s.canPhase = canPhaseForLog;
          s.turnTargetDeg = turnTargetDegForLog;
          s.turnImuDeg = turnImuDegForLog;
          s.turnEncDeg = turnEncDegForLog;
          s.turnFusedDeg = turnFusedDegForLog;
          s.braking = brakingActive ? 1 : 0;
          s.lPwm = motor_l_speed;
          s.rPwm = motor_r_speed;
          s.mL_err_dir = motorL_err_dir;
          s.mL_err_pwm = motorL_err_pwm;
          s.mR_err_dir = motorR_err_dir;
          s.mR_err_pwm = motorR_err_pwm;

          s.cmdL_dir = motor_l_direction;
          s.cmdR_dir = motor_r_direction;
          s.regL_dir = motorL_last_reg_dir;
          s.regL_pwm = motorL_last_reg_pwm;
          s.regR_dir = motorR_last_reg_dir;
          s.regR_pwm = motorR_last_reg_pwm;

          // Extended analysis fields (appended at end)
          s.lateralM = lateralM;
          s.remainingM = remainingM;
          s.nominalTargetRate = nominalTargetForwardRate;
          s.effectiveTargetRate = effectiveTargetForwardRate;
          s.targetRatePps = targetRatePpsForLog;
          s.actualRatePps = actualRatePpsForLog;
          s.speedIntegral = speedIntegral;
          s.steerIntegral = steerIntegral;
          s.encTrimPwm = encTrimPwm;
          s.loopDtMs = (float)dtMs;

          // Extra diagnostics (appended at end)
          s.corrUsed = corr;
          s.motorBiasPwm = biasPwmF;
          s.satShift = satShift;
          s.imuReadUs = (float)runDbg_lastImuReadUs;
          s.motorWriteUs = (float)runDbg_lastMotorWriteUs;
          // Append (decimated) log sample
          if (runLogEveryN <= 1 || ((runLogTick % runLogEveryN) == 0)) {
            runLogAppend(s);
          }
          runLogTick++;
        }
      }
    } else {
      stopAllMotors();
    }
  }

  else if (currentScreen == SCREEN_HW_TEST) {
    // Hold BtnA to exit HW TEST back to menu
    static bool hwLongPressHandled = false;
    if (!M5.BtnA.isPressed()) hwLongPressHandled = false;
    if (!hwLongPressHandled && M5.BtnA.pressedFor(800)) {
      hwLongPressHandled = true;
      suppressNextClick = true;
      enterScreen(SCREEN_MAIN_MENU);
    }

    // Button A: switch selected motor (LEFT <-> RIGHT)
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked() && (now - last_button_ms) > 250) {
      last_button_ms = now;
      selected_motor = (selected_motor == 0) ? 1 : 0;
      stopAllMotors();
    }

    applyHardwareTestDialSteps(dialSteps);

    motor_enabled = (motor_command != 0) ? 1 : 0;
    uint8_t pwm = (uint8_t)abs((int)motor_command);
    const uint8_t dir = (motor_command >= 0) ? 1 : 0;
    motor_l_direction = dir;
    motor_r_direction = dir;

    if (motor_enabled) {
      if (selected_motor == 0) {
        motor_l_speed = pwm;
        motor_r_speed = 0;
      } else {
        motor_l_speed = 0;
        motor_r_speed = pwm;
      }
    } else {
      motor_l_speed = 0;
      motor_r_speed = 0;
    }

    applyMotorOutputs();
  }

  // Render the active screen
  static unsigned long lastDisplay = 0;
  if (now - lastDisplay > DISPLAY_REFRESH_MS) {
    lastDisplay = now;
    renderCurrentScreen();
  }
 
#if SERIAL_STATUS_SPAM
  // Print status every 2 seconds with dial debug info
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.printf("Status: Motor=%s | Sel=%s | Cmd=%d | A=%d B=%d | Lspd=%d Rspd=%d | HB(0x%02X %s e=%u/%u rb=%u/%u) HB(0x%02X %s e=%u/%u rb=%u/%u) | Enc: L(%s)=%ld R(%s)=%ld\n",
                  motor_enabled ? "ON " : "OFF",
                  selected_motor == 0 ? "LEFT" : "RIGHT",
                  (int)motor_command,
                  encoderA_state,
                  encoderB_state,
                  motor_l_speed,
                  motor_r_speed,
                  MOTOR_L_ADDR,
                  motorL_present ? "OK" : "--",
                  (unsigned)motorL_err_dir,
                  (unsigned)motorL_err_pwm,
                  (unsigned)motorL_dir_rb,
                  (unsigned)motorL_spd_rb,
                  MOTOR_R_ADDR,
                  motorR_present ? "OK" : "--",
                  (unsigned)motorR_err_dir,
                  (unsigned)motorR_err_pwm,
                  (unsigned)motorR_dir_rb,
                  (unsigned)motorR_spd_rb,
                  encoderL_found ? "OK" : "--",
                  encoderL_count,
                  encoderR_found ? "OK" : "--",
                  encoderR_count);
  }
#endif
}