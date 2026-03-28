#include <M5Unified.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Preferences.h>
#include <FFat.h>
#define SPIFFS FFat   // Board has FFat partition, not SPIFFS
#include <esp_system.h>

#include <time.h>
#include <sys/time.h>

#include <esp_heap_caps.h>

// WiFi + cloud upload removed — all logging is local SPIFFS only.
// Use the Python tool (tools/download_logs.py) to pull logs over serial.

// Firmware version
// FW_VERSION defined near top
  // Auto-encoder calibration routines removed — manual tuning via ENC CAL screen.
// Provide a default firmware version string if the build system hasn't set one.
#ifndef FW_VERSION
#define FW_VERSION "unknown"
#endif
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

// I2C bus guard delay (µs) between back-to-back transactions.
// Prevents bus contention when 5 devices share the bus (2 motors, 2 encoders, IMU).
#define I2C_GUARD_US 150
static inline void i2cGuard() { delayMicroseconds(I2C_GUARD_US); }

// I2C consecutive error tracking for automatic bus recovery
static uint16_t i2c_consecutive_errors = 0;
static uint32_t i2c_total_errors = 0;

// Encoder-specific read failure tracking
static uint32_t enc_read_attempts = 0;   // total read attempts
static uint32_t enc_read_failures = 0;   // total failed reads (both encoders)
static uint32_t enc_read_recoveries = 0; // successful reads after bus recovery

// === Adaptive Encoder Mode ===
// Determined automatically at boot by spinning each motor and checking which
// encoders respond under load.  BFS DRIVE uses this to decide its distance
// source.  Falls back to time-based dead reckoning if neither encoder works.
enum EncoderMode {
  ENC_MODE_BOTH,       // Average of left + right
  ENC_MODE_LEFT_ONLY,  // Only left encoder usable under motor load
  ENC_MODE_RIGHT_ONLY, // Only right encoder usable under motor load
  ENC_MODE_TIMED,      // Neither encoder works — estimate distance from time
};
static EncoderMode enc_mode = ENC_MODE_RIGHT_ONLY;  // default until boot test runs

// Estimated robot speed (m/s) at BFS_DRIVE_PWM for timed fallback.
// Calibrate this by measuring actual speed over a known distance.
static constexpr float ENC_TIMED_SPEED_MPS = 0.30f;

// I2C bus recovery: toggle SCL to release stuck slaves
static int i2c_sda_pin = -1;  // filled in during setup()
static int i2c_scl_pin = -1;
static void i2cBusRecovery() {
  if (i2c_scl_pin < 0 || i2c_sda_pin < 0) return;
  // Release Wire so we can bit-bang SCL
  Wire.end();
  pinMode(i2c_scl_pin, OUTPUT);
  pinMode(i2c_sda_pin, INPUT_PULLUP);
  // Toggle SCL 16 times to clock out any stuck slave
  for (int i = 0; i < 16; i++) {
    digitalWrite(i2c_scl_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(i2c_scl_pin, HIGH);
    delayMicroseconds(5);
  }
  // Re-init Wire
  Wire.begin(i2c_sda_pin, i2c_scl_pin);
  Wire.setTimeOut(20);
  Wire.setClock(I2C_CLOCK_HZ);
  delayMicroseconds(500);
}

static const uint32_t i2c_clock_hz = I2C_CLOCK_HZ;

// RUN safety: stop if encoders are missing or not advancing.
// This prevents driving blindly when I2C sensor reads fail.
#define RUN_ENCODER_MISSING_TIMEOUT_MS 400
#define RUN_NO_MOTION_TIMEOUT_MS 1200
#define RUN_NO_MOTION_MIN_PULSES 8

// Display refresh interval (ms)
#ifndef DISPLAY_REFRESH_MS
#define DISPLAY_REFRESH_MS 50
#endif

// Boot test defaults (safe fallbacks if not provided elsewhere)
#ifndef BOOT_TEST_MOTOR_ADDR
#define BOOT_TEST_MOTOR_ADDR MOTOR_L_ADDR
#endif
#ifndef BOOT_TEST_DIRECTION
#define BOOT_TEST_DIRECTION 1
#endif
#ifndef BOOT_TEST_PWM
#define BOOT_TEST_PWM 80
#endif
#ifndef BOOT_TEST_DURATION_MS
#define BOOT_TEST_DURATION_MS 500
#endif

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
#define BFS_MAX_NODES 128         // Maximum waypoints (grid + center sub-nodes + virtual)
#define BFS_MAX_EDGES_PER_NODE 12 // Max adjacent nodes (grid + virtual center/gate nodes)
#define BFS_PATH_MAX_LENGTH 256   // Max steps in a path (needs room for multi-gate routes)

// Node structure for graph representation
struct BfsNode {
  uint8_t id;                       // Node ID
  char name[8];                     // Short label (e.g. "2,3")
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
  uint16_t path_length;
  uint16_t path_index;              // Current step along path
  bool path_valid;
  float path_dist_m;                // Total physical distance of last computed path
};

static BfsState bfs_state = {0};
static bool bfs_initialized = false;
static uint8_t bfs_end_center_node = 0;  // Virtual node at end square center
static uint8_t bfs_start_edge_node = 0; // Virtual node at start edge midpoint

// ========================================================================
// GRID-BASED COURSE CONFIGURATION FOR ROBOT TOUR
// ========================================================================
// The course is a grid of intersections. The robot travels along grid lines.
// Walls block edges between adjacent intersections (entered as H/V separately).
// Water bottles sit on grid edges and block that edge (entered as H/V separately).
// Gates are grid squares the robot MUST pass through.

#define GRID_MAX_DIM   8          // Max grid dimension (8x8 = 64 nodes)
#define MAX_WALLS     32          // Max wall segments
#define MAX_BOTTLES   16          // Max water bottle positions
#define MAX_GATES      8          // Max gate positions

struct GridWall {
  uint8_t r1, c1, r2, c2;        // Between grid point (r1,c1) and (r2,c2)
};

struct GridBottle {
  uint8_t r1, c1, r2, c2;        // On edge between (r1,c1) and (r2,c2)
};

struct GridGate {
  uint8_t row, col;               // Square index (row, col)
};

struct GridCourse {
  uint8_t cols;                   // Grid columns (2-8)
  uint8_t rows;                   // Grid rows (2-8)
  uint8_t spacing_cm;             // Distance between grid points (cm), 10-100
  uint8_t start_r1, start_c1;     // Start edge endpoint 1
  uint8_t start_r2, start_c2;     // Start edge endpoint 2 (midpoint = start position)
  uint8_t end_col, end_row;       // End position (square index)

  GridWall walls[MAX_WALLS];
  uint8_t wall_count;

  GridBottle bottles[MAX_BOTTLES];
  uint8_t bottle_count;

  GridGate gates[MAX_GATES];
  uint8_t gate_count;
  int8_t last_gate_idx;           // -1 = auto (optimize), 0+ = index into gates[] that must be visited last
};

static GridCourse grid_course;
static uint8_t bfs_gate_nodes[MAX_GATES]; // Virtual nodes at gate square centers
static uint8_t bfs_sq_center[GRID_MAX_DIM][GRID_MAX_DIM]; // Center sub-node ID for each square (row,col)

// ========================================================================
// MANUAL WAYPOINT SYSTEM (replaces BFS pathfinding for 2026 rule compliance)
// ========================================================================
// The 2026 rules (8.i.v) prohibit algorithms that find the optimal route.
// Instead, competitors manually determine the route and enter waypoints.
// Waypoints are placed on a 25cm sub-grid (11 cols x 9 rows) covering
// the 250cm x 200cm track, allowing diagonal and fractional-square moves.
#define MANUAL_WP_MAX 32
#define MANUAL_GRID_COLS 11   // 0-10 at 25cm = 250cm
#define MANUAL_GRID_ROWS 9    // 0-8 at 25cm = 200cm
#define MANUAL_GRID_SPACING_CM 25

static uint8_t manual_wp_count = 0;
static uint8_t manual_wp_col[MANUAL_WP_MAX]; // col index on 25cm grid (0-10)
static uint8_t manual_wp_row[MANUAL_WP_MAX]; // row index on 25cm grid (0-8)
// Cursor position for waypoint editor
static uint8_t manual_wp_cursor_col = 0;
static uint8_t manual_wp_cursor_row = 0;
// Flat index for dial scrolling: cursor_idx = row * MANUAL_GRID_COLS + col
static int manual_wp_cursor_idx = 0;

// Initialize grid_course with sensible defaults
// Fixed grid: 6 columns x 5 rows of intersections = 5x4 squares = 250x200cm at 50cm spacing
#define GRID_FIXED_COLS   6
#define GRID_FIXED_ROWS   5
#define GRID_FIXED_SPACING_CM 50

static void gridCourseDefaults() {
  memset(&grid_course, 0, sizeof(grid_course));
  grid_course.cols = GRID_FIXED_COLS;
  grid_course.rows = GRID_FIXED_ROWS;
  grid_course.spacing_cm = GRID_FIXED_SPACING_CM;
  // Default start: midpoint of top-left horizontal border edge (0,0)-(0,1)
  grid_course.start_r1 = 0; grid_course.start_c1 = 0;
  grid_course.start_r2 = 0; grid_course.start_c2 = 1;
  grid_course.end_col = (GRID_FIXED_COLS - 1) / 2;  // Square column index (0 to cols-2)
  grid_course.end_row = (GRID_FIXED_ROWS - 1) / 2;  // Square row index (0 to rows-2)
  grid_course.last_gate_idx = -1; // Auto (optimize all permutations)
}

// Convert (row, col) to node index
static inline uint8_t gridNodeId(uint8_t row, uint8_t col) {
  return row * grid_course.cols + col;
}

// Check if a wall exists between two adjacent grid points
static bool gridHasWall(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2) {
  for (uint8_t i = 0; i < grid_course.wall_count; i++) {
    const GridWall& w = grid_course.walls[i];
    if ((w.r1 == r1 && w.c1 == c1 && w.r2 == r2 && w.c2 == c2) ||
        (w.r1 == r2 && w.c1 == c2 && w.r2 == r1 && w.c2 == c1)) {
      return true;
    }

    // (Stall detection relocated to IMU/control update; removed from gridHasWall.)
  }
  return false;
}

// Check if a water bottle is on an edge between two adjacent grid points
static bool gridHasBottle(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2) {
  for (uint8_t i = 0; i < grid_course.bottle_count; i++) {
    const GridBottle& b = grid_course.bottles[i];
    if ((b.r1 == r1 && b.c1 == c1 && b.r2 == r2 && b.c2 == c2) ||
        (b.r1 == r2 && b.c1 == c2 && b.r2 == r1 && b.c2 == c1)) {
      return true;
    }
  }
  return false;
}

// Check if a gate exists at a grid square
static bool gridHasGate(uint8_t row, uint8_t col) {
  for (uint8_t i = 0; i < grid_course.gate_count; i++) {
    if (grid_course.gates[i].row == row && grid_course.gates[i].col == col)
      return true;
  }
  return false;
}

// Toggle a wall between two adjacent grid points (add or remove)
static void gridToggleWall(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2) {
  // Check if wall already exists, remove it
  for (uint8_t i = 0; i < grid_course.wall_count; i++) {
    GridWall& w = grid_course.walls[i];
    if ((w.r1 == r1 && w.c1 == c1 && w.r2 == r2 && w.c2 == c2) ||
        (w.r1 == r2 && w.c1 == c2 && w.r2 == r1 && w.c2 == c1)) {
      // Remove by shifting
      for (uint8_t j = i; j < grid_course.wall_count - 1; j++)
        grid_course.walls[j] = grid_course.walls[j + 1];
      grid_course.wall_count--;
      return;
    }
  }
  // Add new wall
  if (grid_course.wall_count < MAX_WALLS) {
    grid_course.walls[grid_course.wall_count] = {r1, c1, r2, c2};
    grid_course.wall_count++;
  }
}

// Toggle a water bottle on an edge (add or remove)
static void gridToggleBottle(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2) {
  for (uint8_t i = 0; i < grid_course.bottle_count; i++) {
    GridBottle& b = grid_course.bottles[i];
    if ((b.r1 == r1 && b.c1 == c1 && b.r2 == r2 && b.c2 == c2) ||
        (b.r1 == r2 && b.c1 == c2 && b.r2 == r1 && b.c2 == c1)) {
      for (uint8_t j = i; j < grid_course.bottle_count - 1; j++)
        grid_course.bottles[j] = grid_course.bottles[j + 1];
      grid_course.bottle_count--;
      return;
    }
  }
  if (grid_course.bottle_count < MAX_BOTTLES) {
    grid_course.bottles[grid_course.bottle_count] = {r1, c1, r2, c2};
    grid_course.bottle_count++;
  }
}

// Toggle a gate at a grid square (add or remove)
static void gridToggleGate(uint8_t row, uint8_t col) {
  for (uint8_t i = 0; i < grid_course.gate_count; i++) {
    if (grid_course.gates[i].row == row && grid_course.gates[i].col == col) {
      for (uint8_t j = i; j < grid_course.gate_count - 1; j++)
        grid_course.gates[j] = grid_course.gates[j + 1];
      grid_course.gate_count--;
      return;
    }
  }
  if (grid_course.gate_count < MAX_GATES) {
    grid_course.gates[grid_course.gate_count] = {row, col};
    grid_course.gate_count++;
  }
}

// Enumerate all possible edge positions for walls/gates
// Returns total count. Given an index, fills r1,c1,r2,c2.
static uint16_t gridEdgeCount() {
  // Horizontal edges: rows * (cols-1) + Vertical edges: (rows-1) * cols
  uint16_t h = (uint16_t)grid_course.rows * (grid_course.cols > 0 ? grid_course.cols - 1 : 0);
  uint16_t v = (grid_course.rows > 0 ? grid_course.rows - 1 : 0) * (uint16_t)grid_course.cols;
  return h + v;
}

static void gridEdgeFromIndex(uint16_t idx, uint8_t& r1, uint8_t& c1, uint8_t& r2, uint8_t& c2) {
  uint16_t h = (uint16_t)grid_course.rows * (grid_course.cols > 0 ? grid_course.cols - 1 : 0);
  if (idx < h) {
    // Horizontal edge: (row, col) -- (row, col+1)
    uint8_t cols_m1 = grid_course.cols - 1;
    r1 = idx / cols_m1;
    c1 = idx % cols_m1;
    r2 = r1;
    c2 = c1 + 1;
  } else {
    // Vertical edge: (row, col) -- (row+1, col)
    uint16_t vi = idx - h;
    r1 = vi / grid_course.cols;
    c1 = vi % grid_course.cols;
    r2 = r1 + 1;
    c2 = c1;
  }
}

// Enumerate all grid positions for bottles
static uint16_t gridPositionCount() {
  return (uint16_t)grid_course.rows * grid_course.cols;
}

static void gridPositionFromIndex(uint16_t idx, uint8_t& row, uint8_t& col) {
  row = idx / grid_course.cols;
  col = idx % grid_course.cols;
}

// ---- Separate horizontal / vertical edge helpers ----
// Horizontal edges: (r,c)-(r,c+1), count = rows*(cols-1)
static uint16_t gridHEdgeCount() {
  return (uint16_t)grid_course.rows * (grid_course.cols > 0 ? grid_course.cols - 1 : 0);
}
static void gridHEdgeFromIndex(uint16_t idx, uint8_t& r1, uint8_t& c1, uint8_t& r2, uint8_t& c2) {
  uint8_t cols_m1 = grid_course.cols - 1;
  r1 = idx / cols_m1;
  c1 = idx % cols_m1;
  r2 = r1;
  c2 = c1 + 1;
}
// Vertical edges: (r,c)-(r+1,c), count = (rows-1)*cols
static uint16_t gridVEdgeCount() {
  return (grid_course.rows > 0 ? grid_course.rows - 1 : 0) * (uint16_t)grid_course.cols;
}
static void gridVEdgeFromIndex(uint16_t idx, uint8_t& r1, uint8_t& c1, uint8_t& r2, uint8_t& c2) {
  r1 = idx / grid_course.cols;
  c1 = idx % grid_course.cols;
  r2 = r1 + 1;
  c2 = c1;
}

// BFS setup step cursor state for wall/bottle/gate editors
static uint16_t grid_hwall_cursor = 0;
static uint16_t grid_vwall_cursor = 0;
static uint16_t grid_hbottle_cursor = 0;
static uint16_t grid_vbottle_cursor = 0;
static uint16_t grid_gate_cursor = 0;  // Now cycles squares

// ---- Border edge helpers ----
// Enumerate all edges on the grid border (perimeter).
// Goes clockwise: top row L→R, right col T→B, bottom row R→L, left col B→T.
// Start position is at the midpoint of one of these edges.
static uint16_t gridBorderEdgeCount() {
  // Top border: cols-1 horizontal edges
  // Right border: rows-1 vertical edges
  // Bottom border: cols-1 horizontal edges
  // Left border: rows-1 vertical edges
  return 2 * (grid_course.cols - 1) + 2 * (grid_course.rows - 1);
}

static void gridBorderEdgeFromIndex(uint16_t idx, uint8_t &r1, uint8_t &c1, uint8_t &r2, uint8_t &c2) {
  uint8_t cols = grid_course.cols;
  uint8_t rows = grid_course.rows;
  uint16_t total = gridBorderEdgeCount();
  if (total == 0) { r1 = c1 = r2 = c2 = 0; return; }
  idx = idx % total;
  uint16_t top = cols - 1;
  uint16_t right = rows - 1;
  uint16_t bottom = cols - 1;
  // Top border: (0,c)-(0,c+1)
  if (idx < top) { r1 = 0; c1 = idx; r2 = 0; c2 = idx + 1; return; }
  idx -= top;
  // Right border: (r,cols-1)-(r+1,cols-1)
  if (idx < right) { r1 = idx; c1 = cols - 1; r2 = idx + 1; c2 = cols - 1; return; }
  idx -= right;
  // Bottom border: (rows-1,c+1)-(rows-1,c) — reversed direction for clockwise
  if (idx < bottom) { r1 = rows - 1; c1 = cols - 1 - idx; r2 = rows - 1; c2 = cols - 2 - idx; return; }
  idx -= bottom;
  // Left border: (r+1,0)-(r,0) — reversed direction for clockwise
  r1 = rows - 1 - idx; c1 = 0; r2 = rows - 2 - idx; c2 = 0;
}

static uint16_t gridBorderEdgeIndex(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2) {
  uint8_t cols = grid_course.cols;
  uint8_t rows = grid_course.rows;
  // Normalize: try both orderings
  for (int pass = 0; pass < 2; pass++) {
    uint8_t a1 = pass ? r2 : r1, b1 = pass ? c2 : c1;
    uint8_t a2 = pass ? r1 : r2, b2 = pass ? c1 : c2;
    // Check each border segment
    uint16_t idx = 0;
    // Top
    for (uint8_t c = 0; c < cols - 1; c++, idx++)
      if (a1 == 0 && b1 == c && a2 == 0 && b2 == c + 1) return idx;
    // Right
    for (uint8_t r = 0; r < rows - 1; r++, idx++)
      if (a1 == r && b1 == cols - 1 && a2 == r + 1 && b2 == cols - 1) return idx;
    // Bottom (reversed)
    for (uint8_t c = cols - 1; c > 0; c--, idx++)
      if (a1 == rows - 1 && b1 == c && a2 == rows - 1 && b2 == c - 1) return idx;
    // Left (reversed)
    for (uint8_t r = rows - 1; r > 0; r--, idx++)
      if (a1 == r && b1 == 0 && a2 == r - 1 && b2 == 0) return idx;
  }
  return 0;
}

// ---- Square center helpers ----
// Square (sr, sc) has corners at (sr,sc),(sr,sc+1),(sr+1,sc),(sr+1,sc+1).
// Valid range: sr 0..rows-2, sc 0..cols-2.  Total = (rows-1)*(cols-1).
static uint16_t gridSquareCount() {
  return (uint16_t)(grid_course.cols - 1) * (grid_course.rows - 1);
}

static void gridSquarePosition(uint16_t idx, uint8_t &sqRow, uint8_t &sqCol) {
  uint8_t sqCols = grid_course.cols - 1;
  sqRow = idx / sqCols;
  sqCol = idx % sqCols;
}

static uint16_t gridSquareIndex(uint8_t sqRow, uint8_t sqCol) {
  return (uint16_t)sqRow * (grid_course.cols - 1) + sqCol;
}

// Cursor variables for start (border) and end (square center)
static uint16_t grid_start_cursor = 0;
static uint16_t grid_end_cursor = 0;

// --- Wall clearance helper ---
// Returns the minimum distance from a point (px,py) to the line segment (ax,ay)-(bx,by).
static float pointToSegmentDist(float px, float py, float ax, float ay, float bx, float by) {
  float dx = bx - ax, dy = by - ay;
  float lenSq = dx * dx + dy * dy;
  if (lenSq < 1e-12f) return sqrtf((px - ax) * (px - ax) + (py - ay) * (py - ay));
  float t = ((px - ax) * dx + (py - ay) * dy) / lenSq;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  float cx = ax + t * dx, cy = ay + t * dy;
  return sqrtf((px - cx) * (px - cx) + (py - cy) * (py - cy));
}

// Check if two line segments cross in their interiors (excluding endpoints).
static bool segmentsCross(float ax, float ay, float bx, float by,
                          float cx, float cy, float dx, float dy) {
  float d1x = bx - ax, d1y = by - ay;
  float d2x = dx - cx, d2y = dy - cy;
  float denom = d1x * d2y - d1y * d2x;
  if (fabsf(denom) < 1e-8f) return false; // parallel
  float t = ((cx - ax) * d2y - (cy - ay) * d2x) / denom;
  float u = ((cx - ax) * d1y - (cy - ay) * d1x) / denom;
  return t > 0.01f && t < 0.99f && u > 0.01f && u < 0.99f;
}

// Check if edge from A to B is clear of all walls & bottles.
// Uses crossing test for walls sharing an endpoint with the edge,
// and sample-point clearance check for other walls.
static bool bfsEdgeClearOfWalls(float ax_m, float ay_m, float bx_m, float by_m, float clearance_m) {
  const float spacing_m = grid_course.spacing_cm / 100.0f;

  // Helper lambda: check one obstacle segment against the edge
  auto checkObstacle = [&](float wx1, float wy1, float wx2, float wy2) -> bool {
    // Always block if segments cross in their interiors
    if (segmentsCross(ax_m, ay_m, bx_m, by_m, wx1, wy1, wx2, wy2)) return false;
    // Check clearance at sample points along the edge (including near endpoints)
    for (int s = 0; s <= 4; s++) {
      float t = s * 0.25f;
      float px = ax_m + t * (bx_m - ax_m);
      float py = ay_m + t * (by_m - ay_m);
      float dist = pointToSegmentDist(px, py, wx1, wy1, wx2, wy2);
      if (dist < clearance_m - 0.001f) return false;
    }
    // Also check clearance of edge from wall/bottle endpoints (corners)
    float dEp1 = pointToSegmentDist(wx1, wy1, ax_m, ay_m, bx_m, by_m);
    if (dEp1 < clearance_m - 0.001f) return false;
    float dEp2 = pointToSegmentDist(wx2, wy2, ax_m, ay_m, bx_m, by_m);
    if (dEp2 < clearance_m - 0.001f) return false;
    return true;
  };

  for (uint8_t i = 0; i < grid_course.wall_count; i++) {
    const GridWall& w = grid_course.walls[i];
    if (!checkObstacle(w.c1 * spacing_m, w.r1 * spacing_m, w.c2 * spacing_m, w.r2 * spacing_m))
      return false;
  }
  for (uint8_t i = 0; i < grid_course.bottle_count; i++) {
    const GridBottle& b = grid_course.bottles[i];
    if (!checkObstacle(b.c1 * spacing_m, b.r1 * spacing_m, b.c2 * spacing_m, b.r2 * spacing_m))
      return false;
  }
  return true;
}

// Add an edge between two nodes (bidirectional)
static void bfsAddEdge(uint8_t from_id, uint8_t to_id, float distance_m) {
  if (from_id >= BFS_MAX_NODES || to_id >= BFS_MAX_NODES) return;
  
  BfsNode* from = &bfs_state.nodes[from_id];
  if (from->neighbor_count >= BFS_MAX_EDGES_PER_NODE) return;
  
  from->neighbors[from->neighbor_count] = to_id;
  from->distances[from->neighbor_count] = distance_m;
  from->neighbor_count++;
}

// Add an edge between two nodes only if it clears all walls by the required margin.
// clearance_m <= 0 skips the check (used for start node connections outside the grid).
static void bfsAddEdgeIfClear(uint8_t from_id, uint8_t to_id, float distance_m, float clearance_m) {
  if (from_id >= BFS_MAX_NODES || to_id >= BFS_MAX_NODES) return;
  if (clearance_m > 0.0f) {
    float ax = bfs_state.nodes[from_id].x_m, ay = bfs_state.nodes[from_id].y_m;
    float bx = bfs_state.nodes[to_id].x_m,   by = bfs_state.nodes[to_id].y_m;
    if (!bfsEdgeClearOfWalls(ax, ay, bx, by, clearance_m)) return;
  }
  bfsAddEdge(from_id, to_id, distance_m);
}

// Build BFS graph from grid_course configuration
// Creates nodes for each grid intersection, adds edges for adjacent nodes
// that are not blocked by walls or water bottles.
static void bfsInitializeGraph() {
  memset(&bfs_state, 0, sizeof(bfs_state));
  memset(bfs_sq_center, 0xFF, sizeof(bfs_sq_center)); // 0xFF = no center node

  const uint8_t rows = grid_course.rows;
  const uint8_t cols = grid_course.cols;
  const float spacing_m = grid_course.spacing_cm / 100.0f;

  bfs_state.node_count = rows * cols;
  if (bfs_state.node_count > BFS_MAX_NODES) bfs_state.node_count = BFS_MAX_NODES;

  // Create grid intersection nodes
  for (uint8_t r = 0; r < rows; r++) {
    for (uint8_t c = 0; c < cols; c++) {
      uint8_t id = gridNodeId(r, c);
      if (id >= BFS_MAX_NODES) continue;
      bfs_state.nodes[id].id = id;
      snprintf(bfs_state.nodes[id].name, sizeof(bfs_state.nodes[id].name), "%d,%d", r, c);
      bfs_state.nodes[id].x_m = c * spacing_m;
      bfs_state.nodes[id].y_m = r * spacing_m;
      bfs_state.nodes[id].neighbor_count = 0;
    }
  }

  // Wall clearance: 0.25m from any wall/bottle segment (robot half-width + margin)
  const float clearance_m = 0.25f;

  // Add cardinal edges between adjacent grid nodes (skip if wall/bottle blocks)
  for (uint8_t r = 0; r < rows; r++) {
    for (uint8_t c = 0; c < cols; c++) {
      uint8_t id = gridNodeId(r, c);
      if (id >= BFS_MAX_NODES) continue;
      if (c + 1 < cols && !gridHasWall(r, c, r, c + 1) && !gridHasBottle(r, c, r, c + 1))
        bfsAddEdgeIfClear(id, gridNodeId(r, c + 1), spacing_m, clearance_m);
      if (c > 0 && !gridHasWall(r, c, r, c - 1) && !gridHasBottle(r, c, r, c - 1))
        bfsAddEdgeIfClear(id, gridNodeId(r, c - 1), spacing_m, clearance_m);
      if (r + 1 < rows && !gridHasWall(r, c, r + 1, c) && !gridHasBottle(r, c, r + 1, c))
        bfsAddEdgeIfClear(id, gridNodeId(r + 1, c), spacing_m, clearance_m);
      if (r > 0 && !gridHasWall(r, c, r - 1, c) && !gridHasBottle(r, c, r - 1, c))
        bfsAddEdgeIfClear(id, gridNodeId(r - 1, c), spacing_m, clearance_m);
    }
  }

  // --- Direct diagonal edges across each grid square ---
  float full_diag = spacing_m * 1.4142f;  // sqrt(2) * spacing
  for (uint8_t r = 0; r + 1 < rows; r++) {
    for (uint8_t c = 0; c + 1 < cols; c++) {
      uint8_t tl = gridNodeId(r, c), tr = gridNodeId(r, c + 1);
      uint8_t bl = gridNodeId(r + 1, c), br = gridNodeId(r + 1, c + 1);
      if (tl < BFS_MAX_NODES && br < BFS_MAX_NODES) {
        bfsAddEdgeIfClear(tl, br, full_diag, clearance_m);
        bfsAddEdgeIfClear(br, tl, full_diag, clearance_m);
      }
      if (tr < BFS_MAX_NODES && bl < BFS_MAX_NODES) {
        bfsAddEdgeIfClear(tr, bl, full_diag, clearance_m);
        bfsAddEdgeIfClear(bl, tr, full_diag, clearance_m);
      }
    }
  }

  // --- Center sub-nodes: one at the center of every grid square ---
  // Enables routing through square centers (0.25m from grid edges).
  float half_diag = spacing_m * 0.7071f;  // sqrt(2)/2 * spacing
  for (uint8_t r = 0; r + 1 < rows; r++) {
    for (uint8_t c = 0; c + 1 < cols; c++) {
      uint8_t cid = bfs_state.node_count;
      if (cid >= BFS_MAX_NODES) break;
      bfs_state.nodes[cid].id = cid;
      snprintf(bfs_state.nodes[cid].name, sizeof(bfs_state.nodes[cid].name), "c%d%d", r, c);
      bfs_state.nodes[cid].x_m = (c + 0.5f) * spacing_m;
      bfs_state.nodes[cid].y_m = (r + 0.5f) * spacing_m;
      bfs_state.nodes[cid].neighbor_count = 0;
      bfs_state.node_count++;
      bfs_sq_center[r][c] = cid;

      // Connect center to its 4 corner intersections (bidirectional, with wall clearance)
      uint8_t corners[4][2] = {
        {r, c}, {r, (uint8_t)(c+1)},
        {(uint8_t)(r+1), c}, {(uint8_t)(r+1), (uint8_t)(c+1)}
      };
      for (int k = 0; k < 4; k++) {
        uint8_t cr = corners[k][0], cc = corners[k][1];
        if (cr < rows && cc < cols) {
          uint8_t cornerId = gridNodeId(cr, cc);
          bfsAddEdgeIfClear(cid, cornerId, half_diag, clearance_m);
          bfsAddEdgeIfClear(cornerId, cid, half_diag, clearance_m);
        }
      }
    }
  }

  // --- Center-to-center edges for adjacent squares ---
  for (uint8_t r = 0; r + 1 < rows; r++) {
    for (uint8_t c = 0; c + 1 < cols; c++) {
      uint8_t cid = bfs_sq_center[r][c];
      if (cid == 0xFF) continue;
      // Right neighbor center: center(r, c+1)
      if (c + 2 < cols) {
        uint8_t rid = bfs_sq_center[r][c + 1];
        if (rid != 0xFF) {
          bfsAddEdgeIfClear(cid, rid, spacing_m, clearance_m);
          bfsAddEdgeIfClear(rid, cid, spacing_m, clearance_m);
        }
      }
      // Down neighbor center: center(r+1, c)
      if (r + 2 < rows) {
        uint8_t did = bfs_sq_center[r + 1][c];
        if (did != 0xFF) {
          bfsAddEdgeIfClear(cid, did, spacing_m, clearance_m);
          bfsAddEdgeIfClear(did, cid, spacing_m, clearance_m);
        }
      }
    }
  }

  // --- Virtual start node: midpoint of start border edge ---
  // Start node is on the boundary — skip clearance check for its connections
  // since the robot enters from outside the grid.
  uint8_t startVirtId = bfs_state.node_count;
  if (startVirtId < BFS_MAX_NODES) {
    uint8_t sr1 = grid_course.start_r1, sc1 = grid_course.start_c1;
    uint8_t sr2 = grid_course.start_r2, sc2 = grid_course.start_c2;
    bfs_state.nodes[startVirtId].id = startVirtId;
    snprintf(bfs_state.nodes[startVirtId].name, sizeof(bfs_state.nodes[startVirtId].name), "S");
    bfs_state.nodes[startVirtId].x_m = (sc1 + sc2) * 0.5f * spacing_m;
    bfs_state.nodes[startVirtId].y_m = (sr1 + sr2) * 0.5f * spacing_m;
    bfs_state.nodes[startVirtId].neighbor_count = 0;
    bfs_state.node_count++;
    // Connect to both edge endpoints (no clearance — start is on boundary)
    float half_edge = spacing_m * 0.5f;
    uint8_t ep1 = gridNodeId(sr1, sc1), ep2 = gridNodeId(sr2, sc2);
    if (ep1 < rows * cols) {
      bfsAddEdge(startVirtId, ep1, half_edge);
      bfsAddEdge(ep1, startVirtId, half_edge);
    }
    if (ep2 < rows * cols) {
      bfsAddEdge(startVirtId, ep2, half_edge);
      bfsAddEdge(ep2, startVirtId, half_edge);
    }
    // Connect to adjacent square center (most direct entry path)
    if (sr1 == sr2) {
      uint8_t sqRow = (sr1 == 0) ? 0 : (uint8_t)(sr1 - 1);
      uint8_t minC = (sc1 < sc2) ? sc1 : sc2;
      if (sqRow < rows - 1 && minC < cols - 1 && bfs_sq_center[sqRow][minC] != 0xFF) {
        float d = spacing_m * 0.5f;
        bfsAddEdge(startVirtId, bfs_sq_center[sqRow][minC], d);
        bfsAddEdge(bfs_sq_center[sqRow][minC], startVirtId, d);
      }
    } else {
      uint8_t sqCol = (sc1 == 0) ? 0 : (uint8_t)(sc1 - 1);
      uint8_t minR = (sr1 < sr2) ? sr1 : sr2;
      if (minR < rows - 1 && sqCol < cols - 1 && bfs_sq_center[minR][sqCol] != 0xFF) {
        float d = spacing_m * 0.5f;
        bfsAddEdge(startVirtId, bfs_sq_center[minR][sqCol], d);
        bfsAddEdge(bfs_sq_center[minR][sqCol], startVirtId, d);
      }
    }
    bfs_start_edge_node = startVirtId;
  } else {
    bfs_start_edge_node = 0;
  }

  // --- Virtual end node: reuse center sub-node of target square ---
  {
    uint8_t er = grid_course.end_row;
    uint8_t ec = grid_course.end_col;
    if (er < rows - 1 && ec < cols - 1 && bfs_sq_center[er][ec] != 0xFF) {
      bfs_end_center_node = bfs_sq_center[er][ec];
      snprintf(bfs_state.nodes[bfs_end_center_node].name,
               sizeof(bfs_state.nodes[bfs_end_center_node].name), "E%d,%d", er, ec);
    } else {
      uint8_t centerId = bfs_state.node_count;
      if (centerId < BFS_MAX_NODES) {
        bfs_state.nodes[centerId].id = centerId;
        snprintf(bfs_state.nodes[centerId].name, sizeof(bfs_state.nodes[centerId].name), "E%d,%d", er, ec);
        bfs_state.nodes[centerId].x_m = (ec + 0.5f) * spacing_m;
        bfs_state.nodes[centerId].y_m = (er + 0.5f) * spacing_m;
        bfs_state.nodes[centerId].neighbor_count = 0;
        bfs_state.node_count++;
        float hd = spacing_m * 0.7071f;
        uint8_t corners[4][2] = {{er, ec}, {er, (uint8_t)(ec+1)}, {(uint8_t)(er+1), ec}, {(uint8_t)(er+1), (uint8_t)(ec+1)}};
        for (int i = 0; i < 4; i++) {
          uint8_t cr2 = corners[i][0], cc2 = corners[i][1];
          if (cr2 < rows && cc2 < cols) {
            uint8_t cid2 = gridNodeId(cr2, cc2);
            bfsAddEdge(cid2, centerId, hd);
            bfsAddEdge(centerId, cid2, hd);
          }
        }
        bfs_end_center_node = centerId;
      } else {
        bfs_end_center_node = bfs_state.node_count - 1;
      }
    }
  }

  // --- Virtual gate nodes: reuse center sub-nodes of gate squares ---
  memset(bfs_gate_nodes, 0, sizeof(bfs_gate_nodes));
  for (uint8_t gi = 0; gi < grid_course.gate_count; gi++) {
    uint8_t gr = grid_course.gates[gi].row;
    uint8_t gc = grid_course.gates[gi].col;
    if (gr < rows - 1 && gc < cols - 1 && bfs_sq_center[gr][gc] != 0xFF) {
      bfs_gate_nodes[gi] = bfs_sq_center[gr][gc];
      snprintf(bfs_state.nodes[bfs_gate_nodes[gi]].name,
               sizeof(bfs_state.nodes[bfs_gate_nodes[gi]].name), "G%d", gi);
    } else {
      uint8_t gateVirtId = bfs_state.node_count;
      if (gateVirtId >= BFS_MAX_NODES) break;
      bfs_state.nodes[gateVirtId].id = gateVirtId;
      snprintf(bfs_state.nodes[gateVirtId].name, sizeof(bfs_state.nodes[gateVirtId].name), "G%d", gi);
      bfs_state.nodes[gateVirtId].x_m = (gc + 0.5f) * spacing_m;
      bfs_state.nodes[gateVirtId].y_m = (gr + 0.5f) * spacing_m;
      bfs_state.nodes[gateVirtId].neighbor_count = 0;
      bfs_state.node_count++;
      float hd = spacing_m * 0.7071f;
      uint8_t gcorners[4][2] = {{gr, gc}, {gr, (uint8_t)(gc+1)}, {(uint8_t)(gr+1), gc}, {(uint8_t)(gr+1), (uint8_t)(gc+1)}};
      for (int k = 0; k < 4; k++) {
        uint8_t cr2 = gcorners[k][0], cc2 = gcorners[k][1];
        if (cr2 < rows && cc2 < cols) {
          uint8_t cid2 = gridNodeId(cr2, cc2);
          bfsAddEdgeIfClear(cid2, gateVirtId, hd, clearance_m);
          bfsAddEdgeIfClear(gateVirtId, cid2, hd, clearance_m);
        }
      }
      bfs_gate_nodes[gi] = gateVirtId;
    }
  }

  bfs_state.current_node = bfs_start_edge_node;
  bfs_state.goal_node = bfs_end_center_node;
  bfs_state.path_valid = false;
  bfs_state.path_length = 0;
  bfs_state.path_index = 0;

  bfs_initialized = true;

  Serial.printf("BFS: grid %dx%d, spacing=%dcm, clearance=%.0fcm, start=edge(%d,%d)-(%d,%d) end=sq(%d,%d), "
                "walls=%d, bottles=%d, gates=%d, nodes=%d\n",
                rows, cols, grid_course.spacing_cm, (double)(clearance_m * 100),
                grid_course.start_r1, grid_course.start_c1,
                grid_course.start_r2, grid_course.start_c2,
                grid_course.end_row, grid_course.end_col,
                grid_course.wall_count, grid_course.bottle_count,
                grid_course.gate_count, bfs_state.node_count);
}

// Forward declaration (defined after bfsComputePath)
static bool bfsComputePath(uint8_t start_node, uint8_t goal_node);

static void bfsSimplifyPath();  // forward declaration

// Compute BFS path visiting all gates (if any) in optimal order.
// For 0 gates: simple BFS from start to end.
// For 1+ gates: try all permutations (up to MAX_GATES!) to find shortest total path.
// If last_gate_idx is set (>= 0), that gate is forced last; only remaining gates are permuted.
static bool bfsComputePathWithGates() {
  uint8_t startId = bfs_start_edge_node;   // Virtual start at edge midpoint
  uint8_t goalId = bfs_end_center_node;    // Virtual center of end square

  if (grid_course.gate_count == 0) {
    // Simple BFS
    return bfsComputePath(startId, goalId);
  }

  // Collect gate square center virtual nodes as waypoints to visit.
  uint8_t gate_nodes[MAX_GATES];
  uint8_t ng = grid_course.gate_count;
  for (uint8_t i = 0; i < ng; i++) {
    gate_nodes[i] = bfs_gate_nodes[i];
  }

  // For small gate counts, try all permutations
  if (ng > 6) ng = 6;  // Cap at 6! = 720 permutations

  // If a last gate is pinned, separate it out; only permute the rest
  // (2026 rules: gate order is not enforced, so always optimize freely)
  int8_t pinned_last = -1;

  // Build indices to permute (excluding pinned gate if any)
  uint8_t perm[MAX_GATES];
  uint8_t np = 0; // number of gates to permute
  for (uint8_t i = 0; i < ng; i++) {
    if ((int8_t)i != pinned_last) {
      perm[np++] = i;
    }
  }

  uint8_t best_order[MAX_GATES];
  float best_total_dist = 1e9f;
  bool found = false;

  // Evaluate current permutation of the non-pinned gates,
  // then append the pinned gate (if any) at the end.
  auto evalPerm = [&]() {
    float totalDist = 0.0f;
    uint8_t from = startId;
    // Visit permuted gates first
    for (uint8_t i = 0; i < np; i++) {
      if (!bfsComputePath(from, gate_nodes[perm[i]])) return;
      totalDist += bfs_state.path_dist_m;
      from = gate_nodes[perm[i]];
    }
    // Visit pinned last gate
    if (pinned_last >= 0) {
      if (!bfsComputePath(from, gate_nodes[pinned_last])) return;
      totalDist += bfs_state.path_dist_m;
      from = gate_nodes[pinned_last];
    }
    // Final leg to goal
    if (!bfsComputePath(from, goalId)) return;
    totalDist += bfs_state.path_dist_m;
    if (totalDist < best_total_dist) {
      best_total_dist = totalDist;
      // Store full order: permuted gates + pinned gate
      for (uint8_t i = 0; i < np; i++) best_order[i] = perm[i];
      if (pinned_last >= 0) best_order[np] = (uint8_t)pinned_last;
      found = true;
    }
  };

  // Try all permutations using Heap's algorithm iteratively
  uint8_t c[MAX_GATES];
  memset(c, 0, sizeof(c));

  evalPerm();
  uint8_t i = 0;
  while (i < np) {
    if (c[i] < i) {
      if (i % 2 == 0) { uint8_t t = perm[0]; perm[0] = perm[i]; perm[i] = t; }
      else             { uint8_t t = perm[c[i]]; perm[c[i]] = perm[i]; perm[i] = t; }
      evalPerm();
      c[i]++;
      i = 0;
    } else {
      c[i] = 0;
      i++;
    }
  }

  if (!found) return false;

  // Reconstruct the full path using best order
  // Build concatenated path: start → gate1 → gate2 → ... → goal
  uint8_t full_path[BFS_PATH_MAX_LENGTH];
  uint16_t full_len = 0;
  uint8_t from = startId;

  for (uint8_t i = 0; i < ng; i++) {
    bfsComputePath(from, gate_nodes[best_order[i]]);
    // Append path (skip first node if not the first segment to avoid duplicates)
    uint8_t skip = (i == 0) ? 0 : 1;
    for (uint16_t j = skip; j < bfs_state.path_length && full_len < BFS_PATH_MAX_LENGTH; j++) {
      full_path[full_len++] = bfs_state.path[j];
    }
    from = gate_nodes[best_order[i]];
  }
  // Final segment to goal
  bfsComputePath(from, goalId);
  for (uint16_t j = 1; j < bfs_state.path_length && full_len < BFS_PATH_MAX_LENGTH; j++) {
    full_path[full_len++] = bfs_state.path[j];
  }

  // Store the full path back
  memcpy(bfs_state.path, full_path, full_len);
  bfs_state.path_length = full_len;
  bfs_state.path_index = 0;
  bfs_state.path_valid = true;
  bfsSimplifyPath();
  return true;
}

// Dijkstra's algorithm to find shortest-distance path from start to goal.
// Uses the physical distances stored in each node's distances[] array.
static bool bfsComputePath(uint8_t start_node, uint8_t goal_node) {
  if (start_node >= BFS_MAX_NODES || goal_node >= BFS_MAX_NODES) return false;

  float dist[BFS_MAX_NODES];
  bool visited[BFS_MAX_NODES];
  uint8_t parent[BFS_MAX_NODES];

  for (uint8_t i = 0; i < BFS_MAX_NODES; i++) {
    dist[i] = 1e9f;
    visited[i] = false;
    parent[i] = 0xFF;
  }
  dist[start_node] = 0.0f;

  for (uint8_t iter = 0; iter < bfs_state.node_count; iter++) {
    // Find unvisited node with smallest distance
    uint8_t u = 0xFF;
    float best = 1e9f;
    for (uint8_t i = 0; i < bfs_state.node_count; i++) {
      if (!visited[i] && dist[i] < best) {
        best = dist[i];
        u = i;
      }
    }
    if (u == 0xFF) break;  // no reachable unvisited nodes
    visited[u] = true;

    if (u == goal_node) break;  // found shortest path to goal

    // Relax neighbors
    BfsNode* node_ptr = &bfs_state.nodes[u];
    for (uint8_t i = 0; i < node_ptr->neighbor_count; i++) {
      uint8_t v = node_ptr->neighbors[i];
      float w = node_ptr->distances[i];
      if (w <= 0.0f) w = 0.001f;  // safety: avoid zero-weight edges
      float alt = dist[u] + w;
      if (alt < dist[v]) {
        dist[v] = alt;
        parent[v] = u;
      }
    }
  }

  if (!visited[goal_node] && dist[goal_node] >= 1e9f) return false;

  // Reconstruct path
  bfs_state.path_length = 0;
  uint8_t node = goal_node;
  while (node != 0xFF && bfs_state.path_length < BFS_PATH_MAX_LENGTH) {
    bfs_state.path[bfs_state.path_length++] = node;
    node = parent[node];
  }

  // Reverse path (was built backwards)
  for (uint16_t i = 0; i < bfs_state.path_length / 2; i++) {
    uint8_t tmp = bfs_state.path[i];
    bfs_state.path[i] = bfs_state.path[bfs_state.path_length - 1 - i];
    bfs_state.path[bfs_state.path_length - 1 - i] = tmp;
  }

  bfs_state.path_index = 0;
  bfs_state.path_valid = true;
  bfs_state.path_dist_m = dist[goal_node];
  return true;
}

// Check if a node is a gate or end center that must be kept as a waypoint.
static bool bfsIsImportantNode(uint8_t nodeId) {
  if (nodeId == bfs_end_center_node) return true;
  for (uint8_t gi = 0; gi < grid_course.gate_count; gi++) {
    if (nodeId == bfs_gate_nodes[gi]) return true;
  }
  return false;
}

// Remove collinear intermediate waypoints so the robot drives straight through
// instead of stopping at every grid intersection on a straight line.
// Gate centers and end center are always kept (robot must physically visit them).
static void bfsSimplifyPath() {
  if (!bfs_state.path_valid || bfs_state.path_length < 3) return;

  uint8_t simplified[BFS_PATH_MAX_LENGTH];
  uint16_t slen = 0;

  simplified[slen++] = bfs_state.path[0];  // always keep start

  for (uint16_t i = 1; i < bfs_state.path_length - 1; i++) {
    const uint8_t prev = simplified[slen - 1];
    const uint8_t cur  = bfs_state.path[i];
    const uint8_t next = bfs_state.path[i + 1];

    // Always keep gate/end centers — robot must drive into these
    if (bfsIsImportantNode(cur)) {
      simplified[slen++] = cur;
      continue;
    }

    // Check collinearity: cross product of (cur-prev) x (next-prev) ≈ 0
    float ax = bfs_state.nodes[cur].x_m  - bfs_state.nodes[prev].x_m;
    float ay = bfs_state.nodes[cur].y_m  - bfs_state.nodes[prev].y_m;
    float bx = bfs_state.nodes[next].x_m - bfs_state.nodes[prev].x_m;
    float by = bfs_state.nodes[next].y_m - bfs_state.nodes[prev].y_m;
    float cross = ax * by - ay * bx;

    if (fabsf(cross) > 0.001f) {
      // Not collinear — keep this waypoint (direction change needed)
      simplified[slen++] = cur;
    } else {
      // Collinear — only skip if the direct prev→next line is clear of walls
      if (!bfsEdgeClearOfWalls(bfs_state.nodes[prev].x_m, bfs_state.nodes[prev].y_m,
                               bfs_state.nodes[next].x_m, bfs_state.nodes[next].y_m, 0.0f)) {
        simplified[slen++] = cur;  // wall in the way, keep waypoint
      }
      // else: clear path, skip this intermediate node
    }
  }

  simplified[slen++] = bfs_state.path[bfs_state.path_length - 1];  // always keep goal

  if (slen < bfs_state.path_length) {
    Serial.printf("BFS: simplified path %d -> %d waypoints\n",
                  bfs_state.path_length, slen);
    memcpy(bfs_state.path, simplified, slen);
    bfs_state.path_length = slen;
  }
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

// ========================================================================
// Convert manual waypoints into bfs_state path format for the runner.
// This populates bfs_state.nodes[] and path[] so the existing BFS_PHASE_*
// run execution code works unchanged, WITHOUT any route-solving algorithm.
// ========================================================================
static bool manualWaypointsToPath() {
  if (manual_wp_count == 0) return false;

  memset(&bfs_state, 0, sizeof(bfs_state));

  // Node 0: start position (border edge midpoint, matches current grid)
  float spacing_m = (float)grid_course.spacing_cm * 0.01f;
  float sx = 0.5f * ((float)grid_course.start_c1 + (float)grid_course.start_c2) * spacing_m;
  float sy = 0.5f * ((float)grid_course.start_r1 + (float)grid_course.start_r2) * spacing_m;
  bfs_state.nodes[0].id = 0;
  snprintf(bfs_state.nodes[0].name, sizeof(bfs_state.nodes[0].name), "S");
  bfs_state.nodes[0].x_m = sx;
  bfs_state.nodes[0].y_m = sy;

  // Nodes 1..N: manual waypoints on 25cm grid
  for (uint8_t i = 0; i < manual_wp_count; i++) {
    uint8_t nid = i + 1;
    bfs_state.nodes[nid].id = nid;
    snprintf(bfs_state.nodes[nid].name, sizeof(bfs_state.nodes[nid].name), "W%d", (int)i);
    bfs_state.nodes[nid].x_m = (float)manual_wp_col[i] * ((float)MANUAL_GRID_SPACING_CM * 0.01f);
    bfs_state.nodes[nid].y_m = (float)manual_wp_row[i] * ((float)MANUAL_GRID_SPACING_CM * 0.01f);
  }

  bfs_state.node_count = manual_wp_count + 1;

  // Build path: start → wp0 → wp1 → ... → wpN-1
  bfs_state.path[0] = 0; // start node
  for (uint8_t i = 0; i < manual_wp_count; i++) {
    bfs_state.path[i + 1] = i + 1;
  }
  bfs_state.path_length = manual_wp_count + 1;
  bfs_state.path_index = 0;
  bfs_state.current_node = 0;
  bfs_state.path_valid = true;

  // Compute total path distance
  bfs_state.path_dist_m = 0.0f;
  for (uint16_t si = 0; si + 1 < bfs_state.path_length; si++) {
    uint8_t a = bfs_state.path[si], b = bfs_state.path[si + 1];
    float dx = bfs_state.nodes[b].x_m - bfs_state.nodes[a].x_m;
    float dy = bfs_state.nodes[b].y_m - bfs_state.nodes[a].y_m;
    bfs_state.path_dist_m += sqrtf(dx * dx + dy * dy);
  }

  Serial.printf("MANUAL: %d waypoints, path_len=%d, total_dist=%.2fm\n",
                (int)manual_wp_count, (int)bfs_state.path_length,
                (double)bfs_state.path_dist_m);
  return true;
}

// Competition limits (update these when rules are confirmed)
// Allow slight edge adjustments below/above the nominal range.
#define RUN_DISTANCE_MIN_M 6.9f
#define RUN_DISTANCE_MAX_M 10.1f
#define RUN_TIME_MIN_S 10.0f
#define RUN_TIME_MAX_S 100.0f

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

  // Telemetry: left/right wheel distance (m) from encoders for 10Hz CSV
  float distL;
  float distR;
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
static bool runWarnTooFast = false;
// Indicates whether a stall cutoff has been latched during the run (prevents re-arming)
static bool stall_cutoff_latched = false;

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
      "t_ms,meters,forwardM,currentYaw,targetYaw,yawErrDeg,yawI,gz_dps,gzRaw_dps,gzBias_dps,actualRate,leftRate,rightRate,speedErr,basePWM,corrRaw,corrFiltered,corrOut,canPhase,turnTargetDeg,turnImuDeg,turnEncDeg,turnFusedDeg,braking,lPwm,rPwm,mL_err_dir,mL_err_pwm,mR_err_dir,mR_err_pwm,lateralM,remainingM,nomTargetRate,effTargetRate,targetRatePps,actualRatePps,speedI,steerI,encTrimPwm,loopDtMs,corrUsed,motorBiasPwm,satShift,imuReadUs,motorWriteUs,cmdL_dir,cmdR_dir,regL_dir,regL_pwm,regR_dir,regR_pwm,distL,distR");
  runLogStreamFile.flush();
  runLogStreamOpen = true;

  char line[96];
  snprintf(line, sizeof(line), "RUNLOG_STREAM open_ok id=%lu", (unsigned long)runMeta.id);
  startDebugLogLine(line);
}

static void runLogStreamAppend(const RunLogSample& s) {
  if (!runLogStreamOpen) return;
  if (!runLogStreamFile) return;

  char line[768];
  snprintf(line, sizeof(line),
           "%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%.4f,%.4f",
           (unsigned long)s.t_ms,
           (double)s.meters,
           (double)s.forwardM,
           (double)s.yawRelDeg,
           (double)s.targetYawDeg,
           (double)s.yawErrDeg,
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
           (unsigned)s.regR_pwm,
           (double)s.distL,
           (double)s.distR);

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
      "t_ms,meters,forwardM,currentYaw,targetYaw,yawErrDeg,yawI,gz_dps,gzRaw_dps,gzBias_dps,actualRate,leftRate,rightRate,speedErr,basePWM,corrRaw,corrFiltered,corrOut,canPhase,turnTargetDeg,turnImuDeg,turnEncDeg,turnFusedDeg,braking,lPwm,rPwm,mL_err_dir,mL_err_pwm,mR_err_dir,mR_err_pwm,lateralM,remainingM,nomTargetRate,effTargetRate,targetRatePps,actualRatePps,speedI,steerI,encTrimPwm,loopDtMs,corrUsed,motorBiasPwm,satShift,imuReadUs,motorWriteUs,cmdL_dir,cmdR_dir,regL_dir,regL_pwm,regR_dir,regR_pwm,distL,distR");
  for (uint16_t i = 0; i < runLogCount; i++) {
    const RunLogSample& s = runLog[i];
    Serial.printf(
      "%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%.4f,%.4f\n",
                  (unsigned long)s.t_ms,
                  (double)s.meters,
                  (double)s.forwardM,
                  (double)s.yawRelDeg,
                  (double)s.targetYawDeg,
                  (double)s.yawErrDeg,
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
                  (unsigned)s.regR_pwm,
                  (double)s.distL,
                  (double)s.distR);
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
      "t_ms,meters,forwardM,currentYaw,targetYaw,yawErrDeg,yawI,gz_dps,gzRaw_dps,gzBias_dps,actualRate,leftRate,rightRate,speedErr,basePWM,corrRaw,corrFiltered,corrOut,canPhase,turnTargetDeg,turnImuDeg,turnEncDeg,turnFusedDeg,braking,lPwm,rPwm,mL_err_dir,mL_err_pwm,mR_err_dir,mR_err_pwm,lateralM,remainingM,nomTargetRate,effTargetRate,targetRatePps,actualRatePps,speedI,steerI,encTrimPwm,loopDtMs,corrUsed,motorBiasPwm,satShift,imuReadUs,motorWriteUs,cmdL_dir,cmdR_dir,regL_dir,regL_pwm,regR_dir,regR_pwm,distL,distR");
    for (uint16_t i = 0; i < runLogCount; i++) {
      const RunLogSample& s = runLog[i];
      char line[768];
      snprintf(line, sizeof(line),
           "%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%.4f,%.4f",
               (unsigned long)s.t_ms,
               (double)s.meters,
               (double)s.forwardM,
               (double)s.yawRelDeg,
               (double)s.targetYawDeg,
               (double)s.yawErrDeg,
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
               (unsigned)s.regR_pwm,
               (double)s.distL,
               (double)s.distR);
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

  // Tail by scanning newline offsets — static buffer avoids heap fragmentation
  static size_t nlOff[2001];
  const size_t cap = (size_t)lastLines + 1;
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
  if (lastLines > 2000) lastLines = 2000;

  File f = SPIFFS.open("/runlog_last.csv", "r");
  if (!f) {
    Serial.println("RunLog: no saved file (/runlog_last.csv)");
    return;
  }

  // Find the starting offset of the last N lines by scanning once.
  // Static ring buffer avoids heap fragmentation.
  static size_t nlOff[2001];
  const size_t cap = (size_t)lastLines + 1;
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

static void handleSerialCommands() {
  // Non-blocking line reader for simple commands.
  // Commands: dump | dumpid N | tail [N] | listlogs | dump_ram | startdump |
  //           starttail [N] | status | format_spiffs | download_all | delete_all |
  //           ping | help
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
      } else if (strcmp(p, "partinfo") == 0) {
        esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
        while (it) {
          const esp_partition_t* pp = esp_partition_get(it);
          Serial.printf("PART: label=%s type=0x%02x subtype=0x%02x offset=0x%06x size=0x%06x\n",
                        pp->label, pp->type, pp->subtype, (unsigned)pp->address, (unsigned)pp->size);
          it = esp_partition_next(it);
        }
        esp_partition_iterator_release(it);
        Serial.printf("FFat_ok=%d\n", spiffs_ok ? 1 : 0);
      } else if (strcmp(p, "format_spiffs") == 0) {
        Serial.println("FORMAT SPIFFS: erasing filesystem...");
        SPIFFS.end();  // unmount first so format works from any state
        const bool ok = SPIFFS.format();
        spiffs_ok = SPIFFS.begin(true);
        Serial.printf("FORMAT SPIFFS: %s  mount=%s\n", ok ? "OK" : "FAIL", spiffs_ok ? "OK" : "FAIL");
      } else if (strcmp(p, "download_all") == 0) {
        // Dump every runlog file for the Python tool to capture
        if (!spiffs_ok) {
          Serial.println("DOWNLOAD_ALL: SPIFFS not available");
        } else {
          Serial.println("DOWNLOAD_ALL_BEGIN");
          File root = SPIFFS.open("/");
          File f = root.openNextFile();
          while (f) {
            const char* nm = f.name();
            if (nm && strstr(nm, "runlog") && strstr(nm, ".csv")) {
              Serial.printf("FILE_BEGIN %s %u\n", nm, (unsigned)f.size());
              while (f.available()) {
                char buf512[512];
                const int n2 = f.read((uint8_t*)buf512, sizeof(buf512) - 1);
                if (n2 <= 0) break;
                buf512[n2] = 0;
                Serial.print(buf512);
              }
              Serial.println();
              Serial.printf("FILE_END %s\n", nm);
            }
            f.close();
            f = root.openNextFile();
          }
          root.close();
          Serial.println("DOWNLOAD_ALL_END");
        }
      } else if (strcmp(p, "delete_all") == 0) {
        // Delete all runlog CSV files from SPIFFS
        if (!spiffs_ok) {
          Serial.println("DELETE_ALL: SPIFFS not available");
        } else {
          Serial.println("DELETE_ALL_BEGIN");
          // Collect filenames first (can't delete while iterating)
          char paths[32][32];
          int count = 0;
          File root = SPIFFS.open("/");
          File f = root.openNextFile();
          while (f && count < 32) {
            const char* nm = f.name();
            if (nm && strstr(nm, "runlog") && strstr(nm, ".csv")) {
              snprintf(paths[count], sizeof(paths[count]), "/%s", nm);
              // Handle names that already have leading /
              if (nm[0] == '/') snprintf(paths[count], sizeof(paths[count]), "%s", nm);
              count++;
            }
            f.close();
            f = root.openNextFile();
          }
          root.close();
          for (int i2 = 0; i2 < count; i2++) {
            const bool ok2 = SPIFFS.remove(paths[i2]);
            Serial.printf("DELETE %s %s\n", paths[i2], ok2 ? "OK" : "FAIL");
          }
          Serial.printf("DELETE_ALL_END count=%d\n", count);
        }
      } else if (strcmp(p, "ping") == 0) {
        Serial.println("PONG");
      } else if (strcmp(p, "help") == 0 || strcmp(p, "?") == 0) {
        Serial.println("Commands: help | status | dump | dumpid <id> | listlogs | dellog <id> | setepoch <unix_s> | tail [N] | dump_ram | startdump | starttail [N] | format_spiffs | download_all | delete_all | ping");
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
// Encoder-calibration UI: start counts when entering enc-cal screen
static int32_t encCal_start_L = 0;
static int32_t encCal_start_R = 0;

// Motor control
uint8_t selected_motor = 0;  // 0=left, 1=right, 2=FIND MIN
// Per-motor direction (enables true pivot turns)
// 0=reverse, 1=forward
uint8_t motor_l_direction = 1;
uint8_t motor_r_direction = 1;
int16_t motor_command = 0;   // -255..255 (HW test screen; sign = direction, 0 = stop)
uint8_t motor_enabled = 0;   // 0=stopped, 1=running (derived from motor_command)
// motor_l_speed and motor_r_speed moved to top-level

// RUN screen fixed PWM
#define RUN_PWM 255

// IMU attitude (degrees) and raw acceleration (g)
static bool imu_ok = false;
static float imu_pitch = 0.0f;
static float imu_roll = 0.0f;
static float imu_yaw = 0.0f;
// imu_ax_g and imu_ay_g moved to top-level
static float imu_az_g = 0.0f;
static float imu_gz_dps = 0.0f;
static float imu_gz_raw_dps = 0.0f;
static float imu_gz_still_err_dps = 0.0f;
// Missing gyro bias variable (declared/used in several places)
static float imu_gz_bias_dps = 0.0f;
// Timestamp for gyro yaw integration (file-scope so helpers can use it)
static uint32_t lastYawUs = 0;
// Forward accel axes (used by IMU+encoder fusion)
static float imu_ax_g = 0.0f;
static float imu_ay_g = 0.0f;

// Stall detection latched flag moved to top-level (stall_cutoff_latched)
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

// --- Cross-file symbols & macros (minimal declarations to satisfy references) ---
#ifndef ENCODER_L_ADDR
#define ENCODER_L_ADDR 0x58
#define ENCODER_R_ADDR 0x59
#endif

#ifndef MOTOR_L_ADDR
#define MOTOR_L_ADDR 0x21
#define MOTOR_R_ADDR 0x20
#endif

#ifndef INVERT_LEFT_ENCODER_COUNT
#define INVERT_LEFT_ENCODER_COUNT 0
#endif
#ifndef INVERT_RIGHT_ENCODER_COUNT
#define INVERT_RIGHT_ENCODER_COUNT 0
#endif

#ifndef ENCODER_PIN_A
#define ENCODER_PIN_A 41
#define ENCODER_PIN_B 40
#endif

#ifndef DIAL_DIRECTION
#define DIAL_DIRECTION -1
#endif

// Pulses-per-meter globals (ensure available to this compilation unit)
volatile float pulsesPerMeterL = 895.0f;
volatile float pulsesPerMeterR = 865.0f;
volatile float pulsesPerMeter  = 880.0f;

// Motor speed variables may be defined in another translation unit; declare extern.
extern volatile int motor_l_speed;
extern volatile int motor_r_speed;

// Forward declarations for functions defined later but used earlier
void stopAllMotors();
static void brakeToStop(uint8_t brakePwm, unsigned long maxBrakeMs);
void applyMotorOutputs();
static void readWheelEncodersForce();
void readWheelEncoders();
static void calibrateImuGyroBias();

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
  ui.drawString("BLUE VALLEY", 120, 60);
  ui.drawString("NORTH", 120, 90);
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(2);
  ui.drawString("Science Olympiad", 120, 125);
  ui.drawString("Robot Tour. 2025-26", 120, 145);

  // --- Duck mascot ---
  {
    const int dx = 120, dy = 190; // center of duck
    // Body (yellow ellipse)
    ui.fillEllipse(dx, dy + 10, 10, 7, TFT_YELLOW);
    // Head (smaller circle)
    ui.fillCircle(dx + 9, dy + 2, 6, TFT_YELLOW);
    // Eye
    ui.fillCircle(dx + 11, dy, 1, TFT_BLACK);
    // Beak (orange triangle)
    ui.fillTriangle(dx + 15, dy + 2,
                    dx + 15, dy + 5,
                    dx + 20, dy + 3, TFT_ORANGE);
    // Wing
    ui.fillEllipse(dx - 2, dy + 10, 5, 4, 0xFD60);
    // Feet
    ui.drawLine(dx - 4, dy + 17, dx - 7, dy + 20, TFT_ORANGE);
    ui.drawLine(dx + 2, dy + 17, dx - 1, dy + 20, TFT_ORANGE);
  }

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

// Two-stage pre-start calibration driven from SCREEN_CALIBRATE / runImuCalibration().
// Stage 1: 1.5s hands-off wait. Stage 2: 1000 gyro Z samples, then average.
static bool imuCal_running = false;
static bool imuCal_done = false;
static unsigned long imuCal_stage_start_ms = 0;
static int imuCal_stage = 0;   // 0=idle, 1=hands-off wait, 2=sampling, 3=done
static int imuCal_samples = 0;
static double imuCal_sumGz = 0.0;
// Legacy/UI-facing calibration flags (some screens reference these names)
static bool imu_calibrating = false;
static int imu_cal_progress = 0;

void runImuCalibration() {
  const unsigned long now = millis();

  if (!imuCal_running) {
    imuCal_running = true;
    imuCal_done = false;
    imuCal_stage = 1;
    imuCal_stage_start_ms = now;
    imuCal_samples = 0;
    imuCal_sumGz = 0.0;
    imu_calibrating = true;
    imu_cal_progress = 0;
  }

  if (!imu_present) {
    imuCal_done = true;
    imuCal_running = false;
    imu_calibrating = false;
    imu_cal_progress = 100;
    return;
  }

  if (imuCal_stage == 1) {
    // Stage 1: hands-off wait (STAY STILL) — 0.5s is enough with I2C at 100kHz.
    unsigned long elapsed = now - imuCal_stage_start_ms;
    imu_cal_progress = (int)((elapsed * 15u) / 500u); // 0-15% during stage 1
    if (imu_cal_progress > 15) imu_cal_progress = 15;
    if ((now - imuCal_stage_start_ms) >= 500u) {
      imuCal_stage = 2;
      imuCal_stage_start_ms = now;
    }
  } else if (imuCal_stage == 2) {
    // Stage 2: accumulate 500 gyro Z samples (batch 8 per loop call for speed).
    if (imuCal_samples < 500) {
      for (int batch = 0; batch < 8 && imuCal_samples < 500; batch++) {
        float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
        float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps_local = 0.0f;
        float temp_c = 0.0f;
        const bool ok = imu6886Read(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps_local, &temp_c);
        if (ok) {
          imuCal_sumGz += (double)gz_dps_local;
          imuCal_samples += 1;
          imu_cal_progress = 15 + (imuCal_samples * 85) / 500; // 15-100%
          if (imu_cal_progress > 100) imu_cal_progress = 100;
        }
      }
    } else {
      // Finalize bias.
      const double avg = (imuCal_samples > 0) ? (imuCal_sumGz / (double)imuCal_samples) : 0.0;
      imu_gz_bias_dps = (float)avg;

      // Clamp to sane range.
      if (fabsf(imu_gz_bias_dps) > 20.0f) {
        Serial.printf("WARN: IMU gyro bias %.2f dps too large; forcing 0\n", (double)imu_gz_bias_dps);
        imu_gz_bias_dps = 0.0f;
      }
      if (imu_gz_bias_dps > IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = IMU_GZ_BIAS_CLAMP_DPS;
      if (imu_gz_bias_dps < -IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = -IMU_GZ_BIAS_CLAMP_DPS;

      imuCal_stage = 3;
      imuCal_done = true;
      imuCal_running = false;
      imu_calibrating = false;
      imu_cal_progress = 100;
      persistImuGyroBiasIfNeeded();
      Serial.printf("IMU CAL DONE bias=%.3f dps\n", (double)imu_gz_bias_dps);
    }
  }
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
  i2cGuard();  // guard delay after IMU burst read
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

// calibrateImuGyroBias() is implemented below after the UI draw helpers so
// it can update the calibration screen live during the blocking routine.

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
  SCREEN_IMU_CAL,
  SCREEN_SELF_TEST,
  SCREEN_SQUARE_TEST,
  SCREEN_STRAIGHT_TEST,
  SCREEN_TURN_TEST,
  SCREEN_BFS_NAV,
  SCREEN_BFS_RUN,
  SCREEN_CALIBRATE,
};

static ScreenState currentScreen = SCREEN_MAIN_MENU;
static ScreenState imuCal_next_screen = SCREEN_MAIN_MENU;

static const int NUM_MENU_ITEMS = 11;
static const char* menuNames[NUM_MENU_ITEMS] = {
  "NAV",
  "TIME",
  "BIAS TEST",
  "ENC CAL",
  "IMU CAL",
  "HW TEST",
  "SELF TEST",
  "SQ TEST",
  "STRAIGHT",
  "TURN 90",
  "CLR LOGS",
};

static int selectedMenuItem = 0;

// BFS Navigation state
static uint8_t bfs_start_node = 0;     // Starting waypoint (computed from grid)
static uint8_t bfs_goal_node = 0;      // Goal waypoint (computed from grid)
static bool bfs_run_active = false;    // Currently executing BFS path
static uint8_t bfs_current_target = 0; // Current target waypoint in path
static float bfs_target_distance = 0.0f;
static float bfs_target_angle = 0.0f;

// BFS Setup wizard steps (grid size is fixed at 4x5 / 200x250cm):
//  0 = Start position  (dial cycles border intersections)
//  1 = End position    (dial cycles square centers)
//  2 = Walls editor    (dial scrolls edges, click toggles wall)
//  3 = Bottles editor  (dial scrolls nodes, click toggles bottle)
//  4 = Gates editor    (dial scrolls edges, click toggles gate)
//  5 = Confirm / GO    (long-press to start run)
#define BFS_SETUP_STEP_COUNT 9
static int bfs_setup_step = 0;

// BFS Run state machine
enum BfsRunPhase {
  BFS_PHASE_IDLE = 0,
  BFS_PHASE_TURN,        // Pivoting to face next waypoint
  BFS_PHASE_DRIVE,       // Driving toward next waypoint
  BFS_PHASE_BRAKE,       // Brief electronic brake after drive/turn
  BFS_PHASE_SETTLE,      // Wait for motion/accel to settle before next turn/drive
  BFS_PHASE_ARRIVED,     // Reached a waypoint, advance path
  BFS_PHASE_DONE,        // Reached final goal
};
static BfsRunPhase bfs_run_phase = BFS_PHASE_IDLE;
static BfsRunPhase bfs_run_after_brake_phase = BFS_PHASE_IDLE;
static float bfs_run_start_yaw = 0.0f;       // IMU yaw at start of BFS run
static float bfs_run_initial_heading_deg = 0.0f; // Grid heading robot faces at start (from border)
static float bfs_run_target_yaw_deg = 0.0f;  // Target yaw for current segment
static float bfs_run_segment_dist_m = 0.0f;  // Distance for current segment
static float bfs_run_driven_m = 0.0f;        // Distance driven in current segment
static int32_t bfs_run_enc_start_L = 0;
static int32_t bfs_run_enc_start_R = 0;
static float bfs_run_total_driven_m = 0.0f;
static float bfs_run_total_path_dist_m = 0.0f;  // Sum of all segment distances for time scaling
static float bfs_run_effective_drive_time_s = 0.0f; // runTimeS minus estimated non-driving overhead
static unsigned long bfs_run_start_ms = 0;
static bool bfs_run_initialized = false;  // reset when starting new run, set on first BFS_RUN entry
static unsigned long bfs_run_drive_start_ms = 0;  // for ENC_MODE_TIMED fallback
static unsigned long bfs_run_brake_start_ms = 0;
static unsigned long bfs_run_settle_start_ms = 0;
static float bfs_run_cumulative_yaw_deg = 0.0f;  // Accumulated heading from start
static bool bfs_drive_pd_first = true;            // Reset PD state on first DRIVE iteration of each segment
static unsigned long bfs_turn_first_in_tol_ms = 0; // For turn settle hold (like turn test)
static unsigned long bfs_turn_start_ms = 0;         // When current TURN phase started

// BFS turn PID state (reset between segments)
static float bfs_turn_integral = 0.0f;
static float bfs_turn_prev_err = 0.0f;
static float bfs_turn_dFilt = 0.0f;
static uint32_t bfs_turn_pid_last_us = 0;

// BFS drive PID state (reset between segments)
static float bfs_drive_integral = 0.0f;
static float bfs_drive_prev_err = 0.0f;
static float bfs_drive_dFilt = 0.0f;
static uint32_t bfs_drive_pid_last_us = 0;
static unsigned long bfs_drive_steer_ramp_start_ms = 0;  // post-turn steering ramp
#define BFS_DRIVE_STEER_RAMP_MS 150  // suppress steering for this long after turn→drive

// BFS run tuning
#define BFS_TURN_PWM 130
#define BFS_TURN_TOLERANCE_DEG 5.0f
#define BFS_TURN_SETTLE_MS 300
#define BFS_TURN_CRAWL_DEG 15.0f
#define BFS_TURN_CRAWL_PWM 115
#define BFS_DRIVE_PWM 130
#define BFS_DRIVE_STEER_KP 0.5f
#define BFS_DRIVE_STEER_KI 1.0f
#define BFS_DRIVE_STEER_KD 0.06f
#define BFS_ARRIVE_TOLERANCE_M 0.08f

// Unified encoder odometry helper for BFS navigation.
// Uses left and right encoder distances (meters) and handles right-encoder sign.
static float getAverageDistanceMeters() {
  const int32_t dL = encoderL_count - bfs_run_enc_start_L;
  const int32_t dR = encoderR_count - bfs_run_enc_start_R;

  float distL_m = 0.0f;
  float distR_m = 0.0f;

  if (pulsesPerMeterL > 1.0f) {
    const float dLcorr = INVERT_LEFT_ENCODER_COUNT ? -(float)dL : (float)dL;
    distL_m = dLcorr / pulsesPerMeterL;
  }
  if (pulsesPerMeterR > 1.0f) {
    // Apply sign inversion for the right encoder so forward motion aligns with left.
    const float dRcorr = INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR;
    distR_m = dRcorr / pulsesPerMeterR;
  }

  switch (enc_mode) {
    case ENC_MODE_BOTH: {
      const float avg = 0.5f * (distL_m + distR_m);
      return avg;
    }
    case ENC_MODE_LEFT_ONLY:
      return fabsf(distL_m);
    case ENC_MODE_RIGHT_ONLY:
      return fabsf(distR_m);
    case ENC_MODE_TIMED: {
      const float elapsed_s = (millis() - bfs_run_drive_start_ms) / 1000.0f;
      return (elapsed_s > 0.0f) ? (elapsed_s * ENC_TIMED_SPEED_MPS) : 0.0f;
    }
    default:
      return 0.0f;
  }
}

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
// `pulsesPerMeterL`, `pulsesPerMeterR`, and `pulsesPerMeter` were moved
// to top-level globals so they are visible to helpers declared above.

static bool encCalEditingLeft = true;

static inline float clampPpm(float ppm) {
  // Hard floor prevents division-by-near-zero in speed calculations.
  if (ppm < 100.0f) ppm = 100.0f;
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

// Step size sequence for encoder tuning (shared by ENC CAL)
static const float tunePpmSteps[] = {0.1f, 0.5f, 1.0f, 5.0f, 10.0f, 50.0f};
static int tunePpmStepIndex = 0; // index into tunePpmSteps

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

  // Grid course settings
  gridCourseDefaults(); // set defaults first
  // Grid size is fixed (5x4 squares = 6 cols x 5 rows, 50cm spacing)
  // Only load user-configurable fields: start edge, end position, walls, bottles, gates
  grid_course.start_r1 = (uint8_t)prefs.getUChar("gc_sr1", grid_course.start_r1);
  grid_course.start_c1 = (uint8_t)prefs.getUChar("gc_sc1", grid_course.start_c1);
  grid_course.start_r2 = (uint8_t)prefs.getUChar("gc_sr2", grid_course.start_r2);
  grid_course.start_c2 = (uint8_t)prefs.getUChar("gc_sc2", grid_course.start_c2);
  grid_course.end_col = (uint8_t)prefs.getUChar("gc_ec", grid_course.end_col);
  grid_course.end_row = (uint8_t)prefs.getUChar("gc_er", grid_course.end_row);
  // Validate start edge is on border; if invalid, reset to default
  {
    bool valid = false;
    uint16_t bec = gridBorderEdgeCount();
    for (uint16_t i = 0; i < bec; i++) {
      uint8_t r1, c1, r2, c2;
      gridBorderEdgeFromIndex(i, r1, c1, r2, c2);
      if (r1 == grid_course.start_r1 && c1 == grid_course.start_c1 &&
          r2 == grid_course.start_r2 && c2 == grid_course.start_c2) {
        grid_start_cursor = i;
        valid = true;
        break;
      }
    }
    if (!valid) {
      gridBorderEdgeFromIndex(0, grid_course.start_r1, grid_course.start_c1,
                               grid_course.start_r2, grid_course.start_c2);
      grid_start_cursor = 0;
    }
  }
  // Clamp end to square range (0 to cols-2, 0 to rows-2)
  if (grid_course.end_col >= grid_course.cols - 1) grid_course.end_col = grid_course.cols - 2;
  if (grid_course.end_row >= grid_course.rows - 1) grid_course.end_row = grid_course.rows - 2;
  grid_end_cursor = gridSquareIndex(grid_course.end_row, grid_course.end_col);
  // Load walls (packed as 4 bytes each: r1,c1,r2,c2)
  grid_course.wall_count = (uint8_t)prefs.getUChar("gc_wn", 0);
  if (grid_course.wall_count > MAX_WALLS) grid_course.wall_count = 0;
  if (grid_course.wall_count > 0) {
    prefs.getBytes("gc_wd", grid_course.walls, grid_course.wall_count * sizeof(GridWall));
  }
  // Load bottles (packed as 4 bytes each: r1,c1,r2,c2 — edge-based)
  grid_course.bottle_count = (uint8_t)prefs.getUChar("gc_bn", 0);
  if (grid_course.bottle_count > MAX_BOTTLES) grid_course.bottle_count = 0;
  if (grid_course.bottle_count > 0) {
    prefs.getBytes("gc_bd", grid_course.bottles, grid_course.bottle_count * sizeof(GridBottle));
  }
  // Load gates (packed as 2 bytes each: row,col — square-based)
  grid_course.gate_count = (uint8_t)prefs.getUChar("gc_gn", 0);
  if (grid_course.gate_count > MAX_GATES) grid_course.gate_count = 0;
  if (grid_course.gate_count > 0) {
    prefs.getBytes("gc_gd", grid_course.gates, grid_course.gate_count * sizeof(GridGate));
  }
  // Last gate index (-1 = auto, 0+ = force last)
  grid_course.last_gate_idx = (int8_t)prefs.getChar("gc_lgi", -1);
  if (grid_course.last_gate_idx >= grid_course.gate_count) grid_course.last_gate_idx = -1;
  // BFS start/goal determined at graph build time (virtual nodes)
  bfs_start_node = 0;
  bfs_goal_node = 0;

  // Load manual waypoints
  manual_wp_count = (uint8_t)prefs.getUChar("mw_n", 0);
  if (manual_wp_count > MANUAL_WP_MAX) manual_wp_count = 0;
  if (manual_wp_count > 0) {
    prefs.getBytes("mw_c", manual_wp_col, manual_wp_count);
    prefs.getBytes("mw_r", manual_wp_row, manual_wp_count);
  }

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

  // Grid course settings (cols/rows/spacing are fixed, not saved)
  prefs.putUChar("gc_sr1", grid_course.start_r1);
  prefs.putUChar("gc_sc1", grid_course.start_c1);
  prefs.putUChar("gc_sr2", grid_course.start_r2);
  prefs.putUChar("gc_sc2", grid_course.start_c2);
  prefs.putUChar("gc_ec", grid_course.end_col);
  prefs.putUChar("gc_er", grid_course.end_row);
  // Save walls
  prefs.putUChar("gc_wn", grid_course.wall_count);
  if (grid_course.wall_count > 0) {
    prefs.putBytes("gc_wd", grid_course.walls, grid_course.wall_count * sizeof(GridWall));
  }
  // Save bottles
  prefs.putUChar("gc_bn", grid_course.bottle_count);
  if (grid_course.bottle_count > 0) {
    prefs.putBytes("gc_bd", grid_course.bottles, grid_course.bottle_count * sizeof(GridBottle));
  }
  // Save gates
  prefs.putUChar("gc_gn", grid_course.gate_count);
  if (grid_course.gate_count > 0) {
    prefs.putBytes("gc_gd", grid_course.gates, grid_course.gate_count * sizeof(GridGate));
  }
  // Save last gate index
  prefs.putChar("gc_lgi", grid_course.last_gate_idx);

  // Save manual waypoints
  prefs.putUChar("mw_n", manual_wp_count);
  if (manual_wp_count > 0) {
    prefs.putBytes("mw_c", manual_wp_col, manual_wp_count);
    prefs.putBytes("mw_r", manual_wp_row, manual_wp_count);
  }
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
#define RUN_MIN_PWM 170.0f

// Test mode: lock RUN PWM (bypasses speed PI). Useful for heading tuning and diagnosing power/brownout.
// NOTE: v73 phased navigation uses a distance-based S-curve profile instead.
#define RUN_LOCK_PWM_ENABLE 0
#define RUN_LOCK_PWM_VALUE 200.0f

// Distance-based speed profile (S-curve) to reduce wheel slip and improve repeatability.
// Applied per drive phase using down-course distance (forwardM).
#define RUN_SPEED_PROFILE_ENABLE 0
#define RUN_SPEED_PROFILE_MAX_PWM 255.0f
#define RUN_SPEED_PROFILE_MIN_PWM 170.0f
#define RUN_SPEED_PROFILE_ACCEL_M 0.35f
#define RUN_SPEED_PROFILE_DECEL_M 1.00f

// Extra kick at the beginning of each run to break static friction.
// Prevents early NO_MOTION stop if the car doesn't start rolling.
#define RUN_SPEED_PROFILE_START_BOOST_MS 400
#define RUN_SPEED_PROFILE_START_BOOST_PWM 170.0f

// If the car still hasn't moved (encoder pulses) shortly after start,
// keep applying an extra kick for a bit longer.
#define RUN_START_ASSIST_ENABLE 1
#define RUN_START_ASSIST_MS 1200
#define RUN_START_ASSIST_PWM 90.0f
#define RUN_START_ASSIST_MAX_PULSES 60

// No-motion ramp: If no encoder movement, ramp to full 255 PWM over this many ms.
#define NO_MOTION_RAMP_TO_MAX_MS 3000u
#define NO_MOTION_RAMP_DELAY_MS 300u
#define NO_MOTION_RAMP_START_PWM 200.0f
#define NO_MOTION_RAMP_MAX_PWM 255.0f

static float calcNoMotionRampPwm(float currentPwm, uint32_t noMotionMs) {
  if (noMotionMs < NO_MOTION_RAMP_DELAY_MS) {
    return currentPwm;
  }
  uint32_t rampMs = noMotionMs - NO_MOTION_RAMP_DELAY_MS;
  float frac = (float)rampMs / (float)NO_MOTION_RAMP_TO_MAX_MS;
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  float target = NO_MOTION_RAMP_START_PWM + frac * (NO_MOTION_RAMP_MAX_PWM - NO_MOTION_RAMP_START_PWM);
  if (target > NO_MOTION_RAMP_MAX_PWM) target = NO_MOTION_RAMP_MAX_PWM;
  if (target < currentPwm) {
    return currentPwm;
  }
  return target;
}

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
#define DIST_STOP_BRAKE_MS 300
#define DIST_STOP_BRAKE_PWM 145

// Braking / approach tuning near the stop distance.
// As we get within BRAKE_ZONE_M of the target distance, reduce the target speed
// and cap PWM, but still enforce a minimum PWM so we don't stall.
#define BRAKE_ZONE_M 0.60f
#define BRAKE_MIN_SPEED_FACTOR 0.30f
// Cap PWM in the brake zone to prevent coasting past the target.
#define BRAKE_MAX_PWM 155.0f
#define BRAKE_MIN_PWM 90.0f

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
  float r = fmodf(deg + 180.0f, 360.0f);
  return (r < 0.0f) ? r + 180.0f : r - 180.0f;
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

// Timestamp of last pivot command (ms)
static unsigned long lastPivotCmdMs = 0;

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
  // Record last pivot command time so callers can apply post-pivot settle/ramp
  // without requiring synchronous delays here.
  lastPivotCmdMs = millis();
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
  if (next == SCREEN_RUN || next == SCREEN_BFS_RUN) {
    if (imu_present && imu_control_enabled) {
      // Give the car a moment to settle (users often enter RUN while still handling it).
      delay(500);
      calibrateImuGyroBias(); // Declaration is in Robot.cpp
      showImuReadyScreen(imu_gz_bias_dps);
      delay(250);
    }
  }

  // Dedicated IMU CAL screen: run full calibration on entry
  if (next == SCREEN_IMU_CAL) {
    if (imu_present) {
      delay(500);
      calibrateImuGyroBias(); // Declaration is in Robot.cpp
    }
  }

  // Reset self-test to idle on entry
  if (next == SCREEN_SELF_TEST) {
    extern void selfTestReset();
    selfTestReset();
  }

  // Reset square test to idle on entry
  if (next == SCREEN_SQUARE_TEST) {
    extern void sqTestResetExtern();
    sqTestResetExtern();
  }

  // Reset straight test to idle on entry
  if (next == SCREEN_STRAIGHT_TEST) {
    // strTestReset() defined later; use a forward-declared wrapper
    extern void strTestResetExtern();
    strTestResetExtern();
  }

  // Reset turn test to idle on entry
  if (next == SCREEN_TURN_TEST) {
    extern void trnTestResetExtern();
    trnTestResetExtern();
  }

  // When entering encoder-calibration screen, capture encoder baselines
  if (next == SCREEN_SET_ENC_CAL) {
    // Force a fresh read so displayed distances start from zero
    readWheelEncodersForce();
    encCal_start_L = encoderL_count;
    encCal_start_R = encoderR_count;
  }

  // NOTE: AUTO ENC feature removed — tuning handled in ENC CAL screen.

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

  // Ensure we use the latest persisted encoder calibration if available.
  if (prefs_ok) loadPersistedSettings();

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

  // Closed-loop braking: stops each wheel when its encoder reaches zero velocity.
  if (DIST_STOP_BRAKE_ENABLE && lastRunStopReason && strcmp(lastRunStopReason, "DIST") == 0) {
    brakeToStop((uint8_t)DIST_STOP_BRAKE_PWM, (unsigned long)DIST_STOP_BRAKE_MS + 200);
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

// Helper: try reading one encoder with retries. Returns true on success.
static bool readOneEncoder(uint8_t addr, int32_t &out_count) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x00);
    uint8_t err = Wire.endTransmission(true);  // full stop for cleaner bus release
    if (err == 0) {
      delayMicroseconds(50);
      if (Wire.requestFrom((uint8_t)addr, (size_t)4) == 4 && Wire.available() >= 4) {
        int32_t value = 0;
        value |= (int32_t)Wire.read() << 0;
        value |= (int32_t)Wire.read() << 8;
        value |= (int32_t)Wire.read() << 16;
        value |= (int32_t)Wire.read() << 24;
        out_count = value;
        return true;
      }
    }
    // Failed — flush any leftover bytes and retry with increasing delay
    while (Wire.available()) Wire.read();
    delayMicroseconds(300 * (attempt + 1));
  }
  return false;
}

static void readWheelEncodersInternal(bool force) {
  // Throttle reads to keep I2C stable and avoid spamming errors.
  static unsigned long last_read_ms = 0;
  const unsigned long now = millis();
  if (!force && (now - last_read_ms < (unsigned long)control_period_ms)) return;
  last_read_ms = now;

  static uint8_t consecutive_enc_fails = 0;

  enc_read_attempts++;
  encoderL_found = false;
  encoderR_found = false;

  encoderL_found = readOneEncoder(ENCODER_L_ADDR, encoderL_count);
  i2cGuard();
  encoderR_found = readOneEncoder(ENCODER_R_ADDR, encoderR_count);
  i2cGuard();

  if (!encoderL_found || !encoderR_found) {
    enc_read_failures++;
    consecutive_enc_fails++;
    // If we've failed 5+ times in a row, do a full bus recovery
    if (consecutive_enc_fails >= 5) {
      Serial.printf("ENC_I2C: %u consecutive fails — bus recovery!\n", consecutive_enc_fails);
      i2cBusRecovery();
      consecutive_enc_fails = 0;
      // Retry once more after recovery
      delayMicroseconds(500);
      if (!encoderL_found) encoderL_found = readOneEncoder(ENCODER_L_ADDR, encoderL_count);
      i2cGuard();
      if (!encoderR_found) encoderR_found = readOneEncoder(ENCODER_R_ADDR, encoderR_count);
      if (encoderL_found || encoderR_found) enc_read_recoveries++;
    }
  } else {
    consecutive_enc_fails = 0;
  }
}

void driveMotor(uint8_t motor_addr, uint8_t direction, uint8_t speed) {
  // H-Bridge v1.1 protocol (matches the previously working backup code):
  // Register 0x00: Direction (0=STOP, 1=FORWARD, 2=BACKWARD)
  // Register 0x01: Speed PWM (0-255)
  // Per-motor minimum PWM to reliably move (from hardware testing):
  //   Left motor: 100    Right motor: 110

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
    // Use global RUN_MIN_PWM for consistency across all motor commands
    uint8_t minPwm = (uint8_t)RUN_MIN_PWM;
    pwm = (speed < minPwm) ? minPwm : speed;
  }

  uint8_t err_dir = 0;
  uint8_t err_pwm = 0;

  Wire.beginTransmission(motor_addr);
  Wire.write(0x00);
  Wire.write(reg_dir);
  err_dir = Wire.endTransmission(true);

  i2cGuard();  // settle between direction and speed writes

  Wire.beginTransmission(motor_addr);
  Wire.write(0x01);
  Wire.write(pwm);
  err_pwm = Wire.endTransmission(true);

  // Track I2C bus health
  if (err_dir != 0 || err_pwm != 0) {
    i2c_consecutive_errors++;
    i2c_total_errors++;
  } else {
    i2c_consecutive_errors = 0;
  }

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

  i2cGuard();  // guard delay after motor write
}

static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true) == 0;
}

static bool readReg8(uint8_t addr, uint8_t reg, uint8_t* out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated-start
  if (Wire.requestFrom((uint8_t)addr, (size_t)1) != 1) return false;
  if (!Wire.available()) return false;
  *out = Wire.read();
  return true;
}

static void updateMotorDiagnostics() {
  // Infer presence from last write errors (avoids extra i2cPing traffic)
  motorL_present = (motorL_err_dir == 0);
  motorR_present = (motorR_err_dir == 0);

  // Only do full readback when motors are idle (no active run)
  if (motor_l_speed == 0 && motor_r_speed == 0) {
    i2cGuard();
    if (i2cPing(MOTOR_L_ADDR)) {
      motorL_present = true;
      uint8_t v;
      if (readReg8(MOTOR_L_ADDR, 0x00, &v)) motorL_dir_rb = v;
      if (readReg8(MOTOR_L_ADDR, 0x01, &v)) motorL_spd_rb = v;
    } else {
      motorL_dir_rb = 0xFF;
      motorL_spd_rb = 0xFF;
    }

    i2cGuard();
    if (i2cPing(MOTOR_R_ADDR)) {
      motorR_present = true;
      uint8_t v;
      if (readReg8(MOTOR_R_ADDR, 0x00, &v)) motorR_dir_rb = v;
      if (readReg8(MOTOR_R_ADDR, 0x01, &v)) motorR_spd_rb = v;
    } else {
      motorR_dir_rb = 0xFF;
      motorR_spd_rb = 0xFF;
    }
    i2cGuard();
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

  i2cGuard();

  Wire.beginTransmission(motor_addr);
  Wire.write((uint8_t)0x01);
  Wire.write(speed);
  e1 = Wire.endTransmission(true);

  if (err_dir) *err_dir = e0;
  if (err_spd) *err_spd = e1;

  i2cGuard();
}

void stopAllMotors() {
  motor_l_direction = 0;
  motor_r_direction = 0;
  motor_l_speed = 0;
  motor_r_speed = 0;
  // Write direction=STOP to both motors back-to-back (minimise gap)
  // before writing speed=0, so neither motor runs at full PWM while
  // the other is already stopped.
  Wire.beginTransmission(MOTOR_L_ADDR);
  Wire.write(0x00);
  Wire.write((uint8_t)0);  // dir=STOP
  Wire.endTransmission(true);
  Wire.beginTransmission(MOTOR_R_ADDR);
  Wire.write(0x00);
  Wire.write((uint8_t)0);  // dir=STOP
  Wire.endTransmission(true);
  i2cGuard();
  // Now zero the speed registers
  Wire.beginTransmission(MOTOR_L_ADDR);
  Wire.write(0x01);
  Wire.write((uint8_t)0);
  Wire.endTransmission(true);
  Wire.beginTransmission(MOTOR_R_ADDR);
  Wire.write(0x01);
  Wire.write((uint8_t)0);
  Wire.endTransmission(true);
  i2cGuard();
  // Update cached state so applyMotorOutputs will see no change
  motorL_last_reg_dir = 0;
  motorL_last_reg_pwm = 0;
  motorR_last_reg_dir = 0;
  motorR_last_reg_pwm = 0;
  // Invalidate applyMotorOutputs() cache so the next call always writes
  extern volatile bool motorCacheDirty;
  motorCacheDirty = true;
}

// Read IMU once and integrate gyro-Z into imu_yaw.
// Call this during blocking waits so yaw tracking stays current.
static void imuIntegrateOnce() {
  if (!imu_present) return;
  float ax, ay, az, gx, gy, gz, tmp;
  if (!imu6886Read(&ax, &ay, &az, &gx, &gy, &gz, &tmp)) return;
  float gz_corr = gz - imu_gz_bias_dps;
  uint32_t nowUs = (uint32_t)micros();
  if (lastYawUs != 0) {
    float dt_s = (float)(nowUs - lastYawUs) * 1e-6f;
    if (dt_s > 0.0f && dt_s < 0.5f) {
      imu_yaw += gz_corr * dt_s;
      imu_yaw = wrapDeg(imu_yaw);
    }
  }
  lastYawUs = nowUs;
  imu_gz_dps = gz_corr;
  imu_gz_raw_dps = gz;
}

// Closed-loop braking: apply reverse brake equally to both motors, then stop
// each motor individually once its encoder shows zero velocity.  This ensures
// both wheels stop at the same time regardless of motor asymmetry.
static void brakeToStop(uint8_t brakePwm, unsigned long maxBrakeMs) {
  // Progressive braking: start at lower PWM, ramp up only if still moving.
  // Prevents wheel lock/skid on smooth surfaces (hardwood).
  const uint8_t startPwm = (brakePwm > 80) ? (brakePwm / 2) : brakePwm;
  uint8_t currentPwm = startPwm;
  setMotorsReversePwm(currentPwm, currentPwm);
  applyMotorOutputs();

  // If encoders unavailable, fall back to fixed-delay brake
  readWheelEncodersForce();
  if (!encoderL_found && !encoderR_found) {
    // Non-blocking fallback: poll IMU while waiting
    uint32_t fallbackEndUs = (uint32_t)micros() + (maxBrakeMs / 2) * 1000u;
    while ((int32_t)(fallbackEndUs - (uint32_t)micros()) > 0) {
      imuIntegrateOnce();
    }
    stopAllMotors();
    return;
  }

  int32_t prevL = encoderL_count;
  int32_t prevR = encoderR_count;
  bool stoppedL = false, stoppedR = false;
  unsigned long t0 = millis();
  uint32_t lastImuUs = (uint32_t)micros();
  unsigned long lastEncCheckMs = t0;

  while ((millis() - t0) < maxBrakeMs) {
    // Non-blocking IMU polling: read IMU as fast as I2C allows (~1-2ms)
    uint32_t nowUs = (uint32_t)micros();
    if ((nowUs - lastImuUs) >= 1000u) {  // ~1ms minimum between reads
      imuIntegrateOnce();
      lastImuUs = nowUs;
    }

    // Check encoders every 25ms
    if ((millis() - lastEncCheckMs) < 25) continue;
    lastEncCheckMs = millis();
    readWheelEncodersForce();

    int32_t deltaL = abs(encoderL_count - prevL);
    int32_t deltaR = abs(encoderR_count - prevR);

    // Ramp up brake force if wheels are still moving fast
    if (!stoppedL && !stoppedR && (deltaL > 8 || deltaR > 8)) {
      if (currentPwm < brakePwm) {
        currentPwm += 10;
        if (currentPwm > brakePwm) currentPwm = brakePwm;
        setMotorsReversePwm(currentPwm, currentPwm);
        applyMotorOutputs();
      }
    }

    // Wheel is stopped when encoder barely changed in 25ms
    if (!stoppedL && deltaL < 3) {
      hbridgeWriteDirectionSpeed8(MOTOR_L_ADDR, 0, 0, nullptr, nullptr);
      stoppedL = true;
    }
    if (!stoppedR && deltaR < 3) {
      hbridgeWriteDirectionSpeed8(MOTOR_R_ADDR, 0, 0, nullptr, nullptr);
      stoppedR = true;
    }

    if (stoppedL && stoppedR) break;

    prevL = encoderL_count;
    prevR = encoderR_count;
  }

  stopAllMotors();
}

// Flag set by stopAllMotors() to force applyMotorOutputs() to resend on next call
volatile bool motorCacheDirty = false;

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
  bool values_changed = (motor_l_direction != last_ldir) || (motor_r_direction != last_rdir) || (motor_l_speed != last_l) || (motor_r_speed != last_r);
  if (motorCacheDirty) {
    values_changed = true;
    motorCacheDirty = false;
  }
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

// === HW TEST auto-find minimum PWM state ===
static bool hw_find_running = false;   // sweep in progress?
static uint8_t hw_find_pwm = 0;        // current PWM being tested
static uint8_t hw_find_motor = 0;      // 0 = testing left, 1 = testing right
static uint8_t hw_min_pwm_L = 0;       // result: left motor minimum PWM
static uint8_t hw_min_pwm_R = 0;       // result: right motor minimum PWM
static int32_t hw_find_enc_snap_L = 0; // encoder snapshot for current step
static int32_t hw_find_enc_snap_R = 0;
static unsigned long hw_find_step_ms = 0; // when current step started
static unsigned long hw_find_start_ms = 0; // when sweep started (for ramp behavior)
static bool hw_find_done = false;      // sweep completed?
static const uint8_t HW_FIND_START_PWM = 50;  // start ramp from here
static const unsigned long HW_FIND_STEP_DURATION_MS = 300; // time at each PWM level
static const int32_t HW_FIND_PULSE_THRESHOLD = 5; // pulses needed to count as "moving"

// HW test motion tracking for no-motion auto-boost
static unsigned long lastMotionMs_hw = 0;
static int32_t lastMotionEncL_hw = 0;
static int32_t lastMotionEncR_hw = 0;

static void applyHardwareTestDialSteps(int detentSteps) {
  if (detentSteps == 0) return;
  if (selected_motor == 2) return;  // ignore dial in FIND MIN mode
  const int step_per_detent = 1;
  int next_cmd = (int)motor_command + detentSteps * step_per_detent;
  if (next_cmd < -255) next_cmd = -255;
  if (next_cmd > 255) next_cmd = 255;
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
  const int stepCount = (int)(sizeof(tunePpmSteps) / sizeof(tunePpmSteps[0]));
  const float step = tunePpmSteps[tunePpmStepIndex % stepCount];
  volatile float* target = encCalEditingLeft ? &pulsesPerMeterL : &pulsesPerMeterR;
  float next = *target + (float)detentSteps * step;
  next = clampPpm(next);
  if (fabsf(*target - next) > 0.0001f) {
    *target = next;
    syncPulsesPerMeterDerived();
    settings_dirty = true;
    settings_dirty_ms = millis();
  }
}

/* TUNE PPM removed: functionality merged into ENC CAL (SCREEN_SET_ENC_CAL). */

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

// Helper: clamp an int within [lo, hi]
static inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void applyBfsDialSteps(int detentSteps, int bfs_step) {
  if (detentSteps == 0) return;

  switch (bfs_step) {
    case 0: { // Start position (cycle border edge midpoints)
      int bc = (int)gridBorderEdgeCount();
      if (bc > 0) {
        int v = (int)grid_start_cursor + detentSteps;
        while (v < 0) v += bc;
        grid_start_cursor = (uint16_t)(v % bc);
        gridBorderEdgeFromIndex(grid_start_cursor,
          grid_course.start_r1, grid_course.start_c1,
          grid_course.start_r2, grid_course.start_c2);
        manual_wp_count = 0; // layout changed, reset waypoints
      }
      break;
    }
    case 1: { // End position (cycle square centers)
      int sc = (int)gridSquareCount();
      if (sc > 0) {
        int v = (int)grid_end_cursor + detentSteps;
        while (v < 0) v += sc;
        grid_end_cursor = (uint16_t)(v % sc);
        gridSquarePosition(grid_end_cursor, grid_course.end_row, grid_course.end_col);
        manual_wp_count = 0; // layout changed, reset waypoints
      }
      break;
    }
    case 2: { // Horizontal walls editor cursor
      int ec = (int)gridHEdgeCount();
      if (ec > 0) {
        int v = (int)grid_hwall_cursor + detentSteps;
        grid_hwall_cursor = (uint16_t)clampInt(v, 0, ec - 1);
      }
      break;
    }
    case 3: { // Vertical walls editor cursor
      int ec = (int)gridVEdgeCount();
      if (ec > 0) {
        int v = (int)grid_vwall_cursor + detentSteps;
        grid_vwall_cursor = (uint16_t)clampInt(v, 0, ec - 1);
      }
      break;
    }
    case 4: { // Horizontal bottles editor cursor
      int ec = (int)gridHEdgeCount();
      if (ec > 0) {
        int v = (int)grid_hbottle_cursor + detentSteps;
        grid_hbottle_cursor = (uint16_t)clampInt(v, 0, ec - 1);
      }
      break;
    }
    case 5: { // Vertical bottles editor cursor
      int ec = (int)gridVEdgeCount();
      if (ec > 0) {
        int v = (int)grid_vbottle_cursor + detentSteps;
        grid_vbottle_cursor = (uint16_t)clampInt(v, 0, ec - 1);
      }
      break;
    }
    case 6: { // Gates editor cursor (squares)
      int sc = (int)gridSquareCount();
      if (sc > 0) {
        int v = (int)grid_gate_cursor + detentSteps;
        grid_gate_cursor = (uint16_t)clampInt(v, 0, sc - 1);
      }
      break;
    }
    case 7: { // Waypoint editor cursor (25cm grid, row-major order)
      int total = MANUAL_GRID_COLS * MANUAL_GRID_ROWS;
      int v = manual_wp_cursor_idx + detentSteps;
      while (v < 0) v += total;
      v = v % total;
      manual_wp_cursor_idx = v;
      manual_wp_cursor_col = (uint8_t)(v % MANUAL_GRID_COLS);
      manual_wp_cursor_row = (uint8_t)(v / MANUAL_GRID_COLS);
      break;
    }
    default:
      break;
  }
  settings_dirty = true;
  settings_dirty_ms = millis();
}

static void drawMainMenu() {
  ui.fillScreen(TFT_BLACK);

  // Version top-left
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(FW_VERSION, 2, 2);

  // Title
  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("MENU", 120, 14);

  // Scrolling list — show up to 7 items centered on selected
  const int VISIBLE = 7;
  const int itemH = 24;
  const int listTop = 34;
  int startIdx = selectedMenuItem - VISIBLE / 2;
  if (startIdx < 0) startIdx = 0;
  if (startIdx + VISIBLE > NUM_MENU_ITEMS) startIdx = NUM_MENU_ITEMS - VISIBLE;
  if (startIdx < 0) startIdx = 0;

  for (int v = 0; v < VISIBLE && (startIdx + v) < NUM_MENU_ITEMS; v++) {
    int idx = startIdx + v;
    int y = listTop + v * itemH;
    if (idx == selectedMenuItem) {
      ui.fillRoundRect(10, y - 2, 220, itemH - 2, 4, TFT_DARKGREEN);
      ui.drawRoundRect(10, y - 2, 220, itemH - 2, 4, TFT_GREEN);
      ui.setTextColor(TFT_WHITE);
      ui.setTextSize(2);
    } else {
      ui.setTextColor(TFT_DARKGREY);
      ui.setTextSize(1);
    }
    // Number + name
    char label[24];
    snprintf(label, sizeof(label), "%d %s", idx + 1, menuNames[idx]);
    ui.drawString(label, 120, y + itemH / 2 - 2);
  }

  // Scroll indicators
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  if (startIdx > 0) ui.drawString("^", 235, listTop);
  if (startIdx + VISIBLE < NUM_MENU_ITEMS) ui.drawString("v", 235, listTop + VISIBLE * itemH - 10);

  // Status at bottom
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(2);
  const String line1 = String(runDistanceM, 2) + "m / " + String(runTimeS, 1) + "s";
  ui.drawString(line1, 120, 210);

  ui.setTextSize(1);
  ui.setTextColor(TFT_DARKGREY);
  const String line2 = (canDistanceM >= CAN_ENABLE_MIN_M) ? (String("CAN ") + String(canDistanceM, 2) + "m") : String("CAN OFF");
  ui.drawString(line2, 120, 232);
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
  ui.drawString("Dial: adjust selected", 120, 45);
  ui.drawString("Turn: adjust value", 120, 58);
  ui.drawString("Click: cycle step  Hold1s: toggle L/R", 120, 71);
  ui.drawString("Hold2s: save + exit", 120, 84);

  ui.setTextSize(2);
  ui.setTextColor(encCalEditingLeft ? TFT_GREEN : TFT_DARKGREY);
  ui.drawString(String("L ") + String(pulsesPerMeterL, 2), 120, 125);
  ui.setTextColor(encCalEditingLeft ? TFT_DARKGREY : TFT_GREEN);
  ui.drawString(String("R ") + String(pulsesPerMeterR, 2), 120, 155);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  char stepBuf[16];
  snprintf(stepBuf, sizeof(stepBuf), "%.2g", (double)tunePpmSteps[tunePpmStepIndex % (int)(sizeof(tunePpmSteps)/sizeof(tunePpmSteps[0]))]);
  ui.drawString(String("Step: ") + String(stepBuf) + "  avg " + String(pulsesPerMeter, 2), 120, 185);
  ui.setTextSize(1);
  // Live distance since entering this screen (shows per-wheel encoder distance)
  readWheelEncoders();
  const int32_t dL = encoderL_count - encCal_start_L;
  const int32_t dR = encoderR_count - encCal_start_R;
  const float dLcorr = INVERT_LEFT_ENCODER_COUNT ? -(float)dL : (float)dL;
  const float dRcorr = INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR;
  const float distL_m = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
  const float distR_m = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(1);
  ui.drawString(String("L dist: ") + String(distL_m, 3) + " m", 120, 205);
  ui.drawString(String("R dist: ") + String(distR_m, 3) + " m", 120, 220);
}

// Automated encoder calibration using IMU-based distance estimation.
// Runs motors forward at `pwm` for `duration_s` seconds, integrates IMU
// horizontal acceleration to estimate distance, then computes pulses/m.
static void autoCalibrateEncodersUsingImu(float duration_s, uint8_t pwm) {
  if (!imu_present) {
    Serial.println("AutoCal: IMU not present");
    return;
  }

  // Ensure fresh sensor reads
  readWheelEncodersInternal(true);
  int32_t startL = encoderL_count;
  int32_t startR = encoderR_count;

  // Drive forward
  setMotorsForwardPwm(pwm, pwm);
  applyMotorOutputs();

  float vel = 0.0f;
  float dist = 0.0f;
  unsigned long t0 = millis();
  unsigned long last = t0;
  const unsigned long durMs = (unsigned long)(duration_s * 1000.0f);

  while ((unsigned long)(millis() - t0) < durMs) {
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    last = now;

    // Update sensors
    readWheelEncodersForce();

    // Horizontal-plane accel magnitude (g)
    const float horiz_g = sqrtf(imu_ax_g * imu_ax_g + imu_ay_g * imu_ay_g);
    // Ignore tiny readings as noise
    const float accel_g = (horiz_g > 0.02f) ? horiz_g : 0.0f;
    const float accel_ms2 = accel_g * 9.80665f;

    // Leaky integration: decay velocity to prevent unbounded drift from
    // accelerometer bias.  Tau of 0.5s means ~86% of DC bias is removed.
    const float velLeakTau = 0.5f;
    vel *= (1.0f - dt / velLeakTau);
    vel += accel_ms2 * dt;
    dist += vel * dt;

    // Small sleep to yield; sensor updates driven elsewhere but keep loop responsive
    delay(10);
  }

  // Stop motors
  setMotorsStop();
  applyMotorOutputs();

  // Read final encoder counts
  readWheelEncodersForce();
  int32_t endL = encoderL_count;
  int32_t endR = encoderR_count;
  const int32_t dL = INVERT_LEFT_ENCODER_COUNT ? -(endL - startL) : (endL - startL);
  const int32_t dR = INVERT_RIGHT_ENCODER_COUNT ? -(endR - startR) : (endR - startR);

  if (dist > 0.05f) {
    pulsesPerMeterL = (float)dL / dist;
    pulsesPerMeterR = (float)dR / dist;
    syncPulsesPerMeterDerived();
    Serial.printf("AutoCal: dist=%.3fm dL=%ld dR=%ld ppmL=%.1f ppmR=%.1f\n",
                  (double)dist, (long)dL, (long)dR,
                  (double)pulsesPerMeterL, (double)pulsesPerMeterR);
  } else {
    Serial.println("AutoCal: insufficient estimated distance (IMU); try longer run or verify IMU");
  }
}

// Run one trial and return computed ppmL/ppmR via references; returns true if successful
static bool autoCalibrateEncodersTrial(float duration_s, uint8_t pwm, float &out_ppmL, float &out_ppmR, float &out_dist) {
  if (!imu_present) return false;
  readWheelEncodersInternal(true);
  int32_t startL = encoderL_count;
  int32_t startR = encoderR_count;
  setMotorsForwardPwm(pwm, pwm);
  applyMotorOutputs();

  float vel = 0.0f;
  float dist = 0.0f;
  unsigned long t0 = millis();
  unsigned long last = t0;
  const unsigned long durMs = (unsigned long)(duration_s * 1000.0f);
  while ((unsigned long)(millis() - t0) < durMs) {
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    last = now;
    readWheelEncodersForce();
    const float horiz_g = sqrtf(imu_ax_g * imu_ax_g + imu_ay_g * imu_ay_g);
    const float accel_g = (horiz_g > 0.02f) ? horiz_g : 0.0f;
    const float accel_ms2 = accel_g * 9.80665f;
    const float velLeakTau = 0.5f;
    vel *= (1.0f - dt / velLeakTau);
    vel += accel_ms2 * dt;
    dist += vel * dt;
    delay(10);
  }
  setMotorsStop();
  applyMotorOutputs();
  readWheelEncodersForce();
  int32_t endL = encoderL_count;
  int32_t endR = encoderR_count;
  const int32_t dL = INVERT_LEFT_ENCODER_COUNT ? -(endL - startL) : (endL - startL);
  const int32_t dR = INVERT_RIGHT_ENCODER_COUNT ? -(endR - startR) : (endR - startR);
  out_dist = dist;
  if (dist > 0.05f) {
    out_ppmL = (float)dL / dist;
    out_ppmR = (float)dR / dist;
    return true;
  }
  return false;
}

// Interactive multi-trial auto-calibration: shows progress on-screen, averages valid trials, optionally saves
static void autoCalibrateEncodersInteractive(int trials, float duration_s, uint8_t pwm, bool saveResults) {
  if (!imu_present || trials <= 0) return;

  float sumL = 0.0f, sumR = 0.0f;
  int valid = 0;

  for (int t = 0; t < trials; t++) {
    // Update UI
    ui.fillScreen(TFT_BLACK);
    ui.setTextColor(TFT_CYAN);
    ui.setTextSize(2);
    ui.drawString("Auto ENC CAL", 120, 20);
    ui.setTextSize(1);
    ui.setTextColor(TFT_WHITE);
    ui.drawString(String("Trial ") + String(t+1) + "/" + String(trials), 120, 60);
    ui.drawString(String("Duration: ") + String(duration_s,1) + "s PWM:" + String((int)pwm), 120, 80);
    ui.pushSprite(0,0);

    float ppmL=0, ppmR=0, dist=0;
    const bool ok = autoCalibrateEncodersTrial(duration_s, pwm, ppmL, ppmR, dist);

    // Show result for this trial
    ui.setTextColor(ok ? TFT_GREEN : TFT_RED);
    ui.setTextSize(2);
    if (ok) {
      ui.drawString(String("dist:") + String(dist,3) + "m", 120, 120);
      ui.drawString(String("L:") + String(ppmL,1), 120, 150);
      ui.drawString(String("R:") + String(ppmR,1), 120, 180);
      sumL += ppmL; sumR += ppmR; valid++;
    } else {
      ui.drawString("Insufficient IMU data", 120, 120);
    }
    ui.pushSprite(0,0);
    delay(800);
  }

  if (valid > 0) {
    pulsesPerMeterL = sumL / (float)valid;
    pulsesPerMeterR = sumR / (float)valid;
    syncPulsesPerMeterDerived();
    if (saveResults && prefs_ok) savePersistedSettings();

    ui.fillScreen(TFT_BLACK);
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(2);
    ui.drawString("AutoCal COMPLETE", 120, 40);
    ui.setTextSize(1);
    ui.drawString(String("ppmL: ") + String(pulsesPerMeterL,1), 120, 90);
    ui.drawString(String("ppmR: ") + String(pulsesPerMeterR,1), 120, 110);
    ui.drawString(String("avg ppm: ") + String(pulsesPerMeter,1), 120, 130);
    ui.pushSprite(0,0);
  } else {
    ui.fillScreen(TFT_BLACK);
    ui.setTextColor(TFT_RED);
    ui.setTextSize(2);
    ui.drawString("AutoCal FAILED", 120, 80);
    ui.pushSprite(0,0);
  }

  delay(1000);
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

  // Progress bar: visual indicator of distance toward the target `runDistanceM`.
  if (runDistanceM > 0.01f) {
    const int barW = 200;
    const int barH = 10;
    const int barX = 20;
    const int barY = 200;
    float frac = runDistanceM > 0.0f ? (shownM / runDistanceM) : 0.0f;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    ui.fillRect(barX, barY, barW, barH, TFT_DARKGREY);
    ui.fillRect(barX, barY, (int)(barW * frac), barH, runActive ? TFT_GREEN : TFT_YELLOW);
    ui.drawRect(barX, barY, barW, barH, TFT_WHITE);
  }
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
// BFS NAV DISPLAY SCREEN - Grid course setup wizard
// ========================================================================

// Draw a mini grid visualization on the display
static void drawGridMinimap(int16_t ox, int16_t oy, int16_t w, int16_t h,
                            int highlightStep = -1, bool showPath = false) {
  const uint8_t rows = grid_course.rows;
  const uint8_t cols = grid_course.cols;
  if (rows < 2 || cols < 2) return;

  // Vertical layout: row → horizontal (shorter, 4 cells), col → vertical (longer, 5 cells)
  const int16_t cw = w / (rows - 1);   // horizontal = rows-1 cells
  const int16_t ch = h / (cols - 1);   // vertical = cols-1 cells
  const int16_t gw = cw * (rows - 1);
  const int16_t gh = ch * (cols - 1);

  // Grid-to-screen transform: row→x, col→y (makes 5x4 grid taller than wide)
  auto g2s = [&](float r, float c, int16_t &sx, int16_t &sy) {
    sx = ox + (int16_t)(r * cw);
    sy = oy + (int16_t)(c * ch);
  };

  // Draw grid lines (faint)
  for (uint8_t c = 0; c < cols; c++) {
    int16_t y = oy + c * ch;
    ui.drawLine(ox, y, ox + gw, y, TFT_DARKGREY);
  }
  for (uint8_t r = 0; r < rows; r++) {
    int16_t x = ox + r * cw;
    ui.drawLine(x, oy, x, oy + gh, TFT_DARKGREY);
  }

  // Draw walls as thick red lines
  for (uint8_t i = 0; i < grid_course.wall_count; i++) {
    const GridWall& wl = grid_course.walls[i];
    int16_t x1, y1, x2, y2;
    g2s(wl.r1, wl.c1, x1, y1);
    g2s(wl.r2, wl.c2, x2, y2);
    ui.drawLine(x1, y1, x2, y2, TFT_RED);
    if (x1 == x2) { // vertical on screen
      ui.drawLine(x1 - 1, y1, x2 - 1, y2, TFT_RED);
      ui.drawLine(x1 + 1, y1, x2 + 1, y2, TFT_RED);
    } else { // horizontal on screen
      ui.drawLine(x1, y1 - 1, x2, y2 - 1, TFT_RED);
      ui.drawLine(x1, y1 + 1, x2, y2 + 1, TFT_RED);
    }
  }

  // Draw gates as green filled squares
  for (uint8_t i = 0; i < grid_course.gate_count; i++) {
    const GridGate& gt = grid_course.gates[i];
    int16_t x1, y1, x2, y2;
    g2s(gt.row, gt.col, x1, y1);
    g2s(gt.row + 1, gt.col + 1, x2, y2);
    for (int16_t yy = y1 + 1; yy < y2; yy++)
      ui.drawLine(x1 + 1, yy, x2 - 1, yy, 0x03E0);
    ui.drawRect(x1, y1, x2 - x1, y2 - y1, TFT_GREEN);
  }

  // Draw water bottles as blue dots at edge midpoints
  for (uint8_t i = 0; i < grid_course.bottle_count; i++) {
    const GridBottle& bt = grid_course.bottles[i];
    int16_t x1, y1, x2, y2;
    g2s(bt.r1, bt.c1, x1, y1);
    g2s(bt.r2, bt.c2, x2, y2);
    int16_t mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
    ui.fillCircle(mx, my, 3, TFT_BLUE);
  }

  // Draw grid intersection dots
  for (uint8_t r = 0; r < rows; r++) {
    for (uint8_t c = 0; c < cols; c++) {
      int16_t px, py;
      g2s(r, c, px, py);
      ui.fillCircle(px, py, 2, TFT_WHITE);
    }
  }

  // Draw start position at border edge midpoint (green dot)
  {
    int16_t sx, sy;
    g2s((grid_course.start_r1 + grid_course.start_r2) * 0.5f,
        (grid_course.start_c1 + grid_course.start_c2) * 0.5f, sx, sy);
    ui.fillCircle(sx, sy, 4, TFT_GREEN);
  }

  // Draw end position at square center (orange X)
  {
    int16_t ex, ey;
    g2s(grid_course.end_row + 0.5f, grid_course.end_col + 0.5f, ex, ey);
    ui.fillCircle(ex, ey, 4, TFT_ORANGE);
    ui.drawLine(ex - 3, ey - 3, ex + 3, ey + 3, TFT_ORANGE);
    ui.drawLine(ex - 3, ey + 3, ex + 3, ey - 3, TFT_ORANGE);
  }

  // Optionally draw the planned path (if computed)
  if (showPath && bfs_state.path_valid && bfs_state.path_length > 0) {
    const float spacing_m = (float)grid_course.spacing_cm / 100.0f;
    int prev_x = 0, prev_y = 0;
    bool havePrev = false;
    for (uint16_t pi = 0; pi < bfs_state.path_length; pi++) {
      const uint8_t nid = bfs_state.path[pi];
      // Convert node coordinates (x_m = col*spacing, y_m = row*spacing)
      float node_r = bfs_state.nodes[nid].y_m / spacing_m;
      float node_c = bfs_state.nodes[nid].x_m / spacing_m;
      int16_t nx, ny;
      g2s(node_r, node_c, nx, ny);
      // Draw waypoint marker
      ui.fillCircle(nx, ny, 3, TFT_CYAN);
      if (havePrev) {
        ui.drawLine(prev_x, prev_y, nx, ny, TFT_CYAN);
        // Draw direction arrow at midpoint of segment
        float mx = (prev_x + nx) * 0.5f, my = (prev_y + ny) * 0.5f;
        float dx = (float)(nx - prev_x), dy = (float)(ny - prev_y);
        float len = sqrtf(dx * dx + dy * dy);
        if (len > 6.0f) {
          float ux = dx / len, uy = dy / len; // unit vector along segment
          float sz = 4.0f; // arrowhead size
          // Arrowhead tip at midpoint + half-size forward
          float tipx = mx + ux * sz * 0.5f, tipy = my + uy * sz * 0.5f;
          // Two base points perpendicular to direction
          float bx1 = mx - ux * sz * 0.5f + uy * sz * 0.4f;
          float by1 = my - uy * sz * 0.5f - ux * sz * 0.4f;
          float bx2 = mx - ux * sz * 0.5f - uy * sz * 0.4f;
          float by2 = my - uy * sz * 0.5f + ux * sz * 0.4f;
          ui.fillTriangle((int16_t)tipx, (int16_t)tipy,
                          (int16_t)bx1, (int16_t)by1,
                          (int16_t)bx2, (int16_t)by2, TFT_CYAN);
        }
      }
      prev_x = nx; prev_y = ny; havePrev = true;
    }
  }

  // Highlight current cursor in editor modes
  if (highlightStep == 2) {
    // H-Wall editor: highlight horizontal edge under cursor
    uint8_t r1, c1, r2, c2;
    gridHEdgeFromIndex(grid_hwall_cursor, r1, c1, r2, c2);
    int16_t x1, y1, x2, y2;
    g2s(r1, c1, x1, y1);
    g2s(r2, c2, x2, y2);
    int16_t mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
    ui.fillCircle(mx, my, 3, TFT_YELLOW);
  } else if (highlightStep == 3) {
    // V-Wall editor: highlight vertical edge under cursor
    uint8_t r1, c1, r2, c2;
    gridVEdgeFromIndex(grid_vwall_cursor, r1, c1, r2, c2);
    int16_t x1, y1, x2, y2;
    g2s(r1, c1, x1, y1);
    g2s(r2, c2, x2, y2);
    int16_t mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
    ui.fillCircle(mx, my, 3, TFT_YELLOW);
  } else if (highlightStep == 4) {
    // H-Bottle editor: highlight horizontal edge under cursor
    uint8_t r1, c1, r2, c2;
    gridHEdgeFromIndex(grid_hbottle_cursor, r1, c1, r2, c2);
    int16_t x1, y1, x2, y2;
    g2s(r1, c1, x1, y1);
    g2s(r2, c2, x2, y2);
    int16_t mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
    ui.fillCircle(mx, my, 3, TFT_YELLOW);
  } else if (highlightStep == 5) {
    // V-Bottle editor: highlight vertical edge under cursor
    uint8_t r1, c1, r2, c2;
    gridVEdgeFromIndex(grid_vbottle_cursor, r1, c1, r2, c2);
    int16_t x1, y1, x2, y2;
    g2s(r1, c1, x1, y1);
    g2s(r2, c2, x2, y2);
    int16_t mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
    ui.fillCircle(mx, my, 3, TFT_YELLOW);
  } else if (highlightStep == 6) {
    // Gate editor: highlight square under cursor
    uint8_t row, col;
    gridSquarePosition(grid_gate_cursor, row, col);
    int16_t x1, y1, x2, y2;
    g2s(row, col, x1, y1);
    g2s(row + 1, col + 1, x2, y2);
    ui.drawRect(x1, y1, x2 - x1, y2 - y1, TFT_YELLOW);
  }
}

static void drawImuCalScreen() {
  ui.fillScreen(TFT_BLACK);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("IMU CAL", 120, 20);

  if (!imu_present) {
    ui.setTextColor(TFT_RED);
    ui.setTextSize(2);
    ui.drawString("NO IMU", 120, 120);
  } else {
    // Gyro Z bias
    ui.setTextColor(TFT_DARKGREY);
    ui.setTextSize(1);
    ui.drawString("Gyro Z bias (dps)", 120, 50);
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(2);
    ui.drawString(String(imu_gz_bias_dps, 3), 120, 70);

    // Live gyro Z (corrected)
    ui.setTextColor(TFT_DARKGREY);
    ui.setTextSize(1);
    ui.drawString("Live Gz (dps)", 120, 95);
    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(2);
    ui.drawString(String(imu_gz_dps, 2), 120, 115);

    // Live yaw
    ui.setTextColor(TFT_DARKGREY);
    ui.setTextSize(1);
    ui.drawString("Yaw (deg)", 120, 140);
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    ui.drawString(String(imu_yaw, 1), 120, 160);

    // Raw Gz
    ui.setTextColor(TFT_DARKGREY);
    ui.setTextSize(1);
    ui.drawString("Raw Gz (dps)", 120, 185);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString(String(imu_gz_raw_dps, 2), 120, 198);
  }

  // If a calibration routine is running, show smooth animated progress bar + duck.
  if (imu_calibrating) {
    // Stage label
    ui.setTextSize(1);
    if (imu_cal_progress < 15) {
      ui.setTextColor(TFT_YELLOW);
      ui.drawString("Hold still...", 120, 192);
    } else if (imu_cal_progress < 100) {
      ui.setTextColor(TFT_GREEN);
      ui.drawString("Sampling gyro", 120, 192);
    } else {
      ui.setTextColor(TFT_GREEN);
      ui.drawString("Done!", 120, 192);
    }

    // Smooth animated progress bar
    static float imucal_displayed_pct = 0.0f;
    float target_pct = (float)imu_cal_progress;
    if (imu_cal_progress == 0) imucal_displayed_pct = 0.0f;
    float diff = target_pct - imucal_displayed_pct;
    if (diff > 0.5f) {
      imucal_displayed_pct += diff * 0.15f;
      if (imucal_displayed_pct > target_pct) imucal_displayed_pct = target_pct;
    } else {
      imucal_displayed_pct = target_pct;
    }

    const int bx = 20, by = 208, bw = 200, bh = 20, radius = 6;
    int filled = (int)((bw * imucal_displayed_pct) / 100.0f);
    if (filled < 0) filled = 0;
    if (filled > bw) filled = bw;

    // Background (rounded)
    ui.fillRoundRect(bx, by, bw, bh, radius, TFT_DARKGREY);
    // Filled portion (rounded, clipped)
    if (filled > 2 * radius) {
      ui.fillRoundRect(bx, by, filled, bh, radius, TFT_CYAN);
    } else if (filled > 0) {
      ui.fillRect(bx, by, filled, bh, TFT_CYAN);
    }
    // Border (rounded)
    ui.drawRoundRect(bx, by, bw, bh, radius, TFT_WHITE);

    // Percentage text centered on bar
    char pctBuf[8];
    snprintf(pctBuf, sizeof(pctBuf), "%d%%", (int)imucal_displayed_pct);
    ui.setTextColor(TFT_BLACK);
    ui.setTextSize(1);
    ui.drawString(pctBuf, bx + bw / 2, by + bh / 2);

    // Duck rides the progress bar
    {
      const int duckX = bx + filled - 6;
      const int duckY = by - 22;
      ui.fillEllipse(duckX, duckY + 10, 10, 7, TFT_YELLOW);
      ui.fillCircle(duckX + 9, duckY + 2, 6, TFT_YELLOW);
      ui.fillCircle(duckX + 11, duckY, 1, TFT_BLACK);
      ui.fillTriangle(duckX + 15, duckY + 2,
                      duckX + 15, duckY + 5,
                      duckX + 20, duckY + 3, TFT_ORANGE);
      ui.fillEllipse(duckX - 2, duckY + 10, 5, 4, 0xFD60);
      ui.drawLine(duckX - 4, duckY + 17, duckX - 7, duckY + 20, TFT_ORANGE);
      ui.drawLine(duckX + 2, duckY + 17, duckX - 1, duckY + 20, TFT_ORANGE);
    }
  }

  if (!imu_calibrating) {
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(1);
    ui.drawString("Click:recal  Hold:menu", 120, 225);
  }
}

// Calibration status screen for SCREEN_CALIBRATE with smooth progress bar.
static void drawCalibrateScreen() {
  ui.fillScreen(TFT_BLACK);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("CALIBRATING", 120, 25);

  // Stage label
  ui.setTextSize(1);
  if (imuCal_stage <= 1) {
    ui.setTextColor(TFT_YELLOW);
    ui.drawString("Hold still...", 120, 55);
  } else if (imuCal_stage == 2) {
    ui.setTextColor(TFT_GREEN);
    ui.drawString("Sampling gyro", 120, 55);
  } else {
    ui.setTextColor(TFT_GREEN);
    ui.drawString("Done!", 120, 55);
  }

  // --- Smooth animated progress bar ---
  // Smoothly interpolate displayed progress towards actual progress
  static float displayed_pct = 0.0f;
  float target_pct = (float)imu_cal_progress;
  // Reset on new calibration start
  if (imu_cal_progress == 0 && !imu_calibrating) displayed_pct = 0.0f;
  // Ease toward target (fast catch-up, smooth motion)
  float diff = target_pct - displayed_pct;
  if (diff > 0.5f) {
    displayed_pct += diff * 0.15f; // smooth ease
    if (displayed_pct > target_pct) displayed_pct = target_pct;
  } else {
    displayed_pct = target_pct;
  }

  const int bx = 20, by = 85, bw = 200, bh = 20, radius = 6;
  int filled = (int)((bw * displayed_pct) / 100.0f);
  if (filled < 0) filled = 0;
  if (filled > bw) filled = bw;

  // Background (rounded)
  ui.fillRoundRect(bx, by, bw, bh, radius, TFT_DARKGREY);
  // Filled portion (rounded, clipped)
  if (filled > 2 * radius) {
    ui.fillRoundRect(bx, by, filled, bh, radius, TFT_CYAN);
  } else if (filled > 0) {
    ui.fillRect(bx, by, filled, bh, TFT_CYAN);
  }
  // Border (rounded)
  ui.drawRoundRect(bx, by, bw, bh, radius, TFT_WHITE);

  // Percentage text centered on bar
  char pctBuf[8];
  snprintf(pctBuf, sizeof(pctBuf), "%d%%", (int)displayed_pct);
  ui.setTextColor(TFT_BLACK);
  ui.setTextSize(1);
  ui.drawString(pctBuf, bx + bw / 2, by + bh / 2);

  // Gyro bias result (shows live during cal, final when done)
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Gyro Z bias (dps)", 120, 125);
  ui.setTextColor(TFT_GREEN);
  ui.setTextSize(2);
  ui.drawString(String(imu_gz_bias_dps, 3), 120, 145);

  // --- Duck ---
  // The duck rides the progress bar, sliding right as calibration progresses
  {
    const int duckX = bx + filled - 6;  // duck sits at the leading edge of the bar
    const int duckY = by - 22;          // duck stands on top of the bar

    // Body (yellow ellipse)
    ui.fillEllipse(duckX, duckY + 10, 10, 7, TFT_YELLOW);
    // Head (smaller circle)
    ui.fillCircle(duckX + 9, duckY + 2, 6, TFT_YELLOW);
    // Eye
    ui.fillCircle(duckX + 11, duckY, 1, TFT_BLACK);
    // Beak (orange triangle)
    ui.fillTriangle(duckX + 15, duckY + 2,
                    duckX + 15, duckY + 5,
                    duckX + 20, duckY + 3, TFT_ORANGE);
    // Wing (darker arc on body)
    ui.fillEllipse(duckX - 2, duckY + 10, 5, 4, 0xFD60); // dark yellow
    // Feet (two small orange lines under body)
    ui.drawLine(duckX - 4, duckY + 17, duckX - 7, duckY + 20, TFT_ORANGE);
    ui.drawLine(duckX + 2, duckY + 17, duckX - 1, duckY + 20, TFT_ORANGE);
  }
}

static void drawBfsNavScreen(int bfs_setup_step) {
  ui.fillScreen(TFT_BLACK);

  // Step indicator at top
  char stepBuf[24];
  snprintf(stepBuf, sizeof(stepBuf), "SETUP %d/%d", bfs_setup_step + 1, BFS_SETUP_STEP_COUNT);
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString(stepBuf, 120, 12);

  char buf[48];

  switch (bfs_setup_step) {
    case 0: // Start position (border edge midpoint)
      ui.setTextColor(TFT_CYAN);
      ui.setTextSize(2);
      ui.drawString("START POS", 120, 35);
      ui.setTextColor(TFT_YELLOW);
      ui.setTextSize(1);
      snprintf(buf, sizeof(buf), "Edge (%d,%d)-(%d,%d)",
               grid_course.start_r1, grid_course.start_c1,
               grid_course.start_r2, grid_course.start_c2);
      ui.drawString(buf, 120, 60);
      ui.setTextColor(TFT_DARKGREY);
      ui.setTextSize(1);
      ui.drawString("Border edge midpoint", 120, 80);
      ui.drawString("5x4 grid, 250x200cm", 120, 95);
      drawGridMinimap(70, 110, 100, 125);
      break;

    case 1: // End position (square center)
      ui.setTextColor(TFT_CYAN);
      ui.setTextSize(2);
      ui.drawString("END POS", 120, 35);
      ui.setTextColor(TFT_YELLOW);
      ui.setTextSize(2);
      snprintf(buf, sizeof(buf), "Sq(%d, %d)", grid_course.end_row, grid_course.end_col);
      ui.drawString(buf, 120, 65);
      ui.setTextColor(TFT_DARKGREY);
      ui.setTextSize(1);
      ui.drawString("Square center", 120, 90);
      drawGridMinimap(70, 105, 100, 125);
      break;

    case 2: { // Horizontal walls editor
      ui.setTextColor(TFT_RED);
      ui.setTextSize(2);
      snprintf(buf, sizeof(buf), "H-WALLS (%d)", grid_course.wall_count);
      ui.drawString(buf, 120, 30);

      uint8_t r1, c1, r2, c2;
      gridHEdgeFromIndex(grid_hwall_cursor, r1, c1, r2, c2);
      bool hasWall = gridHasWall(r1, c1, r2, c2);

      ui.setTextColor(hasWall ? TFT_RED : TFT_WHITE);
      ui.setTextSize(1);
      snprintf(buf, sizeof(buf), "(%d,%d)-(%d,%d) %s", r1, c1, r2, c2,
               hasWall ? "[WALL]" : "[open]");
      ui.drawString(buf, 120, 55);

      drawGridMinimap(70, 70, 100, 125, 2);

      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(1);
      ui.drawString("Dial:move Click:toggle", 120, 200);
      ui.drawString("Hold:next  2s:menu", 120, 215);
      break;
    }

    case 3: { // Vertical walls editor
      ui.setTextColor(TFT_RED);
      ui.setTextSize(2);
      snprintf(buf, sizeof(buf), "V-WALLS (%d)", grid_course.wall_count);
      ui.drawString(buf, 120, 30);

      uint8_t r1, c1, r2, c2;
      gridVEdgeFromIndex(grid_vwall_cursor, r1, c1, r2, c2);
      bool hasWall = gridHasWall(r1, c1, r2, c2);

      ui.setTextColor(hasWall ? TFT_RED : TFT_WHITE);
      ui.setTextSize(1);
      snprintf(buf, sizeof(buf), "(%d,%d)-(%d,%d) %s", r1, c1, r2, c2,
               hasWall ? "[WALL]" : "[open]");
      ui.drawString(buf, 120, 55);

      drawGridMinimap(70, 70, 100, 125, 3);

      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(1);
      ui.drawString("Dial:move Click:toggle", 120, 200);
      ui.drawString("Hold:next  2s:menu", 120, 215);
      break;
    }

    case 4: { // Horizontal bottles editor
      ui.setTextColor(TFT_BLUE);
      ui.setTextSize(2);
      snprintf(buf, sizeof(buf), "H-BOTTLES (%d)", grid_course.bottle_count);
      ui.drawString(buf, 120, 30);

      uint8_t r1, c1, r2, c2;
      gridHEdgeFromIndex(grid_hbottle_cursor, r1, c1, r2, c2);
      bool hasBottle = gridHasBottle(r1, c1, r2, c2);

      ui.setTextColor(hasBottle ? TFT_BLUE : TFT_WHITE);
      ui.setTextSize(1);
      snprintf(buf, sizeof(buf), "(%d,%d)-(%d,%d) %s", r1, c1, r2, c2,
               hasBottle ? "[BOTTLE]" : "[empty]");
      ui.drawString(buf, 120, 55);

      drawGridMinimap(70, 70, 100, 125, 4);

      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(1);
      ui.drawString("Dial:move Click:toggle", 120, 200);
      ui.drawString("Hold:next  2s:menu", 120, 215);
      break;
    }

    case 5: { // Vertical bottles editor
      ui.setTextColor(TFT_BLUE);
      ui.setTextSize(2);
      snprintf(buf, sizeof(buf), "V-BOTTLES (%d)", grid_course.bottle_count);
      ui.drawString(buf, 120, 30);

      uint8_t r1, c1, r2, c2;
      gridVEdgeFromIndex(grid_vbottle_cursor, r1, c1, r2, c2);
      bool hasBottle = gridHasBottle(r1, c1, r2, c2);

      ui.setTextColor(hasBottle ? TFT_BLUE : TFT_WHITE);
      ui.setTextSize(1);
      snprintf(buf, sizeof(buf), "(%d,%d)-(%d,%d) %s", r1, c1, r2, c2,
               hasBottle ? "[BOTTLE]" : "[empty]");
      ui.drawString(buf, 120, 55);

      drawGridMinimap(70, 70, 100, 125, 5);

      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(1);
      ui.drawString("Dial:move Click:toggle", 120, 200);
      ui.drawString("Hold:next  2s:menu", 120, 215);
      break;
    }

    case 6: { // Gates editor (squares) — 3-state: none → gate → last gate → remove
      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(2);
      snprintf(buf, sizeof(buf), "GATES (%d)", grid_course.gate_count);
      ui.drawString(buf, 120, 30);

      uint8_t row, col;
      gridSquarePosition(grid_gate_cursor, row, col);
      bool hasGate = gridHasGate(row, col);
      ui.setTextColor(hasGate ? TFT_GREEN : TFT_WHITE);
      ui.setTextSize(1);
      snprintf(buf, sizeof(buf), "Sq(%d,%d) %s", row, col,
               hasGate ? "[GATE]" : "[none]");
      ui.drawString(buf, 120, 55);

      drawGridMinimap(70, 80, 100, 115, 6);

      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(1);
      ui.drawString("Click: +gate/remove", 120, 200);
      ui.drawString("Hold:next  2s:menu", 120, 215);
      break;
    }

    case 7: { // Waypoint editor (manual waypoint plotting on 25cm sub-grid)
      ui.setTextColor(TFT_YELLOW);
      ui.setTextSize(2);
      snprintf(buf, sizeof(buf), "WAYPTS (%d)", manual_wp_count);
      ui.drawString(buf, 120, 30);

      // Draw minimap with 25cm waypoint overlay
      // Map dimensions: fit 250x200cm into display area
      const int mapX = 20, mapY = 50, mapW = 200, mapH = 160;
      const float scaleX = (float)mapW / 2.50f; // pixels per meter
      const float scaleY = (float)mapH / 2.00f;

      // Draw 50cm grid lines (light grey)
      for (int c = 0; c <= 5; c++) {
        int px = mapX + (int)(c * 0.50f * scaleX);
        ui.drawLine(px, mapY, px, mapY + mapH, TFT_DARKGREY);
      }
      for (int r = 0; r <= 4; r++) {
        int py = mapY + mapH - (int)(r * 0.50f * scaleY);
        ui.drawLine(mapX, py, mapX + mapW, py, TFT_DARKGREY);
      }

      // Draw 2x4 obstacles (red) if any walls
      for (uint8_t wi = 0; wi < grid_course.wall_count; wi++) {
        const GridWall &w = grid_course.walls[wi];
        float spacing_m = (float)grid_course.spacing_cm * 0.01f;
        int wx1 = mapX + (int)(w.c1 * spacing_m * scaleX);
        int wy1 = mapY + mapH - (int)(w.r1 * spacing_m * scaleY);
        int wx2 = mapX + (int)(w.c2 * spacing_m * scaleX);
        int wy2 = mapY + mapH - (int)(w.r2 * spacing_m * scaleY);
        ui.drawLine(wx1, wy1, wx2, wy2, TFT_RED);
        // Draw thick
        ui.drawLine(wx1-1, wy1, wx2-1, wy2, TFT_RED);
        ui.drawLine(wx1+1, wy1, wx2+1, wy2, TFT_RED);
      }

      // Draw gate zones (green rectangles)
      for (uint8_t gi = 0; gi < grid_course.gate_count; gi++) {
        float spacing_m = (float)grid_course.spacing_cm * 0.01f;
        int gx = mapX + (int)(grid_course.gates[gi].col * spacing_m * scaleX);
        int gy = mapY + mapH - (int)((grid_course.gates[gi].row + 1) * spacing_m * scaleY);
        int gw = (int)(spacing_m * scaleX);
        int gh = (int)(spacing_m * scaleY);
        ui.drawRect(gx, gy, gw, gh, TFT_GREEN);
        // Gate label
        char gl[4];
        snprintf(gl, sizeof(gl), "%c", 'A' + gi);
        ui.setTextColor(TFT_GREEN);
        ui.setTextSize(1);
        ui.drawString(gl, gx + gw/2, gy + gh/2);
      }

      // Draw start position (cyan dot)
      {
        float spacing_m = (float)grid_course.spacing_cm * 0.01f;
        float sx = 0.5f * ((float)grid_course.start_c1 + (float)grid_course.start_c2) * spacing_m;
        float sy = 0.5f * ((float)grid_course.start_r1 + (float)grid_course.start_r2) * spacing_m;
        int spx = mapX + (int)(sx * scaleX);
        int spy = mapY + mapH - (int)(sy * scaleY);
        ui.fillCircle(spx, spy, 3, TFT_CYAN);
      }

      // Draw existing waypoints and path lines
      float prevX_m = 0.5f * ((float)grid_course.start_c1 + (float)grid_course.start_c2) * ((float)grid_course.spacing_cm * 0.01f);
      float prevY_m = 0.5f * ((float)grid_course.start_r1 + (float)grid_course.start_r2) * ((float)grid_course.spacing_cm * 0.01f);
      for (uint8_t wi = 0; wi < manual_wp_count; wi++) {
        float wx = (float)manual_wp_col[wi] * 0.25f;
        float wy = (float)manual_wp_row[wi] * 0.25f;
        int px1 = mapX + (int)(prevX_m * scaleX);
        int py1 = mapY + mapH - (int)(prevY_m * scaleY);
        int px2 = mapX + (int)(wx * scaleX);
        int py2 = mapY + mapH - (int)(wy * scaleY);
        ui.drawLine(px1, py1, px2, py2, TFT_YELLOW);
        ui.fillCircle(px2, py2, 2, TFT_WHITE);
        // Waypoint number label
        char wl[4];
        snprintf(wl, sizeof(wl), "%d", (int)(wi + 1));
        ui.setTextColor(TFT_YELLOW);
        ui.setTextSize(1);
        ui.drawString(wl, px2 + 4, py2 - 5);
        prevX_m = wx;
        prevY_m = wy;
      }

      // Draw cursor (blinking magenta dot) with preview line from last point
      {
        float cx = (float)manual_wp_cursor_col * 0.25f;
        float cy = (float)manual_wp_cursor_row * 0.25f;
        int cpx = mapX + (int)(cx * scaleX);
        int cpy = mapY + mapH - (int)(cy * scaleY);
        // Preview line: dashed from last waypoint (or start) to cursor
        int ppx = mapX + (int)(prevX_m * scaleX);
        int ppy = mapY + mapH - (int)(prevY_m * scaleY);
        // Draw dashed preview line (alternating pixels)
        {
          float dx = (float)(cpx - ppx), dy = (float)(cpy - ppy);
          float len = sqrtf(dx * dx + dy * dy);
          if (len > 2.0f) {
            float ux = dx / len, uy = dy / len;
            for (float t = 0; t < len; t += 4.0f) {
              int x1 = ppx + (int)(ux * t);
              int y1 = ppy + (int)(uy * t);
              int x2 = ppx + (int)(ux * (t + 2.0f));
              int y2 = ppy + (int)(uy * (t + 2.0f));
              ui.drawLine(x1, y1, x2, y2, TFT_DARKGREY);
            }
          }
        }
        uint16_t curColor = ((millis() / 300) % 2 == 0) ? TFT_MAGENTA : TFT_WHITE;
        ui.fillCircle(cpx, cpy, 4, curColor);
        // Show cursor position and segment distance
        float segDist = sqrtf((cx - prevX_m) * (cx - prevX_m) + (cy - prevY_m) * (cy - prevY_m));
        snprintf(buf, sizeof(buf), "(%.2f,%.2f)m d=%.2f", (double)cx, (double)cy, (double)segDist);
        ui.setTextColor(TFT_WHITE);
        ui.setTextSize(1);
        ui.drawString(buf, 120, 218);
      }

      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(1);
      ui.drawString("Clk:add Hold:GO 2s:undo", 120, 230);
      break;
    }

    case 8: { // Confirm / GO
      ui.setTextColor(TFT_CYAN);
      ui.setTextSize(2);
      ui.drawString("CONFIRM", 120, 30);

      ui.setTextColor(TFT_WHITE);
      ui.setTextSize(1);
      snprintf(buf, sizeof(buf), "Grid: 5x4 (250x200cm)");
      ui.drawString(buf, 120, 55);
      snprintf(buf, sizeof(buf), "Start:E(%d,%d)-(%d,%d) End:Sq(%d,%d)",
               grid_course.start_r1, grid_course.start_c1,
               grid_course.start_r2, grid_course.start_c2,
               grid_course.end_row, grid_course.end_col);
      ui.drawString(buf, 120, 70);
      snprintf(buf, sizeof(buf), "WPs:%d Gates:%d Time:%.0fs",
               manual_wp_count, grid_course.gate_count, (double)runTimeS);
      ui.drawString(buf, 120, 85);

      // Compute and show total path distance
      {
        float totalDist = 0.0f;
        float spacing_m = (float)grid_course.spacing_cm * 0.01f;
        float px = 0.5f * ((float)grid_course.start_c1 + (float)grid_course.start_c2) * spacing_m;
        float py = 0.5f * ((float)grid_course.start_r1 + (float)grid_course.start_r2) * spacing_m;
        for (uint8_t i = 0; i < manual_wp_count; i++) {
          float wx = (float)manual_wp_col[i] * 0.25f;
          float wy = (float)manual_wp_row[i] * 0.25f;
          totalDist += sqrtf((wx - px) * (wx - px) + (wy - py) * (wy - py));
          px = wx; py = wy;
        }
        snprintf(buf, sizeof(buf), "Path: %.2fm", (double)totalDist);
        ui.drawString(buf, 120, 100);
      }

      drawGridMinimap(70, 115, 100, 85, -1, true);

      ui.setTextColor(TFT_GREEN);
      ui.setTextSize(1);
      ui.drawString("Click:GO!  Hold:back", 120, 205);
      ui.drawString("2s:menu", 120, 218);
      break;
    }

    default:
      break;
  }

  // Common hint for position steps (0-1)
  if (bfs_setup_step >= 0 && bfs_setup_step <= 1) {
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(1);
    ui.drawString("Dial:change  Click:next", 120, 218);
  }
}

// ========================================================================
// BFS RUN DISPLAY SCREEN
// ========================================================================
static void drawBfsRunScreen() {
  ui.fillScreen(TFT_BLACK);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("BFS RUN", 120, 30);

  // Phase indicator
  const char* phaseStr = "IDLE";
  uint16_t phaseColor = TFT_DARKGREY;
  switch (bfs_run_phase) {
    case BFS_PHASE_TURN:    phaseStr = "TURNING";  phaseColor = TFT_ORANGE;  break;
    case BFS_PHASE_DRIVE:   phaseStr = "DRIVING";  phaseColor = TFT_GREEN;   break;
    case BFS_PHASE_BRAKE:   phaseStr = "BRAKE";    phaseColor = TFT_RED;     break;
    case BFS_PHASE_SETTLE:  phaseStr = "SETTLE";   phaseColor = TFT_BLUE;    break;
    case BFS_PHASE_ARRIVED: phaseStr = "ARRIVED";  phaseColor = TFT_YELLOW;  break;
    case BFS_PHASE_DONE:    phaseStr = "DONE!";    phaseColor = TFT_MAGENTA; break;
    default: break;
  }
  ui.setTextColor(phaseColor);
  ui.setTextSize(2);
  ui.drawString(phaseStr, 120, 60);

  // Current path progress
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(1);
  if (bfs_state.path_valid && bfs_state.path_length > 0) {
    // Show: from -> to  (step N/M)
    uint8_t fromIdx = (bfs_state.path_index > 0) ? bfs_state.path[bfs_state.path_index - 1]
                                                  : bfs_state.path[0];
    uint8_t toIdx = bfs_state.path[bfs_state.path_index];
    if (bfs_run_phase == BFS_PHASE_DONE) {
      toIdx = bfs_state.path[bfs_state.path_length - 1];
      fromIdx = toIdx;
    }
    char pathBuf[48];
    snprintf(pathBuf, sizeof(pathBuf), "%s -> %s",
             bfs_state.nodes[fromIdx].name, bfs_state.nodes[toIdx].name);
    ui.drawString(pathBuf, 120, 85);

    char stepBuf[24];
    snprintf(stepBuf, sizeof(stepBuf), "Step %d / %d",
             (int)bfs_state.path_index + 1, (int)bfs_state.path_length);
    ui.drawString(stepBuf, 120, 100);
  }

  // Segment distance (include current in-progress segment)
  ui.setTextColor(TFT_WHITE);
  ui.setTextSize(3);
  char distBuf[16];
  float displayDist = bfs_run_total_driven_m + bfs_run_driven_m;
  snprintf(distBuf, sizeof(distBuf), "%.2fm", (double)displayDist);
  ui.drawString(distBuf, 120, 135);

  // Elapsed time
  float elapsed = 0.0f;
  if (bfs_run_start_ms > 0) {
    elapsed = (millis() - bfs_run_start_ms) / 1000.0f;
  }
  ui.setTextSize(2);
  char timeBuf[16];
  snprintf(timeBuf, sizeof(timeBuf), "%.1fs", (double)elapsed);
  ui.drawString(timeBuf, 120, 165);

  // Yaw info
  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  char yawBuf[32];
  snprintf(yawBuf, sizeof(yawBuf), "Yaw: %.1f tgt: %.1f",
           (double)imu_yaw, (double)bfs_run_target_yaw_deg);
  ui.drawString(yawBuf, 120, 190);

  // Encoder health + mode
  {
    const char* encModeStr = "???";
    uint16_t encModeColor = TFT_WHITE;
    switch (enc_mode) {
      case ENC_MODE_BOTH:       encModeStr = "ENC:BOTH";  encModeColor = TFT_GREEN;  break;
      case ENC_MODE_LEFT_ONLY:  encModeStr = "ENC:L";     encModeColor = TFT_YELLOW; break;
      case ENC_MODE_RIGHT_ONLY: encModeStr = "ENC:R";     encModeColor = TFT_YELLOW; break;
      case ENC_MODE_TIMED:      encModeStr = "TIMED";     encModeColor = TFT_RED;    break;
    }
    ui.setTextColor(encModeColor);
    ui.setTextSize(1);
    if (enc_read_attempts > 0) {
      uint8_t pctOk = (uint8_t)(100u - (100u * enc_read_failures / enc_read_attempts));
      char encBuf[32];
      snprintf(encBuf, sizeof(encBuf), "%s %u%% ok", encModeStr, pctOk);
      ui.drawString(encBuf, 120, 205);
    } else {
      ui.drawString(encModeStr, 120, 205);
    }
  }

  // Button hint
  ui.setTextColor(TFT_GREEN);
  ui.drawString("Click: STOP", 120, 225);
}

// ========================================================================
// Automated Self-Test
// ========================================================================
enum SelfTestPhase {
  ST_IDLE,
  ST_I2C_SCAN,
  ST_IMU_CHECK,
  ST_ENC_STATIC,         // Read encoders with motors off
  ST_MOTOR_L_START,
  ST_MOTOR_L_RUN,
  ST_MOTOR_L_STOP,
  ST_MOTOR_R_START,
  ST_MOTOR_R_RUN,
  ST_MOTOR_R_STOP,
  ST_DONE,
};

struct SelfTestResult {
  // I2C scan
  bool i2c_encL;
  bool i2c_encR;
  bool i2c_motorL;
  bool i2c_motorR;
  bool i2c_imu;
  // IMU
  bool imu_alive;
  // Encoder static (should not change when motors off)
  bool enc_static_ok;
  // Motor + encoder combined
  bool motorL_i2c_ok;     // motor L responded to I2C commands
  bool motorL_enc_moved;  // encoder registered movement when motor L ran
  int32_t motorL_pulses;  // pulses measured on LEFT encoder when LEFT motor ran
  int32_t motorL_cross;   // pulses measured on RIGHT encoder when LEFT motor ran
  bool motorR_i2c_ok;
  bool motorR_enc_moved;
  int32_t motorR_pulses;  // pulses measured on RIGHT encoder when RIGHT motor ran
  int32_t motorR_cross;   // pulses measured on LEFT encoder when RIGHT motor ran
  bool swap_detected;     // true if cross-encoder moved more than same-side encoder
  // Encoder I2C under load
  uint32_t enc_reads_total;
  uint32_t enc_reads_fail;
  uint8_t enc_pct_ok;
};

static SelfTestPhase st_phase = ST_IDLE;
static SelfTestResult st_result;
static unsigned long st_phase_start_ms = 0;
static int32_t st_enc_snapshot_L = 0;
static int32_t st_enc_snapshot_R = 0;
static uint32_t st_enc_reads = 0;
static uint32_t st_enc_fails = 0;

static const uint8_t ST_TEST_PWM = 130;       // PWM for motor spin test
static const unsigned long ST_SPIN_MS = 1500;  // How long to spin each motor
static const unsigned long ST_SETTLE_MS = 300; // Settle after stop

void selfTestReset() {
  st_phase = ST_IDLE;
}

static void selfTestStart() {
  memset(&st_result, 0, sizeof(st_result));
  st_phase = ST_I2C_SCAN;
  st_phase_start_ms = millis();
  st_enc_reads = 0;
  st_enc_fails = 0;
  stopAllMotors();
  Serial.println("=== SELF TEST START ===");
}

// Runs one tick of the self-test state machine. Called from loop().
// Returns true while test is still running.
static bool selfTestTick() {
  const unsigned long now = millis();
  const unsigned long elapsed = now - st_phase_start_ms;

  switch (st_phase) {
    case ST_IDLE:
      return false;

    case ST_I2C_SCAN: {
      // Probe all 5 expected devices
      auto probe = [](uint8_t addr) -> bool {
        Wire.beginTransmission(addr);
        return Wire.endTransmission() == 0;
      };
      st_result.i2c_encL = probe(ENCODER_L_ADDR);
      i2cGuard();
      st_result.i2c_encR = probe(ENCODER_R_ADDR);
      i2cGuard();
      st_result.i2c_motorL = probe(MOTOR_L_ADDR);
      i2cGuard();
      st_result.i2c_motorR = probe(MOTOR_R_ADDR);
      i2cGuard();
      st_result.i2c_imu = probe(IMU_6886_ADDR);
      Serial.printf("ST I2C: encL=%d encR=%d motL=%d motR=%d imu=%d\n",
        st_result.i2c_encL, st_result.i2c_encR,
        st_result.i2c_motorL, st_result.i2c_motorR, st_result.i2c_imu);
      st_phase = ST_IMU_CHECK;
      st_phase_start_ms = now;
      break;
    }

    case ST_IMU_CHECK: {
      float ax, ay, az, gx, gy, gz, temp;
      st_result.imu_alive = imu_present && imu6886Read(&ax, &ay, &az, &gx, &gy, &gz, &temp);
      Serial.printf("ST IMU: alive=%d\n", st_result.imu_alive);
      st_phase = ST_ENC_STATIC;
      st_phase_start_ms = now;
      // Take encoder snapshot
      readWheelEncodersInternal(true);
      st_enc_snapshot_L = encoderL_count;
      st_enc_snapshot_R = encoderR_count;
      break;
    }

    case ST_ENC_STATIC: {
      // Wait 500ms with motors off, check encoders didn't drift
      if (elapsed >= 500) {
        readWheelEncodersInternal(true);
        int32_t driftL = abs(encoderL_count - st_enc_snapshot_L);
        int32_t driftR = abs(encoderR_count - st_enc_snapshot_R);
        st_result.enc_static_ok = (driftL < 10 && driftR < 10);
        Serial.printf("ST ENC_STATIC: driftL=%ld driftR=%ld ok=%d\n",
          (long)driftL, (long)driftR, st_result.enc_static_ok);
        st_phase = ST_MOTOR_L_START;
        st_phase_start_ms = now;
      }
      break;
    }

    case ST_MOTOR_L_START: {
      // Snapshot BOTH encoders, start left motor
      readWheelEncodersInternal(true);
      st_enc_snapshot_L = encoderL_count;
      st_enc_snapshot_R = encoderR_count;
      st_enc_reads = 0;
      st_enc_fails = 0;
      motor_l_direction = 1;
      motor_l_speed = ST_TEST_PWM;
      motor_r_direction = 0;
      motor_r_speed = 0;
      applyMotorOutputs();
      st_result.motorL_i2c_ok = (motorL_err_dir == 0 && motorL_err_pwm == 0);
      Serial.printf("ST MOTOR_L: start pwm=%d i2c_ok=%d\n", ST_TEST_PWM, st_result.motorL_i2c_ok);
      st_phase = ST_MOTOR_L_RUN;
      st_phase_start_ms = now;
      break;
    }

    case ST_MOTOR_L_RUN: {
      // Keep reading encoders while motor spins
      readWheelEncodersInternal(true);
      st_enc_reads++;
      if (!encoderL_found) st_enc_fails++;
      // Accumulate for overall reliability
      st_result.enc_reads_total += 1;
      if (!encoderL_found || !encoderR_found) st_result.enc_reads_fail += 1;
      if (elapsed >= ST_SPIN_MS) {
        stopAllMotors();
        delay(50);
        readWheelEncodersInternal(true);
        int32_t pulsesL = encoderL_count - st_enc_snapshot_L;
        int32_t pulsesR = encoderR_count - st_enc_snapshot_R;
        // Apply inversion
        if (INVERT_LEFT_ENCODER_COUNT) pulsesL = -pulsesL;
        if (INVERT_RIGHT_ENCODER_COUNT) pulsesR = -pulsesR;
        st_result.motorL_pulses = pulsesL;  // same-side encoder
        st_result.motorL_cross = pulsesR;   // cross encoder
        st_result.motorL_enc_moved = (abs(pulsesL) > 50);
        // Detect swap: cross encoder moved more than same-side
        if (abs(pulsesR) > 50 && abs(pulsesL) < 50) {
          st_result.swap_detected = true;
        }
        Serial.printf("ST MOTOR_L: encL=%ld encR=%ld (cross) moved=%d reads=%lu fails=%lu\n",
          (long)pulsesL, (long)pulsesR, st_result.motorL_enc_moved,
          (unsigned long)st_enc_reads, (unsigned long)st_enc_fails);
        st_phase = ST_MOTOR_L_STOP;
        st_phase_start_ms = now;
      }
      break;
    }

    case ST_MOTOR_L_STOP: {
      stopAllMotors();
      if (elapsed >= ST_SETTLE_MS) {
        st_phase = ST_MOTOR_R_START;
        st_phase_start_ms = now;
      }
      break;
    }

    case ST_MOTOR_R_START: {
      readWheelEncodersInternal(true);
      st_enc_snapshot_L = encoderL_count;
      st_enc_snapshot_R = encoderR_count;
      st_enc_reads = 0;
      st_enc_fails = 0;
      motor_l_direction = 0;
      motor_l_speed = 0;
      motor_r_direction = 1;
      motor_r_speed = ST_TEST_PWM;
      applyMotorOutputs();
      st_result.motorR_i2c_ok = (motorR_err_dir == 0 && motorR_err_pwm == 0);
      Serial.printf("ST MOTOR_R: start pwm=%d i2c_ok=%d\n", ST_TEST_PWM, st_result.motorR_i2c_ok);
      st_phase = ST_MOTOR_R_RUN;
      st_phase_start_ms = now;
      break;
    }

    case ST_MOTOR_R_RUN: {
      readWheelEncodersInternal(true);
      st_enc_reads++;
      if (!encoderR_found) st_enc_fails++;
      // Accumulate total for final report
      st_result.enc_reads_total += 1;
      if (!encoderL_found || !encoderR_found) st_result.enc_reads_fail += 1;
      if (elapsed >= ST_SPIN_MS) {
        stopAllMotors();
        delay(50);
        readWheelEncodersInternal(true);
        int32_t pulsesR = encoderR_count - st_enc_snapshot_R;
        int32_t pulsesL = encoderL_count - st_enc_snapshot_L;
        if (INVERT_RIGHT_ENCODER_COUNT) pulsesR = -pulsesR;
        if (INVERT_LEFT_ENCODER_COUNT) pulsesL = -pulsesL;
        st_result.motorR_pulses = pulsesR;  // same-side encoder
        st_result.motorR_cross = pulsesL;   // cross encodePID
        st_result.motorR_enc_moved = (abs(pulsesR) > 50);
        // Detect swap: cross encoder moved more than same-side
        if (abs(pulsesL) > 50 && abs(pulsesR) < 50) {
          st_result.swap_detected = true;
        }
        Serial.printf("ST MOTOR_R: encR=%ld encL=%ld (cross) moved=%d reads=%lu fails=%lu\n",
          (long)pulsesR, (long)pulsesL, st_result.motorR_enc_moved,
          (unsigned long)st_enc_reads, (unsigned long)st_enc_fails);
        // Compute overall encoder reliability during motor runs
        if (st_result.enc_reads_total > 0) {
          st_result.enc_pct_ok = (uint8_t)(100u - (100u * st_result.enc_reads_fail / st_result.enc_reads_total));
        } else {
          st_result.enc_pct_ok = 0;
        }
        st_phase = ST_MOTOR_R_STOP;
        st_phase_start_ms = now;
      }
      break;
    }

    case ST_MOTOR_R_STOP: {
      stopAllMotors();
      if (elapsed >= ST_SETTLE_MS) {
        st_phase = ST_DONE;
        Serial.println("=== SELF TEST DONE ===");
      }
      break;
    }

    case ST_DONE:
      return false;
  }
  return true;
}

static void drawSelfTestScreen() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextDatum(middle_center);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("SELF TEST", 120, 18);

  if (st_phase != ST_DONE && st_phase != ST_IDLE) {
    // Show progress
    const char* phaseNames[] = {
      "Idle", "I2C Scan", "IMU Check", "Enc Static",
      "Motor L Init", "Motor L Run", "Motor L Stop",
      "Motor R Init", "Motor R Run", "Motor R Stop", "Done"
    };
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    ui.drawString(phaseNames[st_phase], 120, 80);

    // Progress bar
    int progress = ((int)st_phase * 100) / (int)ST_DONE;
    int barW = (200 * progress) / 100;
    ui.drawRect(20, 120, 200, 16, TFT_WHITE);
    ui.fillRect(20, 120, barW, 16, TFT_GREEN);

    ui.setTextColor(TFT_DARKGREY);
    ui.setTextSize(1);
    ui.drawString("Testing...", 120, 160);
    ui.drawString("Hold BtnA: abort", 120, 220);
    return;
  }

  if (st_phase == ST_IDLE) {
    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(2);
    ui.drawString("Click: START", 120, 120);
    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("Hold BtnA: back", 120, 220);
    return;
  }

  // ST_DONE — show results
  int y = 38;
  const int lineH = 17;
  ui.setTextSize(1);

  // I2C devices
  auto drawResult = [&](const char* label, bool pass) {
    ui.setTextColor(pass ? TFT_GREEN : TFT_RED);
    char buf[32];
    snprintf(buf, sizeof(buf), "%s: %s", label, pass ? "OK" : "FAIL");
    ui.drawString(buf, 120, y);
    y += lineH;
  };

  drawResult("I2C EncL", st_result.i2c_encL);
  drawResult("I2C EncR", st_result.i2c_encR);
  drawResult("I2C MotL", st_result.i2c_motorL);
  drawResult("I2C MotR", st_result.i2c_motorR);
  drawResult("IMU", st_result.imu_alive);
  drawResult("Enc Static", st_result.enc_static_ok);

  // Motor tests with pulse counts (same-side / cross-encoder)
  auto drawMotor = [&](const char* label, bool moved, int32_t pulses, int32_t cross) {
    ui.setTextColor(moved ? TFT_GREEN : TFT_RED);
    char buf[48];
    snprintf(buf, sizeof(buf), "%s: %s  L:%ld R:%ld", label, moved ? "OK" : "FAIL", (long)pulses, (long)cross);
    ui.drawString(buf, 120, y);
    y += lineH;
  };

  drawMotor("Mot L", st_result.motorL_enc_moved, st_result.motorL_pulses, st_result.motorL_cross);
  drawMotor("Mot R", st_result.motorR_enc_moved, st_result.motorR_cross, st_result.motorR_pulses);

  // Swap warning
  if (st_result.swap_detected) {
    ui.setTextColor(TFT_RED);
    ui.drawString("!! ENCODER SWAP !!", 120, y);
    y += lineH;
  }

  // Encoder I2C reliability under load
  uint8_t pct = st_result.enc_pct_ok;
  ui.setTextColor(pct > 90 ? TFT_GREEN : (pct > 50 ? TFT_YELLOW : TFT_RED));
  char encBuf[32];
  snprintf(encBuf, sizeof(encBuf), "Enc I2C: %u%% ok", pct);
  ui.drawString(encBuf, 120, y);
  y += lineH;

  // Overall
  bool allPass = st_result.i2c_encL && st_result.i2c_encR &&
                 st_result.i2c_motorL && st_result.i2c_motorR &&
                 st_result.imu_alive && st_result.enc_static_ok &&
                 st_result.motorL_enc_moved && st_result.motorR_enc_moved &&
                 pct > 80;
  ui.setTextColor(allPass ? TFT_GREEN : TFT_RED);
  ui.setTextSize(2);
  ui.drawString(allPass ? "ALL PASS" : "FAIL", 120, y + 5);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Click: retest  Hold: back", 120, 228);
}

// ========================================================================
// Lightweight log helpers for test modes (square / straight / turn)
// that don't use startRun() / stopRun().
// ========================================================================
static void testLogInit(const char* testName, float distM) {
  runMeta.id = runMetaNextId++;
  if (prefs_ok) prefs.putULong("run_id_next", runMetaNextId);
  runMeta.runDistanceM = distM;
  runMeta.pulsesPerMeter = pulsesPerMeter;
  runMeta.pulsesPerMeterL = pulsesPerMeterL;
  runMeta.pulsesPerMeterR = pulsesPerMeterR;
  runMeta.motor_bias_pwm = motor_bias_pwm;
  runMeta.i2c_clock_hz = i2c_clock_hz;
  runMeta.imu_present = imu_present ? 1 : 0;
  runMeta.imu_ok_at_start = imu_ok ? 1 : 0;
  runLogReset();
  runLogCount = 0;
  runLogOverflow = false;
  Serial.printf("TestLog: init id=%lu test=%s\n", (unsigned long)runMeta.id, testName);
}

static void testLogAppendSample(unsigned long t_ms, float meters, float yaw, float targetYaw,
                                float yawErr, float gz, float lateralM, float remainingM,
                                float basePwm, float steerCorr, uint8_t lPwm, uint8_t rPwm,
                                float distL, float distR, float steerI = 0.0f) {
  if (!runLog || runLogCapacity == 0) return;
  RunLogSample s;
  memset(&s, 0, sizeof(s));
  s.t_ms = (uint32_t)t_ms;
  s.meters = meters;
  s.forwardM = meters;
  s.yawRelDeg = yaw;
  s.targetYawDeg = targetYaw;
  s.yawErrDeg = yawErr;
  s.gz_dps = gz;
  s.lateralM = lateralM;
  s.remainingM = remainingM;
  s.basePWM = basePwm;
  s.corrOut = steerCorr;
  s.lPwm = lPwm;
  s.rPwm = rPwm;
  s.distL = distL;
  s.distR = distR;
  s.steerIntegral = steerI;
  runLogAppend(s);
}

// ========================================================================
// Square Drive Test
// Drives a 50cm square (4 sides, 4 right turns) and measures drift.
// Reports: per-side distance error, heading error, and final position
// offset from start (how far the robot drifted from a perfect square).
// ========================================================================
enum SqTestPhase {
  SQ_IDLE,
  SQ_CALIBRATING,   // IMU calibration at start
  SQ_TURN,          // Pivoting 90° before next side
  SQ_DRIVE,         // Driving one side of the square
  SQ_DONE,          // All 4 sides complete — show results
};

struct SqTestResult {
  float side_dist_m[4];     // actual distance driven per side
  float side_dist_err_m[4]; // error vs target per side
  float heading_after[4];   // IMU yaw after each turn+drive
  float heading_err[4];     // heading error vs expected
  float total_dist_m;       // total distance driven
  float yaw_drift_deg;      // final yaw vs expected (should be ≈ start)
  float enc_dist_L_m;       // total left encoder distance
  float enc_dist_R_m;       // total right encoder distance
  float position_err_m;     // dead-reckoned distance from start (XY)
  float pos_x_m;            // dead-reckoned X offset from start
  float pos_y_m;            // dead-reckoned Y offset from start
  unsigned long elapsed_ms; // total test time
};

static SqTestPhase sq_phase = SQ_IDLE;
static SqTestResult sq_result;
static uint8_t sq_side = 0;                  // current side (0-3)
static float sq_side_len_m = 0.50f;          // side length (adjustable with dial)
static float sq_target_yaw = 0.0f;           // target yaw for current turn
static float sq_start_yaw = 0.0f;            // yaw at test start
static int32_t sq_enc_start_L = 0;
static int32_t sq_enc_start_R = 0;
static int32_t sq_enc_total_start_L = 0;     // encoder snapshot at test start
static int32_t sq_enc_total_start_R = 0;
static float sq_driven_m = 0.0f;             // distance driven in current side
static unsigned long sq_start_ms = 0;
static unsigned long sq_drive_start_ms = 0;  // for timed fallback
static float sq_imu_vel = 0.0f;
static float sq_imu_dist = 0.0f;
static unsigned long sq_imu_last_ms = 0;
// Square-test motion tracking for no-motion auto-boost
static unsigned long lastMotionMs_sq = 0;
static int32_t lastMotionEncL_sq = 0;
static int32_t lastMotionEncR_sq = 0;
static float lastDrivenM_sq = 0.0f;
// Dead-reckoning position tracking (X=forward at start, Y=left)
static float sq_pos_x = 0.0f;
static float sq_pos_y = 0.0f;

static void sqTestReset() {
  sq_phase = SQ_IDLE;
  stopAllMotors();
}

// Extern wrapper for enterScreen() which is defined before sqTestReset()
void sqTestResetExtern() { sqTestReset(); }

// Steering PID state for square test (drive phase)
static float sq_steerI = 0.0f;
static float sq_prev_yaw_err = 0.0f;
static uint32_t sq_pid_last_us = 0;
static float sq_dFilt = 0.0f;  // low-pass filtered derivative
// Turn PID state for square test (turn phase)
static float sq_turnI = 0.0f;
static float sq_turn_prev_err = 0.0f;
static float sq_turn_dFilt = 0.0f;
static uint32_t sq_turn_pid_last_us = 0;
// Turn settle state (matches turn test logic)
static unsigned long sq_first_in_tol_ms = 0;
static unsigned long sq_turn_start_ms = 0;

static void sqTestStart() {
  memset(&sq_result, 0, sizeof(sq_result));
  sq_side = 0;
  sq_phase = SQ_CALIBRATING;
  sq_start_ms = millis();
  stopAllMotors();
  testLogInit("SQUARE", sq_side_len_m * 4.0f);
  sq_steerI = 0.0f;
  sq_prev_yaw_err = 0.0f;
  sq_pid_last_us = 0;
  sq_dFilt = 0.0f;
  sq_turnI = 0.0f;
  sq_turn_prev_err = 0.0f;
  sq_turn_dFilt = 0.0f;
  sq_turn_pid_last_us = 0;
  Serial.printf("=== SQUARE TEST START  side=%.2fm ===\n", (double)sq_side_len_m);
}

// Returns driven distance for current side using adaptive encoder mode
static float sqComputeDriven() {
  const int32_t dL = encoderL_count - sq_enc_start_L;
  const int32_t dR = encoderR_count - sq_enc_start_R;
  const float dLcorr = (INVERT_LEFT_ENCODER_COUNT  ? -(float)dL : (float)dL);
  const float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR);
  const float sL = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
  const float sR = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
  const float aL = fabsf(sL);
  const float aR = fabsf(sR);
  switch (enc_mode) {
    case ENC_MODE_BOTH:
      if (encoderL_found || encoderR_found) {
        float lhs = encoderL_found ? aL : 0.0f;
        float rhs = encoderR_found ? aR : 0.0f;
        float denom = (encoderL_found && encoderR_found) ? 2.0f : 1.0f;
        return fabsf((lhs + rhs) / denom);
      }
      // Neither encoder found — prefer IMU over timed
      if (imu_present) return sq_imu_dist;
      break;  // fall through to TIMED
    case ENC_MODE_LEFT_ONLY:
      if (encoderL_found) return aL;
      if (imu_present) return sq_imu_dist;
      break;
    case ENC_MODE_RIGHT_ONLY:
      if (encoderR_found) return aR;
      if (imu_present) return sq_imu_dist;
      break;
    default:
      break;
  }
  // Timed fallback if no encoder or IMU available
  float elapsed_s = (millis() - sq_drive_start_ms) / 1000.0f;
  return elapsed_s * ENC_TIMED_SPEED_MPS;
}

// Called from loop() each tick. Returns true while test is running.
static unsigned long sq_lastLogMs = 0;
static bool sqTestTick() {
  const unsigned long now = millis();

  switch (sq_phase) {
    case SQ_IDLE:
    case SQ_DONE:
      return false;

    case SQ_CALIBRATING: {
      // Brief IMU calibration (already includes 1.5s hands-off settle internally)
      if (imu_present) {
        calibrateImuGyroBias();
        imu_yaw = 0.0f;
      }
      readWheelEncodersForce();
      sq_start_yaw = imu_yaw;
      sq_target_yaw = sq_start_yaw;  // first side: drive straight ahead
      sq_enc_total_start_L = encoderL_count;
      sq_enc_total_start_R = encoderR_count;
      sq_enc_start_L = encoderL_count;
      sq_enc_start_R = encoderR_count;
      sq_driven_m = 0.0f;
      sq_drive_start_ms = now;
      // reset IMU integration state for distance fallback
      sq_imu_vel = 0.0f;
      sq_imu_dist = 0.0f;
      sq_imu_last_ms = now;
      sq_phase = SQ_DRIVE;  // start driving first side (no turn needed)
      // Initialize square-test motion trackers
      lastMotionEncL_sq = encoderL_count;
      lastMotionEncR_sq = encoderR_count;
      lastMotionMs_sq = now;
      lastDrivenM_sq = 0.0f;
      // Reset dead-reckoning position
      sq_pos_x = 0.0f;
      sq_pos_y = 0.0f;
      Serial.printf("SQ: calibrated yaw=%.1f, starting side 0\n", (double)sq_start_yaw);
      break;
    }

    case SQ_TURN: {
      // Pivot to face the next side — matches turn test logic
      float yaw_err = wrapDeg(sq_target_yaw - imu_yaw);

      const float SQ_TURN_TOLERANCE = 5.0f;       // degrees (matches BFS tolerance)
      const unsigned long SQ_TURN_SETTLE_MS = 300; // must hold in tolerance this long

      if (fabsf(yaw_err) < SQ_TURN_TOLERANCE) {
        // Require BOTH position in tolerance AND low angular rate (prevents coast-through)
        bool gyroStill = fabsf(imu_gz_dps) < 8.0f;
        if (sq_first_in_tol_ms == 0 && gyroStill) {
          sq_first_in_tol_ms = now;
          unsigned long elT = now - sq_turn_start_ms;
          Serial.printf("SQT%d SETTLE_START t=%lu err=%.2f gz=%.1f yaw=%.2f\n",
                        sq_side, elT, (double)yaw_err, (double)imu_gz_dps, (double)imu_yaw);
        }
        if (!gyroStill) sq_first_in_tol_ms = 0;  // reset if still rotating
        if (sq_first_in_tol_ms != 0 && (now - sq_first_in_tol_ms) >= SQ_TURN_SETTLE_MS) {
          // Turn settled — log final state
          { unsigned long elT = now - sq_turn_start_ms;
            Serial.printf("SQT%d SETTLED t=%lu err=%.2f gz=%.1f yaw=%.2f finalI=%.2f\n",
                          sq_side, elT, (double)yaw_err, (double)imu_gz_dps, (double)imu_yaw, (double)sq_turnI); }
          // IMU-aware pause (keeps imu_yaw fresh)
          stopAllMotors();
          { unsigned long t0s = millis();
            uint32_t lastImuPollUs = (uint32_t)micros();
            while ((millis() - t0s) < 150) {
              uint32_t nUs = (uint32_t)micros();
              if ((nUs - lastImuPollUs) >= 1000u) { imuIntegrateOnce(); lastImuPollUs = nUs; }
            } }
          readWheelEncodersForce();
          sq_enc_start_L = encoderL_count;
          sq_enc_start_R = encoderR_count;
          sq_driven_m = 0.0f;
          sq_drive_start_ms = millis();
          // reset IMU integration state for distance fallback
          sq_imu_vel = 0.0f;
          sq_imu_dist = 0.0f;
          sq_imu_last_ms = millis();
          sq_phase = SQ_DRIVE;
          // Initialize motion trackers for this drive segment
          lastMotionEncL_sq = encoderL_count;
          lastMotionEncR_sq = encoderR_count;
          lastMotionMs_sq = now;
          Serial.printf("SQ: turn done yaw=%.1f target=%.1f, driving side %d\n",
                  (double)imu_yaw, (double)sq_target_yaw, sq_side);
          break;
        }
        // In tolerance but still settling — stop motors and wait
        stopAllMotors();
      } else {
        // Left tolerance — reset settle timer
        sq_first_in_tol_ms = 0;

        // PID-based turn: compute real dt (microsecond precision)
        float dt;
        { uint32_t nowUs = (uint32_t)micros();
          if (sq_turn_pid_last_us == 0) {
            dt = 0.010f;
          } else {
            uint32_t dtUs = nowUs - sq_turn_pid_last_us;
            if (dtUs < 100) dtUs = 100;
            if (dtUs > 50000) dtUs = 50000;
            dt = (float)dtUs * 1e-6f;
          }
          sq_turn_pid_last_us = nowUs;
        }

        // Leaky integral (corrects steady-state bias in pivot)
        const float sqTurnKi = 2.0f;
        const float sqTurnKd = 0.05f;
        const float sqTurnILeakTau = 0.8f;
        sq_turnI -= sq_turnI * (dt / sqTurnILeakTau);
        if (fabsf(yaw_err) > 0.5f) {
          sq_turnI += yaw_err * dt;
        }
        // Anti-windup: clamp integral contribution to ±20 PWM
        const float sqTurnIMaxPwm = 20.0f;
        float iLim = sqTurnIMaxPwm / sqTurnKi;
        if (sq_turnI > iLim) sq_turnI = iLim;
        if (sq_turnI < -iLim) sq_turnI = -iLim;

        // Low-pass filtered derivative
        float rawDeriv = (yaw_err - sq_turn_prev_err) / dt;
        sq_turn_prev_err = yaw_err;
        const float sqTurnDFilterTau = 0.030f;
        float dAlpha = dt / (sqTurnDFilterTau + dt);
        sq_turn_dFilt += dAlpha * (rawDeriv - sq_turn_dFilt);

        // Base PWM from proportional scaling
        float absErr = fabsf(yaw_err);
        float basePwm;
        if (absErr > 45.0f) {
          basePwm = BFS_TURN_PWM;  // full speed for large errors
        } else if (absErr > 15.0f) {
          basePwm = 110.0f + (absErr - 15.0f) * (BFS_TURN_PWM - 110.0f) / 30.0f;
        } else if (absErr > 5.0f) {
          // Crawl zone: ramp from 110 down to 100 between 15° and 5°
          float t = (absErr - 5.0f) / 10.0f;
          basePwm = 100.0f + t * 10.0f;
        } else {
          // Fine zone: stays at 100 (motor minimum)
          basePwm = 100.0f;
        }

        // I and D corrections: sign-aware relative to error direction.
        float errSign = (yaw_err > 0.0f) ? 1.0f : -1.0f;
        float correction = (sqTurnKi * sq_turnI + sqTurnKd * sq_turn_dFilt) * errSign;
        float turnPwmF = basePwm + correction;
        if (turnPwmF > 200.0f) turnPwmF = 200.0f;
        if (turnPwmF < 100) turnPwmF = 100;

        int sign = (yaw_err > 0.0f) ? 1 : -1;
        setMotorsPivotPwm(sign, (uint8_t)(turnPwmF + 0.5f));
        applyMotorOutputs();

        // Detailed turn logging every ~50ms
        static unsigned long sq_turn_last_log_ms = 0;
        if ((now - sq_turn_last_log_ms) >= 50u) {
          sq_turn_last_log_ms = now;
          unsigned long elT = now - sq_turn_start_ms;
          Serial.printf("SQT%d t=%lu err=%.2f yaw=%.2f tgt=%.2f gz=%.1f pwm=%d I=%.2f D=%.1f base=%.0f corr=%.1f\n",
                        sq_side, elT, (double)yaw_err, (double)imu_yaw, (double)sq_target_yaw,
                        (double)imu_gz_dps, (int)(turnPwmF + 0.5f),
                        (double)sq_turnI, (double)sq_turn_dFilt, (double)basePwm, (double)correction);
        }
      }

      // Timeout safety
      if ((now - sq_turn_start_ms) > 5000) {
        stopAllMotors();
        { unsigned long t0s = millis();
          uint32_t lastImuPollUs = (uint32_t)micros();
          while ((millis() - t0s) < 150) {
            uint32_t nUs = (uint32_t)micros();
            if ((nUs - lastImuPollUs) >= 1000u) { imuIntegrateOnce(); lastImuPollUs = nUs; }
          } }
        readWheelEncodersForce();
        sq_enc_start_L = encoderL_count;
        sq_enc_start_R = encoderR_count;
        sq_driven_m = 0.0f;
        sq_drive_start_ms = millis();
        sq_imu_vel = 0.0f;
        sq_imu_dist = 0.0f;
        sq_imu_last_ms = millis();
        sq_phase = SQ_DRIVE;
        lastMotionEncL_sq = encoderL_count;
        lastMotionEncR_sq = encoderR_count;
        lastMotionMs_sq = now;
        Serial.printf("SQ: turn TIMEOUT yaw=%.1f target=%.1f, forcing drive side %d\n",
                (double)imu_yaw, (double)sq_target_yaw, sq_side);
      }
      break;
    }

    case SQ_DRIVE: {
      // Update IMU-based distance integration (failsafe)
      if (imu_present) {
        const unsigned long nowMs = millis();
        float dt = (nowMs - sq_imu_last_ms) / 1000.0f;
        if (dt > 0.0f && dt < 0.5f) {
          const float horiz_g = sqrtf(imu_ax_g * imu_ax_g + imu_ay_g * imu_ay_g);
          const float accel_g = (horiz_g > 0.02f) ? horiz_g : 0.0f;
          const float accel_ms2 = accel_g * 9.80665f;
          sq_imu_vel += accel_ms2 * dt;
          sq_imu_dist += sq_imu_vel * dt;
        }
        sq_imu_last_ms = nowMs;
      }
      readWheelEncodersForce();
      sq_driven_m = sqComputeDriven();

      if (sq_driven_m >= sq_side_len_m - 0.08f) {
        // Side complete — closed-loop reverse brake stops each wheel when its encoder reaches zero
        brakeToStop(140, 400);
        // IMU-aware settle wait — keeps imu_yaw fresh during pause
        { unsigned long t0settle = millis();
          uint32_t lastImuPollUs = (uint32_t)micros();
          while ((millis() - t0settle) < 300) {
            uint32_t nUs = (uint32_t)micros();
            if ((nUs - lastImuPollUs) >= 1000u) { imuIntegrateOnce(); lastImuPollUs = nUs; }
          } }

        sq_result.side_dist_m[sq_side] = sq_driven_m;
        sq_result.side_dist_err_m[sq_side] = sq_driven_m - sq_side_len_m;
        sq_result.heading_after[sq_side] = imu_yaw;
        // Expected heading after side N: start_yaw - N*90 (turning right = negative yaw for CCW-positive IMU)
        float expected_yaw = wrapDeg(sq_start_yaw - (float)(sq_side) * 90.0f);
        sq_result.heading_err[sq_side] = wrapDeg(imu_yaw - expected_yaw);

        Serial.printf("SQ: side %d done dist=%.3f err=%.3f yaw=%.1f yawErr=%.1f\n",
                      sq_side, (double)sq_driven_m, (double)sq_result.side_dist_err_m[sq_side],
                      (double)imu_yaw, (double)sq_result.heading_err[sq_side]);

        // Update dead-reckoned position: heading during this side is the target yaw
        {
          float headingRad = sq_target_yaw * (M_PI / 180.0f);
          sq_pos_x += sq_driven_m * cosf(headingRad);
          sq_pos_y += sq_driven_m * sinf(headingRad);
          Serial.printf("SQ: DR pos (%.3f, %.3f)m after side %d\n",
                        (double)sq_pos_x, (double)sq_pos_y, sq_side);
        }

        sq_side++;
        if (sq_side >= 4) {
          // All 4 sides complete — compute results directly (no final turn)
          stopAllMotors();
          sq_result.elapsed_ms = now - sq_start_ms;
          sq_result.total_dist_m = 0;
          for (int i = 0; i < 4; i++) sq_result.total_dist_m += sq_result.side_dist_m[i];
          sq_result.yaw_drift_deg = wrapDeg(imu_yaw - sq_start_yaw);
          readWheelEncodersForce();
          int32_t totDL = encoderL_count - sq_enc_total_start_L;
          int32_t totDR = encoderR_count - sq_enc_total_start_R;
          float totDLcorr = (INVERT_LEFT_ENCODER_COUNT ? -(float)totDL : (float)totDL);
          float totDRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)totDR : (float)totDR);
          sq_result.enc_dist_L_m = (pulsesPerMeterL > 1.0f) ? (totDLcorr / pulsesPerMeterL) : 0.0f;
          sq_result.enc_dist_R_m = (pulsesPerMeterR > 1.0f) ? (totDRcorr / pulsesPerMeterR) : 0.0f;
          sq_result.pos_x_m = sq_pos_x;
          sq_result.pos_y_m = sq_pos_y;
          sq_result.position_err_m = sqrtf(sq_pos_x * sq_pos_x + sq_pos_y * sq_pos_y);
          sq_phase = SQ_DONE;
          runLogSaveToSpiffs();
          Serial.printf("SQ: DONE total=%.2fm yawDrift=%.1f posErr=%.3fm (%.3f,%.3f) encL=%.2f encR=%.2f time=%lums\n",
                        (double)sq_result.total_dist_m, (double)sq_result.yaw_drift_deg,
                        (double)sq_result.position_err_m, (double)sq_result.pos_x_m, (double)sq_result.pos_y_m,
                        (double)sq_result.enc_dist_L_m, (double)sq_result.enc_dist_R_m,
                        sq_result.elapsed_ms);
        } else {
          // Turn 90° right relative to fresh imu_yaw (kept current by
          // imuIntegrateOnce() during brakeToStop + settle wait above).
          sq_target_yaw = imu_yaw - 90.0f;
          sq_steerI = 0.0f;
          sq_prev_yaw_err = 0.0f;
          sq_pid_last_us = 0;
          sq_dFilt = 0.0f;
          sq_turnI = 0.0f;
          // Initialize prev_err to actual initial error to avoid derivative spike
          sq_turn_prev_err = wrapDeg(sq_target_yaw - imu_yaw);
          sq_turn_dFilt = 0.0f;
          sq_turn_pid_last_us = 0;
          sq_first_in_tol_ms = 0;
          sq_turn_start_ms = millis();
          sq_phase = SQ_TURN;
          Serial.printf("SQ: turning to %.1f for side %d (initErr=%.1f)\n", (double)sq_target_yaw, sq_side,
                        (double)sq_turn_prev_err);
        }
      } else {
        // Drive with yaw PID correction (P + I + D)
        float yaw_err = wrapDeg(sq_target_yaw - imu_yaw);

        // Leaky integral for steady-state bias correction
        const float sq_steerKi = 4.0f;
        const float sq_steerKd = 0.20f;
        const float sq_iLeakTau = 1.0f;
        // Measure real dt; clamp to [100us, 50ms] to prevent spikes after stalls
        float dt;
        { uint32_t nowUs = (uint32_t)micros();
          if (sq_pid_last_us == 0) {
            dt = 0.010f;
          } else {
            uint32_t dtUs = nowUs - sq_pid_last_us;
            if (dtUs < 100) dtUs = 100;
            if (dtUs > 50000) dtUs = 50000;
            dt = (float)dtUs * 1e-6f;
          }
          sq_pid_last_us = nowUs;
        }
        sq_steerI -= sq_steerI * (dt / sq_iLeakTau);
        if (fabsf(yaw_err) > 0.1f) {
          sq_steerI += yaw_err * dt;
        }
        const float sq_iMaxCorr = 25.0f;
        const float sqILim = sq_iMaxCorr / sq_steerKi;
        if (sq_steerI > sqILim) sq_steerI = sqILim;
        if (sq_steerI < -sqILim) sq_steerI = -sqILim;

        // Low-pass filtered derivative (alpha ~0.3 at 10ms, adapts with real dt)
        float rawDeriv = (yaw_err - sq_prev_yaw_err) / dt;
        sq_prev_yaw_err = yaw_err;
        const float dFilterTau = 0.030f; // 30ms time constant
        float dAlpha = dt / (dFilterTau + dt);
        sq_dFilt += dAlpha * (rawDeriv - sq_dFilt);

        // Boost P gain during deceleration
        float remaining = sq_side_len_m - sq_driven_m;
        float effectiveKp = BFS_DRIVE_STEER_KP;
        if (remaining < 0.30f) {
          effectiveKp = BFS_DRIVE_STEER_KP * 1.5f;
        }

        float steerCorr = effectiveKp * yaw_err
                        + sq_steerKi * sq_steerI
                        + sq_steerKd * sq_dFilt;
        if (steerCorr > 60.0f) steerCorr = 60.0f;
        if (steerCorr < -60.0f) steerCorr = -60.0f;

        // Decel near end — short zone + high floor to maintain steering authority
        float speedFactor = 1.0f;
        if (remaining < 0.20f) {
          speedFactor = remaining / 0.20f;
          if (speedFactor < 0.30f) speedFactor = 0.30f;
        }

        float basePwm = BFS_DRIVE_PWM * speedFactor;
        if (basePwm < 110.0f) basePwm = 110.0f;

        float leftPwm = basePwm - steerCorr + (float)motor_bias_pwm;
        float rightPwm = basePwm + steerCorr - (float)motor_bias_pwm;
        if (leftPwm < 0.0f) leftPwm = 0.0f;
        if (rightPwm < 0.0f) rightPwm = 0.0f;
        if (leftPwm > 255.0f) leftPwm = 255.0f;
        if (rightPwm > 255.0f) rightPwm = 255.0f;
        // Prevent one-wheel stall: clamp above hardware minimum
        if (leftPwm > 0.0f && leftPwm < 100.0f) leftPwm = 100.0f;
        if (rightPwm > 0.0f && rightPwm < 110.0f) rightPwm = 110.0f;

        // Motion detection + adaptive auto-boost for Square Test
        {
          // Prefer distance-based detection (handles low-pulse movement); fallback to encoder-pulse check
          const float MIN_DRIVEN_MOTION_M = 0.005f; // 5 mm
          float drivenDelta = sq_driven_m - lastDrivenM_sq;
          if (drivenDelta >= MIN_DRIVEN_MOTION_M) {
            lastDrivenM_sq = sq_driven_m;
            lastMotionEncL_sq = encoderL_count;
            lastMotionEncR_sq = encoderR_count;
            lastMotionMs_sq = now;
          } else {
            const int32_t dML = encoderL_count - lastMotionEncL_sq;
            const int32_t dMR = encoderR_count - lastMotionEncR_sq;
            const float motionAbs = (fabsf((float)dML) + fabsf((float)dMR)) * 0.5f;
            if (motionAbs >= (float)RUN_NO_MOTION_MIN_PULSES) {
              lastMotionEncL_sq = encoderL_count;
              lastMotionEncR_sq = encoderR_count;
              lastMotionMs_sq = now;
              lastDrivenM_sq = sq_driven_m;
            }
          }

          const uint32_t sinceEncMoveMs_sq = (lastMotionMs_sq != 0) ? ((uint32_t)now - (uint32_t)lastMotionMs_sq) : 0u;
          if (sinceEncMoveMs_sq >= NO_MOTION_RAMP_DELAY_MS && sinceEncMoveMs_sq < (uint32_t)RUN_NO_MOTION_TIMEOUT_MS) {
            float targetRamp = calcNoMotionRampPwm(0.0f, sinceEncMoveMs_sq);
            const float MAX_SQ_SAFE_PWM = 160.0f;
            if (leftPwm > 0.0f) {
              leftPwm = fmaxf(leftPwm, targetRamp);
              if (leftPwm > MAX_SQ_SAFE_PWM) leftPwm = MAX_SQ_SAFE_PWM;
            }
            if (rightPwm > 0.0f) {
              rightPwm = fmaxf(rightPwm, targetRamp);
              if (rightPwm > MAX_SQ_SAFE_PWM) rightPwm = MAX_SQ_SAFE_PWM;
            }
          }
        }

        setMotorsForwardPwm((uint8_t)(leftPwm + 0.5f), (uint8_t)(rightPwm + 0.5f));
        applyMotorOutputs();

        // Log sample every ~100ms
        if ((now - sq_lastLogMs) >= 100u) {
          sq_lastLogMs = now;
          float totDriven = 0;
          for (int i2 = 0; i2 < sq_side; i2++) totDriven += sq_result.side_dist_m[i2];
          totDriven += sq_driven_m;
          float rem = sq_side_len_m - sq_driven_m;
          const int32_t dL3 = encoderL_count - sq_enc_total_start_L;
          const int32_t dR3 = encoderR_count - sq_enc_total_start_R;
          float dLm3 = (INVERT_LEFT_ENCODER_COUNT ? -(float)dL3 : (float)dL3);
          float dRm3 = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR3 : (float)dR3);
          if (pulsesPerMeterL > 1.0f) dLm3 /= pulsesPerMeterL;
          if (pulsesPerMeterR > 1.0f) dRm3 /= pulsesPerMeterR;
          testLogAppendSample(now - sq_start_ms, totDriven, imu_yaw, sq_target_yaw,
                              yaw_err, imu_gz_dps, 0, rem,
                              basePwm, steerCorr,
                              (uint8_t)(leftPwm + 0.5f), (uint8_t)(rightPwm + 0.5f),
                              dLm3, dRm3, sq_steerI);
        }
      }
      break;
    }

    default:
      break;
  }
  return true;
}

static void drawSquareTestScreen() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextDatum(middle_center);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("SQ TEST", 120, 18);

  if (sq_phase == SQ_IDLE) {
    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(1);
    ui.drawString("Drives a square to", 120, 55);
    ui.drawString("measure drift/errors", 120, 70);

    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    char sideBuf[16];
    snprintf(sideBuf, sizeof(sideBuf), "%.2fm", (double)sq_side_len_m);
    ui.drawString(sideBuf, 120, 105);
    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("Dial: adjust side len", 120, 128);

    // Show encoder mode
    const char* modeStr = "???";
    switch (enc_mode) {
      case ENC_MODE_BOTH:       modeStr = "Enc: BOTH";  break;
      case ENC_MODE_LEFT_ONLY:  modeStr = "Enc: LEFT";  break;
      case ENC_MODE_RIGHT_ONLY: modeStr = "Enc: RIGHT"; break;
      case ENC_MODE_TIMED:      modeStr = "Enc: TIMED"; break;
    }
    ui.setTextColor(enc_mode == ENC_MODE_TIMED ? TFT_RED : TFT_GREEN);
    ui.drawString(modeStr, 120, 150);

    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(2);
    ui.drawString("Click: GO", 120, 185);
    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("Hold: back", 120, 220);
    return;
  }

  if (sq_phase == SQ_CALIBRATING) {
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    ui.drawString("Calibrating...", 120, 120);
    return;
  }

  if (sq_phase == SQ_TURN || sq_phase == SQ_DRIVE) {
    // Running — show live status
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(1);
    char phaseBuf[24];
    snprintf(phaseBuf, sizeof(phaseBuf), "%s side %d/4",
             sq_phase == SQ_TURN ? "TURNING" : "DRIVING", sq_side + 1);
    ui.drawString(phaseBuf, 120, 45);

    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(2);
    char distBuf[16];
    snprintf(distBuf, sizeof(distBuf), "%.3fm", (double)sq_driven_m);
    ui.drawString(distBuf, 120, 75);

    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    char yawBuf[32];
    snprintf(yawBuf, sizeof(yawBuf), "Yaw: %.1f  tgt: %.1f", (double)imu_yaw, (double)sq_target_yaw);
    ui.drawString(yawBuf, 120, 100);

    // Show completed sides
    for (int i = 0; i < (int)sq_side; i++) {
      char buf[32];
      snprintf(buf, sizeof(buf), "S%d: %.3fm  err:%.3f", i + 1,
               (double)sq_result.side_dist_m[i], (double)sq_result.side_dist_err_m[i]);
      ui.setTextColor(fabsf(sq_result.side_dist_err_m[i]) < 0.05f ? TFT_GREEN : TFT_YELLOW);
      ui.drawString(buf, 120, 125 + i * 15);
    }

    // Elapsed
    float elapsed = (millis() - sq_start_ms) / 1000.0f;
    ui.setTextColor(TFT_DARKGREY);
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%.1fs", (double)elapsed);
    ui.drawString(timeBuf, 120, 200);

    ui.setTextColor(TFT_RED);
    ui.drawString("Click: ABORT", 120, 225);
    return;
  }

  // SQ_DONE — results screen
  int y = 40;
  ui.setTextSize(1);

  // Per-side results
  for (int i = 0; i < 4; i++) {
    char buf[48];
    snprintf(buf, sizeof(buf), "S%d: %.3fm  dErr:%.3f  hErr:%.1f",
             i + 1, (double)sq_result.side_dist_m[i],
             (double)sq_result.side_dist_err_m[i],
             (double)sq_result.heading_err[i]);
    bool distOk = fabsf(sq_result.side_dist_err_m[i]) < 0.05f;
    bool headOk = fabsf(sq_result.heading_err[i]) < 10.0f;
    ui.setTextColor((distOk && headOk) ? TFT_GREEN : TFT_YELLOW);
    ui.drawString(buf, 120, y);
    y += 14;
  }

  y += 4;
  // Totals
  char totalBuf[32];
  snprintf(totalBuf, sizeof(totalBuf), "Total: %.2fm (%.2f exp)",
           (double)sq_result.total_dist_m, (double)(sq_side_len_m * 4.0f));
  ui.setTextColor(TFT_WHITE);
  ui.drawString(totalBuf, 120, y);
  y += 14;

  // Yaw drift
  char yawBuf[32];
  snprintf(yawBuf, sizeof(yawBuf), "Yaw drift: %.1f deg", (double)sq_result.yaw_drift_deg);
  ui.setTextColor(fabsf(sq_result.yaw_drift_deg) < 15.0f ? TFT_GREEN : TFT_RED);
  ui.drawString(yawBuf, 120, y);
  y += 14;

  // Position error (dead-reckoned return-to-start distance)
  char posBuf[48];
  snprintf(posBuf, sizeof(posBuf), "Pos err: %.0fmm (%.0f,%.0f)",
           (double)(sq_result.position_err_m * 1000.0f),
           (double)(sq_result.pos_x_m * 1000.0f),
           (double)(sq_result.pos_y_m * 1000.0f));
  ui.setTextColor(sq_result.position_err_m < 0.05f ? TFT_GREEN :
                  sq_result.position_err_m < 0.10f ? TFT_YELLOW : TFT_RED);
  ui.drawString(posBuf, 120, y);
  y += 14;

  // Encoder L/R totals
  char encBuf[48];
  snprintf(encBuf, sizeof(encBuf), "EncL: %.2fm  EncR: %.2fm",
           (double)sq_result.enc_dist_L_m, (double)sq_result.enc_dist_R_m);
  ui.setTextColor(TFT_BLUE);
  ui.drawString(encBuf, 120, y);
  y += 14;

  // Time
  char timeBuf[24];
  snprintf(timeBuf, sizeof(timeBuf), "Time: %.1fs", sq_result.elapsed_ms / 1000.0f);
  ui.setTextColor(TFT_DARKGREY);
  ui.drawString(timeBuf, 120, y);
  y += 14;

  // Overall verdict
  float avgDistErr = 0;
  for (int i = 0; i < 4; i++) avgDistErr += fabsf(sq_result.side_dist_err_m[i]);
  avgDistErr /= 4.0f;
  bool pass = avgDistErr < 0.05f && fabsf(sq_result.yaw_drift_deg) < 20.0f;
  ui.setTextColor(pass ? TFT_GREEN : TFT_RED);
  ui.setTextSize(2);
  ui.drawString(pass ? "PASS" : "DRIFT!", 120, y + 5);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Click: retest  Hold: back", 120, 228);
}

// ========================================================================
// STRAIGHT-LINE TEST
// Drives forward a configurable distance to verify straightness.
// Reports yaw drift, L/R encoder mismatch, and IMU-derived lateral drift.
// ========================================================================
enum StraightTestPhase {
  STR_IDLE,
  STR_CALIBRATING,
  STR_DRIVING,
  STR_DONE,
};

struct StraightTestResult {
  float target_m;
  float driven_m;
  float dist_err_m;
  float yaw_drift_deg;
  float enc_L_m;
  float enc_R_m;
  float enc_diff_mm;     // |L - R| in mm
  float lateral_drift_m; // estimated sideways drift from IMU
  unsigned long elapsed_ms;
};

static StraightTestPhase str_phase = STR_IDLE;
static StraightTestResult str_result;
static float str_target_m = 1.00f;
static float str_start_yaw = 0.0f;
static int32_t str_enc_start_L = 0;
static int32_t str_enc_start_R = 0;
static unsigned long str_start_ms = 0;
static float str_driven_m = 0.0f;
static float str_lateral_m = 0.0f;
// Motion tracking for auto-boost
static unsigned long str_lastMotionMs = 0;
static int32_t str_lastMotionEncL = 0;
static int32_t str_lastMotionEncR = 0;
static float str_lastDrivenM = 0.0f;
static float str_lateral_prev_driven = 0.0f;
static unsigned long str_lastLogMs = 0;
// Steering PID state for straight test
static float str_steerI = 0.0f;
static float str_prev_yaw_err = 0.0f;
static uint32_t str_pid_last_us = 0;
static float str_dFilt = 0.0f;  // low-pass filtered derivative

static void strTestReset() {
  str_phase = STR_IDLE;
  stopAllMotors();
}
void strTestResetExtern() { strTestReset(); }

static void strTestStart() {
  memset(&str_result, 0, sizeof(str_result));
  str_result.target_m = str_target_m;
  str_phase = STR_CALIBRATING;
  str_start_ms = millis();
  stopAllMotors();
  testLogInit("STRAIGHT", str_target_m);
  str_lastLogMs = 0;
  str_steerI = 0.0f;
  str_prev_yaw_err = 0.0f;
  str_pid_last_us = 0;
  str_dFilt = 0.0f;
  Serial.printf("=== STRAIGHT TEST START  target=%.2fm ===\n", (double)str_target_m);
}

static float strComputeDriven() {
  const int32_t dL = encoderL_count - str_enc_start_L;
  const int32_t dR = encoderR_count - str_enc_start_R;
  const float dLcorr = (INVERT_LEFT_ENCODER_COUNT  ? -(float)dL : (float)dL);
  const float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR);
  const float sL = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
  const float sR = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
  if (encoderL_found && encoderR_found) return (fabsf(sL) + fabsf(sR)) * 0.5f;
  if (encoderL_found) return fabsf(sL);
  if (encoderR_found) return fabsf(sR);
  return 0.0f;
}

static bool strTestTick() {
  const unsigned long now = millis();
  switch (str_phase) {
    case STR_IDLE:
    case STR_DONE:
      return false;

    case STR_CALIBRATING: {
      if (imu_present) {
        calibrateImuGyroBias();
        imu_yaw = 0.0f;
      }
      readWheelEncodersForce();
      str_start_yaw = imu_yaw;
      str_enc_start_L = encoderL_count;
      str_enc_start_R = encoderR_count;
      str_driven_m = 0.0f;
      str_lateral_m = 0.0f;
      str_lastMotionEncL = encoderL_count;
      str_lastMotionEncR = encoderR_count;
      str_lastMotionMs = now;
      str_lastDrivenM = 0.0f;
      str_lateral_prev_driven = 0.0f;
      str_phase = STR_DRIVING;
      Serial.printf("STR: calibrated yaw=%.1f, driving %.2fm\n", (double)str_start_yaw, (double)str_target_m);
      break;
    }

    case STR_DRIVING: {
      readWheelEncodersForce();
      str_driven_m = strComputeDriven();

      // Accumulate lateral drift from IMU heading
      if (imu_present) {
        float headingRad = imu_yaw * (M_PI / 180.0f);
        float dd = str_driven_m - str_lateral_prev_driven;
        if (dd > 0.0f) {
          str_lateral_m += dd * sinf(headingRad);
        }
        str_lateral_prev_driven = str_driven_m;
      }

      if (str_driven_m >= str_target_m - 0.08f) {
        // Done — closed-loop reverse brake stops each wheel when its encoder reaches zero
        brakeToStop(140, 400);
        // IMU-aware settle wait
        { unsigned long t0settle = millis();
          uint32_t lastImuPollUs = (uint32_t)micros();
          while ((millis() - t0settle) < 300) {
            uint32_t nUs = (uint32_t)micros();
            if ((nUs - lastImuPollUs) >= 1000u) { imuIntegrateOnce(); lastImuPollUs = nUs; }
          } }

        const int32_t totDL = encoderL_count - str_enc_start_L;
        const int32_t totDR = encoderR_count - str_enc_start_R;
        float dLcorr = (INVERT_LEFT_ENCODER_COUNT ? -(float)totDL : (float)totDL);
        float dRcorr = (INVERT_RIGHT_ENCODER_COUNT ? -(float)totDR : (float)totDR);
        str_result.enc_L_m = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
        str_result.enc_R_m = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
        str_result.enc_diff_mm = fabsf(str_result.enc_L_m - str_result.enc_R_m) * 1000.0f;
        str_result.driven_m = str_driven_m;
        str_result.dist_err_m = str_driven_m - str_target_m;
        str_result.yaw_drift_deg = imu_yaw - str_start_yaw;
        str_result.lateral_drift_m = str_lateral_m;
        str_result.elapsed_ms = now - str_start_ms;

        str_phase = STR_DONE;
        runLogSaveToSpiffs();
        Serial.printf("STR: DONE dist=%.3f err=%.3f yaw=%.1f latDrift=%.3f encDiff=%.1fmm time=%lums\n",
                      (double)str_result.driven_m, (double)str_result.dist_err_m,
                      (double)str_result.yaw_drift_deg, (double)str_result.lateral_drift_m,
                      (double)str_result.enc_diff_mm, str_result.elapsed_ms);
      } else {
        // Drive with yaw PID correction (P + I + D)
        float yaw_err = wrapDeg(str_start_yaw - imu_yaw);

        // Leaky integral to fight steady-state drift from motor/encoder asymmetry
        const float str_steerKi = 0.8f;
        const float str_steerKd = 0.05f;
        const float str_iLeakTau = 1.0f;
        // Measure real dt; clamp to [100us, 50ms] to prevent spikes after stalls
        float dt;
        { uint32_t nowUs = (uint32_t)micros();
          if (str_pid_last_us == 0) {
            dt = 0.010f;
          } else {
            uint32_t dtUs = nowUs - str_pid_last_us;
            if (dtUs < 100) dtUs = 100;
            if (dtUs > 50000) dtUs = 50000;
            dt = (float)dtUs * 1e-6f;
          }
          str_pid_last_us = nowUs;
        }
        str_steerI -= str_steerI * (dt / str_iLeakTau);
        if (fabsf(yaw_err) > 0.1f) {
          str_steerI += yaw_err * dt;
        }
        // Anti-windup: cap I contribution
        const float str_iMaxCorr = 25.0f;
        const float iLim = str_iMaxCorr / str_steerKi;
        if (str_steerI > iLim) str_steerI = iLim;
        if (str_steerI < -iLim) str_steerI = -iLim;

        // Low-pass filtered derivative (alpha ~0.3 at 10ms, adapts with real dt)
        float rawDeriv = (yaw_err - str_prev_yaw_err) / dt;
        str_prev_yaw_err = yaw_err;
        const float dFilterTau = 0.030f; // 30ms time constant
        float dAlpha = dt / (dFilterTau + dt);
        str_dFilt += dAlpha * (rawDeriv - str_dFilt);

        // Boost P gain during deceleration to maintain steering authority at low PWM
        float remaining = str_target_m - str_driven_m;
        float effectiveKp = BFS_DRIVE_STEER_KP;
        if (remaining < 0.30f) {
          effectiveKp = BFS_DRIVE_STEER_KP * 1.5f;
        }

        float steerCorr = effectiveKp * yaw_err
                        + str_steerKi * str_steerI
                        + str_steerKd * str_dFilt;
        if (steerCorr > 30.0f) steerCorr = 30.0f;
        if (steerCorr < -30.0f) steerCorr = -30.0f;

        // Decel near end — short zone + high floor to maintain steering authority
        float speedFactor = 1.0f;
        if (remaining < 0.20f) {
          speedFactor = remaining / 0.20f;
          if (speedFactor < 0.30f) speedFactor = 0.30f;
        }

        float basePwm = BFS_DRIVE_PWM * speedFactor;
        if (basePwm < 110.0f) basePwm = 110.0f;

        // Apply steering correction (inverted for correct left/right response)
        float leftPwm = basePwm + steerCorr + (float)motor_bias_pwm;
        float rightPwm = basePwm - steerCorr - (float)motor_bias_pwm;
        if (leftPwm < 0.0f) leftPwm = 0.0f;
        if (rightPwm < 0.0f) rightPwm = 0.0f;
        if (leftPwm > 255.0f) leftPwm = 255.0f;
        if (rightPwm > 255.0f) rightPwm = 255.0f;
        // Allow steering to reduce one motor below basePwm (but not below 0)

        // Auto-boost if not moving
        {
          const float MIN_DRIVEN_MOTION_M = 0.005f;
          float drivenDelta = str_driven_m - str_lastDrivenM;
          if (drivenDelta >= MIN_DRIVEN_MOTION_M) {
            str_lastDrivenM = str_driven_m;
            str_lastMotionEncL = encoderL_count;
            str_lastMotionEncR = encoderR_count;
            str_lastMotionMs = now;
          }
          const uint32_t sinceMotionMs = (str_lastMotionMs != 0) ? ((uint32_t)now - (uint32_t)str_lastMotionMs) : 0u;
          if (sinceMotionMs >= NO_MOTION_RAMP_DELAY_MS && sinceMotionMs < (uint32_t)RUN_NO_MOTION_TIMEOUT_MS) {
            float targetRamp = calcNoMotionRampPwm(0.0f, sinceMotionMs);
            if (leftPwm > 0.0f) {
              leftPwm = fmaxf(leftPwm, targetRamp);
              if (leftPwm > 255.0f) leftPwm = 255.0f;
            }
            if (rightPwm > 0.0f) {
              rightPwm = fmaxf(rightPwm, targetRamp);
              if (rightPwm > 255.0f) rightPwm = 255.0f;
            }
          }
        }

        setMotorsForwardPwm((uint8_t)(leftPwm + 0.5f), (uint8_t)(rightPwm + 0.5f));
        applyMotorOutputs();

        // Log sample every ~100ms
        if ((now - str_lastLogMs) >= 100u) {
          str_lastLogMs = now;
          float yawErr = str_start_yaw - imu_yaw;
          float rem = str_target_m - str_driven_m;
          const int32_t dL2 = encoderL_count - str_enc_start_L;
          const int32_t dR2 = encoderR_count - str_enc_start_R;
          float dLm = (INVERT_LEFT_ENCODER_COUNT ? -(float)dL2 : (float)dL2);
          float dRm = (INVERT_RIGHT_ENCODER_COUNT ? -(float)dR2 : (float)dR2);
          if (pulsesPerMeterL > 1.0f) dLm /= pulsesPerMeterL;
          if (pulsesPerMeterR > 1.0f) dRm /= pulsesPerMeterR;
          testLogAppendSample(now - str_start_ms, str_driven_m, imu_yaw, str_start_yaw,
                              yawErr, imu_gz_dps, str_lateral_m, rem,
                              basePwm, steerCorr,
                              (uint8_t)(leftPwm + 0.5f), (uint8_t)(rightPwm + 0.5f),
                              dLm, dRm, str_steerI);
        }
      }
      break;
    }

    default:
      break;
  }
  return true;
}

static void drawStraightTestScreen() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextDatum(middle_center);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("STRAIGHT", 120, 18);

  if (str_phase == STR_IDLE) {
    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(1);
    ui.drawString("Drive straight to test", 120, 50);
    ui.drawString("heading hold accuracy", 120, 65);

    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2fm", (double)str_target_m);
    ui.drawString(buf, 120, 100);
    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("Dial: adjust distance", 120, 125);

    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(2);
    ui.drawString("Click: GO", 120, 170);
    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("Hold: back", 120, 210);
    return;
  }

  if (str_phase == STR_CALIBRATING) {
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    ui.drawString("Calibrating...", 120, 120);
    return;
  }

  if (str_phase == STR_DRIVING) {
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(1);
    ui.drawString("DRIVING", 120, 45);

    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(2);
    char distBuf[16];
    snprintf(distBuf, sizeof(distBuf), "%.3fm", (double)str_driven_m);
    ui.drawString(distBuf, 120, 80);

    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    char yawBuf[32];
    snprintf(yawBuf, sizeof(yawBuf), "Yaw: %.1f deg", (double)imu_yaw);
    ui.drawString(yawBuf, 120, 110);

    float elapsed = (millis() - str_start_ms) / 1000.0f;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%.1fs", (double)elapsed);
    ui.drawString(timeBuf, 120, 180);

    ui.setTextColor(TFT_RED);
    ui.drawString("Click: ABORT", 120, 225);
    return;
  }

  // STR_DONE — results
  int y = 40;
  ui.setTextSize(1);

  char buf[48];
  snprintf(buf, sizeof(buf), "Dist: %.3fm (tgt %.2f)", (double)str_result.driven_m, (double)str_result.target_m);
  ui.setTextColor(fabsf(str_result.dist_err_m) < 0.05f ? TFT_GREEN : TFT_YELLOW);
  ui.drawString(buf, 120, y); y += 16;

  snprintf(buf, sizeof(buf), "Yaw drift: %.1f deg", (double)str_result.yaw_drift_deg);
  ui.setTextColor(fabsf(str_result.yaw_drift_deg) < 5.0f ? TFT_GREEN : TFT_RED);
  ui.drawString(buf, 120, y); y += 16;

  snprintf(buf, sizeof(buf), "Lateral: %.0fmm", (double)(str_result.lateral_drift_m * 1000.0f));
  ui.setTextColor(fabsf(str_result.lateral_drift_m) < 0.03f ? TFT_GREEN :
                  fabsf(str_result.lateral_drift_m) < 0.06f ? TFT_YELLOW : TFT_RED);
  ui.drawString(buf, 120, y); y += 16;

  snprintf(buf, sizeof(buf), "EncL: %.3f  EncR: %.3f", (double)str_result.enc_L_m, (double)str_result.enc_R_m);
  ui.setTextColor(TFT_BLUE);
  ui.drawString(buf, 120, y); y += 16;

  snprintf(buf, sizeof(buf), "L-R diff: %.1fmm", (double)str_result.enc_diff_mm);
  ui.setTextColor(str_result.enc_diff_mm < 20.0f ? TFT_GREEN : TFT_RED);
  ui.drawString(buf, 120, y); y += 16;

  snprintf(buf, sizeof(buf), "Time: %.1fs", str_result.elapsed_ms / 1000.0f);
  ui.setTextColor(TFT_DARKGREY);
  ui.drawString(buf, 120, y); y += 20;

  bool pass = fabsf(str_result.yaw_drift_deg) < 5.0f && str_result.enc_diff_mm < 20.0f;
  ui.setTextColor(pass ? TFT_GREEN : TFT_RED);
  ui.setTextSize(2);
  ui.drawString(pass ? "STRAIGHT" : "VEERING", 120, y);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Click: retest  Hold: back", 120, 228);
}

// ========================================================================
// 90-DEGREE TURN-IN-PLACE TEST
// Performs an exact 90° right pivot and reports accuracy.
// ========================================================================
enum TurnTestPhase {
  TRN_IDLE,
  TRN_CALIBRATING,
  TRN_TURNING,
  TRN_DONE,
};

struct TurnTestResult {
  float target_deg;
  float actual_deg;
  float error_deg;
  float overshoot_deg;   // max overshoot past target
  unsigned long elapsed_ms;
  unsigned long settle_ms; // time from first entering tolerance to final stop
};

static TurnTestPhase trn_phase = TRN_IDLE;
static TurnTestResult trn_result;
static float trn_target_deg = 90.0f;  // adjustable: positive = right turn
static float trn_start_yaw = 0.0f;
static unsigned long trn_start_ms = 0;
static float trn_max_overshoot = 0.0f;
static unsigned long trn_first_in_tol_ms = 0;
static int trn_direction = -1;  // -1 = right (yaw decreases), +1 = left

static void trnTestReset() {
  trn_phase = TRN_IDLE;
  stopAllMotors();
}
void trnTestResetExtern() { trnTestReset(); }

static void trnTestStart() {
  memset(&trn_result, 0, sizeof(trn_result));
  trn_result.target_deg = trn_target_deg;
  trn_phase = TRN_CALIBRATING;
  trn_start_ms = millis();
  trn_max_overshoot = 0.0f;
  trn_first_in_tol_ms = 0;
  stopAllMotors();
  testLogInit("TURN", trn_target_deg);
  Serial.printf("=== TURN TEST START  target=%.1f deg dir=%s ===\n",
                (double)trn_target_deg, trn_direction < 0 ? "RIGHT" : "LEFT");
}

static unsigned long trn_lastLogMs = 0;

static bool trnTestTick() {
  const unsigned long now = millis();
  switch (trn_phase) {
    case TRN_IDLE:
    case TRN_DONE:
      return false;

    case TRN_CALIBRATING: {
      if (imu_present) {
        calibrateImuGyroBias();
        imu_yaw = 0.0f;
      }
      trn_start_yaw = imu_yaw;
      trn_max_overshoot = 0.0f;
      trn_first_in_tol_ms = 0;
      trn_start_ms = millis();  // reset AFTER calibration so timeout counts from actual turn start
      trn_phase = TRN_TURNING;
      Serial.printf("TRN: calibrated yaw=%.1f, turning %.1f deg %s\n",
                    (double)trn_start_yaw, (double)trn_target_deg,
                    trn_direction < 0 ? "RIGHT" : "LEFT");
      break;
    }

    case TRN_TURNING: {
      // Target yaw: start + direction * target_deg
      // For right turn: yaw decreases (trn_direction = -1)
      float target_yaw = trn_start_yaw + trn_direction * trn_target_deg;
      float yaw_err = wrapDeg(target_yaw - imu_yaw);

      float turned_deg = fabsf(imu_yaw - trn_start_yaw);

      // Track overshoot
      float overshoot = turned_deg - trn_target_deg;
      if (overshoot > trn_max_overshoot) trn_max_overshoot = overshoot;

      const float TOLERANCE = 2.0f;  // degrees
      const unsigned long SETTLE_HOLD_MS = 300;  // must hold in tolerance this long

      if (fabsf(yaw_err) < TOLERANCE) {
        if (trn_first_in_tol_ms == 0) trn_first_in_tol_ms = now;
        if ((now - trn_first_in_tol_ms) >= SETTLE_HOLD_MS) {
          // Done — settled in tolerance
          stopAllMotors();
          delay(100);
          trn_result.actual_deg = fabsf(imu_yaw - trn_start_yaw);
          trn_result.error_deg = trn_result.actual_deg - trn_target_deg;
          trn_result.overshoot_deg = trn_max_overshoot > 0.0f ? trn_max_overshoot : 0.0f;
          trn_result.elapsed_ms = now - trn_start_ms;
          trn_result.settle_ms = (trn_first_in_tol_ms > trn_start_ms) ? (trn_first_in_tol_ms - trn_start_ms) : 0;
          trn_phase = TRN_DONE;
          runLogSaveToSpiffs();
          Serial.printf("TRN: DONE actual=%.1f err=%.1f overshoot=%.1f settle=%lums time=%lums\n",
                        (double)trn_result.actual_deg, (double)trn_result.error_deg,
                        (double)trn_result.overshoot_deg, trn_result.settle_ms, trn_result.elapsed_ms);
          break;
        }
        // In tolerance but still settling — stop motors and wait
        stopAllMotors();
      } else {
        // Reset settle timer if we leave tolerance
        trn_first_in_tol_ms = 0;

        // Pivot with proportional speed
        int sign = (yaw_err > 0.0f) ? 1 : -1;
        float pwmScale = fabsf(yaw_err) / 45.0f;
        if (pwmScale > 1.0f) pwmScale = 1.0f;
        if (pwmScale < 0.35f) pwmScale = 0.35f;
        uint8_t turnPwm = (uint8_t)(BFS_TURN_PWM * pwmScale);
        if (turnPwm < 115) turnPwm = 115;
        setMotorsPivotPwm(sign, turnPwm);
        applyMotorOutputs();

        // Log sample every ~100ms
        if ((now - trn_lastLogMs) >= 100u) {
          trn_lastLogMs = now;
          testLogAppendSample(now - trn_start_ms, 0, imu_yaw, target_yaw,
                              yaw_err, imu_gz_dps, 0, fabsf(yaw_err),
                              (float)turnPwm, 0,
                              motor_l_speed, motor_r_speed, 0, 0);
        }
      }

      // Timeout safety
      if ((now - trn_start_ms) > 5000) {
        stopAllMotors();
        trn_result.actual_deg = fabsf(imu_yaw - trn_start_yaw);
        trn_result.error_deg = trn_result.actual_deg - trn_target_deg;
        trn_result.overshoot_deg = trn_max_overshoot > 0.0f ? trn_max_overshoot : 0.0f;
        trn_result.elapsed_ms = now - trn_start_ms;
        trn_result.settle_ms = 0;
        trn_phase = TRN_DONE;
        runLogSaveToSpiffs();
        Serial.printf("TRN: TIMEOUT actual=%.1f err=%.1f\n",
                      (double)trn_result.actual_deg, (double)trn_result.error_deg);
      }
      break;
    }

    default:
      break;
  }
  return true;
}

static void drawTurnTestScreen() {
  ui.fillScreen(TFT_BLACK);
  ui.setTextDatum(middle_center);

  ui.setTextColor(TFT_CYAN);
  ui.setTextSize(2);
  ui.drawString("TURN 90", 120, 18);

  if (trn_phase == TRN_IDLE) {
    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(1);
    ui.drawString("In-place pivot test", 120, 50);
    ui.drawString("measures turn accuracy", 120, 65);

    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    char buf[24];
    snprintf(buf, sizeof(buf), "%.0f %s", (double)trn_target_deg,
             trn_direction < 0 ? "RIGHT" : "LEFT");
    ui.drawString(buf, 120, 100);
    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("Dial: adjust angle", 120, 125);
    ui.drawString("Hold 1s: toggle L/R", 120, 140);

    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(2);
    ui.drawString("Click: GO", 120, 175);
    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    ui.drawString("Hold 2s: back", 120, 210);
    return;
  }

  if (trn_phase == TRN_CALIBRATING) {
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(2);
    ui.drawString("Calibrating...", 120, 120);
    return;
  }

  if (trn_phase == TRN_TURNING) {
    float turned = fabsf(imu_yaw - trn_start_yaw);
    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(1);
    ui.drawString(trn_direction < 0 ? "TURNING RIGHT" : "TURNING LEFT", 120, 45);

    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(2);
    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f / %.0f", (double)turned, (double)trn_target_deg);
    ui.drawString(buf, 120, 80);

    ui.setTextSize(1);
    ui.setTextColor(TFT_DARKGREY);
    char yawBuf[32];
    snprintf(yawBuf, sizeof(yawBuf), "Yaw: %.1f deg", (double)imu_yaw);
    ui.drawString(yawBuf, 120, 110);

    float elapsed = (millis() - trn_start_ms) / 1000.0f;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%.1fs", (double)elapsed);
    ui.drawString(timeBuf, 120, 180);

    ui.setTextColor(TFT_RED);
    ui.drawString("Click: ABORT", 120, 225);
    return;
  }

  // TRN_DONE — results
  int y = 45;
  ui.setTextSize(1);

  char buf[48];
  snprintf(buf, sizeof(buf), "Target: %.0f deg %s",
           (double)trn_result.target_deg, trn_direction < 0 ? "RIGHT" : "LEFT");
  ui.setTextColor(TFT_WHITE);
  ui.drawString(buf, 120, y); y += 18;

  snprintf(buf, sizeof(buf), "Actual: %.1f deg", (double)trn_result.actual_deg);
  ui.setTextColor(fabsf(trn_result.error_deg) < 5.0f ? TFT_GREEN : TFT_YELLOW);
  ui.drawString(buf, 120, y); y += 18;

  snprintf(buf, sizeof(buf), "Error: %.1f deg", (double)trn_result.error_deg);
  ui.setTextColor(fabsf(trn_result.error_deg) < 5.0f ? TFT_GREEN :
                  fabsf(trn_result.error_deg) < 10.0f ? TFT_YELLOW : TFT_RED);
  ui.drawString(buf, 120, y); y += 18;

  snprintf(buf, sizeof(buf), "Overshoot: %.1f deg", (double)trn_result.overshoot_deg);
  ui.setTextColor(trn_result.overshoot_deg < 5.0f ? TFT_GREEN : TFT_RED);
  ui.drawString(buf, 120, y); y += 18;

  snprintf(buf, sizeof(buf), "Settle: %lums  Total: %lums",
           trn_result.settle_ms, trn_result.elapsed_ms);
  ui.setTextColor(TFT_DARKGREY);
  ui.drawString(buf, 120, y); y += 22;

  bool pass = fabsf(trn_result.error_deg) < 5.0f && trn_result.overshoot_deg < 8.0f;
  ui.setTextColor(pass ? TFT_GREEN : TFT_RED);
  ui.setTextSize(2);
  ui.drawString(pass ? "ACCURATE" : "IMPRECISE", 120, y);

  ui.setTextColor(TFT_DARKGREY);
  ui.setTextSize(1);
  ui.drawString("Click: retest  Hold: back", 120, 228);
}

static void renderCurrentScreen() {
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
    /* SCREEN_TUNE_PPM removed */
    case SCREEN_SET_CTRL_MODE:
      drawSetControlMode();
      break;
    case SCREEN_BFS_NAV:
      drawBfsNavScreen(bfs_setup_step);
      break;
    case SCREEN_BFS_RUN:
      drawBfsRunScreen();
      break;
    case SCREEN_HW_TEST:
      updateDisplay();
      break;
    case SCREEN_IMU_CAL:
      drawImuCalScreen();
      break;
    case SCREEN_SELF_TEST:
      drawSelfTestScreen();
      break;
    case SCREEN_SQUARE_TEST:
      drawSquareTestScreen();
      break;
    case SCREEN_STRAIGHT_TEST:
      drawStraightTestScreen();
      break;
    case SCREEN_TURN_TEST:
      drawTurnTestScreen();
      break;
    case SCREEN_CALIBRATE:
      drawCalibrateScreen();
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
 
  // Selected motor / mode
  ui.setTextColor(TFT_YELLOW);
  ui.setTextSize(1);
  ui.drawString("Mode (click):", 120, 60);

  if (selected_motor <= 1) {
    // --- Manual motor test mode ---
    ui.setTextColor(selected_motor == 0 ? TFT_GREEN : TFT_WHITE);
    ui.setTextSize(2);
    ui.drawString(selected_motor == 0 ? "LEFT" : "RIGHT", 120, 78);

    // Speed display — show commanded and actual (clamped) PWM
    ui.setTextColor(TFT_YELLOW);
    ui.setTextSize(1);
    ui.drawString("Dial: +/- 1 PWM", 120, 96);

    ui.setTextColor(TFT_GREEN);
    ui.setTextSize(2);
    // Compute actual clamped PWM that driveMotor() will send
    uint8_t cmdAbs = (uint8_t)abs((int)motor_command);
    uint8_t actualPwm = 0;
    if (cmdAbs > 0) {
      uint8_t minPwm = 110;
      actualPwm = (cmdAbs < minPwm) ? minPwm : cmdAbs;
    }
    char spd_str[24];
    snprintf(spd_str, sizeof(spd_str), "%d (=%d)", (int)motor_command, (int)actualPwm);
    ui.drawString(spd_str, 120, 120);

    // Legend (use sign for direction)
    ui.setTextColor(TFT_WHITE);
    ui.setTextSize(1);
#if INVERT_MOTOR_DIRECTION
    ui.drawString("+ = REV   - = FWD", 120, 142);
#else
    ui.drawString("+ = FWD   - = REV", 120, 142);
#endif

    // IMU yaw (compact)
    ui.setTextColor(TFT_DARKGREY);
    ui.setTextSize(1);
    if (imu_ok) {
      char yawBuf[24];
      snprintf(yawBuf, sizeof(yawBuf), "Yaw: %.1f", (double)imu_yaw);
      ui.drawString(yawBuf, 120, 158);
    }

    // Wheel encoder values
    ui.setTextColor(TFT_BLUE);
    ui.setTextSize(2);
    char lbuf[32];
    char rbuf[32];
    if (encoderL_found) snprintf(lbuf, sizeof(lbuf), "L:%ld", (long)encoderL_count);
    else snprintf(lbuf, sizeof(lbuf), "L:--");
    if (encoderR_found) snprintf(rbuf, sizeof(rbuf), "R:%ld", (long)encoderR_count);
    else snprintf(rbuf, sizeof(rbuf), "R:--");
    ui.drawString(lbuf, 120, 182);
    ui.drawString(rbuf, 120, 205);

    // Exit hint
    ui.setTextColor(TFT_DARKGREY);
    ui.setTextSize(1);
    ui.drawString("Hold: MENU", 120, 228);
  } else {
    // --- FIND MIN PWM mode ---
    ui.setTextColor(TFT_MAGENTA);
    ui.setTextSize(2);
    ui.drawString("FIND MIN", 120, 78);

    if (hw_find_running) {
      // Show sweep progress
      ui.setTextColor(TFT_YELLOW);
      ui.setTextSize(1);
      ui.drawString("Sweeping...", 120, 100);
      char progBuf[32];
      snprintf(progBuf, sizeof(progBuf), "Motor %s  PWM %d",
               hw_find_motor == 0 ? "LEFT" : "RIGHT", (int)hw_find_pwm);
      ui.setTextSize(2);
      ui.drawString(progBuf, 120, 125);
    } else {
      // Show results or prompt
      ui.setTextSize(1);
      if (hw_find_done) {
        ui.setTextColor(TFT_GREEN);
        ui.drawString("Results:", 120, 100);
        ui.setTextSize(2);
        char rL[24], rR[24];
        if (hw_min_pwm_L > 0)
          snprintf(rL, sizeof(rL), "L min: %d", (int)hw_min_pwm_L);
        else
          snprintf(rL, sizeof(rL), "L: NO MOVE");
        if (hw_min_pwm_R > 0)
          snprintf(rR, sizeof(rR), "R min: %d", (int)hw_min_pwm_R);
        else
          snprintf(rR, sizeof(rR), "R: NO MOVE");
        ui.setTextColor(hw_min_pwm_L > 0 ? TFT_GREEN : TFT_RED);
        ui.drawString(rL, 120, 125);
        ui.setTextColor(hw_min_pwm_R > 0 ? TFT_GREEN : TFT_RED);
        ui.drawString(rR, 120, 155);
      } else {
        ui.setTextColor(TFT_WHITE);
        ui.drawString("Click to start", 120, 110);
        ui.drawString("auto-sweep", 120, 125);
        ui.setTextColor(TFT_DARKGREY);
        ui.drawString("Ramps PWM 50-255", 120, 145);
        ui.drawString("detects encoder move", 120, 160);
      }
    }

    // Encoder values
    ui.setTextColor(TFT_BLUE);
    ui.setTextSize(2);
    char lbuf[32], rbuf[32];
    if (encoderL_found) snprintf(lbuf, sizeof(lbuf), "L:%ld", (long)encoderL_count);
    else snprintf(lbuf, sizeof(lbuf), "L:--");
    if (encoderR_found) snprintf(rbuf, sizeof(rbuf), "R:%ld", (long)encoderR_count);
    else snprintf(rbuf, sizeof(rbuf), "R:--");
    ui.drawString(lbuf, 120, 195);
    ui.drawString(rbuf, 120, 218);
  }
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

// ========================================================================
// Boot-time adaptive encoder auto-test
// Spins each motor briefly and checks which encoders respond under load.
// Sets enc_mode so BFS DRIVE uses the best available distance source.
// Called once from setup() after I2C, motors and encoders are initialised.
// ========================================================================
static void bootEncoderAutoTest() {
  Serial.println("=== BOOT ENCODER AUTO-TEST ===");

  // Show status on display
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextColor(TFT_CYAN);
  M5.Lcd.setTextSize(2);
  M5.Lcd.drawString("ENC TEST", 120, 30);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.drawString("Testing encoders...", 120, 60);

  // ---- helper lambda: try reading one encoder safely ----
  auto readEnc = [](uint8_t addr, int32_t &val) -> bool {
    return readOneEncoder(addr, val);
  };

  // ---- Phase 1: Read baseline (motors off) ----
  readWheelEncodersInternal(true);
  int32_t baseL = encoderL_count;
  int32_t baseR = encoderR_count;
  delay(100);

  // ---- Phase 2: Spin LEFT motor, measure BOTH encoders ----
  readWheelEncodersInternal(true);
  int32_t preL_L = encoderL_count;
  int32_t preL_R = encoderR_count;

  motor_l_direction = 1;
  motor_l_speed = 130;
  motor_r_direction = 0;
  motor_r_speed = 0;
  applyMotorOutputs();

  // Spin for 1 second, reading encoders periodically
  uint32_t encReadsL = 0, encFailsL_L = 0, encFailsL_R = 0;
  unsigned long spinStart = millis();
  while (millis() - spinStart < 1000) {
    readWheelEncodersInternal(true);
    encReadsL++;
    if (!encoderL_found) encFailsL_L++;
    if (!encoderR_found) encFailsL_R++;
    delay(20);
  }
  stopAllMotors();
  delay(200);  // settle
  readWheelEncodersInternal(true);

  int32_t dL_whenMotL = encoderL_count - preL_L;
  int32_t dR_whenMotL = encoderR_count - preL_R;
  if (INVERT_LEFT_ENCODER_COUNT)  dL_whenMotL = -dL_whenMotL;
  if (INVERT_RIGHT_ENCODER_COUNT) dR_whenMotL = -dR_whenMotL;

  Serial.printf("BOOT ENC: motL -> encL=%ld encR=%ld  failsL=%lu failsR=%lu / %lu\n",
                (long)dL_whenMotL, (long)dR_whenMotL,
                (unsigned long)encFailsL_L, (unsigned long)encFailsL_R,
                (unsigned long)encReadsL);

  M5.Lcd.drawString("Left motor done", 120, 80);

  // ---- Phase 3: Spin RIGHT motor, measure BOTH encoders ----
  readWheelEncodersInternal(true);
  int32_t preR_L = encoderL_count;
  int32_t preR_R = encoderR_count;

  motor_l_direction = 0;
  motor_l_speed = 0;
  motor_r_direction = 1;
  motor_r_speed = 130;
  applyMotorOutputs();

  uint32_t encReadsR = 0, encFailsR_L = 0, encFailsR_R = 0;
  spinStart = millis();
  while (millis() - spinStart < 1000) {
    readWheelEncodersInternal(true);
    encReadsR++;
    if (!encoderL_found) encFailsR_L++;
    if (!encoderR_found) encFailsR_R++;
    delay(20);
  }
  stopAllMotors();
  delay(200);
  readWheelEncodersInternal(true);

  int32_t dL_whenMotR = encoderL_count - preR_L;
  int32_t dR_whenMotR = encoderR_count - preR_R;
  if (INVERT_LEFT_ENCODER_COUNT)  dL_whenMotR = -dL_whenMotR;
  if (INVERT_RIGHT_ENCODER_COUNT) dR_whenMotR = -dR_whenMotR;

  Serial.printf("BOOT ENC: motR -> encL=%ld encR=%ld  failsL=%lu failsR=%lu / %lu\n",
                (long)dL_whenMotR, (long)dR_whenMotR,
                (unsigned long)encFailsR_L, (unsigned long)encFailsR_R,
                (unsigned long)encReadsR);

  M5.Lcd.drawString("Right motor done", 120, 100);

  // ---- Phase 4: Decide which encoder(s) actually work under motor load ----
  // An encoder "works" if it registered ≥ 50 pulses with its own-side motor
  // AND had < 50% I2C read failures across BOTH motor tests.
  // We check each encoder's total failure rate across all motor tests.
  uint32_t totalReads = encReadsL + encReadsR;
  uint32_t totalFailsL = encFailsL_L + encFailsR_L;
  uint32_t totalFailsR = encFailsL_R + encFailsR_R;

  // "own-side moved" — did the encoder see pulses when its motor ran?
  // Because of possible cross-wiring we also check the cross case.
  bool leftEncMoved  = (abs(dL_whenMotL) > 50) || (abs(dL_whenMotR) > 50);
  bool rightEncMoved = (abs(dR_whenMotR) > 50) || (abs(dR_whenMotL) > 50);

  // Encoder I2C reliable enough? (> 50% success rate during motor tests)
  bool leftEncReliable  = (totalReads > 0) && (totalFailsL * 100 / totalReads < 50);
  bool rightEncReliable = (totalReads > 0) && (totalFailsR * 100 / totalReads < 50);

  bool leftOk  = leftEncMoved  && leftEncReliable;
  bool rightOk = rightEncMoved && rightEncReliable;

  if (leftOk && rightOk) {
    enc_mode = ENC_MODE_BOTH;
  } else if (rightOk) {
    enc_mode = ENC_MODE_RIGHT_ONLY;
  } else if (leftOk) {
    enc_mode = ENC_MODE_LEFT_ONLY;
  } else {
    enc_mode = ENC_MODE_TIMED;
  }

  // ---- Display result ----
  const char* modeStr = "???";
  uint16_t modeColor = TFT_WHITE;
  switch (enc_mode) {
    case ENC_MODE_BOTH:       modeStr = "BOTH OK";    modeColor = TFT_GREEN;  break;
    case ENC_MODE_LEFT_ONLY:  modeStr = "LEFT ONLY";  modeColor = TFT_YELLOW; break;
    case ENC_MODE_RIGHT_ONLY: modeStr = "RIGHT ONLY"; modeColor = TFT_YELLOW; break;
    case ENC_MODE_TIMED:      modeStr = "TIMED (no enc)"; modeColor = TFT_RED; break;
  }

  Serial.printf("BOOT ENC: mode=%s  leftOk=%d(moved=%d rel=%d)  rightOk=%d(moved=%d rel=%d)\n",
                modeStr, leftOk, leftEncMoved, leftEncReliable,
                rightOk, rightEncMoved, rightEncReliable);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(modeColor);
  M5.Lcd.drawString(modeStr, 120, 140);

  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_WHITE);
  char buf[48];
  snprintf(buf, sizeof(buf), "L: %s  R: %s", leftOk ? "OK" : "FAIL", rightOk ? "OK" : "FAIL");
  M5.Lcd.drawString(buf, 120, 170);
  snprintf(buf, sizeof(buf), "L pulses: %ld / %ld", (long)dL_whenMotL, (long)dL_whenMotR);
  M5.Lcd.drawString(buf, 120, 190);
  snprintf(buf, sizeof(buf), "R pulses: %ld / %ld", (long)dR_whenMotL, (long)dR_whenMotR);
  M5.Lcd.drawString(buf, 120, 205);

  delay(2000);  // Show result briefly before continuing to menu
  Serial.println("=== BOOT ENCODER AUTO-TEST DONE ===");
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
  // Auto-format on first boot so FFat works out of the box.
  spiffs_ok = SPIFFS.begin(true);
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
    Serial.printf("Loaded settings: ppm=%.2f ppmL=%.2f ppmR=%.2f mtr_bias=%d\n",
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
  // Store pins for bus recovery
  i2c_sda_pin = ex_sda;
  i2c_scl_pin = ex_scl;
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
  // Store default pins for bus recovery (ESP32-S3 defaults)
  i2c_sda_pin = SDA;
  i2c_scl_pin = SCL;
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

  // Initialize BFS graph for navigation
  bfsInitializeGraph();

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

  // Adaptive encoder auto-test: spin each motor, detect which encoders work
  // under load, and select the best distance source for BFS navigation.
  bootEncoderAutoTest();
 
  renderCurrentScreen();
}

void loop() {
  M5.update();

  // Allow post-run interaction over USB serial (e.g., dump saved log).
  handleSerialCommands();

  const unsigned long now = millis();

  // === Track screen changes and reset BFS state ===
  static ScreenState lastScreen = SCREEN_MAIN_MENU;
  
  if (currentScreen != lastScreen) {
    // Screen changed - reset state
    if (currentScreen == SCREEN_BFS_NAV) {
      bfs_setup_step = 0;  // Reset to selecting start node
    }
    lastScreen = currentScreen;
  }

  // Auto-save calibration settings after the dial stops.
  if (settings_dirty && (now - settings_dirty_ms) >= (unsigned long)SETTINGS_AUTOSAVE_IDLE_MS) {
    if (currentScreen == SCREEN_SET_ENC_CAL || currentScreen == SCREEN_SET_CTRL_MODE
        || currentScreen == SCREEN_BFS_NAV) {
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
  if (now - lastMotorDiag > 1000) {   // reduced from 500ms to 1000ms to cut bus traffic
    lastMotorDiag = now;
    updateMotorDiagnostics();
  }

  // Read IMU attitude periodically (degrees)
  static unsigned long lastImuMs = 0;
  if (now - lastImuMs >= (unsigned long)control_period_ms) {
    lastImuMs = now;

    float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
    float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;
    float temp_c = 0.0f;
    static uint8_t imu_consecutive_fails = 0;
    const uint32_t t0us = (uint32_t)micros();
    const bool ok = imu_present && imu6886Read(&ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps, &temp_c);
    const uint32_t t1us = (uint32_t)micros();
    runDbg_lastImuReadUs = (t1us >= t0us) ? (t1us - t0us) : 0;
    imu_ok = ok;

    if (!ok) {
      imu_consecutive_fails++;
      if (imu_consecutive_fails >= 5) {
        Serial.println("IMU_I2C: consecutive fails — bus recovery!");
        i2cBusRecovery();
        imu_consecutive_fails = 0;
      }
    } else {
      imu_consecutive_fails = 0;
    }

    imu_ax_g = ok ? ax_g : 0.0f;
    imu_ay_g = ok ? ay_g : 0.0f;
    imu_az_g = ok ? az_g : 0.0f;
    imu_gz_raw_dps = ok ? gz_dps : 0.0f;
    imu_gz_dps = ok ? (gz_dps - imu_gz_bias_dps) : 0.0f;

    // Stall detection (relocated from gridHasWall): if commanded PWM high
    // but horizontal acceleration is very low for >400ms, latch cutoff.
    {
      const int maxPwm = (abs(motor_l_speed) > abs(motor_r_speed)) ? abs(motor_l_speed) : abs(motor_r_speed);
      const bool pwmHigh = (maxPwm > 100);
      const float horiz_g = sqrtf(imu_ax_g * imu_ax_g + imu_ay_g * imu_ay_g);
      const bool accelLow = fabsf(horiz_g) < 0.05f;  // ~0.05g threshold
      static unsigned long stallStartMs = 0;

      if (pwmHigh && accelLow && !stall_cutoff_latched) {
        unsigned long nowSt = millis();
        if (stallStartMs == 0) stallStartMs = nowSt;
        if ((nowSt - stallStartMs) > 400u) {
          Serial.println("STALL: High PWM with low acceleration — emergency cutoff");
          stopAllMotors();
          stall_cutoff_latched = true;
        }
      } else {
        stallStartMs = 0;
        if (!pwmHigh) stall_cutoff_latched = false;
      }
    }

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
    // Also block bias updates during any test/run (square, straight, BFS)
    const bool testActive = (sq_phase != SQ_IDLE && sq_phase != SQ_DONE) ||
                            (str_phase != STR_IDLE && str_phase != STR_DONE) ||
                            (bfs_run_phase != BFS_PHASE_IDLE);
    if (!runActive && !testActive && motorsStopped && accelStill && ok) {
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

      // Integrate gyro Z (deg/s) into a yaw-ish angle using bias-compensated rate.
      const uint32_t nowUs = (uint32_t)micros();
      if (lastYawUs == 0) lastYawUs = nowUs;
      const uint32_t dtUs = nowUs - lastYawUs;
      lastYawUs = nowUs;
      const float dt_s = (float)dtUs * 1e-6f;
      if (dt_s > 0.0f && dt_s < 0.5f) {
        // currentYaw += (rawGyroZ - gyroBiasZ) * deltaTime
        imu_yaw += imu_gz_dps * dt_s;
        imu_yaw = wrapDeg(imu_yaw);
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
          enterScreen(SCREEN_BFS_NAV);
          break;
        case 1:
          enterScreen(SCREEN_SET_TIME);
          break;
        case 2:
          enterScreen(SCREEN_BIAS_TEST);
          break;
        case 3:
          enterScreen(SCREEN_SET_ENC_CAL);
          break;
        case 4:
          enterScreen(SCREEN_IMU_CAL);
          break;
        case 5:
          enterScreen(SCREEN_HW_TEST);
          break;
        case 6:
          enterScreen(SCREEN_SELF_TEST);
          break;
        case 7:
          enterScreen(SCREEN_SQUARE_TEST);
          break;
        case 8:
          enterScreen(SCREEN_STRAIGHT_TEST);
          break;
        case 9:
          enterScreen(SCREEN_TURN_TEST);
          break;
        case 10: {
          // Clear all runlog files from SPIFFS
          if (spiffs_ok) {
            int delCount = 0;
            char paths[32][32];
            int count = 0;
            File root = SPIFFS.open("/");
            File f = root.openNextFile();
            while (f && count < 32) {
              const char* nm = f.name();
              if (nm && strstr(nm, "runlog") && strstr(nm, ".csv")) {
                snprintf(paths[count], sizeof(paths[count]), "%s%s",
                         (nm[0] == '/') ? "" : "/", nm);
                count++;
              }
              f.close();
              f = root.openNextFile();
            }
            root.close();
            for (int i2 = 0; i2 < count; i2++) {
              if (SPIFFS.remove(paths[i2])) delCount++;
            }
            // Also remove start_debug.txt if present
            SPIFFS.remove("/start_debug.txt");
            // Show confirmation on screen
            ui.fillScreen(TFT_BLACK);
            ui.setTextColor(TFT_GREEN);
            ui.setTextSize(2);
            ui.drawString("LOGS CLEARED", 120, 100);
            ui.setTextSize(1);
            ui.drawString(String(delCount) + " file(s) deleted", 120, 130);
            ui.pushSprite(0, 0);
            delay(1000);
            Serial.printf("CLR_LOGS: deleted %d file(s)\n", delCount);
          } else {
            ui.fillScreen(TFT_BLACK);
            ui.setTextColor(TFT_RED);
            ui.setTextSize(2);
            ui.drawString("FS ERROR", 120, 110);
            ui.pushSprite(0, 0);
            delay(1000);
          }
          break;
        }
      }
    }

    // In menu: motors always off
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_BFS_NAV) {
    // BFS Navigation grid course setup wizard
    // Dial changes the value for the current setup step
    applyBfsDialSteps(dialSteps, bfs_setup_step);

    // Long-press tracking
    static bool bfsNavLongPressHandled = false;
    static bool bfsNavExitHandled = false;
    if (!M5.BtnA.isPressed()) {
      bfsNavLongPressHandled = false;
      bfsNavExitHandled = false;
    }

    // Click behavior depends on current step
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      if (bfs_setup_step >= 0 && bfs_setup_step <= 1) {
        // Position steps 0-1: click advances to next step
        bfs_setup_step++;
      } else if (bfs_setup_step == 2) {
        // H-Wall editor: click toggles wall at cursor
        uint8_t r1, c1, r2, c2;
        gridHEdgeFromIndex(grid_hwall_cursor, r1, c1, r2, c2);
        gridToggleWall(r1, c1, r2, c2);
        manual_wp_count = 0; // layout changed, reset waypoints
      } else if (bfs_setup_step == 3) {
        // V-Wall editor: click toggles wall at cursor
        uint8_t r1, c1, r2, c2;
        gridVEdgeFromIndex(grid_vwall_cursor, r1, c1, r2, c2);
        gridToggleWall(r1, c1, r2, c2);
        manual_wp_count = 0; // layout changed, reset waypoints
      } else if (bfs_setup_step == 4) {
        // H-Bottle editor: click toggles bottle at cursor
        uint8_t r1, c1, r2, c2;
        gridHEdgeFromIndex(grid_hbottle_cursor, r1, c1, r2, c2);
        gridToggleBottle(r1, c1, r2, c2);
        manual_wp_count = 0; // layout changed, reset waypoints
      } else if (bfs_setup_step == 5) {
        // V-Bottle editor: click toggles bottle at cursor
        uint8_t r1, c1, r2, c2;
        gridVEdgeFromIndex(grid_vbottle_cursor, r1, c1, r2, c2);
        gridToggleBottle(r1, c1, r2, c2);
        manual_wp_count = 0; // layout changed, reset waypoints
      } else if (bfs_setup_step == 6) {
        // Gate editor: 2-state toggle (none → gate → remove)
        uint8_t row, col;
        gridSquarePosition(grid_gate_cursor, row, col);
        bool hasGate = gridHasGate(row, col);
        if (!hasGate) {
          // No gate → place gate
          if (grid_course.gate_count < MAX_GATES) {
            grid_course.gates[grid_course.gate_count] = {row, col};
            grid_course.gate_count++;
          }
        } else {
          // Gate exists → remove it
          int8_t gateIdx = -1;
          for (uint8_t gi = 0; gi < grid_course.gate_count; gi++) {
            if (grid_course.gates[gi].row == row && grid_course.gates[gi].col == col) {
              gateIdx = (int8_t)gi;
              break;
            }
          }
          if (gateIdx >= 0) {
            for (uint8_t j = (uint8_t)gateIdx; j < grid_course.gate_count - 1; j++)
              grid_course.gates[j] = grid_course.gates[j + 1];
            grid_course.gate_count--;
          }
        }
        manual_wp_count = 0; // layout changed, reset waypoints
      } else if (bfs_setup_step == 7) {
        // Waypoint editor: click on last waypoint deletes it, otherwise adds new
        if (manual_wp_count > 0 &&
            manual_wp_col[manual_wp_count - 1] == manual_wp_cursor_col &&
            manual_wp_row[manual_wp_count - 1] == manual_wp_cursor_row) {
          manual_wp_count--;
          Serial.printf("WP: deleted #%d at col=%d row=%d\n",
                        (int)(manual_wp_count + 1), (int)manual_wp_cursor_col, (int)manual_wp_cursor_row);
        } else if (manual_wp_count < MANUAL_WP_MAX) {
          manual_wp_col[manual_wp_count] = manual_wp_cursor_col;
          manual_wp_row[manual_wp_count] = manual_wp_cursor_row;
          manual_wp_count++;
          Serial.printf("WP: added #%d at col=%d row=%d\n",
                        (int)manual_wp_count, (int)manual_wp_cursor_col, (int)manual_wp_cursor_row);
        }
      } else if (bfs_setup_step == 8) {
        // Confirm screen: click starts run using manual waypoints.
        bool pathOk = manualWaypointsToPath();
        if (pathOk) {
          bfs_run_active = true;
          bfs_current_target = bfsGetNextTarget();
          savePersistedSettings();
          bfs_run_phase = BFS_PHASE_IDLE;
          bfs_run_total_driven_m = 0.0f;
          // Compute total path distance for time-based speed scaling
          bfs_run_total_path_dist_m = 0.0f;
          for (uint16_t si = 0; si + 1 < bfs_state.path_length; si++) {
            uint8_t a = bfs_state.path[si], b = bfs_state.path[si + 1];
            float ddx = bfs_state.nodes[b].x_m - bfs_state.nodes[a].x_m;
            float ddy = bfs_state.nodes[b].y_m - bfs_state.nodes[a].y_m;
            bfs_run_total_path_dist_m += sqrtf(ddx * ddx + ddy * ddy);
          }
          Serial.printf("NAV: total_path_dist=%.2fm runTimeS=%.1fs\n",
                        (double)bfs_run_total_path_dist_m, (double)runTimeS);
          // Estimate non-driving overhead for time-based speed scaling.
          {
            int numSeg = (int)(bfs_state.path_length - 1);
            float overheadS = 1.5f + (float)numSeg * 1.5f;
            bfs_run_effective_drive_time_s = runTimeS - overheadS;
            if (bfs_run_effective_drive_time_s < runTimeS * 0.3f)
              bfs_run_effective_drive_time_s = runTimeS * 0.3f;
            Serial.printf("NAV: numSeg=%d overhead=%.1fs effectiveDrive=%.1fs\n",
                          numSeg, (double)overheadS, (double)bfs_run_effective_drive_time_s);
          }
          bfs_run_start_ms = 0;  // Will be set when robot first moves (2026 rules)
          bfs_run_initialized = false;
          bfs_run_cumulative_yaw_deg = 0.0f;

          // Two-stage IMU calibration before entering BFS_RUN.
          imuCal_next_screen = SCREEN_BFS_RUN;
          enterScreen(SCREEN_CALIBRATE);
        }
      }
      settings_dirty = true;
      settings_dirty_ms = millis();
    }

    // Hold (800ms): advance from editor steps, undo waypoint, or go back from confirm
    if (!bfsNavLongPressHandled && M5.BtnA.pressedFor(800)) {
      bfsNavLongPressHandled = true;
      if (bfs_setup_step >= 2 && bfs_setup_step <= 6) {
        // Editor steps: advance to next step
        bfs_setup_step++;
        if (bfs_setup_step == 7) {
          // Entering waypoint editor — init cursor near start position
          float spacing_m = (float)grid_course.spacing_cm * 0.01f;
          float sx = 0.5f * ((float)grid_course.start_c1 + (float)grid_course.start_c2) * spacing_m;
          float sy = 0.5f * ((float)grid_course.start_r1 + (float)grid_course.start_r2) * spacing_m;
          manual_wp_cursor_col = (uint8_t)(sx / 0.25f + 0.5f);
          manual_wp_cursor_row = (uint8_t)(sy / 0.25f + 0.5f);
          if (manual_wp_cursor_col >= MANUAL_GRID_COLS) manual_wp_cursor_col = MANUAL_GRID_COLS - 1;
          if (manual_wp_cursor_row >= MANUAL_GRID_ROWS) manual_wp_cursor_row = MANUAL_GRID_ROWS - 1;
          manual_wp_cursor_idx = manual_wp_cursor_row * MANUAL_GRID_COLS + manual_wp_cursor_col;
        }
      } else if (bfs_setup_step == 7) {
        // Waypoint editor: hold advances to confirm screen
        if (manual_wp_count > 0) {
          bfs_setup_step = 8;
          // Pre-compute path for preview display
          manualWaypointsToPath();
        }
      } else if (bfs_setup_step == 8) {
        // Confirm screen: hold goes back to waypoint editor
        bfs_setup_step = 7;
      }
    }

    // Very long press (2s): undo waypoint in step 7, exit to menu otherwise
    if (!bfsNavExitHandled && M5.BtnA.pressedFor(2000)) {
      bfsNavExitHandled = true;
      if (bfs_setup_step == 7 && manual_wp_count > 0) {
        // Undo last waypoint
        manual_wp_count--;
        Serial.printf("WP: undo, now %d waypoints\n", (int)manual_wp_count);
      } else {
        savePersistedSettings();
        enterScreen(SCREEN_MAIN_MENU);
      }
    }

    stopAllMotors();
  }

  else if (currentScreen == SCREEN_BFS_RUN) {
    // Fresh IMU read every loop iteration — prevents yaw drift during display pushSprite blocks
    if (imu_present && imu_control_enabled) imuIntegrateOnce();

    // BFS path-following run screen
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      // Stop BFS run
      bfs_run_phase = BFS_PHASE_DONE;
      bfs_run_active = false;
      stopAllMotors();
      enterScreen(SCREEN_BFS_NAV);
    }

    // Initialize on first entry (bfs_run_start_ms is already set when user clicked start)
    if (bfs_run_phase == BFS_PHASE_IDLE && !bfs_run_initialized) {
      bfs_run_initialized = true;
      // bfs_run_start_ms will be set when robot first moves (2026 rule: Run Time begins when Robot begins to move)
      bfs_run_start_yaw = imu_yaw;
      bfs_run_cumulative_yaw_deg = imu_yaw;
      // Reset encoder health counters for this run
      enc_read_attempts = 0;
      enc_read_failures = 0;
      enc_read_recoveries = 0;

      // Compute the grid-frame heading the robot faces at the start position.
      // The robot is placed at a border edge midpoint, facing INWARD.
        {
          uint8_t sr1 = grid_course.start_r1;
          uint8_t sc1 = grid_course.start_c1;
        uint8_t sr2 = grid_course.start_r2, sc2 = grid_course.start_c2;
        // Heading is computed in SCREEN frame (row→screenX, col→screenY)
        // so the atan2 args are swapped vs raw grid (x=col, y=row).
        if (sr1 == sr2) {
          // Horizontal border edge: top (row=0) faces screen-right (+row), bottom faces screen-left
          float dr = (sr1 == 0) ? 1.0f : -1.0f;
          bfs_run_initial_heading_deg = atan2f(0.0f, dr) * (180.0f / (float)M_PI);
        } else {
          // Vertical border edge: left (col=0) faces screen-down (+col), right faces screen-up
          float dc = (sc1 == 0) ? 1.0f : -1.0f;
          bfs_run_initial_heading_deg = atan2f(dc, 0.0f) * (180.0f / (float)M_PI);
        }
      }

      // Start by computing first segment
      bfs_run_phase = BFS_PHASE_ARRIVED;  // Will immediately compute first segment
      Serial.printf("BFS_RUN: init yaw=%.1f initial_heading=%.1f path_len=%d path_idx=%d\n",
                    (double)imu_yaw, (double)bfs_run_initial_heading_deg,
                    (int)bfs_state.path_length, (int)bfs_state.path_index);
    }

    // Safety timeout: stop BFS run after 120 seconds max
    if (bfs_run_start_ms > 0 && (now - bfs_run_start_ms) > 120000) {
      Serial.println("BFS_RUN: SAFETY TIMEOUT");
      bfs_run_phase = BFS_PHASE_DONE;
      bfs_run_active = false;
      stopAllMotors();
    }

    // Settling statistics for BFS_PHASE_SETTLE (accelerometer variance)
    static float bfs_settle_mean_g = 0.0f;
    static float bfs_settle_m2_g = 0.0f;
    static int   bfs_settle_n = 0;

    if (bfs_run_phase == BFS_PHASE_DONE) {
      stopAllMotors();
    }
    else if (bfs_run_phase == BFS_PHASE_ARRIVED) {
      // Advance to next segment or finish
      stopAllMotors();
      Serial.printf("BFS_RUN: ARRIVED idx=%d len=%d\n", (int)bfs_state.path_index, (int)bfs_state.path_length);
      if (bfs_state.path_index >= bfs_state.path_length - 1) {
        // Reached final goal
        bfs_run_phase = BFS_PHASE_DONE;
        bfs_run_active = false;
      } else {
        // Compute heading and distance to next waypoint
        uint8_t curNode = bfs_state.path[bfs_state.path_index];
        bfsAdvancePath();
        uint8_t nextNode = bfs_state.path[bfs_state.path_index];

        float dx = bfs_state.nodes[nextNode].x_m - bfs_state.nodes[curNode].x_m;
        float dy = bfs_state.nodes[nextNode].y_m - bfs_state.nodes[curNode].y_m;
        bfs_run_segment_dist_m = sqrtf(dx * dx + dy * dy);

        // Target heading in SCREEN frame (row→screenX, col→screenY):
        // screen_dx ∝ dy (row change), screen_dy ∝ dx (col change)
        float target_heading_deg = atan2f(dx, dy) * (180.0f / (float)M_PI);

        // Compute the grid heading the robot CURRENTLY faces.
        // At run start: IMU=start_yaw corresponded to grid heading=initial_heading_deg.
        // Current grid heading = initial_heading + (start_yaw - current_imu_yaw)
        // (IMU yaw decreases when turning CW in grid frame)
        // Integrate IMU so imu_yaw is up-to-date before computing turn target.
        imuIntegrateOnce();
        float current_grid_heading = bfs_run_initial_heading_deg + (bfs_run_start_yaw - imu_yaw);
        // The turn delta in grid frame is (target - current).
        // Convert to IMU frame: IMU turns opposite to grid, so negate.
        float turn_delta = wrapDeg(target_heading_deg - current_grid_heading);
        // Apply relative to current fresh IMU yaw
        bfs_run_target_yaw_deg = imu_yaw - turn_delta;

        // Reset segment encoder tracking
        bfs_run_enc_start_L = encoderL_count;
        bfs_run_enc_start_R = encoderR_count;
        bfs_run_driven_m = 0.0f;

        bfs_run_phase = BFS_PHASE_TURN;
        bfs_turn_first_in_tol_ms = 0;  // reset settle timer for new turn
        bfs_turn_start_ms = now;       // record when turn started
        // Reset ALL turn PID state to prevent cumulative error
        bfs_turn_integral = 0.0f;
        // Initialize prev_err to actual initial error to avoid derivative spike
        bfs_turn_prev_err = wrapDeg(bfs_run_target_yaw_deg - imu_yaw);
        bfs_turn_dFilt = 0.0f;
        bfs_turn_pid_last_us = (uint32_t)micros();
        Serial.printf("BFS_RUN: segment %s->%s dist=%.2f heading=%.1f target_yaw=%.1f\n",
                      bfs_state.nodes[curNode].name, bfs_state.nodes[nextNode].name,
                      (double)bfs_run_segment_dist_m, (double)(target_heading_deg),
                      (double)bfs_run_target_yaw_deg);
      }
    }
    else if (bfs_run_phase == BFS_PHASE_TURN) {
      // Pivot in place to face the next waypoint
      float yaw_err = wrapDeg(bfs_run_target_yaw_deg - imu_yaw);

      if (fabsf(yaw_err) < BFS_TURN_TOLERANCE_DEG) {
        // In tolerance — stop motors and hold for settle time (like turn test)
        stopAllMotors();
        // Require BOTH position in tolerance AND low angular rate
        bool gyroStill = fabsf(imu_gz_dps) < 8.0f;
        if (bfs_turn_first_in_tol_ms == 0 && gyroStill) bfs_turn_first_in_tol_ms = now;
        if (!gyroStill) bfs_turn_first_in_tol_ms = 0;
        if (bfs_turn_first_in_tol_ms != 0 && (now - bfs_turn_first_in_tol_ms) >= BFS_TURN_SETTLE_MS) {
          // Settled — enter BRAKE then SETTLE before driving
          bfs_turn_first_in_tol_ms = 0;
          bfs_run_brake_start_ms = now;
          bfs_run_after_brake_phase = BFS_PHASE_SETTLE;
          bfs_run_phase = BFS_PHASE_BRAKE;
          Serial.printf("BFS_RUN: TURN done, entering BRAKE/SETTLE yaw=%.1f err=%.1f encL=%ld encR=%ld\n",
                        (double)imu_yaw, (double)yaw_err,
                        (long)encoderL_count, (long)encoderR_count);
        }
      } else {
        // Out of tolerance — reset settle timer
        bfs_turn_first_in_tol_ms = 0;

        // PID-based turn: compute real dt (microsecond precision)
        float dt;
        { uint32_t nowUs = (uint32_t)micros();
          uint32_t dtUs = nowUs - bfs_turn_pid_last_us;
          if (dtUs < 100) dtUs = 100;
          if (dtUs > 50000) dtUs = 50000;
          dt = (float)dtUs * 1e-6f;
          bfs_turn_pid_last_us = nowUs;
        }

        // Leaky integral (only accumulate when error is meaningful)
        const float turnKi = 1.0f;
        const float turnKd = 0.03f;
        const float turnILeakTau = 1.0f;
        bfs_turn_integral -= bfs_turn_integral * (dt / turnILeakTau);
        if (fabsf(yaw_err) > 0.5f) {
          bfs_turn_integral += yaw_err * dt;
        }
        // Anti-windup: clamp integral contribution to ±12 PWM
        const float turnIMaxPwm = 12.0f;
        float iLim = turnIMaxPwm / turnKi;
        if (bfs_turn_integral > iLim) bfs_turn_integral = iLim;
        if (bfs_turn_integral < -iLim) bfs_turn_integral = -iLim;

        // Low-pass filtered derivative
        float rawDeriv = (yaw_err - bfs_turn_prev_err) / dt;
        bfs_turn_prev_err = yaw_err;
        const float turnDFilterTau = 0.030f;
        float dAlpha = dt / (turnDFilterTau + dt);
        bfs_turn_dFilt += dAlpha * (rawDeriv - bfs_turn_dFilt);

        // Base PWM from proportional scaling with proper crawl zone
        float absErr = fabsf(yaw_err);
        float basePwm;
        if (absErr > 45.0f) {
          basePwm = BFS_TURN_PWM;  // full speed for large errors
        } else if (absErr > 15.0f) {
          basePwm = 110.0f + (absErr - 15.0f) * (BFS_TURN_PWM - 110.0f) / 30.0f;
        } else if (absErr > 5.0f) {
          // Crawl zone: ramp from 110 down to 100 between 15° and 5°
          float t = (absErr - 5.0f) / 10.0f;
          basePwm = 100.0f + t * 10.0f;
        } else {
          // Fine zone: stays at 100 (motor minimum)
          basePwm = 100.0f;
        }

        // I and D corrections: sign-aware relative to error direction.
        float errSign = (yaw_err > 0.0f) ? 1.0f : -1.0f;
        float correction = (turnKi * bfs_turn_integral + turnKd * bfs_turn_dFilt) * errSign;
        float turnPwmF = basePwm + correction;
        if (turnPwmF > 200.0f) turnPwmF = 200.0f;
        if (turnPwmF < 100) turnPwmF = 100;

        int sign = (yaw_err > 0.0f) ? 1 : -1;
        setMotorsPivotPwm(sign, (uint8_t)(turnPwmF + 0.5f));
        applyMotorOutputs();
        // 2026 rule: Run Time begins when Robot begins to move
        if (bfs_run_start_ms == 0) {
          bfs_run_start_ms = millis();
          Serial.printf("BFS_RUN: runtime started at first motor move\n");
        }
      }

      // Timeout safety: 5 seconds max
      if (bfs_turn_start_ms > 0 && (now - bfs_turn_start_ms) > 5000) {
        stopAllMotors();
        bfs_turn_first_in_tol_ms = 0;
        bfs_run_brake_start_ms = now;
        bfs_run_after_brake_phase = BFS_PHASE_SETTLE;
        bfs_run_phase = BFS_PHASE_BRAKE;
        Serial.println("BFS_RUN: TURN timeout, forcing BRAKE/SETTLE");
      }
    }
    else if (bfs_run_phase == BFS_PHASE_BRAKE) {
      // Brief electronic brake with motors commanded to 0 PWM.
      stopAllMotors();
      if ((now - bfs_run_brake_start_ms) >= 150u) {
        if (bfs_run_after_brake_phase == BFS_PHASE_SETTLE) {
          bfs_run_settle_start_ms = now;
          bfs_settle_mean_g = 0.0f;
          bfs_settle_m2_g = 0.0f;
          bfs_settle_n = 0;
          bfs_run_phase = BFS_PHASE_SETTLE;
        } else if (bfs_run_after_brake_phase == BFS_PHASE_ARRIVED) {
          bfs_run_phase = BFS_PHASE_ARRIVED;
        } else {
          bfs_run_phase = BFS_PHASE_ARRIVED;
        }
        bfs_run_after_brake_phase = BFS_PHASE_IDLE;
      }
    }
    else if (bfs_run_phase == BFS_PHASE_SETTLE) {
      // Wait until accelerometer variance is near-zero before transitioning to DRIVE.
      // Use horizontal-plane acceleration (ax, ay) as proxy for residual linear motion.
      const float horiz_g = sqrtf(imu_ax_g * imu_ax_g + imu_ay_g * imu_ay_g);
      bfs_settle_n += 1;
      const float delta = horiz_g - bfs_settle_mean_g;
      bfs_settle_mean_g += delta / (float)bfs_settle_n;
      const float delta2 = horiz_g - bfs_settle_mean_g;
      bfs_settle_m2_g += delta * delta2;

      float var_g2 = 0.0f;
      if (bfs_settle_n > 1) {
        var_g2 = bfs_settle_m2_g / (float)(bfs_settle_n - 1);
      }

      const bool timeOk = (now - bfs_run_settle_start_ms) >= 80u;
      const bool varSmall = var_g2 < (0.015f * 0.015f);

      if (timeOk && varSmall) {
        // Fresh encoder snapshot for the upcoming DRIVE
        readWheelEncodersInternal(true);
        bfs_run_enc_start_L = encoderL_count;
        bfs_run_enc_start_R = encoderR_count;
        bfs_run_drive_start_ms = millis();  // for ENC_MODE_TIMED fallback
        bfs_run_phase = BFS_PHASE_DRIVE;
        bfs_drive_pd_first = true;  // reset PD state for new segment
        // Reset ALL drive PID state to prevent cumulative error between segments
        bfs_drive_integral = 0.0f;
        bfs_drive_prev_err = 0.0f;
        bfs_drive_dFilt = 0.0f;
        bfs_drive_pid_last_us = 0;
        bfs_drive_steer_ramp_start_ms = millis();  // post-turn steering ramp
        Serial.printf("BFS_RUN: SETTLE done, starting DRIVE encL=%ld encR=%ld var_g2=%.6f\n",
                      (long)encoderL_count, (long)encoderR_count, (double)var_g2);
      }
    }
    else if (bfs_run_phase == BFS_PHASE_DRIVE) {
      // Force a fresh encoder read every iteration during DRIVE
      readWheelEncodersInternal(true);

      // Compute distance driven using unified encoder odometry helper
      bfs_run_driven_m = getAverageDistanceMeters();

      if (bfs_run_driven_m >= bfs_run_segment_dist_m - BFS_ARRIVE_TOLERANCE_M) {
        // Reached waypoint: closed-loop reverse brake (like straight test)
        brakeToStop(140, 400);
        // IMU-aware settle wait
        { unsigned long t0settle = millis();
          uint32_t lastImuPollUs = (uint32_t)micros();
          while ((millis() - t0settle) < 300) {
            uint32_t nUs = (uint32_t)micros();
            if ((nUs - lastImuPollUs) >= 1000u) { imuIntegrateOnce(); lastImuPollUs = nUs; }
          } }
        bfs_run_total_driven_m += bfs_run_driven_m;
        bfs_run_phase = BFS_PHASE_ARRIVED;
        Serial.printf("BFS_RUN: DRIVE done dist=%.2f/%.2f total=%.2f (brakeToStop)\n",
                      (double)bfs_run_driven_m, (double)bfs_run_segment_dist_m,
                      (double)bfs_run_total_driven_m);
      } else {
        // Drive with yaw correction
        float yaw_err = wrapDeg(bfs_run_target_yaw_deg - imu_yaw);

        // Full PID steering (matching square test quality)
        // Full PID steering: compute real dt (microsecond precision)
        float dt;
        { uint32_t nowUs = (uint32_t)micros();
          if (bfs_drive_pid_last_us == 0) {
            dt = 0.010f;
            bfs_drive_prev_err = yaw_err;
          } else {
            uint32_t dtUs = nowUs - bfs_drive_pid_last_us;
            if (dtUs < 100) dtUs = 100;
            if (dtUs > 50000) dtUs = 50000;
            dt = (float)dtUs * 1e-6f;
          }
          bfs_drive_pid_last_us = nowUs;
        }

        // Leaky integral for steady-state bias correction
        const float driveILeakTau = 1.0f;
        bfs_drive_integral -= bfs_drive_integral * (dt / driveILeakTau);
        if (fabsf(yaw_err) > 0.1f) {
          bfs_drive_integral += yaw_err * dt;
        }
        // Anti-windup: clamp integral contribution to ±25 PWM
        const float driveIMaxCorr = 25.0f;
        float driveILim = driveIMaxCorr / BFS_DRIVE_STEER_KI;
        if (bfs_drive_integral > driveILim) bfs_drive_integral = driveILim;
        if (bfs_drive_integral < -driveILim) bfs_drive_integral = -driveILim;

        // Low-pass filtered derivative
        float rawDeriv = (yaw_err - bfs_drive_prev_err) / dt;
        bfs_drive_prev_err = yaw_err;
        const float driveDFilterTau = 0.030f;
        float dAlpha = dt / (driveDFilterTau + dt);
        bfs_drive_dFilt += dAlpha * (rawDeriv - bfs_drive_dFilt);

        // Deceleration zone: boost KP in last 0.30m (like straight test)
        float remaining = bfs_run_segment_dist_m - bfs_run_driven_m;
        float effectiveKp = BFS_DRIVE_STEER_KP;
        if (remaining < 0.30f) {
          effectiveKp = BFS_DRIVE_STEER_KP * 1.5f;
        }

        float steerCorr = effectiveKp * yaw_err
                        + BFS_DRIVE_STEER_KI * bfs_drive_integral
                        + BFS_DRIVE_STEER_KD * bfs_drive_dFilt;
        if (steerCorr > 60.0f) steerCorr = 60.0f;
        if (steerCorr < -60.0f) steerCorr = -60.0f;

        // Post-turn steering ramp: suppress correction briefly to avoid jerk
        if (bfs_drive_steer_ramp_start_ms > 0) {
          unsigned long rampElapsed = now - bfs_drive_steer_ramp_start_ms;
          if (rampElapsed < BFS_DRIVE_STEER_RAMP_MS) {
            float rampFactor = (float)rampElapsed / (float)BFS_DRIVE_STEER_RAMP_MS;
            steerCorr *= rampFactor;
          } else {
            bfs_drive_steer_ramp_start_ms = 0;  // ramp complete
          }
        }

        // Deceleration zone: ramp down over last 0.20m (matching straight test)
        float speedFactor = 1.0f;
        if (remaining < 0.20f) {
          speedFactor = remaining / 0.20f;
          if (speedFactor < 0.30f) speedFactor = 0.30f;
        }

        // Time-based drive PWM: scale speed to finish path within runTimeS.
        // Subtract estimated non-driving overhead so nominalMps reflects actual drive speed needed.
        float drivePwm = (float)BFS_DRIVE_PWM;
        if (runTimeS > 0.0f && bfs_run_effective_drive_time_s > 0.1f && bfs_run_total_path_dist_m > 0.1f) {
          float elapsedS = (float)(now - bfs_run_start_ms) * 0.001f;
          float remainPathM = bfs_run_total_path_dist_m - bfs_run_total_driven_m - bfs_run_driven_m;
          if (remainPathM < 0.0f) remainPathM = 0.0f;
          // Subtract estimated future overhead from remaining wall-clock time
          // Current segment brakeToStop+IMU_settle = 0.7s, each future segment ≈ 1.5s
          int segsAfterCurrent = (int)(bfs_state.path_length - bfs_state.path_index) - 1;
          if (segsAfterCurrent < 0) segsAfterCurrent = 0;
          float futureOverheadS = 0.7f + (float)segsAfterCurrent * 1.5f;
          float remainTimeS = runTimeS - elapsedS - futureOverheadS;
          if (remainTimeS < 0.5f) remainTimeS = 0.5f;
          // Target speed to use remaining effective drive time (1.05x bias → prefer overshoot)
          float targetMps = remainPathM / (remainTimeS * 1.05f);
          float nominalMps = bfs_run_total_path_dist_m / bfs_run_effective_drive_time_s;
          // Clamp: never go below 60% or above 130% of nominal
          if (targetMps < nominalMps * 0.60f) targetMps = nominalMps * 0.60f;
          if (targetMps > nominalMps * 1.30f) targetMps = nominalMps * 1.30f;
          drivePwm = (float)BFS_DRIVE_PWM * (targetMps / nominalMps);
        }
        if (drivePwm < 110.0f) drivePwm = 110.0f;
        if (drivePwm > 220.0f) drivePwm = 220.0f;

        float basePwm = drivePwm * speedFactor;
        if (basePwm < 110.0f) basePwm = 110.0f;
        // Positive yaw_err → need to turn LEFT (CCW, increase IMU yaw)
        // → slow left wheel, speed up right wheel
        float leftPwm = basePwm - steerCorr + (float)motor_bias_pwm;
        float rightPwm = basePwm + steerCorr - (float)motor_bias_pwm;

        if (leftPwm < 0.0f) leftPwm = 0.0f;
        if (rightPwm < 0.0f) rightPwm = 0.0f;
        if (leftPwm > 255.0f) leftPwm = 255.0f;
        if (rightPwm > 255.0f) rightPwm = 255.0f;
        // Prevent one-wheel stall (matching straight test)
        if (leftPwm > 0.0f && leftPwm < 100.0f) leftPwm = 100.0f;
        if (rightPwm > 0.0f && rightPwm < 110.0f) rightPwm = 110.0f;

        setMotorsForwardPwm((uint8_t)(leftPwm + 0.5f), (uint8_t)(rightPwm + 0.5f));
        applyMotorOutputs();
        // 2026 rule: Run Time begins when Robot begins to move
        if (bfs_run_start_ms == 0) {
          bfs_run_start_ms = millis();
          Serial.printf("BFS_RUN: runtime started at first drive move\n");
        }

        // Periodic debug
        static unsigned long lastBfsDriveLog = 0;
        if (now - lastBfsDriveLog > 200) {
          lastBfsDriveLog = now;
          const int32_t dbgDL = encoderL_count - bfs_run_enc_start_L;
          const int32_t dbgDR = encoderR_count - bfs_run_enc_start_R;
          Serial.printf("BFS_DRIVE: driven=%.3f/%.2f rem=%.2f yawErr=%.1f Lpwm=%d Rpwm=%d encDL=%ld encDR=%ld encOK=%c%c rawL=%ld rawR=%ld\n",
                        (double)bfs_run_driven_m, (double)bfs_run_segment_dist_m,
                        (double)remaining, (double)yaw_err,
                        (int)(leftPwm + 0.5f), (int)(rightPwm + 0.5f),
                        (long)dbgDL, (long)dbgDR,
                        encoderL_found ? 'Y' : 'N', encoderR_found ? 'Y' : 'N',
                        (long)encoderL_count, (long)encoderR_count);
        }
      }
    }
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
    // Single click cycles step size; hold 1s toggles L/R; hold 2s saves & exits
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      const int stepCount = (int)(sizeof(tunePpmSteps) / sizeof(tunePpmSteps[0]));
      tunePpmStepIndex = (tunePpmStepIndex + 1) % stepCount;
      settings_dirty = true;
      settings_dirty_ms = millis();
    }

    static bool encCalHold1Handled = false;
    static bool encCalHold2Handled = false;
    if (!M5.BtnA.isPressed()) {
      encCalHold1Handled = false;
      encCalHold2Handled = false;
    }
    if (!encCalHold1Handled && M5.BtnA.pressedFor(1000)) {
      encCalHold1Handled = true;
      encCalEditingLeft = !encCalEditingLeft;
    }
    if (!encCalHold2Handled && M5.BtnA.pressedFor(2000)) {
      encCalHold2Handled = true;
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
            // Bleed the speed integral in the brake zone so the PI controller
            // can actually reduce basePWM instead of coasting on stale history.
            // Deeper into the zone → faster bleed (factor ranges 0.30..1.0).
            speedIntegral *= (0.90f + 0.10f * factor);  // at edge: ×1.0, at end: ×0.93
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

          // Fuse IMU-integrated forward velocity with encoder-derived rate for a more robust speed estimate.
          // Static fusion state is reset via the run control reset above.
          static float imu_vel_est = 0.0f;    // integrated IMU velocity (m/s)
          static float imu_vel_lp = 0.0f;     // low-pass filtered IMU velocity
          const float imu_fuse_alpha = 0.85f; // weight for encoder-derived velocity (0..1)

          float fusedPathRate = actualPathRate;
          float fusedForwardRate = actualForwardRate;
          if (imu_present && imu_ok && dt > 0.0f && dt < 0.5f) {
            // Use forward-axis accel (imu_ax_g) → m/s^2
            float accel_ms2 = imu_ax_g * 9.80665f;
            if (fabsf(accel_ms2) < 0.05f) accel_ms2 = 0.0f; // deadband
            imu_vel_est += accel_ms2 * dt;
            imu_vel_lp = 0.95f * imu_vel_lp + 0.05f * imu_vel_est;

            // Fuse: prefer encoder estimate but back off to IMU when encoder quantization/noise present
            fusedForwardRate = imu_fuse_alpha * actualForwardRate + (1.0f - imu_fuse_alpha) * imu_vel_lp;
            // Derive path rate from fused forward by dividing by cosYaw when safe
            if (fabsf(cosYaw) > 1e-3f) fusedPathRate = fusedForwardRate / cosYaw;
            else fusedPathRate = actualPathRate;
          } else {
            // No IMU available/healthy: decay integrated state to avoid long drift
            imu_vel_est *= 0.995f;
            imu_vel_lp *= 0.995f;
            fusedPathRate = actualPathRate;
            fusedForwardRate = actualForwardRate;
          }

          // Speed control
          // - Normal: PI in pulse-rate units
          // - Test: locked PWM to isolate heading control and reduce startup current surge
          const float actualRateForControl_mps = (fabsf(cosYaw) >= SPEED_TURN_COS_THRESHOLD) ? fusedForwardRate : fusedPathRate;
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

          // Enforce a maximum safe forward speed by scaling back base PWM when
          // the fused forward-rate estimate exceeds `MAX_ALLOWED_SPEED_MPS`.
          // Uses a small hysteresis to avoid chatter.
          {
            const float MAX_ALLOWED_SPEED_MPS = 0.25f;
            const float SPEED_HYST_MPS = 0.02f;
            static unsigned long run_last_overspeed_ms = 0;
            if (actualRateForControl_mps > (MAX_ALLOWED_SPEED_MPS + SPEED_HYST_MPS)) {
              // Aggressive cut: scale down proportionally but allow cutting to 10% as safe floor
              float scale = MAX_ALLOWED_SPEED_MPS / actualRateForControl_mps;
              if (scale < 0.10f) scale = 0.10f; // allow aggressive reduction
              basePWMf *= scale;
              // Also reduce integrator to avoid immediate re-raise of basePWMf
              speedIntegral *= 0.25f;
              runWarnTooFast = true;
              run_last_overspeed_ms = now;
            } else if (actualRateForControl_mps < (MAX_ALLOWED_SPEED_MPS - SPEED_HYST_MPS)) {
              // Clear warning and allow integrator to recover slowly
              runWarnTooFast = false;
            }
          }

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
                  beginTurnPhase(CANP_TURN_TO_PARALLEL, 0.0f);
                } else if (canPhase == CANP_DRIVE_PARALLEL) {
                  beginTurnPhase(CANP_TURN_BACK_IN, -thetaDeg);
                } else if (canPhase == CANP_DRIVE_DIAG_IN) {
                  beginTurnPhase(CANP_TURN_TO_CENTER, 0.0f);
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
            // Auto-boost: if we're commanding forward motion but encoders show no movement,
            // ramp PWM toward 255 over NO_MOTION_RAMP_TO_MAX_MS.
            const uint32_t sinceEncMoveMs = (lastMotionMs != 0) ? ((uint32_t)now - (uint32_t)lastMotionMs) : 0u;
            if (effectiveTargetForwardRate > 0.0f && sinceEncMoveMs >= NO_MOTION_RAMP_DELAY_MS && sinceEncMoveMs < (uint32_t)RUN_NO_MOTION_TIMEOUT_MS) {
              float targetRamp = calcNoMotionRampPwm(0.0f, sinceEncMoveMs);
              if (leftPWMf > 0.0f) {
                leftPWMf = fmaxf(leftPWMf, targetRamp);
                if (leftPWMf > 255.0f) leftPWMf = 255.0f;
              }
              if (rightPWMf > 0.0f) {
                rightPWMf = fmaxf(rightPWMf, targetRamp);
                if (rightPWMf > 255.0f) rightPWMf = 255.0f;
              }
            }
            // Prefer pivot-in-place whenever forward target is essentially zero.
            // This issues opposing wheel commands (one forward, one reverse) so the robot
            // turns without large lateral drift. Pivot PWM is clamped for safety.
            const float PIVOT_MAX_PWM = 200.0f; // cap pivot PWM for safety
            float finalLeftPwmF = leftPWMf;
            float finalRightPwmF = rightPWMf;
            bool usePivot = (effectiveTargetForwardRate <= 0.05f);
            float pivotPwmF = 0.0f;
            if (usePivot) {
              int turnSign = (yawErrDegForLog >= 0.0f) ? 1 : -1;
              pivotPwmF = basePWMf + fabsf(corr);
              if (pivotPwmF < 110.0f) pivotPwmF = 110.0f;
              if (pivotPwmF > PIVOT_MAX_PWM) pivotPwmF = PIVOT_MAX_PWM;
              // Represent pivot as opposing wheel pwms for overspeed handling
              finalLeftPwmF = pivotPwmF;
              finalRightPwmF = pivotPwmF;
            } else {
              finalLeftPwmF = leftPWMf;
              finalRightPwmF = rightPWMf;
            }

            // Apply aggressive overspeed throttling to the final PWM outputs every tick.
            {
              const float MAX_ALLOWED_SPEED_MPS = 0.25f;
              const float SPEED_HYST_MPS = 0.02f;
              if (actualRateForControl_mps > (MAX_ALLOWED_SPEED_MPS + SPEED_HYST_MPS)) {
                float scale = MAX_ALLOWED_SPEED_MPS / actualRateForControl_mps;
                if (scale < 0.10f) scale = 0.10f;
                finalLeftPwmF *= scale;
                finalRightPwmF *= scale;
                speedIntegral *= 0.25f;
                runWarnTooFast = true;
              } else if (actualRateForControl_mps < (MAX_ALLOWED_SPEED_MPS - SPEED_HYST_MPS)) {
                runWarnTooFast = false;
              }
            }

            if (usePivot) {
              int turnSign = (yawErrDegForLog >= 0.0f) ? 1 : -1;
              // Use the pivotPwmF magnitude but scaled per overspeed above
              uint8_t applyPwm = (uint8_t)(fminf(fmaxf((finalLeftPwmF + 0.5f), 0.0f), 255.0f));
              setMotorsPivotPwm(turnSign, applyPwm);
            } else {
              setMotorsForwardPwm((uint8_t)(finalLeftPwmF + 0.5f), (uint8_t)(finalRightPwmF + 0.5f));
            }
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
          // Telemetry: left/right wheel distance (m) for CSV
          {
            const int32_t dL = encoderL_count - runStartEncL;
            const int32_t dR = encoderR_count - runStartEncR;
            const float dLcorr = INVERT_LEFT_ENCODER_COUNT ? -(float)dL : (float)dL;
            const float dRcorr = INVERT_RIGHT_ENCODER_COUNT ? -(float)dR : (float)dR;
            s.distL = (pulsesPerMeterL > 1.0f) ? (dLcorr / pulsesPerMeterL) : 0.0f;
            s.distR = (pulsesPerMeterR > 1.0f) ? (dRcorr / pulsesPerMeterR) : 0.0f;
          }
          // Append at 10 Hz (every 100 ms) so CSV has currentYaw, targetYaw, distL, distR at 10 Hz
          static unsigned long lastRunLogMs = 0;
          if ((now - runStartMs) < 100u) lastRunLogMs = 0;
          if ((now - lastRunLogMs) >= 100u) {
            lastRunLogMs = now;
            if (runLogEveryN <= 1 || ((runLogTick % runLogEveryN) == 0)) {
              runLogAppend(s);
            }
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

    // Button A click: cycle mode LEFT → RIGHT → FIND MIN
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked() && (now - last_button_ms) > 250) {
      last_button_ms = now;
      if (selected_motor == 2 && !hw_find_running && !hw_find_done) {
        // In FIND MIN mode, clicking starts the sweep
        hw_find_running = true;
        hw_find_done = false;
        hw_find_motor = 0; // start with left motor
        hw_find_pwm = HW_FIND_START_PWM;
        hw_find_start_ms = millis();
        hw_min_pwm_L = 0;
        hw_min_pwm_R = 0;
        stopAllMotors();
        delay(100);
        readWheelEncodersInternal(true);
        hw_find_enc_snap_L = encoderL_count;
        hw_find_enc_snap_R = encoderR_count;
        hw_find_step_ms = millis();
        // Start first motor at starting PWM
        motor_l_direction = 1;
        motor_l_speed = hw_find_pwm;
        motor_r_direction = 0;
        motor_r_speed = 0;
        applyMotorOutputs();
        Serial.printf("HW_FIND: starting sweep motor=LEFT pwm=%d\n", hw_find_pwm);
      } else if (selected_motor == 2 && hw_find_done) {
        // Results shown, click again resets to allow re-run
        hw_find_done = false;
      } else {
        // Cycle mode: 0 → 1 → 2
        selected_motor = (selected_motor + 1) % 3;
        stopAllMotors();
        motor_command = 0;
        if (selected_motor == 2) {
          hw_find_running = false;
          hw_find_done = false;
        }
      }
    }

    // --- FIND MIN sweep state machine ---
    if (selected_motor == 2 && hw_find_running) {
      if (now - hw_find_step_ms >= HW_FIND_STEP_DURATION_MS) {
        // Read encoders and check for movement
        readWheelEncodersInternal(true);
        int32_t deltaL = abs((int32_t)(encoderL_count - hw_find_enc_snap_L));
        int32_t deltaR = abs((int32_t)(encoderR_count - hw_find_enc_snap_R));

        bool moved = false;
        if (hw_find_motor == 0) {
          // Testing left motor — check left encoder (or right if cross-wired)
          moved = (deltaL > HW_FIND_PULSE_THRESHOLD) || (deltaR > HW_FIND_PULSE_THRESHOLD);
          if (moved) {
            hw_min_pwm_L = hw_find_pwm;
            Serial.printf("HW_FIND: LEFT min PWM = %d (dL=%ld dR=%ld)\n",
                          hw_find_pwm, (long)deltaL, (long)deltaR);
          }
        } else {
          // Testing right motor
          moved = (deltaL > HW_FIND_PULSE_THRESHOLD) || (deltaR > HW_FIND_PULSE_THRESHOLD);
          if (moved) {
            hw_min_pwm_R = hw_find_pwm;
            Serial.printf("HW_FIND: RIGHT min PWM = %d (dL=%ld dR=%ld)\n",
                          hw_find_pwm, (long)deltaL, (long)deltaR);
          }
        }

        if (moved || hw_find_pwm >= 255) {
          // Found threshold or reached max — move to next motor or finish
          stopAllMotors();
          delay(200);
          if (hw_find_motor == 0) {
            // Switch to right motor
            hw_find_motor = 1;
            hw_find_pwm = HW_FIND_START_PWM;
            hw_find_start_ms = millis();
            readWheelEncodersInternal(true);
            hw_find_enc_snap_L = encoderL_count;
            hw_find_enc_snap_R = encoderR_count;
            hw_find_step_ms = millis();
            motor_l_direction = 0;
            motor_l_speed = 0;
            motor_r_direction = 1;
            motor_r_speed = hw_find_pwm;
            applyMotorOutputs();
            Serial.printf("HW_FIND: switching to RIGHT motor pwm=%d\n", hw_find_pwm);
          } else {
            // Both done
            hw_find_running = false;
            hw_find_done = true;
            Serial.printf("HW_FIND: DONE  L_min=%d  R_min=%d\n",
                          (int)hw_min_pwm_L, (int)hw_min_pwm_R);
          }
        } else {
          // No movement yet — ramp PWM toward 255 in first 3 seconds 
          uint32_t sinceStartMs = (uint32_t)(now - hw_find_start_ms);
          float frac = (float)sinceStartMs / (float)NO_MOTION_RAMP_TO_MAX_MS;
          if (frac < 0.0f) frac = 0.0f;
          if (frac > 1.0f) frac = 1.0f;
          uint8_t targetPwm = (uint8_t)(HW_FIND_START_PWM + frac * (NO_MOTION_RAMP_MAX_PWM - NO_MOTION_RAMP_START_PWM) + 0.5f);
          if (targetPwm < hw_find_pwm + 1) targetPwm = hw_find_pwm + 1;
          if (targetPwm > 255) targetPwm = 255;
          hw_find_pwm = targetPwm;
          readWheelEncodersInternal(true);
          hw_find_enc_snap_L = encoderL_count;
          hw_find_enc_snap_R = encoderR_count;
          hw_find_step_ms = millis();
          if (hw_find_motor == 0) {
            motor_l_speed = hw_find_pwm;
            motor_r_speed = 0;
          } else {
            motor_l_speed = 0;
            motor_r_speed = hw_find_pwm;
          }
          motor_l_direction = 1;
          motor_r_direction = 1;
          applyMotorOutputs();
        }
      }
    }
    // --- Normal manual motor control (modes 0 and 1) ---
    else if (selected_motor <= 1) {
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

      // HW test: motion detection + conservative auto-boost for manual motor control
      {
        if (motor_enabled && lastMotionMs_hw == 0) {
          // initialize trackers on first enable
          readWheelEncodersInternal(true);
          lastMotionEncL_hw = encoderL_count;
          lastMotionEncR_hw = encoderR_count;
          lastMotionMs_hw = now;
        }
        // check for encoder movement
        const int32_t dML = encoderL_count - lastMotionEncL_hw;
        const int32_t dMR = encoderR_count - lastMotionEncR_hw;
        const float motionAbs = (fabsf((float)dML) + fabsf((float)dMR)) * 0.5f;
        if (motionAbs >= (float)RUN_NO_MOTION_MIN_PULSES) {
          lastMotionEncL_hw = encoderL_count;
          lastMotionEncR_hw = encoderR_count;
          lastMotionMs_hw = now;
        }
        const uint32_t sinceEncMoveMs_hw = (lastMotionMs_hw != 0) ? ((uint32_t)now - (uint32_t)lastMotionMs_hw) : 0u;
        if (motor_enabled && sinceEncMoveMs_hw >= NO_MOTION_RAMP_DELAY_MS && sinceEncMoveMs_hw < (uint32_t)RUN_NO_MOTION_TIMEOUT_MS) {
          float targetRamp = calcNoMotionRampPwm(0.0f, sinceEncMoveMs_hw);
          if (motor_l_speed > 0) {
            float f = fmaxf((float)motor_l_speed, targetRamp);
            if (f > 255.0f) f = 255.0f;
            motor_l_speed = (uint8_t)f;
          }
          if (motor_r_speed > 0) {
            float f = fmaxf((float)motor_r_speed, targetRamp);
            if (f > 255.0f) f = 255.0f;
            motor_r_speed = (uint8_t)f;
          }
        }
      }

      applyMotorOutputs();
    }
  }

  else if (currentScreen == SCREEN_IMU_CAL) {
    // Click: re-run calibration
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked()) {
      if (imu_present) {
        delay(500);
        calibrateImuGyroBias(); // Declaration is in Robot.cpp
        imu_yaw = 0.0f;  // Reset yaw after recalibration
      }
    }
    // Hold: back to menu
    static bool imuCalLongPressHandled = false;
    if (!M5.BtnA.isPressed()) imuCalLongPressHandled = false;
    if (!imuCalLongPressHandled && M5.BtnA.pressedFor(800)) {
      imuCalLongPressHandled = true;
      suppressNextClick = true;
      enterScreen(SCREEN_MAIN_MENU);
    }
    stopAllMotors();
  }

  else if (currentScreen == SCREEN_CALIBRATE) {
    // Run the non-blocking two-stage IMU calibration.
    runImuCalibration();

    // When calibration completes, automatically transition to the next screen
    // (typically SCREEN_BFS_RUN) as configured by the caller.
    if (!imuCal_running && imuCal_done) {
      imuCal_done = false;
      enterScreen(imuCal_next_screen);
    }

    stopAllMotors();
  }

  else if (currentScreen == SCREEN_SELF_TEST) {
    // Run the self-test state machine
    if (st_phase != ST_IDLE && st_phase != ST_DONE) {
      selfTestTick();
    }

    // Hold: abort/back to menu
    static bool stLongPressHandled = false;
    if (!M5.BtnA.isPressed()) stLongPressHandled = false;
    if (!stLongPressHandled && M5.BtnA.pressedFor(800)) {
      stLongPressHandled = true;
      suppressNextClick = true;
      stopAllMotors();
      st_phase = ST_IDLE;
      enterScreen(SCREEN_MAIN_MENU);
    }

    // Click: start test (if idle or done)
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked() && (now - last_button_ms) > 250) {
      last_button_ms = now;
      if (st_phase == ST_IDLE || st_phase == ST_DONE) {
        selfTestStart();
      }
    }
  }

  else if (currentScreen == SCREEN_SQUARE_TEST) {
    // Run the square test state machine
    if (sq_phase != SQ_IDLE && sq_phase != SQ_DONE) {
      sqTestTick();
    }

    // Hold: abort/back to menu
    static bool sqLongPressHandled = false;
    if (!M5.BtnA.isPressed()) sqLongPressHandled = false;
    if (!sqLongPressHandled && M5.BtnA.pressedFor(800)) {
      sqLongPressHandled = true;
      suppressNextClick = true;
      stopAllMotors();
      sq_phase = SQ_IDLE;
      enterScreen(SCREEN_MAIN_MENU);
    }

    // Click actions depend on phase
    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked() && (now - last_button_ms) > 250) {
      last_button_ms = now;
      if (sq_phase == SQ_IDLE || sq_phase == SQ_DONE) {
        sqTestStart();
      } else {
        // Abort running test
        stopAllMotors();
        sq_phase = SQ_IDLE;
      }
    }

    // Dial adjusts side length only when idle
    if (sq_phase == SQ_IDLE && dialSteps != 0) {
      sq_side_len_m += dialSteps * 0.05f;
      if (sq_side_len_m < 0.20f) sq_side_len_m = 0.20f;
      if (sq_side_len_m > 2.00f) sq_side_len_m = 2.00f;
    }
  }

  // ---- STRAIGHT TEST input ----
  else if (currentScreen == SCREEN_STRAIGHT_TEST) {
    if (str_phase != STR_IDLE && str_phase != STR_DONE) {
      strTestTick();
    }

    static bool strLongPressHandled = false;
    if (!M5.BtnA.isPressed()) strLongPressHandled = false;
    if (!strLongPressHandled && M5.BtnA.pressedFor(800)) {
      strLongPressHandled = true;
      suppressNextClick = true;
      stopAllMotors();
      str_phase = STR_IDLE;
      enterScreen(SCREEN_MAIN_MENU);
    }

    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked() && (now - last_button_ms) > 250) {
      last_button_ms = now;
      if (str_phase == STR_IDLE || str_phase == STR_DONE) {
        strTestStart();
      } else {
        stopAllMotors();
        str_phase = STR_IDLE;
      }
    }

    if (str_phase == STR_IDLE && dialSteps != 0) {
      str_target_m += dialSteps * 0.10f;
      if (str_target_m < 0.20f) str_target_m = 0.20f;
      if (str_target_m > 5.00f) str_target_m = 5.00f;
    }
  }

  // ---- TURN TEST input ----
  else if (currentScreen == SCREEN_TURN_TEST) {
    if (trn_phase != TRN_IDLE && trn_phase != TRN_DONE) {
      trnTestTick();
    }

    // 2s hold: back to menu
    static bool trnExitHandled = false;
    if (!M5.BtnA.isPressed()) trnExitHandled = false;
    if (!trnExitHandled && M5.BtnA.pressedFor(2000)) {
      trnExitHandled = true;
      suppressNextClick = true;
      stopAllMotors();
      trn_phase = TRN_IDLE;
      enterScreen(SCREEN_MAIN_MENU);
    }

    // 1s hold: toggle direction (only when idle)
    static bool trnDirHandled = false;
    if (!M5.BtnA.isPressed()) trnDirHandled = false;
    if (!trnDirHandled && M5.BtnA.pressedFor(1000) && (trn_phase == TRN_IDLE || trn_phase == TRN_DONE)) {
      trnDirHandled = true;
      suppressNextClick = true;
      trn_direction = -trn_direction;  // toggle LEFT/RIGHT
      Serial.printf("TRN: direction toggled to %s\n", trn_direction < 0 ? "RIGHT" : "LEFT");
    }

    if ((now - boot_ms) > 500 && M5.BtnA.wasClicked() && (now - last_button_ms) > 250) {
      last_button_ms = now;
      if (trn_phase == TRN_IDLE || trn_phase == TRN_DONE) {
        trnTestStart();
      } else {
        stopAllMotors();
        trn_phase = TRN_IDLE;
      }
    }

    if (trn_phase == TRN_IDLE && dialSteps != 0) {
      trn_target_deg += dialSteps * 5.0f;
      if (trn_target_deg < 5.0f) trn_target_deg = 5.0f;
      if (trn_target_deg > 360.0f) trn_target_deg = 360.0f;
    }
  }

  // Render the active screen — throttle during active tests to reduce loop jitter
  static unsigned long lastDisplay = 0;
  const bool testRunning = (sq_phase != SQ_IDLE && sq_phase != SQ_DONE) ||
                           (str_phase != STR_IDLE && str_phase != STR_DONE) ||
                           (bfs_run_phase != BFS_PHASE_IDLE) || runActive;
  const unsigned long displayInterval = testRunning ? 200u : DISPLAY_REFRESH_MS;
  if (now - lastDisplay > displayInterval) {
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

// Blocking calibration implementation that updates the IMU CAL screen live.
static void calibrateImuGyroBias() {
  if (!imu_present) return;

  imu_calibrating = true;
  imu_cal_progress = 0;

  const unsigned long t0 = millis();
  // 0.5s hands-off wait (enough for post-button vibrations to settle)
  while ((millis() - t0) < 500u) {
    float ax, ay, az, gx, gy, gz, tmp;
    (void)imu6886Read(&ax, &ay, &az, &gx, &gy, &gz, &tmp);
    imu_cal_progress = (int)(((millis() - t0) * 15u) / 500u);
    if (imu_cal_progress < 0) imu_cal_progress = 0;
    if (imu_cal_progress > 15) imu_cal_progress = 15;
    drawImuCalScreen();
    ui.pushSprite(0, 0);
    yield();
  }

  double sumGz = 0.0;
  int samples = 0;
  int attempts = 0;
  const int MAX_ATTEMPTS = 1500;  // bail if IMU fails repeatedly
  while (samples < 500 && attempts < MAX_ATTEMPTS) {
    float ax, ay, az, gx, gy, gz, tmp;
    const bool ok = imu6886Read(&ax, &ay, &az, &gx, &gy, &gz, &tmp);
    attempts++;
    if (ok) {
      sumGz += (double)gz;
      samples += 1;
      imu_cal_progress = 15 + (samples * 85) / 500;
      if (imu_cal_progress > 100) imu_cal_progress = 100;
    }
    if ((attempts & 0xF) == 0) {
      // update display every 16 attempts for smooth progress bar animation
      drawImuCalScreen();
      ui.pushSprite(0, 0);
    }
    yield();
  }
  if (samples < 500) {
    Serial.printf("IMU CAL: only got %d/500 samples in %d attempts — sensor may be disconnected\n", samples, attempts);
  }

  const double avg = (samples > 0) ? (sumGz / (double)samples) : 0.0;
  imu_gz_bias_dps = (float)avg;
  if (fabsf(imu_gz_bias_dps) > 20.0f) imu_gz_bias_dps = 0.0f;
  if (imu_gz_bias_dps > IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = IMU_GZ_BIAS_CLAMP_DPS;
  if (imu_gz_bias_dps < -IMU_GZ_BIAS_CLAMP_DPS) imu_gz_bias_dps = -IMU_GZ_BIAS_CLAMP_DPS;

  imu_gz_dps = imu_gz_raw_dps - imu_gz_bias_dps;
  persistImuGyroBiasIfNeeded();

  imu_cal_progress = 100;
  drawImuCalScreen();
  ui.pushSprite(0, 0);
  imu_calibrating = false;
}
