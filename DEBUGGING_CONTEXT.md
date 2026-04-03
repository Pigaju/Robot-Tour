# Robot Tour Debugging Context

Use this file to bootstrap a new Copilot chat session with full context.

## Platform
- **MCU**: ESP32-S3 (Adafruit Feather ESP32-S3 Reverse TFT)
- **Build**: PlatformIO (`~/.platformio/penv/bin/platformio run`)
- **Upload**: `platformio run --target upload --upload-port /dev/cu.usbmodem1101`
- **Log download**: `python3 tools/download_logs.py` (pyserial required)
- **Workspace**: `/Users/fang/RobotTourRepo/Robot-Tour`
- **Main firmware**: `src/main.cpp` (~12,466 lines)
- **Original backup**: `main_copy_backup.cpp` (identical to `~/Downloads/main (1).cpp`)

## Hardware
- **Motor drivers**: I2C H-Bridge modules
  - MOTOR_L_ADDR = 0x21, MOTOR_R_ADDR = 0x20
  - Register 0x00 = direction (0=STOP, 1=FWD, 2=BWD)
  - Register 0x01 = speed (0–255 PWM)
- **Encoders**: I2C at 0x58 (left), 0x59 (right)
  - ppmL = 922.9 pulses/meter, ppmR = 889.3 pulses/meter (3.8% difference)
- **IMU**: MPU6886 — heading-hold PID for steering
- **Motor direction inversions**: ALL DISABLED (`INVERT_MOTOR_DIRECTION`, `INVERT_LEFT_MOTOR_DIRECTION`, `INVERT_RIGHT_MOTOR_DIRECTION` — none are `#define`d)
- **Encoder inversions**: Both 0
- **motor_bias_pwm** = 1 (adds +1 left, −1 right)

---

## Core Problem
The robot drifts **rightward** during straight driving. In run #90 (straight test, 1m), yaw drifted to −24°. Left wheel traveled 0.9048m vs right 0.8377m (8% asymmetry) at identical commanded PWM.

The PID tried to compensate (lPwm dropped to ~100, rPwm rose to ~189), but a `driveMotor()` minimum PWM clamp of 170 erased all differential — both motors got clamped to 170.

## Root Cause Investigation — STILL OPEN

**Why is there a ~120 PWM gap needed between motors?** The PID wants lPwm≈100 and rPwm≈189 to hold a straight line. This is a huge asymmetry for what should be a symmetric drivetrain.

### What we've checked
1. **Motor addresses**: Confirmed correct (0x21=left, 0x20=right). Swapping them causes the robot to **spin** (one motor forward, one backward), confirming the physical wiring expects this mapping.
2. **Direction inversion macros**: None are `#define`d, all `#if` blocks are inactive.
3. **Encoder calibration**: ppmL=922.9 vs ppmR=889.3 is only 3.8% — not enough to explain 8%+ speed difference or −24° yaw drift.
4. **Register readback**: `regL_dir`, `regL_pwm`, `regR_dir`, `regR_pwm` are ALL ZEROS in run #90 logs. **Reason**: The straight test uses `testLogAppendSample()` (line 7028) which does `memset(&s, 0, sizeof(s))` and never populates cmd/reg fields. Only the RUN mode log path (line 11912) populates them.
5. **PWM clamping layers** — 7 layers identified and all reverted to original values (see below).

### Hypotheses NOT yet tested
- One motor is physically weaker (worn brushes, different friction, bad H-bridge)
- Encoder-to-motor cross-wiring (encoder L reading motor R's wheel?)
- H-bridge output voltage differs between 0x20 and 0x21 at same PWM
- Battery voltage sag under load affecting one channel more
- Wheel diameter difference
- **Test idea**: Use WHEEL TEST screen to run each motor individually at fixed PWM and compare encoder tick rates

---

## Changes Made (from original `main_copy_backup.cpp`)

### Reverted to original (were bugs introduced during earlier debugging)
| Item | Was changed to | Reverted back to |
|------|---------------|-------------------|
| MOTOR_L_ADDR | 0x20 | **0x21** (original) |
| MOTOR_R_ADDR | 0x21 | **0x20** (original) |
| Pivot turn directions | swapped | **original** |
| `RUN_MIN_PWM` | 170 | **95** |
| `driveMotor()` minPwm | `(uint8_t)RUN_MIN_PWM` (=170) | **110** (hardcoded, original) |
| `RUN_SPEED_PROFILE_MAX_PWM` | 255 | **235** |
| `RUN_SPEED_PROFILE_MIN_PWM` | 170 | **110** |
| `RUN_SPEED_PROFILE_START_BOOST_PWM` | 170 | **90** |

### Retained new features/fixes (keep these)
- **WHEEL TEST screen**: New UI screen for testing individual wheel response
- **Square test zero-crossing integral reset**: Prevents PID jitter at turn boundaries
- **`calcNoMotionRampPwm()` centralized function**: Extracted from duplicated code
- **No-motion ramp constants**: Defined at top level

---

## Key Code Locations

| Function/Feature | Lines (approx) |
|-----------------|----------------|
| `RunLogSample` struct | ~1320–1365 |
| `testLogAppendSample()` | 7028–7052 |
| `driveMotor()` | 4547–4600 |
| `setMotorsForwardPwm()` | 4018 |
| `setMotorsPivotPwm()` | 4036 |
| `applyMotorOutputs()` | 4837 |
| Straight test PID loop | 7940–8060 |
| Square test PID loop | 7480–7610 |
| BFS drive PID loop | 9920–10050 |
| RUN mode main loop | 10500–11850 |
| RUN mode log population (cmdL_dir etc) | 11900–11945 |

## PWM Control Flow (straight test)
```
basePwm = 130 (BFS_DRIVE_PWM)
steerCorr = KP*yawErr + KI*integral + KD*derivative  (clamped ±60)
leftPwm  = basePwm - steerCorr + motor_bias_pwm
rightPwm = basePwm + steerCorr - motor_bias_pwm
→ driveMotor(MOTOR_L_ADDR, direction, leftPwm)
→ driveMotor(MOTOR_R_ADDR, direction, rightPwm)
    └→ if (speed < 110) speed = 110;  // minPwm floor in driveMotor()
```

## Steering PID Parameters
| Mode | KP | KI | KD | Clamp |
|------|----|----|-----|-------|
| Straight test | 2.0 | 0.8 | 0.05 | ±60 |
| BFS drive | 2.0 | 1.0 | 0.06 | — |
| RUN mode | — | — | — | Uses saturation shifting to preserve differential |

## Mixer Convention
**Positive yaw error = robot turned right of target → steerCorr > 0 → leftPwm decreases, rightPwm increases → corrects left.** This convention is identical across main.cpp, main_copy_backup.cpp, and Downloads/main(1).cpp — all three files have `leftPwm = basePwm - steerCorr`.

---

## Log Files Available
| File | Description |
|------|-------------|
| `logs/20260401_185646_runlog_90.csv` | Run #90, straight test 1m. After address revert, before minPwm fix. **Best log for analyzing 120 PWM gap.** |
| `logs/20260401_183902_runlog_88.csv` | Run #88, straight test 1m. With swapped addresses + RUN_MIN_PWM=170. |
| `logs/20260401_011859_runlog_80.csv` | Run #80 (earlier session) |

### Log format notes
- CSV, 53 columns, header on row 3
- Columns 46–51 (`cmdL_dir`, `cmdR_dir`, `regL_dir`, `regL_pwm`, `regR_dir`, `regR_pwm`) are **always zero in test modes** — only populated in RUN mode
- Key data columns: `t_ms`(1), `meters`(2), `currentYaw`(4), `targetYaw`(5), `lPwm`(25), `rPwm`(26), `distL`(52), `distR`(53)

## Plot Scripts
- `logs/plot_run90_pid.py` — 8-panel PID diagnostic (time-series of yaw, PWM, encoder distances, PID terms)
- `logs/plot_run88.py`, `logs/plot_runs.py` — earlier analysis

---

## Important Behavioral Notes
- When motor addresses are **swapped** (L=0x20, R=0x21), the robot **spins in place** — one motor goes forward, one backward. This confirms the physical H-bridge wiring expects the current mapping.
- The `driveMotor()` function has an `#if INVERT_LEFT_MOTOR_DIRECTION` block that flips direction for the left motor — but this macro is **NOT defined**, so no software inversion occurs.
- `motor_bias_pwm = 1` is a tiny static correction (+1 left, −1 right). Not nearly enough to compensate the actual asymmetry.

## Git Workflow
- Always commit after every successful upload to the robot
- Commit message should describe the firmware changes made
