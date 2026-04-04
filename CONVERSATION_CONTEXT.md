# Robot Tour — Conversation Context (April 3, 2026, Evening)

Use this file to bootstrap a new chat window. Paste/attach it so the assistant has full context.

---

## Project Overview

- **Robot**: ESP32-S3 (Adafruit Feather ESP32-S3 Reverse TFT / M5Dial) — Science Olympiad 2026 Robot Tour event
- **Firmware**: `src/main.cpp` (~12,500+ lines), PlatformIO, single-file architecture
- **Navigation**: BFS waypoint navigation state machine: IDLE → TURN → BRAKE → SETTLE → DRIVE → ARRIVED
- **Build**: `~/.platformio/penv/bin/platformio run`
- **Upload**: `platformio run --target upload --upload-port /dev/cu.usbmodem101`
- **Download logs**: `python3 tools/download_logs.py`

## Key Parameters (current values in firmware)

| Parameter | Value | Location |
|---|---|---|
| BFS_DRIVE_STEER_KP | 2.5f | ~line 3371 |
| BFS_DRIVE_STEER_KI | 1.5f | ~line 3372 |
| BFS_DRIVE_STEER_KD | 0.25f | ~line 3373 |
| steerCorr clamp | ±60 PWM | ~line 9978 |
| motor_bias_pwm | -12 | NVS key "mtr_bias3" |
| BFS_DRIVE_PWM | 160 | ~line 3370 |
| drivePwm floor | 130 | ~line 10025 |
| basePwm floor | 110 | ~line 10042 |
| BFS_TURN_PWM | 130 | |
| BFS_TURN_TOLERANCE_DEG | 2.0f | |
| BFS_TURN_SETTLE_MS | 300 | |
| Turn PID | Ki=1.0, Kd=0.15, err/45 base, floor 115 | |
| Encoder PPM | L=924.5, R=888.0 | NVS |
| PWM floor (satshift) | L=90, R=95 | ~line 10050 |
| Launch ramp | Adaptive: cubic t³ 130→target, 800ms, encoder feedback + speed limiter | ~line 10075 |
| Decel zone | Logarithmic decay, 30cm zone, ln(1+t*(e-1)), floor 0.45, coast-stop at arrival | ~line 10045 |
| Leaky integral tau | 1.0s | |
| Anti-windup | ±25 PWM |
| steerI carry cap | ±5.0 between segments | | |
| Heading abort | >30° for 300ms → skip segment | |
| Stall detector | no avg progress for 2s → skip | |
| Single-wheel stall | one encoder stopped 500ms → skip | |
| Heading-hold settle | KP=1.5, ±30 clamp, >1° dead zone | |
| Steer ramp post-turn | 150ms suppress | |

## Mixer Convention

```
leftPwm  = basePwm - steerCorr + motor_bias_pwm
rightPwm = basePwm + steerCorr - motor_bias_pwm
```
Same in square test, straight test, BFS DRIVE, and RUN mode.

## Motor / Encoder Addresses

- MOTOR_L_ADDR = 0x21, MOTOR_R_ADDR = 0x20
- ENCODER_L_ADDR = 0x58, ENCODER_R_ADDR = 0x59
- **Never swap motor addresses** — the full PID sign chain is tuned for these

## All Fixes Applied (chronological)

### 1. Run 117 — Post-turn drive stall (fixed)
- **Problem**: After turning, drive PWM was too low to overcome static friction
- **Fix**: Launch boost after turn/settle completes (now: smooth 160→target ramp over 500ms)

### 2. Run 118 — BFS turns failing while standalone turns worked (fixed)
- **Problem**: BFS turn PID parameters didn't match standalone turn test
- **Fix**: Matched BFS turn PID to standalone: PWM floor 95→115, removed gyro-still gate, tolerance 3°→2°, settle 400→300ms, kept I+D terms (Ki=1.0, Kd=0.15)

### 3. Runs 124-125 — Severe rightward drift during forward driving (fixed)
- **Problem**: PWM floor clamping destroyed steering differential at low basePwm
- **Fix**: Saturation shift — when one wheel clips at floor, deficit transfers to OTHER wheel

### 4. Run 126 — Wrong distance / skipping segments (fixed)
- **Problem**: Boot encoder auto-test was failing, setting `enc_mode = ENC_MODE_TIMED`
- **Fix**: Override TIMED→BOTH after boot test. Added `enc=` field to log header.

### 5. Runs 127-129 — Inconsistent PWM causing traversal failures (fixed)
- **Root cause analysis**: Speed scaling drove basePWM to 90-100 range (68% of samples). With motor_bias=-12, left wheel was permanently below floor. Launch boost created 40-50 PWM cliff. KP=3.5 saturated on moderate errors. Post-turn heading errors were 5-22°.
- **Fixes applied**:
  - Raised drivePwm floor from 100→130
  - Replaced hard launch boost (140 PWM for 300ms) with smooth 160→target ramp over 500ms
  - Reduced BFS_DRIVE_STEER_KP from 3.5→2.5
  - Added heading-hold PID during SETTLE phase (KP=1.5, ±30 clamp, >1° triggers correction)

### 6. Runs 127-129 — Later turns degrade from gyro drift (fixed)
- **Problem**: Turn target yaw was computed from accumulated grid heading (`start_yaw - imu_yaw`), so MPU6886 drift compounded across segments
- **Fix**: Drift-immune relative turns — turn delta computed purely from grid geometry (`atan2(next_seg) - atan2(prev_seg)`), snapped to ±90°. Target = `current_imu_yaw - geometric_delta`. Each turn is a fresh relative rotation.

### 7. Run 131 — Single-wheel stall + deceleration zone failure (fixed)
- **Problem**: Seg 6 left encoder stopped at 0.424m while right kept going. Robot pivoted in place. Average-distance stall detector didn't fire because right wheel still moved. Deceleration zone dropped basePWM to 90×0.30=27 effective, causing inability to correct heading.
- **Fixes applied**:
  - Single-wheel stall detection: if one encoder stops while other moves for 500ms, skip segment
  - Deceleration zone: shortened 0.20m→0.15m, raised floor 0.30→0.60, raised basePwm minimum 90→110
  - Carry forward steerI between segments (mechanical bias is persistent, saves ~1s convergence)

### 8. Run 134 — Instant PWM spike after every turn (fixed)
- **Problem**: Old launch boost ramped 160→drivePwm, but time-based speed scaling pushed drivePwm to 220. Since 160 < 220, the ramp was completely bypassed — robot jumped from 0→220 PWM instantly. Caused heading errors of 4-25° (seg 5 worst: 25.3° error, corrOut clamped at ±60).
- **Fix**: Adaptive closed-loop launch ramp using encoder feedback:
  - Quadratic ease-in from 130→drivePwm over 500ms (normal case)
  - Exponential `exp(4t)` increase when stuck (encoder < 3mm after 100ms)
  - Early ramp exit when encoder speed ≥ 85% of target speed
  - At 250ms (midpoint), PWM is only 25% of the way to target → gives steering PID time to lock on

### 9. Run 135 — Launch ramp still too aggressive + steerI catastrophic carry-forward (fixed)
- **Problem 1**: Quadratic ramp (500ms) reached ~200 PWM by 430ms. Heading swings 5-11° on segments 4 and 6 during first 300ms of driving.
- **Problem 2**: Physical bump on seg 10 caused 468°/s spin. steerI grew to -16.3 during stall. Carried into seg 11, producing 14 PWM correction with only 0.17° error → robot spiraled out.
- **Fixes applied**:
  - Cubic (t³) ease-in over 800ms: at 400ms midpoint, only 12.5% of PWM range (was 64% with quadratic)
  - Speed limiter: if encoder speed > 130% of ramp target, hold at launchFloor
  - Softer exponential stuck-detect: exp(3t) vs exp(4t), 150ms delay
  - Cap steerI carry between segments to ±5.0 (≈±7.5 PWM max bias correction)

### 10. Run 136 — Abrupt braking causing catastrophic yaw errors (fixed)
- **Problem**: Linear deceleration over only 15cm (speedFactor 1.0→0.60) caused 220→132 PWM drops in ~150ms. Differential wheel friction during sudden braking induced massive yaw torque:
  - Seg 8: 24.7° yaw swing (gz=65°/s) during decel
  - Seg 10: 23° yaw jump, gz=216°/s
  - Seg 14: gz=-325°/s, 35° error, complete loss of control
- **Fix**: Logarithmic deceleration curve over 30cm + coast-stop:
  - `speedFactor = 0.45 + 0.55 * ln(1 + t*(e-1))` where t = remaining/0.30
  - At 30cm: 1.0 (220 PWM), 15cm: 0.83 (183), 7.5cm: 0.71 (156), 0cm: 0.45 (→110 floor)
  - Replaced `brakeToStop(140, 400)` (reverse PWM, per-wheel independent stop) with `stopAllMotors()` coast
  - No more asymmetric braking — both wheels coast equally, steering PID active until arrival
  - Settle reduced 300→200ms since arrival speed is much lower
  - Log curve onset is gentle at high speed, preventing abrupt torque imbalance

## Current Status (Run 137 pending)

- Run 136: Multiple catastrophic yaw errors caused by abrupt braking + asymmetric reverse-brake
- Logarithmic deceleration + coast-stop uploaded (no more reverse-brake at waypoints)
- Decel floor lowered to 0.45 so robot reaches motor-floor speed (~110 PWM) before arrival
- Remaining concerns: boot encoder test can still fail for right encoder (EMI-sensitive)

## Encoder Mode System

- Boot test (`bootEncoderAutoTest()` ~line 8617): spins each motor for 1s at PWM 130, checks which encoders register ≥50 pulses with <50% I2C failures
- `getAverageDistanceMeters()` (~line 3375): uses `enc_mode` to select distance source
- Modes: `ENC_MODE_BOTH` (avg L+R), `LEFT_ONLY`, `RIGHT_ONLY`, `TIMED` (time × 0.30 m/s)
- TIMED is overridden to BOTH at boot (~line 8742)
- **Known issue**: Right encoder I2C sometimes fails boot test → falls back to LEFT_ONLY

## BFS Drive Control Flow (key lines in src/main.cpp)

| Component | Approx Line |
|---|---|
| Encoder mode enum + TIMED speed | 60-73 |
| BFS state vars + PID state | 3330-3365 |
| BFS defines (KP, KI, KD, PWM, etc) | 3367-3374 |
| getAverageDistanceMeters() | 3375-3410 |
| motor_bias_pwm | 2805-2820 |
| imuIntegrateOnce() | 4720-4737 |
| imu_gz_bias_dps + persist | 2740-2770 |
| Boot encoder auto-test | 8617-8775 |
| Heading-hold during SETTLE | ~9789 |
| SETTLE → DRIVE transition | ~9840 |
| BFS DRIVE main loop | ~9855-10100 |
| Heading abort (>30° for 300ms) | ~9870-9890 |
| Average-distance stall (2s) | ~9895-9915 |
| Single-wheel stall (500ms) | ~9918-9960 |
| Arrive check + brakeToStop | ~9962-9980 |
| Steering PID (P+I+D) | ~9985-9978 |
| KP boost in decel zone (1.5×) | ~9970 |
| Post-turn steer ramp (150ms) | ~9983-9990 |
| Deceleration zone (last 0.15m) | ~9993-9998 |
| Time-based speed scaling | ~10000-10023 |
| drivePwm floor (130) | ~10070 |
| Adaptive launch ramp (encoder feedback) | ~10075-10107 |
| basePwm floor (110) | ~10042 |
| Saturation shift (floor preserve) | ~10050-10065 |
| PWM clamp + motor output | ~10067-10075 |
| 10Hz SPIFFS log append | ~10095-10110 |

## Turn Target Calculation (drift-immune)

```
next_heading = atan2(dx_next, dy_next)    // grid heading of next segment
prev_heading = atan2(dx_prev, dy_prev)    // grid heading of prev segment (or initial_heading for seg 1)
turn_delta = snap_to_90(next_heading - prev_heading)
target_yaw = current_imu_yaw - turn_delta  // pure relative turn
```
No accumulated grid heading tracking. Each turn is independent of prior drift.

## Log Column Reference

`t_ms, meters, forwardM, currentYaw, targetYaw, yawErrDeg, yawI, gz_dps, ..., basePWM, ..., corrOut, ..., lPwm, rPwm, ..., lateralM, remainingM, ..., steerI, ..., distL, distR`

- `meters` / `forwardM`: total distance (forwardM = cumulative across segments)
- `corrOut` = steerCorr (steering correction applied)
- `steerI` = bfs_drive_integral (steering I-term, now carried between segments)
- `lPwm/rPwm` = actual motor PWMs sent to drivers
- `distL/distR` = per-segment encoder distances (meters)
- `remainingM` = distance left in current segment

## Git Workflow

- Always git commit after every successful upload to the robot
- Commit message should describe the firmware changes made
