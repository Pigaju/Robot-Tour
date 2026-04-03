# Science Olympiad Robot Tour - Manual Waypoint Navigation

This firmware drives a pre-programmed sequence of waypoints.
Competitors determine the route manually; the robot executes the programmed path.

## Features

- Manual waypoint navigation (no automatic route computation)
- IMU gyro heading hold
- Dual wheel encoders for distance tracking
- Motor bias compensation
- CSV telemetry logging to SPIFFS
- Serial command interface for log retrieval
- Persistent NVS settings

## Control Modes

1. IMU Mode - Gyroscope heading hold
2. Encoder Mode (default) - Wheel encoder straight-line
3. Hybrid Mode - Fused IMU + encoder

## Compilation

Requires PlatformIO with ESP32 Arduino, M5Unified, M5Dial.

    platformio run

## Troubleshooting

- Verify encoders in HW TEST screen
- Check motor I2C addresses (0x20 right, 0x21 left)
- Keep robot still during boot-time gyro calibration
- Use BIAS TEST for straight-line motor trim
