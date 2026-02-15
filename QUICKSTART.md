# Quick Start: Robot Tour BFS Navigation

## What Changed?

Your Electric Vehicle firmware has been adapted for **Robot Tour** with a **graph-based BFS navigation** system while keeping all the original hardware features (motors, encoders, IMU, data logging, hardware tests).

## Quick Setup (5 minutes)

### 1. Define Your Venue Layout

Edit `bfsInitializeGraph()` in `src/main.cpp` (around line 220):

```cpp
static void bfsInitializeGraph() {
  bfs_state.node_count = 3;  // Number of rooms/waypoints
  
  // Room 0: Start
  bfs_state.nodes[0].id = 0;
  strcpy(bfs_state.nodes[0].name, "Start");
  bfs_state.nodes[0].x_m = 0.0f;
  bfs_state.nodes[0].y_m = 0.0f;
  
  // Room 1: Office
  bfs_state.nodes[1].id = 1;
  strcpy(bfs_state.nodes[1].name, "Office");
  bfs_state.nodes[1].x_m = 3.0f;
  bfs_state.nodes[1].y_m = 0.0f;
  
  // Room 2: Lab
  bfs_state.nodes[2].id = 2;
  strcpy(bfs_state.nodes[2].name, "Lab");
  bfs_state.nodes[2].x_m = 6.0f;
  bfs_state.nodes[2].y_m = 0.0f;
  
  // Connect the rooms (bidirectional edges)
  bfsAddEdge(0, 1, 3.0f);  // Start to Office: 3 meters
  bfsAddEdge(1, 0, 3.0f);  // Office back to Start
  bfsAddEdge(1, 2, 3.0f);  // Office to Lab: 3 meters
  bfsAddEdge(2, 1, 3.0f);  // Lab back to Office
}
```

### 2. Compile & Upload

```bash
cd "c:\Users\piika\OneDrive\Documents\PlatformIO\Projects\Robot tour"
python -m platformio run
python -m platformio run --target upload
```

### 3. Test on Robot

1. Power on the robot
2. Wait for IMU calibration (keep still for 3 seconds)
3. From main menu: Select **"BFS NAV"**
4. Turn dial to select starting waypoint
5. Click button to switch to goal selection
6. Turn dial to select goal waypoint
7. Press & hold button to start
8. Robot computes shortest path and navigates!

## Key Menu Options

| Option | What It Does |
|--------|-------------|
| **BFS NAV** | Set start/goal rooms and auto-navigate (NEW) |
| CAR RUN | Manual speed/distance run (original) |
| DISTANCE | Adjust target distance (original) |
| TIME | Adjust target time (original) |
| BIAS TEST | Tune motor straight-line compensation (original) |
| ENC CAL | Calibrate encoder distance accuracy (original) |
| CTRL MODE | Pick IMU/Encoder/Hybrid steering (original) |
| HW TEST | Test motors, encoders, IMU (original) |

## Typical Venue Layouts

### Linear (3 rooms)
```
[Start] → [Middle] → [End]
```
```cpp
bfs_state.node_count = 3;
bfsAddEdge(0, 1, 2.0f);
bfsAddEdge(1, 0, 2.0f);
bfsAddEdge(1, 2, 2.0f);
bfsAddEdge(2, 1, 2.0f);
```

### Grid (4 rooms)
```
[Start]  [Middle]
   ↓        ↓
[Left]   [Right]
```
```cpp
bfs_state.node_count = 4;
// Vertical connections
bfsAddEdge(0, 2, 2.0f); bfsAddEdge(2, 0, 2.0f);
bfsAddEdge(1, 3, 2.0f); bfsAddEdge(3, 1, 2.0f);
// Horizontal connections
bfsAddEdge(0, 1, 2.0f); bfsAddEdge(1, 0, 2.0f);
bfsAddEdge(2, 3, 2.0f); bfsAddEdge(3, 2, 2.0f);
```

### Star (multiple rooms around central hub)
```
        [Top]
         ↑
[Left] ← [Hub] → [Right]
         ↓
       [Bottom]
```
```cpp
bfs_state.node_count = 5;
// All rooms connect to hub (node 0)
for (int i = 1; i < 5; i++) {
  bfsAddEdge(0, i, 2.0f);
  bfsAddEdge(i, 0, 2.0f);
}
```

## Serial Commands (For Debugging)

Connect via USB and use any serial monitor:

```
status          → Show firmware, run stats, BFS state
dump            → Print last run's telemetry CSV
tail 50         → Show last 50 lines of log
listlogs        → List all saved run files
help            → Show all commands
```

## Hardware Calibration (Before Competition)

### 1. Encoder Accuracy
- From menu: **DISTANCE**
- Measure your floor (measure 1 meter accurately)
- Test: Target 1.0m, check actual distance
- Adjust "pulses/meter" until accurate
- Save: Click to save

### 2. Motor Straight-Line
- From menu: **BIAS TEST**
- Click to start motors running
- Turn dial to adjust `motor_bias_pwm` (±40 max)
- Goal: Robot drives straight without steering correction
- If left drifts: increase bias (positive = boost left motor)
- If right drifts: decrease bias (negative = boost right motor)

### 3. IMU Calibration
- Keep robot still during 3-second countdown at startup
- Display shows "READY TO RACE" when done
- Gyro bias auto-saves for next startup

## FAQ

**Q: Robot picks wrong path?**
A: Check your `bfsAddEdge()` calls match the physical layout. BFS finds mathematically shortest (fewest transitions), not necessarily shortest distance.

**Q: How many rooms max?**
A: Up to 16 waypoints (`BFS_MAX_NODES = 16`), 8 connections per room.

**Q: Can I add rooms without recompiling?**
A: Not yet—rooms are defined at compile-time. You can modify `bfsInitializeGraph()` and recompile quickly (~45 seconds).

**Q: What if robot gets stuck?**
A: Press button A to kill motors immediately. Check for obstacles, encoder issues, or IMU drift.

**Q: Where are my run logs saved?**
A: `/runlog_<id>.csv` on the robot's internal flash (SPIFFS). Dump via `dump` serial command.

## Preserved Features ✅

Everything from the original E.V. firmware still works:

- ✅ **Motor Control** - Dual independent motors with direction control
- ✅ **Encoders** - Wheel position tracking with calibration
- ✅ **IMU** - 6-DOF gyro/accel with boot-time bias estimation
- ✅ **Data Logging** - Full telemetry CSV, auto-saves to SPIFFS
- ✅ **Hardware Tests** - Motor spin test, encoder check, IMU readback
- ✅ **Settings Persistence** - Auto-saves calibration to NVS
- ✅ **Serial Interface** - Dump logs, change settings, check status

## Next Steps

1. **Configure your venue** in `bfsInitializeGraph()` 
2. **Compile**: `python -m platformio run`
3. **Upload** to robot
4. **Calibrate** encoders and motor bias
5. **Test** BFS navigation menu
6. **Run** some practice paths
7. **Review logs** with `dump` command
8. **Compete!**

## Still Have Questions?

- See **IMPLEMENTATION_SUMMARY.md** for architecture overview
- See **BFS_README.md** for detailed BFS configuration
- Check **platformio.ini** for build configuration

Good luck at the competition! 🏆
