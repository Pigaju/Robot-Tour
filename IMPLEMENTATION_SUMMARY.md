# Science Olympiad Robot Tour - BFS Navigation Implementation

## Summary of Changes

This firmware has been successfully adapted from the **Electric Vehicle (E.V.)** competition to the **Robot Tour** event while preserving all hardware testing features.

### Key Features Added

#### 1. **BFS Graph Navigation System**
- **Breadth-First Search** pathfinding algorithm for finding shortest paths
- Graph-based representation with nodes (waypoints/rooms) and edges (connections)
- Configurable up to 16 nodes with 8 connections per node
- Automatic path computation from start to goal waypoint

#### 2. **Integrated Menu System**
- New "BFS NAV" menu option in the main menu
- Interactive node selection using the dial
- Real-time path visualization on display

#### 3. **Hardware Testing Preserved**
All original features remain fully functional:
- ✅ Motor control (left/right independent)
- ✅ Motor bias compensation (feedforward trim)
- ✅ Encoder calibration (pulses/meter)
- ✅ IMU gyro calibration & heading control
- ✅ Data logging to SPIFFS
- ✅ Hardware diagnostic screens
- ✅ Serial command interface

### Code Structure

```
main.cpp
├── BFS Graph Definition
│   ├── BfsNode structure (nodes, positions, edges)
│   ├── BfsState structure (path state machine)
│   ├── bfsInitializeGraph() - Configure venue layout
│   ├── bfsAddEdge() - Add room connections
│   ├── bfsComputePath() - BFS search algorithm
│   └── bfsGetNextTarget()/bfsAdvancePath() - Path execution
│
├── Menu Integration  
│   ├── SCREEN_BFS_NAV enum value
│   ├── BFS menu item in main menu
│   ├── drawBfsNavScreen() - Node selection UI
│   ├── BFS input handling in loop()
│   └── BFS settings persistence
│
├── Original Features (Preserved)
│   ├── Motor control & testing
│   ├── Encoder calibration
│   ├── IMU gyro bias estimation
│   ├── Data logging & CSV export
│   └── Serial command interface
│
└── Documentation
    └── BFS_README.md (detailed configuration guide)
```

### Configuration Constants

```cpp
#define BFS_MAX_NODES 16          // Maximum waypoints
#define BFS_MAX_EDGES_PER_NODE 8  // Max connections per waypoint
#define BFS_PATH_MAX_LENGTH 16    // Max steps in a path
```

### How to Customize

Edit `bfsInitializeGraph()` in `main.cpp` to configure your competition venue:

**Example: 4-room layout**
```cpp
void bfsInitializeGraph() {
  bfs_state.node_count = 4;
  
  // Define each room/waypoint
  strcpy(bfs_state.nodes[0].name, "Start");
  bfs_state.nodes[0].x_m = 0.0f;
  bfs_state.nodes[0].y_m = 0.0f;
  
  // Add more nodes...
  
  // Connect rooms
  bfsAddEdge(0, 1, 2.0f);  // Start to Node 1, 2 meters
}
```

### Menu Usage

1. **From Main Menu**: Select **"BFS NAV"**
2. **Configure Path**:
   - Rotate dial to select **Start Node** (highlighted in yellow)
   - Click button to switch to goal selection
   - Rotate dial to select **Goal Node** (highlighted in yellow)
3. **Start Navigation**: Press & hold the button
   - BFS computes the shortest path
   - Automatically enters RUN screen
   - Robot executes sequential waypoint-to-waypoint navigation

### Data Logging

All original telemetry is captured and includes new navigation fields:
- `lateralM` - Distance from centerline
- `remainingM` - Distance to next waypoint
- `canPhase` - Navigation phase indicator

Logs are saved to SPIFFS and can be retrieved via Serial commands:
```
dump            - Dump last run CSV
tail [N]        - Show last N lines
listlogs        - List saved logs
dumpid <id>     - Dump specific run
```

### Control Modes

The system supports three steering approaches:
1. **IMU Mode** - Gyroscope-based heading hold
2. **Encoder Mode** (default) - Wheel encoder-based straight-line
3. **Hybrid Mode** - Fused IMU + encoder data

Select via "CTRL MODE" menu option.

### Persistence

All settings are saved to ESP32's NVS:
- BFS start/goal node selection
- Motor calibration values
- Encoder pulses-per-meter
- Gyro bias estimates
- Control mode preference

Settings auto-save after changes with 1-second idle timeout.

### Testing Checklist

Before competition:
- [ ] Configure graph in `bfsInitializeGraph()`
- [ ] Test encoder calibration (DISTANCE menu, verify 1m accuracy)
- [ ] Calibrate motor bias (BIAS TEST screen for straight-line)
- [ ] Verify IMU boot-time calibration (keep robot still during countdown)
- [ ] Test BFS navigation menu (select start/goal nodes)
- [ ] Run practice path (verify shortest path is correct)
- [ ] Check data logging (dump sample run, verify timestamps)
- [ ] Test hardware motors (HW TEST screen)

### File Structure

```
Robot tour/
├── platformio.ini           (project config with M5Unified)
├── src/
│   ├── main.cpp            (complete firmware with BFS)
│   └── test.cpp            (hardware testing utilities)
├── include/
│   └── README
├── lib/
│   └── README
├── BFS_README.md           (detailed BFS configuration guide)
└── .pio/libdeps/           (M5Unified, M5Dial, etc.)
```

### Compilation

Requires:
- PlatformIO Core
- ESP32 Arduino Framework
- M5Unified library (M5Stack)
- M5Dial library (M5Stack)

Compile with:
```bash
platformio run
```

### Troubleshooting

**Robot doesn't move after BFS start:**
- Verify encoders detected in HW TEST screen
- Check motor I2C addresses (0x20 right, 0x21 left)
- Confirm `bfs_initialized` is true after setup

**Wrong path selected:**
- Review `bfsInitializeGraph()` edge definitions
- Verify node IDs are sequential (0, 1, 2, ...)
- Ensure all rooms are reachable from start node

**Gyro drift during navigation:**
- Keep robot still during boot-time calibration
- Use "BIAS TEST" to find optimal `motor_bias_pwm`
- Gyro bias auto-persists in NVS

### Future Enhancements

- Dynamic graph updates from sensor input
- Multi-path comparison & optimization
- Obstacle detection & avoidance routing
- Telemetry visualization dashboard
- Real-time path replay/debugging

---

**Firmware Version**: `v1_robot_tour_bfs_2026-02-14`

**Status**: ✅ Compiles successfully, ready for testing

**Contact**: For configuration questions, refer to BFS_README.md
