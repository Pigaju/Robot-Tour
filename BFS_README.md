# Robot Tour BFS Navigation System

This firmware adapts the Science Olympiad Electric Vehicle platform for the **Robot Tour** event using **Breadth-First Search (BFS)** pathfinding.

## Overview

The robot uses a graph-based navigation system where:
- **Nodes** represent rooms or waypoints
- **Edges** represent connections between rooms with known distances
- **BFS** finds the shortest path (fewest room transitions) to reach the goal

## Hardware Features Preserved

✅ **Motor Control & Testing**
- Independent left/right motor control
- Motor bias compensation (feedforward trim)
- Encoder calibration (pulses per meter)
- Hardware test screen for motor diagnostics

✅ **IMU & Gyro Calibration**
- Boot-time gyro bias estimation
- Real-time heading hold
- Persistent bias storage across reboots

✅ **Encoder Support**
- Dual wheel encoders (I2C)
- Per-wheel distance calibration
- Position tracking

✅ **Data Logging**
- Full CSV telemetry during runs
- SPIFFS storage with automatic pruning
- Serial dump/tail commands for analysis

## Customizing the BFS Graph

Edit `bfsInitializeGraph()` in `main.cpp` to configure your venue:

```cpp
static void bfsInitializeGraph() {
  bfs_state.node_count = 4;  // Number of rooms/waypoints
  
  // Node 0: Start room
  bfs_state.nodes[0].id = 0;
  strcpy(bfs_state.nodes[0].name, "Start");
  bfs_state.nodes[0].x_m = 0.0f;    // X position (meters)
  bfs_state.nodes[0].y_m = 0.0f;    // Y position (meters)
  
  // Add more nodes as needed...
  
  // Add edges (connections) between nodes
  bfsAddEdge(0, 1, 2.0f);  // From node 0 to node 1, 2.0m distance
  bfsAddEdge(1, 0, 2.0f);  // Reverse connection (bidirectional)
}
```

### Example 4-Room Layout

```
    [Side]
      |
[Start]--[Middle]--[End]
```

Configuration:
```cpp
bfs_state.node_count = 4;

// Node 0: Start
strcpy(bfs_state.nodes[0].name, "Start");
bfs_state.nodes[0].x_m = 0.0f;
bfs_state.nodes[0].y_m = 0.0f;

// Node 1: Middle
strcpy(bfs_state.nodes[1].name, "Middle");
bfs_state.nodes[1].x_m = 2.0f;
bfs_state.nodes[1].y_m = 0.0f;

// Node 2: End
strcpy(bfs_state.nodes[2].name, "End");
bfs_state.nodes[2].x_m = 4.0f;
bfs_state.nodes[2].y_m = 0.0f;

// Node 3: Side
strcpy(bfs_state.nodes[3].name, "Side");
bfs_state.nodes[3].x_m = 2.0f;
bfs_state.nodes[3].y_m = 2.0f;

// Edges
bfsAddEdge(0, 1, 2.0f);    // Start -> Middle
bfsAddEdge(1, 0, 2.0f);    // Middle -> Start
bfsAddEdge(1, 2, 2.0f);    // Middle -> End
bfsAddEdge(2, 1, 2.0f);    // End -> Middle
bfsAddEdge(1, 3, 2.83f);   // Middle -> Side
bfsAddEdge(3, 1, 2.83f);   // Side -> Middle
```

## Using the BFS Navigation

1. **From Main Menu**: Select "BFS NAV"
2. **Configure Path**:
   - Dial selects start waypoint
   - Click button to switch to goal selection
   - Dial selects goal waypoint
3. **Start Navigation**: Press & hold button
   - BFS computes shortest path
   - Enters RUN screen automatically
   - Robot follows path, executing navigation to each waypoint

## Data Fields in RunLogSample

Each logged sample includes navigation-specific fields:

```cpp
float lateralM;              // Lateral distance from centerline
float remainingM;            // Distance to next waypoint
uint8_t canPhase;            // Current navigation phase (optional)
```

## Serial Commands

Existing commands still work:
```
status          - Show firmware version, run metrics
dump            - Dump last saved CSV log
tail [N]        - Show last N lines of log
help            - List all commands
```

## Control Modes

The system supports three steering control modes (selectable from menu):

1. **IMU Mode** - Uses gyroscope for heading control (susceptible to drift)
2. **Encoder Mode** (default) - Uses wheel encoders for straight-line (more stable)
3. **Hybrid Mode** - Fuses IMU + encoder data

## Important Configuration Constants

```cpp
#define BFS_MAX_NODES 16          // Max waypoints
#define BFS_MAX_EDGES_PER_NODE 8  // Max connections per node
#define BFS_PATH_MAX_LENGTH 16    // Max steps in a path
#define BFS_MAX_NODES 16          // Must match actual node_count
```

## Troubleshooting

**Robot doesn't move at startup:**
- Ensure `bfs_initialized` is true
- Check that `bfs_start_node` and `bfs_goal_node` are valid
- Verify encoders are responding (check HW TEST screen)

**Wrong path selected:**
- Verify `bfsInitializeGraph()` edges match your venue layout
- Check node IDs are sequential (0, 1, 2, ...)

**Motor bias compensation:**
- Use "BIAS TEST" screen to tune `motor_bias_pwm` for straight-line motion
- Save setting, then test "CAR RUN" mode

**Gyro drift issues:**
- Keep robot still during boot-time calibration countdown
- Gyro bias is persisted in NVS across reboots

## Future Enhancements

- Dynamic node discovery via onboard sensors
- Obstacle avoidance integration
- Multi-hop path visualization on display
- Path completion feedback (waypoint reached indication)
