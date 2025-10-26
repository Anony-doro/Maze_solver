# MazeRouter - NoC Maze Routing Algorithm Implementation

This implementation brings the Network-on-Chip (NoC) maze routing algorithm from the paper "A Low-Overhead, Fully-Distributed, Guaranteed-Delivery Routing Algorithm for Faulty Network-on-Chips" to robot maze navigation.

## Overview

The MazeRouter implements Algorithm 1 from the NoC paper, providing:
- **Guaranteed delivery** if a path exists
- **Fully distributed** routing (no central controller)
- **Low overhead** (no routing tables)
- **On-the-fly reconfiguration** when obstacles appear

## Key Concepts

### Routing Modes
- **NORMAL**: Move directly toward goal using productive directions
- **TRAVERSAL_LEFT/RIGHT**: Wall-following around obstacles using left/right-hand rule

### Productive Directions
A direction is "productive" if it reduces the Manhattan distance to the goal:
```
MD = |x_current - x_goal| + |y_current - y_goal|
```

### Wall Following
When blocked, the router switches to traversal mode and follows the obstacle boundary until it can resume normal progress.

## Files

- `MazeRouter.hpp` - Header file with class definition
- `MazeRouter.cpp` - Implementation of the routing algorithm
- `test_maze_router.cpp` - Standalone test program
- `main.cpp` - Integration with bicycle simulation

## Usage

### Basic Usage

```cpp
#include "MazeRouter.hpp"

// Create maze (0=free, 1=wall, 2=start, 3=goal)
std::vector<std::vector<int>> maze = {
    {1, 1, 1, 1, 1},
    {1, 0, 0, 0, 1},
    {1, 0, 1, 0, 1},
    {1, 0, 0, 0, 1},
    {1, 1, 1, 1, 1}
};

// Initialize router
MazeRouter router(maze);
MazeRouter::Cell start(1, 1);
MazeRouter::Cell goal(3, 3);
router.initializeRouting(start, goal);

// Route step by step
MazeRouter::Cell current = start;
MazeRouter::Direction lastDir = MazeRouter::Direction::NORTH;

while (current.x != goal.x || current.y != goal.y) {
    auto decision = router.routeStep(current, goal, lastDir);
    
    if (decision.unreachable) {
        std::cout << "Goal unreachable!" << std::endl;
        break;
    }
    
    if (decision.dir != MazeRouter::Direction::LOCAL) {
        current = router.step(current, decision.dir);
        lastDir = decision.dir;
    }
}
```

### Integration with Bicycle Model

The router provides discrete grid-based decisions that can be converted to continuous robot control:

```cpp
// Get routing decision
auto decision = router.routeStep(currentPos, goal, lastDirection);

// Convert to steering command
double steerCmd = 0.0;
switch (decision.dir) {
    case MazeRouter::Direction::NORTH: steerCmd = 0.0; break;
    case MazeRouter::Direction::EAST: steerCmd = 0.1; break;
    case MazeRouter::Direction::SOUTH: steerCmd = 0.0; break;
    case MazeRouter::Direction::WEST: steerCmd = -0.1; break;
    default: break;
}

car.setSteerCmd(steerCmd);
```

## Building

```bash
mkdir build
cd build
cmake ..
make
```

This creates two executables:
- `MazeGenerator` - Full simulation with bicycle model and visualization
- `TestMazeRouter` - Standalone router test

## Algorithm Details

### State Machine
1. **NORMAL**: Try to move productively toward goal
2. **TRAVERSAL**: Wall-follow around obstacles
3. **Exit condition**: Return to NORMAL when productive direction available

### Unreachable Detection
The algorithm detects unreachable goals by checking if it returns to the same starting cell and direction during traversal.

### Deadlock/Livelock Freedom
- Uses deflection routing (no waiting)
- Guaranteed progress through Manhattan distance tracking
- Face traversal ensures eventual goal reach or unreachable detection

## Performance

- **Time complexity**: O(V + E) for reachability check
- **Space complexity**: O(1) per router (no routing tables)
- **Reconfiguration time**: O(1) (on-the-fly)

## Integration Notes

### Grid to World Coordinates
The router works in discrete grid coordinates (i,j). Convert to world coordinates (x,y) for robot control:
```cpp
double worldX = gridY + 0.5;  // Convert (i,j) to (x,y)
double worldY = gridX + 0.5;
```

### Dynamic Obstacles
Update the maze grid when new obstacles are detected. The router will automatically switch to traversal mode to navigate around them.

### Bicycle Model Constraints
The discrete grid path may not be feasible for a bicycle model due to turning radius constraints. Consider:
- Using Pure Pursuit or Stanley controllers
- Post-processing with Dubins/Reeds-Shepp curves
- Increasing effective corridor width

## Example Output

```
=== Maze Router Test ===
Start: (1, 1)
Goal: (3, 3)
Goal reachable: Yes

=== Routing Simulation ===
Step  Pos      Mode         Dir      MDbest
--------------------------------------------------
   0  (1,1)    NORMAL       EAST         3
   1  (1,2)    NORMAL       EAST         2
   2  (1,3)    NORMAL       SOUTH        1
   3  (2,3)    NORMAL       EAST         0
   4  (3,3)    NORMAL       LOCAL        0

Goal reached in 4 steps!
```

## References

- "A Low-Overhead, Fully-Distributed, Guaranteed-Delivery Routing Algorithm for Faulty Network-on-Chips" (NOCS 2015)
- Face routing in wireless ad-hoc networks
- Graph theory and planar graphs



