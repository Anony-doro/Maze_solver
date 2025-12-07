# Bicycle Navigation In a Maze

## Overview

This project implements an autonomous bicycle navigation system for a 10×10 maze with dynamic wall-following and dead-end recovery. It basically follows right handrule. The robot uses a 360-degree sensor system to detect walls and obstacles, implements state-based navigation modes, and records its journey as an MP4 video.

## Key Features

- **360-Degree Wall Detection**: Ray-casting based detection in main 4 directions
- **Multi-State Navigation**: NORMAL, ALIGNING, REVERSING, and NORMAL_LEFT modes
- **Dead-End Recovery**: Intelligent reversal and escape from dead ends
- **Cardinal Direction Snapping**: Angles are snapped to the nearest cardinal (North/South/East/West)
- **MP4 Recording**: Automatically captures and saves simulation as MP4 video
- **Real-Time Visualization**: OpenCV-based visualization with sensor displays

## Architecture

### Core Components

1. **SimpleMaze10x10** - 10×10 maze generator with wall creation in the maze
2. **MazeRouter100x100** - Pathfinding router for optimal navigation. Have not been used much in this project. this was adopted from another project of mine. (MazeRouter)
3. **Bicycle** - Physics-based bicycle model with steering and velocity control
4. **Navigation State Machine** - Multi-mode navigation controller

### Navigation Modes

#### NORMAL Mode
- Follows productive directions toward goal - only for the initial set up of the robot, robot was made to face the productive direction toward the goal.
- Detects walls and obstacles. However obstacles are not fully avoidable due to time limit for this project
- Switches to ALIGNING when between walls
- Turns 90° when encountering walls

#### ALIGNING Mode
- Centers robot between two walls
- Maintains heading alignment
- Switches to NORMAL when properly aligned
- Uses control theory with two inputs - 1. position based error and 2. heading direction error
- uses PD control

#### REVERSING Mode
- Detects dead ends
- Reverses backward when not enough space to take a turn 90 degrees.
- Uses distance checks to determine escape conditions
- Transitions to NORMAL_LEFT for recovery from a dead end corner. This is because robot is designed to do a backward movement when it faces dead end, so to follow the right hand rule, it has to take a left turn once when left turn is available.

#### NORMAL_LEFT Mode
- Turns left when leaving dead ends (Only once, and return to normal mode)
- Returns to NORMAL when path is clear and Left turn is done

## Coordinate Systems

The project uses **OpenCV coordinates** throughout:
- Two coordinate systems:
one is the maze vector coordinate where x is the row and y is the column
second is the world coordinate based on opencv coordinate, x is the rightward and y is downward.
- **X-axis**: Horizontal (left-right)
- **Y-axis**: Vertical (top-down)
- **Phi (heading)**: 0° = East, 90° = South, 180° = West, -90° = North   : positive angle change means clockwise rotation

### Angle Calculations
- All angles use `atan2()` to match OpenCV's coordinate system
- Phi is normalized to `[-π, π]` range
- Cardinal snapping uses `±45°` bands

## Wall Detection

### Sensor Configuration
- **Wall Threshold**: 8.0 units
- **Ray Step Size**: 0.05 units (for precision)
- **Maximum Range**: 10.0 units (maze cell size)
- **Minimum Distance**: 1.0 units (prevents false positives)

### Detection Logic
```cpp
auto getWallDistance = [&](double angle) -> double {
    double step = 0.05;
    double dist = 1.0;  // Start 1 unit away
    while (dist < 10.0) {
        dist += step;
        if (maze.hasWallBetween(currentPos, checkPos)) {
            return dist;
        }
    }
    return 10.0;  // No wall found
};
```

## Control System

### Steering Controller
- **Proportional (Kp)**: 10.0
- **Derivative (Kd)**: 0.1
- **Control Output**: `kp * error + kd * errorDerivative`
used trial error method

### PID Control in ALIGNING
- **Position Kp**: 0.1
- **Heading Kp**: 0.1
- **Derivative Kd**: 0.0
- **Combined Output**: Maintains both position and heading alignment


## State Transitions

### NORMAL → ALIGNING
Trigger: `wallDistLeft < 9.0 && wallDistRight < 9.0 && !wallAhead && !wallBehind`
- Robot enters narrow corridor
- Maintains current heading
- Centers between walls

### NORMAL → REVERSING
Trigger: `wallAhead && wallLeft && wallRight`
- Dead end detected
- Reverses to escape

### ALIGNING → NORMAL
Trigger: `positionError < 0.05 && headingError < 0.05`
- Successfully centered and aligned
- Continues navigation

### REVERSING → NORMAL_LEFT
Trigger: `deadEnd && certain conditions`
- Exited dead end
- Turns left to follow wall
- Resumes navigation

## MP4 Recording

The simulation automatically records its execution:

### Configuration
- **Frame Capture**: Every 100th frame
- **Output FPS**: 5 frames/second
- **Codec**: H.264
- **Output**: `bicycle_navigation.mp4`

### Usage
To compile:
OpenCV required
Either MSVC or G++ MINGW required
CMAKE required
mkdir build
cd build
cmake ..
cmake --build . --config Release

To run:
Go to Release folder and 
./BicycleNavigation.exe
In the same folder, the mp4 file is saved.
To reduce the speed of simulation, edit the line 835 and 663

```

## Building

### Prerequisites
- CMake 3.10+
- OpenCV (with video support)
- C++17 compiler


### Controls
- **ESC**: Exit simulation
- **SPACE**: Pause/Resume
- **Window**: Close to quit

### Output
- Console: Distance to goal updates
- Terminal: Debug information (if enabled)
- MP4: Recorded simulation video

## Algorithm Details

### Cardinal Direction Snapping
```cpp
double snapToCardinal(double angle) {
    // Snap to nearest: East (0°), South (90°), West (180°), North (-90°)
    // Uses ±45° bands
    if (angle >= -45° && angle < 45°) return 0°;     // East
    if (angle >= 45° && angle < 135°) return 90°;    // South
    if (angle >= 135° || angle <= -135°) return 180°; // West
    return -90°;  // North
}
```

### Distance-Based Wall Detection
- Rays cast from robot position
- Step incrementally along ray
- Stop at first wall detection
- Returns distance to wall

### Productive Direction Integration
- Uses MazeRouter100x100 for pathfinding
- Gets productive directions at each cell
- Prefers directions that reduce Manhattan distance to goal
- Falls back to Right Hand Rule if no productive directions

## Performance Optimizations

### Fast Mode Settings
- **Time Step**: `dt = 0.05` (50ms)
- **Visualization**: Every 100th frame
- **WaitKey**: 1ms delay
- **Frame Recording**: Every 100th step  ()

### Normal Mode Settings
- **Time Step**: `dt = 0.05` (standard precision)
- **Visualization**: Every 2nd frame
- **WaitKey**: 10ms delay
- **Frame Recording**: Every 10th step

## Troubleshooting

### Navigation Issues
- **Stuck in ALIGNING**: Reduce `kp` and `kp_heading` gains
- **Oscillating behavior**: Add derivative term (Kd)
- **Missing goal**: Increase `maxTime` or adjust `goalThreshold`

### MP4 Generation Issues
- **Codec not found**: Install OpenCV with H.264 support

### Build Issues
- **OpenCV not found**: Install OpenCV and set `OPENCV_DIR`
- **Missing files**: Check CMakeLists.txt includes all source files
- **Compilation errors**: Ensure C++17 standard is enabled

## File Structure

```
.
├── main_bicycle_navigation.cpp    # Main simulation file
├── Bicycle.cpp/hpp                 # Bicycle physics model
├── SimpleMaze10x10.cpp/hpp         # Maze generation
├── MazeRouter100x100.cpp/hpp       # Pathfinding router
├── CMakeLists.txt                  # Build configuration
└── README_BicycleNavigation.md     # This file
```

## Future Enhancements

- [ ] Path smoothing for better trajectories
- [ ] Dynamic obstacle avoidance
- [ ] Multi-robot coordination
- [ ] Real-time sensor fusion
- [ ] Machine learning-based navigation
