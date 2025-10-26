# Bicycle Navigation with Wall Following and Dead-End Recovery

## Overview

This project implements an autonomous bicycle navigation system for a 10×10 maze with dynamic wall-following, dead-end recovery, and obstacle avoidance capabilities. The robot uses a 360-degree sensor system to detect walls and obstacles, implements state-based navigation modes, and records its journey as an MP4 video.

## Key Features

- **360-Degree Wall Detection**: Ray-casting based detection in all directions
- **Multi-State Navigation**: NORMAL, ALIGNING, REVERSING, and NORMAL_LEFT modes
- **Dead-End Recovery**: Intelligent reversal and escape from dead ends
- **Cardinal Direction Snapping**: Angles snapped to nearest cardinal (North/South/East/West)
- **MP4 Recording**: Automatically captures and saves simulation as MP4 video
- **Real-Time Visualization**: OpenCV-based visualization with sensor displays

## Architecture

### Core Components

1. **SimpleMaze10x10** - 10×10 maze generator with wall management
2. **MazeRouter100x100** - Pathfinding router for optimal navigation
3. **Bicycle** - Physics-based bicycle model with steering and velocity control
4. **Navigation State Machine** - Multi-mode navigation controller

### Navigation Modes

#### NORMAL Mode
- Follows productive directions toward goal
- Detects walls and obstacles
- Switches to ALIGNING when between walls
- Turns 90° when encountering walls

#### ALIGNING Mode
- Centers robot between two walls
- Maintains heading alignment
- Switches to NORMAL when properly aligned

#### REVERSING Mode
- Detects dead ends
- Reverses backward when walls in front and both sides
- Uses distance checks to determine escape conditions
- Transitions to NORMAL_LEFT for recovery

#### NORMAL_LEFT Mode
- Turns left when leaving dead ends
- Follows wall on right side
- Returns to NORMAL when path is clear

## Coordinate Systems

The project uses **OpenCV coordinates** throughout:
- **X-axis**: Horizontal (left-right)
- **Y-axis**: Vertical (top-down)
- **Phi (heading)**: 0° = East, 90° = South, 180° = West, -90° = North

### Angle Calculations
- All angles use `atan2(-dy, dx)` to match OpenCV's coordinate system
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
```bash
# After simulation completes
./build/Debug/BicycleNavigation.exe

# Generated files
bicycle_navigation.mp4  # Video recording
```

### Conversion to GIF (Optional)
```bash
ffmpeg -i bicycle_navigation.mp4 bicycle_navigation.gif
```

## Building

### Prerequisites
- CMake 3.10+
- OpenCV (with video support)
- C++17 compiler

### Build Commands
```bash
# Configure
cmake -B build

# Build
cmake --build build --config Debug --target BicycleNavigation

# Run
./build/Debug/BicycleNavigation.exe
```

## Usage

### Running the Simulation
```bash
./build/Debug/BicycleNavigation.exe
```

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
- **Frame Recording**: Every 100th step

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
- **Empty frames**: Check frame buffer size
- **Corrupted video**: Verify frame dimensions match

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

## License

Educational project for autonomous navigation research.

## Contact

For questions or contributions, please refer to the project repository.
