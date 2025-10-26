#include "SimpleMaze10x10.hpp"
#include "MazeRouter100x100.hpp"
#include "BicycleTemp.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

//to snap angle to nearest cardinal direction using ±45° bands
double snapToCardinal(double angle) {
    // Normalize angle
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    
    const double pi4 = M_PI / 4.0;  // 45 degrees
    
    if (angle >= -pi4 && angle < pi4) {
        return 0.0;  // East
    }
    else if (angle >= pi4 && angle < 3*pi4) {
        return M_PI / 2.0;  // South
    }
    else if (angle >= 3*pi4 || angle < -3*pi4) {
        if (angle < 0) {
            return -M_PI;  
        } else {
            return M_PI;   
        }
    }
    else {
        return -M_PI / 2.0;  // North
    }
}

#define SNAP_TARGET(targetDesiredPhi) targetDesiredPhi = snapToCardinal(targetDesiredPhi)

int main()
{
    // 1. Generate Maze
    SimpleMaze10x10 maze;
    maze.generateMaze();
    maze.setRandomStartGoal();
    maze.generateObstacles(0.01);  // 1% obstacle density (reduced from 15%)

    
    // Get start and goal from maze (in major cell coordinates)
    // maze.getStartPosition() returns (row, col) but we want (x, y) convention
    auto startPos = maze.getStartPosition();
    auto goalPos = maze.getGoalPosition();
    int startMajorX = startPos.second;  // col = horizontal = x
    int startMajorY = startPos.first;   // row = vertical = y
    int goalMajorX = goalPos.second;    // col = horizontal = x
    int goalMajorY = goalPos.first;     // row = vertical = y
    
    if (startMajorX == -1 || goalMajorX == -1) {
        std::cout << "Could not find start or goal!" << std::endl;
        return -1;
    }
    
    //converting into world coordinates (each major cell = 10 units of subcells)
    double startX = startMajorX * 10.0 + 5.0;  // x = horizontal
    double startY = startMajorY * 10.0 + 5.0;  // y = vertical
    double goalX = goalMajorX * 10.0 + 5.0;    // x = horizontal
    double goalY = goalMajorY * 10.0 + 5.0;    // y = vertical
    
    // 2. Initialize MazeRouter
    // Router uses (x=horizontal, y=vertical) to match OpenCV convention
    MazeRouter100x100::Cell startCell(startMajorX * 10 + 5, startMajorY * 10 + 5);
    MazeRouter100x100::Cell goalCell(goalMajorX * 10 + 5, goalMajorY * 10 + 5);
    
    MazeRouter100x100 router(&maze);
    router.initializeRouting(startCell, goalCell);
    
    if (!router.isGoalReachable()) {
        std::cout << "\nERROR: Goal is unreachable from start!" << std::endl;
        return -1;
    }
    
    std::cout << "MazeRouter initialized - Goal is reachable!" << std::endl;
    
    // using productive directions
    MazeRouter100x100::Cell currentCell = startCell;
    auto productiveDirs = router.getProductiveDirections(currentCell, goalCell);
    
    double initialPhi = 0.0;
    if (!productiveDirs.empty()) {
        // Use first productive direction for initial heading
        switch (productiveDirs[0]) {
            case MazeRouter100x100::Direction::NORTH:
                initialPhi = -M_PI / 2.0;  // -90 degrees
                break;
            case MazeRouter100x100::Direction::SOUTH:
                initialPhi = M_PI / 2.0;   // 90 degrees
                break;
            case MazeRouter100x100::Direction::EAST:
                initialPhi = 0.0;          // 0 degrees
                break;
            case MazeRouter100x100::Direction::WEST:
                initialPhi = M_PI;         // 180 degrees
                break;
            default:
                initialPhi = 0.0;
                break;
        }
        std::cout << "Initial heading set from productive direction: " << (initialPhi * 180.0 / M_PI) << " degrees" << std::endl;
        std::cout << "Start position: (" << startX << ", " << startY << ")" << std::endl;
        std::cout << "Start major cell: (" << (int)(startX/10.0) << ", " << (int)(startY/10.0) << ")" << std::endl;
    }
    
    //  3. Initialize Bicycle 
    Bicycle car(startX, startY, initialPhi);
    car.setVelocity(1.0);  // 1 unit/s  just initialization
    
    std::cout << "Bicycle initialized at (" << startX << ", " << startY << ")" << std::endl;
    std::cout << "Initial heading: " << (initialPhi * 180.0 / M_PI) << "degrees" << std::endl;
    
    //  4. Simulation Parameters 
    const double dt = 0.05;  // 50ms time step
    const int cellPixels = 100;  // Each major cell = 100 pixels
    
    const double goalThreshold = 4.0;  // Within 4 units = reached goal
    
    double t = 0.0;
    const double maxTime = 3000.0;
    bool goalReached = false;
    
    // Navigation modes
    enum class NavigationMode {
        NORMAL,
        REVERSING,        
        ALIGNING,         
        TURN_AROUND,      
        NORMAL_LEFT,      
    };
    
    NavigationMode navMode = NavigationMode::NORMAL;
    
    std::cout << "\nStarting Bicycle Navigation" << std::endl;
    std::cout << "Press ESC to exit, SPACE to pause" << std::endl;
    
    bool paused = false;
    int stepCount = 0;
    
    // Frame buffer for MP4 recording
    std::vector<cv::Mat> frames;

    
    // 5. Main Simulation Loop
    while (t < maxTime && !goalReached) {
        stepCount++;
        
        // initial pose
        auto pose = car.pose();
        double x = std::get<0>(pose);
        double y = std::get<1>(pose);
        double phi = std::get<2>(pose);
        
        // distance to goal
        double distToGoal = std::sqrt((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY));
        
        // if goal reached
        if (distToGoal < goalThreshold) {
            goalReached = true;
            std::cout << "\n🎉 GOAL REACHED at t=" << t << "s!" << std::endl;
            std::cout << "Final position: (" << x << ", " << y << ")" << std::endl;
            std::cout << "Distance from goal: " << distToGoal << std::endl;
            break;
        }
        
        // update current cell based on robot position
        currentCell.x = static_cast<int>(x);
        currentCell.y = static_cast<int>(y);
        
        // distance-based wall and obstacle detection
        double maxDetectionDist = 10.0;  // Maximum detection distance
        int currentMajorX = static_cast<int>(x / 10.0);
        int currentMajorY = static_cast<int>(y / 10.0);
        
        // function to calculate distance to nearest wall in a direction
        auto getWallDistance = [&](double angle) -> double {
            double step = 0.2; 
            double dist = 0.0;
            int prevMajorX = currentMajorX;
            int prevMajorY = currentMajorY;
            
            while (dist < maxDetectionDist) {
                dist += step;
                double checkX = x + dist * std::cos(angle);
                double checkY = y + dist * std::sin(angle);
                int checkMajorX = static_cast<int>(std::floor(checkX / 10.0));
                int checkMajorY = static_cast<int>(std::floor(checkY / 10.0));
                
                // if moved to a different major cell
                if (checkMajorX != prevMajorX || checkMajorY != prevMajorY) {
                    if (checkMajorX < 0 || checkMajorX >= 10 || checkMajorY < 0 || checkMajorY >= 10) {
                        return dist;  
                    }
                    if (maze.hasWallBetween(prevMajorX, prevMajorY, checkMajorX, checkMajorY)) {
                        return dist;  
                    }
                    // update previous cell for next iteration
                    prevMajorX = checkMajorX;
                    prevMajorY = checkMajorY;
                }
            }
            return maxDetectionDist;  // No wall found
        };
        
        // function to calculate distance to nearest obstacle in a direction
        auto getObstacleDistance = [&](double angle) -> double {
            double step = 0.1;  // Smaller step
            double dist = 0.0;
            
            while (dist < maxDetectionDist) {
                dist += step;
                double checkX = x + dist * std::cos(angle);
                double checkY = y + dist * std::sin(angle);
         
                // if obstacle found
                if (!maze.isObstacleFree(checkX, checkY)) {
                    return dist;
                }
            }
            return maxDetectionDist;  // No obstacle found 
        };
        
        double wallDistAhead = getWallDistance(phi);
        double wallDistBehind = getWallDistance(phi + M_PI);
        double wallDistLeft = getWallDistance(phi - (M_PI/2.0));
        double wallDistRight = getWallDistance(phi + (M_PI/2.0));
        
        double obstacleDistAhead = getObstacleDistance(phi);
        double obstacleDistBehind = getObstacleDistance(phi + M_PI);
        double obstacleDistLeft = getObstacleDistance(phi - M_PI/2.0);
        double obstacleDistRight = getObstacleDistance(phi + M_PI/2.0);
        double obstacleDistLeftAhead = getObstacleDistance(phi - M_PI/4.0);
        double obstacleDistRightAhead = getObstacleDistance(phi + M_PI/4.0);
        
        // convert distances to boolean flags
        double wallThreshold = 8.0;  // Consider wall "detected" if within 8 units
        double obstacleThreshold = 4.0;  // Consider obstacle "detected" if within 4 units
        
        bool wallAhead = wallDistAhead < wallThreshold;
        bool wallBehind = wallDistBehind < wallThreshold;
        bool wallLeft = wallDistLeft < wallThreshold;
        bool wallRight = wallDistRight < wallThreshold;
        
        bool obstacleAhead = obstacleDistAhead < obstacleThreshold;
        bool obstacleBehind = obstacleDistBehind < obstacleThreshold;
        bool obstacleLeft = obstacleDistLeft < obstacleThreshold;
        bool obstacleRight = obstacleDistRight < obstacleThreshold;
        bool obstacleLeftAhead = obstacleDistLeftAhead < obstacleThreshold;
        bool obstacleRightAhead = obstacleDistRightAhead < obstacleThreshold;
        
        
        double desiredPhi = -0.00;
        double steerCmd = -0.00;  
        double velocity = 0.50;

        double wallError = 0.0;
        double errorDerivative = 0.0;
        double error = 0.0;
        double targetDistance = 0.0;

        //distance from center of cell
        double dX = x-(currentMajorX*10.0);
        double dY = y-(currentMajorY*10.0);
        double distFromCenter = std::sqrt(dX*dX + dY*dY);
        
        // execute current state
        switch (navMode) {
            case NavigationMode::NORMAL: {
                static double targetDesiredPhi = 0.0;
                static bool targetSet = false;
                static bool firstRun = true;
                static NavigationMode lastNavMode = NavigationMode::NORMAL;  // Track previous mode
                static double lastError = 0.0;

                // Set target only once (or when it's reached)
                const double headingThreshold = 0.05;  // 0.05 radians ≈ 2.9 degrees
                
                // Detect when we JUST entered NORMAL mode from ALIGNING
                if (lastNavMode != NavigationMode::NORMAL) {
                    // We just entered NORMAL from ALIGNING - keep current heading
                    targetSet = true;  // Set targetSet to true to prevent immediate re-entry to ALIGNING
                    targetDesiredPhi = phi;  // Keep current heading
                    firstRun = true;
                    std::cout << "Entering NORMAL from ALIGNING: phi=" << phi << ", targetDesiredPhi=" << targetDesiredPhi << ", keeping current heading" << std::endl;
                    
                    // Check if walls still present - if so, go back to ALIGNING
                    if (wallDistLeft < 10.0 && wallDistRight < 10.0 && !wallAhead && !wallBehind) {
                        std::cout << "Still between walls, switching back to ALIGNING" << std::endl;
                        navMode = NavigationMode::ALIGNING;
                        break;
                    }
                }
                lastNavMode = navMode;
                
                                
                

                if (!targetSet && wallDistAhead <4.0 && wallLeft && wallRight && !wallBehind) { // wall ahead and both sides
                    std::cout << "wall ahead and both sides--------------------------------------------" << std::endl;
                    navMode = NavigationMode::REVERSING;
                    std::cout << "Reversing to leave the dead end" << std::endl;
                }
                if (!targetSet && wallDistRight <9.0 && wallDistLeft <9.0 &&!wallAhead && !wallBehind) {  // between wall and wall  hiher priority than other conditions
                    navMode = NavigationMode::ALIGNING;
                    
                    std::cout << "Aligning" << std::endl;
                }
                
                
                
                if ((!targetSet && wallAhead && !wallLeft && !wallRight && wallBehind) || (!targetSet && wallBehind && !wallAhead && !wallLeft && !wallRight)) { //only wall ahead and behind or only wall behind and ahead
                    if (wallDistBehind > 3.0) { //tooclose to wall, reverse
                        navMode = NavigationMode::REVERSING;
                        
                    }
                    targetDesiredPhi = phi + M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 5: " << targetDesiredPhi << std::endl;
                }
                
                if (!targetSet && !wallAhead && wallLeft && !wallRight && wallBehind) { //only wall left and behind
                    if (wallDistBehind > 3.0){
                        navMode = NavigationMode::REVERSING;
                        
                    }
                    targetDesiredPhi = phi + M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 7: " << targetDesiredPhi << std::endl;
                }
                if (!targetSet && wallAhead && wallBehind && wallLeft && !wallRight) { //initial starting orientation 1
                    if (wallDistBehind > 3.0){
                        navMode = NavigationMode::REVERSING;
                    }
                    targetDesiredPhi = phi + M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 8: " << targetDesiredPhi << std::endl;
                }
                if (!targetSet && wallAhead && wallBehind && wallRight && !wallLeft) { //initial starting orientation 2
                    if (wallDistBehind > 3.0){
                        navMode = NavigationMode::REVERSING;
                    }
                    targetDesiredPhi = phi - M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 9: " << targetDesiredPhi << std::endl;
                }
                if (!targetSet && wallAhead && !wallLeft && !wallRight && !wallBehind) { //only wall ahead
                    targetDesiredPhi = phi + M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 6: " << targetDesiredPhi << std::endl;
                }
                if (!targetSet && wallAhead && wallDistLeft <8.0 && !wallRight && !wallBehind) { // only wall ahead and left
                    // Set new target
                    targetDesiredPhi = phi + M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 1: " << targetDesiredPhi << std::endl;
                }

                if (!targetSet && wallAhead && wallDistRight <8.0 && !wallLeft && !wallBehind) { // only wall ahead and right
                    // Set new target
                    targetDesiredPhi = phi - M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 2: " << targetDesiredPhi << std::endl;
                }
                
                
                if (!targetSet && !wallAhead && wallLeft && !wallRight && !wallBehind) { //only wall left
                    targetDesiredPhi = phi + M_PI/180.0 * 90;
                    SNAP_TARGET(targetDesiredPhi);
                    targetSet = true;
                    std::cout << "Setting new target heading 4: " << targetDesiredPhi << std::endl;
                }
                

               
                
                // Use the fixed target
                if (targetSet) {
                    desiredPhi = targetDesiredPhi;
                }
                else {
                    desiredPhi = phi;
                }
                std::cout << "NORMAL: targetSet=" << targetSet << ", targetDesiredPhi=" << targetDesiredPhi << ", phi=" << phi << ", desiredPhi=" << desiredPhi << std::endl;
                
                
                error = desiredPhi - phi;
                while (error > M_PI) error -= 2.0 * M_PI;
                while (error < -M_PI) error += 2.0 * M_PI;
                
                
                if (!firstRun) {
                    errorDerivative = (error - lastError) / dt;
                }
                lastError = error;
                double kp = 10.0;
                std::cout << "errorderivative: " << errorDerivative << std::endl;
                double kd = 0.1;
                double controlOutput = kp * error + kd * errorDerivative;
                steerCmd = controlOutput;
                
                // Reset when target is reached
                std::cout << "error2: " << error << std::endl;
                if (std::abs(error) < 1e-3 ) {
                    velocity = 1.0;
                    targetSet = false;
                    std::cout << "Target reached! Ready for new heading." << std::endl;
                }
                
                firstRun = false;
                
                break;   
            }
            
            case NavigationMode::REVERSING: {
                static bool deadEnd = false;
                static double checkDY = phi > 0 ? 2.0 : 8.0; 
                static double checkDX = (std::abs(phi) < M_PI/4.0) ? 2.0 : 8.0;    //check east or west if going east, reverse is west so dx should be 2 and if going west, reverse is east so dx should be 8
                static bool lateral = ((std::abs(phi) < M_PI/4.0) || (std::abs(phi) > 3*M_PI/4.0)) ? true : false;    //check east or west if going east, reverse is west so dx should be 2 and if going west, reverse is east so dx should be 8
                
                checkDY = phi > 0 ? 2.0 : 8.0;
                checkDX = (std::abs(phi) < M_PI/4.0) ? 2.0 : 8.0;
                lateral = ((std::abs(phi) < M_PI/4.0) || (std::abs(phi) > 3*M_PI/4.0)) ? true : false;
                // boolean to check if lateral or vertical
                 std::cout << "lateral: " << lateral << std::endl;
                std::cout << "checkDY: " << checkDY << std::endl;
                std::cout << "checkDX: " << checkDX << std::endl;
                // std::cout << "checkDX: " << checkDX << std::endl;
                std::cout << "dY: " << dY << std::endl;
                std::cout << "dX: " << dX << std::endl;
                // std::cout << "DEBUG line 449 check: wallDistBehind=" << wallDistBehind << std::endl;
                bool dYCheck = (dY >= checkDY-0.5 && dY <= checkDY+0.5) && !lateral;
                bool dXCheck = (dX >= checkDX-0.5 && dX <= checkDX+0.5) && lateral;
                std::cout << "dYCheck=" << dYCheck << ", dXCheck=" << dXCheck << ", lateral=" << lateral << std::endl;
                std::cout << "dY=" << dY << ", checkDY=" << checkDY << ", dX=" << dX << ", checkDX=" << checkDX << std::endl;
                std::cout << "deadEnd: " << deadEnd << std::endl;
                if (!wallBehind && wallLeft && wallRight) {
                    deadEnd = true;
                    std::cout << "Dead end detected" << std::endl;
                    velocity = -0.50;
                }
                if (deadEnd){
                    velocity = -0.50;
                }
                
                // if (wallDistAhead < 8.0 && !wallLeft && !wallRight && !deadEnd) {
                //     velocity = -0.5;
                // }
                // else if (wallDistAhead >= 8.0 && wallDistAhead < 10.0 && !deadEnd){
                //     navMode = NavigationMode::NORMAL;
                // }
                std::cout << "wallDistBehind: " << wallDistBehind << std::endl;
                if ((((dY >= checkDY-0.5 && dY <= checkDY+0.5) && !lateral) || ((dX >= checkDX-0.5 && dX <= checkDX+0.5) && lateral)) && deadEnd && !wallLeft) {
                    std::cout << "Line 446 condition triggered!" << std::endl;
                    deadEnd = false;
                    navMode = NavigationMode::NORMAL_LEFT; // turn left, and then follow the wall
                }
                if (wallDistBehind > 2.9 && !deadEnd){  // for condition where only wall left and behind or right and behind
                    velocity = -1.0;
                }
                else if (wallDistBehind <= 2.9  && !deadEnd){  // for condition where only wall left and behind or right and behind
                    navMode = NavigationMode::NORMAL;
                }
                else if (wallDistBehind <= 3.0 && !wallAhead && deadEnd){
                    deadEnd = false;
                    navMode = NavigationMode::NORMAL_LEFT; // turn left, and then follow the wall
                }
                else if (wallDistAhead == 10.0 && wallDistLeft == 10.0 && wallDistRight == 10.0 && (((dY >= checkDY-0.5 && dY <= checkDY+0.5) && !lateral) || ((dX >= checkDX-0.5 && dX <= checkDX+0.5) && lateral)) && wallDistBehind == 10.0 && deadEnd) { //dY is the distance from the center of the cell in y direction
                    std::cout << "Line 449 condition triggered!" << std::endl;
                    deadEnd = false;
                    navMode = NavigationMode::NORMAL_LEFT; // turn left, and then follow the wall
                    
                                                                                   //phi positive is south (reverse is going north so dy should be 2) and negative is north
                     // turn left, and then follow the wall this is 
                }
                // Debug output to check why conditions at 449 and 457 aren't met
                
                else if (wallDistAhead == 10.0 && wallDistLeft == 10.0 && wallDistRight < 10.0 && (((dY >= checkDY-0.5 && dY <= checkDY+0.2) && !lateral) || ((dX >= checkDX-0.5 && dX <= checkDX+0.2) && lateral)) && wallDistBehind == 10.0 && deadEnd) { //when right wall is present before the deadend
                    std::cout << "Line 465 condition triggered!" << std::endl;
                    deadEnd = false;
                    navMode = NavigationMode::NORMAL_LEFT; // turn left, and then follow the wall
                    
                    
                }
                
                
            
                
                    break;
            }
            case NavigationMode::ALIGNING: {
                static bool firstRun = true;
                static double lastError = 0.0;
                static double errorDerivative = 0.0;
                static double targetHeadingInAligning = 0.0;  // Will be set when entering mode
                static bool headingSet = false;
                static NavigationMode lastMode = NavigationMode::NORMAL;  // Track previous mode
                
                std::cout << "ALIGNING: lastMode=" << (int)lastMode << ", navMode=" << (int)navMode << ", headingSet=" << headingSet << ", entering check result: " << (lastMode != NavigationMode::ALIGNING) << std::endl;
                
                // Detect when we JUST entered ALIGNING mode
                if (lastMode != NavigationMode::ALIGNING) {
                    // We just entered ALIGNING - set target heading to current heading
                    targetHeadingInAligning = phi;
                    headingSet = true;
                    firstRun = true;
                    std::cout << "ENTERING ALIGNING: Target heading set to: " << targetHeadingInAligning << std::endl;
                    std::cout << "Current phi: " << phi << std::endl;
                }
                lastMode = NavigationMode::ALIGNING;
                
                if (wallDistLeft <9.0 && wallDistRight <9.0 && !wallAhead && !wallBehind) {
                    if (!headingSet) {
                        // Failed to initialize - do it now as fallback
                        targetHeadingInAligning = phi;
                        headingSet = true;
                        firstRun = true;
                        std::cout << "Warning: headingSet was false - initializing now with phi=" << phi << std::endl;
                    }
                    
                    // Calculate position error
                    targetDistance = (wallDistLeft + wallDistRight) / 2.0;
                    error = (targetDistance - wallDistLeft)/targetDistance;
                    
                    // Calculate heading error
                    std::cout << "targetHeadingInAligning: " << targetHeadingInAligning << std::endl;
                    std::cout << "phi: " << phi << std::endl;
                    double headingError = targetHeadingInAligning - phi;
                    while (headingError > M_PI) headingError -= 2.0 * M_PI;
                    while (headingError < -M_PI) headingError += 2.0 * M_PI;
                    
                    std::cout << "ALIGNING error: " << error << ", headingError: " << headingError << std::endl;
                    
                    if (!firstRun) {
                        errorDerivative = (error - lastError) / dt;
                    }
                    lastError = error;
                    
                    // Combined control: position + heading
                    double kp = 0.1;  // Position gain
                    double kp_heading = 0.1;  // Heading gain
                    double kd = 0.0;
                    double controlOutput = kp * error + kp_heading * headingError + kd * errorDerivative;
                    steerCmd = controlOutput;
                    
                    // Switch to NORMAL only when BOTH position AND heading are aligned
                    const double positionThreshold = 5e-2;
                    const double headingThreshold = 0.05;  // ~2.9 degrees
                    
                    if (std::abs(error) < positionThreshold && std::abs(headingError) < headingThreshold) {
                        navMode = NavigationMode::NORMAL;
                        headingSet = false;  // Reset for next time
                        firstRun = true;     // Reset firstRun
                        std::cout << "ALIGNING complete! Switching to NORMAL" << std::endl;
                    }
                } else if (! (wallDistLeft<9.0) || ! (wallDistRight<9.0)) {
                    navMode = NavigationMode::NORMAL;
                    headingSet = false;  // Reset for next time
                    firstRun = true;
                } else if (wallAhead && wallLeft && wallRight && !wallBehind) {
                    navMode = NavigationMode::REVERSING;
                    headingSet = false;  // Reset for next time
                    firstRun = true;
                }
                velocity = 0.4;  // Slower speed in ALIGNING mode

                break;
            }
            
            case NavigationMode::NORMAL_LEFT: {
                static double targetDesiredPhi = phi;
                static bool targetSet = false;
                static bool firstRun = true;
                static double lastError = 0.0;

                
                
                targetDesiredPhi = phi - M_PI/180.0 * 90;
                std::cout << "NORMAL_LEFT: Before snap, targetDesiredPhi=" << targetDesiredPhi << ", phi=" << phi << std::endl;
                
                
                 // go left unless left wall is present
                if (wallLeft) {
                    targetDesiredPhi = phi;
                    std::cout << "NORMAL_LEFT: Left wall detected, keeping phi=" << phi << std::endl;
                }
                SNAP_TARGET(targetDesiredPhi);
                desiredPhi = targetDesiredPhi;  // Assign snapped value to desiredPhi
                std::cout << "NORMAL_LEFT: After snap, targetDesiredPhi=" << targetDesiredPhi << ", desiredPhi=" << desiredPhi << std::endl;
            
                
                std::cout << "desiredPhi set to: " << desiredPhi << ", phi: " << phi << ", targetSet: " << targetSet << std::endl;
                
                
                error = desiredPhi - phi;
                while (error > M_PI) error -= 2.0 * M_PI;
                while (error < -M_PI) error += 2.0 * M_PI;
                
                
                if (!firstRun) {
                    errorDerivative = (error - lastError) / dt;
                }
                lastError = error;
                double kp = 10.0;
                std::cout << "errorderivative: " << errorDerivative << std::endl;
                double kd = 0.1;
                double controlOutput = kp * error + kd * errorDerivative;
                steerCmd = controlOutput;
                
                // Reset when target is reached
                std::cout << "error2: " << error << std::endl;
                if (std::abs(error) < 1e-3 ) {
                    velocity = 1.00;
                    targetSet = false;
                    std::cout << "Target reached! Ready for new heading." << std::endl;
                    navMode = NavigationMode::NORMAL;
                }
                
                firstRun = false;
                break;
            }
        }
        
        // Apply controls
        car.setSteerAngle(steerCmd);  // Now using direct angle control
        car.setVelocity(velocity);
        
        // Update bicycle physics
        car.step(dt);
        
        
        // === 6. Visualization ===
        if (stepCount % 100 == 0) {  // Draw every 10th frame for speed
            cv::Mat image = maze.visualizeWithSubcells(cellPixels, 10);
            
            // Convert coordinates to pixels
            // Major cells: use major cell indices directly (center of cell)
            // OpenCV: Point(x, y) where x=horizontal (column), y=vertical (row)
            
            // Draw goal (red circle) - using major cell coordinates
            // Now using consistent (x=horizontal, y=vertical) convention
            int goalPixX = goalMajorX * cellPixels + cellPixels / 2;  // x = horizontal
            int goalPixY = goalMajorY * cellPixels + cellPixels / 2;  // y = vertical
            cv::circle(image, cv::Point(goalPixX, goalPixY), 10, cv::Scalar(0, 0, 255), -1);
            
            // Draw start (green circle) - using major cell coordinates
            int startPixX = startMajorX * cellPixels + cellPixels / 2;  // x = horizontal
            int startPixY = startMajorY * cellPixels + cellPixels / 2;  // y = vertical
            cv::circle(image, cv::Point(startPixX, startPixY), 8, cv::Scalar(0, 255, 0), -1);
            
            // Draw bicycle (blue circle) - using world coordinates
            // World coordinates now use (x=horizontal, y=vertical) like OpenCV
            int carPixX = static_cast<int>(x * cellPixels / 10.0);  // x = horizontal
            int carPixY = static_cast<int>(y * cellPixels / 10.0);  // y = vertical
            cv::circle(image, cv::Point(carPixX, carPixY), 6, cv::Scalar(255, 0, 0), -1);
            
            // Draw line to goal (cyan dashed)
            int goalPixX_viz = goalMajorX * cellPixels + cellPixels / 2;  // x = horizontal
            int goalPixY_viz = goalMajorY * cellPixels + cellPixels / 2;  // y = vertical
            cv::line(image, cv::Point(carPixX, carPixY), cv::Point(goalPixX_viz, goalPixY_viz),
                     cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
            
            // Draw 360-degree wall sensors
            double sensorLength = 10*12;
            
            // Front sensor
            int frontX = carPixX + static_cast<int>(sensorLength * std::cos(phi));
            int frontY = carPixY + static_cast<int>(sensorLength * std::sin(phi));
            cv::line(image, cv::Point(carPixX, carPixY), cv::Point(frontX, frontY),
                    wallAhead ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 3);
            
            // Left sensor
            int leftX = carPixX + static_cast<int>(sensorLength * std::cos(phi - M_PI/2));
            int leftY = carPixY + static_cast<int>(sensorLength * std::sin(phi - M_PI/2));
            cv::line(image, cv::Point(carPixX, carPixY), cv::Point(leftX, leftY),
                    wallLeft ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 200, 200), 2);
            
            // Right sensor
            int rightX = carPixX + static_cast<int>(sensorLength * std::cos(phi + M_PI/2));
            int rightY = carPixY + static_cast<int>(sensorLength * std::sin(phi + M_PI/2));
            cv::line(image, cv::Point(carPixX, carPixY), cv::Point(rightX, rightY),
                    wallRight ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 200, 200), 2);
            
            // Back sensor (thinner)
            int backX = carPixX - static_cast<int>(sensorLength * 0.7 * std::cos(phi));
            int backY = carPixY - static_cast<int>(sensorLength * 0.7 * std::sin(phi));
            cv::line(image, cv::Point(carPixX, carPixY), cv::Point(backX, backY),
                    wallBehind ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 200, 200), 1);
            
            // Draw motion indicator based on mode
            if (navMode == NavigationMode::REVERSING) {
                // Backward arrow
                cv::arrowedLine(image, cv::Point(carPixX, carPixY), cv::Point(backX, backY),
                               cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.3);
            }
            
            // Draw status text
            std::string timeText = "Time: " + std::to_string(static_cast<int>(t)) + "s";
            cv::putText(image, timeText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                        0.6, cv::Scalar(0, 0, 0), 2);
            
            std::string distText = "Goal dist: " + std::to_string(static_cast<int>(distToGoal));
            cv::putText(image, distText, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 
                        0.6, cv::Scalar(0, 0, 0), 2);
            
            std::string velText = "Velocity: " + std::to_string(velocity).substr(0, 5);
            cv::putText(image, velText, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 
                        0.6, cv::Scalar(0, 0, 0), 2);

            
            
            // Mode display
            std::string modeText;
            cv::Scalar modeColor;
            switch (navMode) {
                case NavigationMode::NORMAL:
                    modeText = "Mode: NORMAL";
                    modeColor = cv::Scalar(0, 200, 0);
                    break;
                
                case NavigationMode::REVERSING:
                    modeText = "Mode: REVERSING (parallel to wall)";
                    modeColor = cv::Scalar(0, 0, 255);
                    break;
                
                
                case NavigationMode::ALIGNING:
                    modeText = "Mode: ALIGNING";
                    modeColor = cv::Scalar(255, 255, 0);
                    break;
                case NavigationMode::NORMAL_LEFT   : 
                    modeText = "Mode: NORMAL_LEFT";
                    modeColor = cv::Scalar(0, 0, 255);
                    break;
                
            }
            cv::putText(image, modeText, cv::Point(10, 120), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, modeColor, 2);
            
            // Wall distances
            std::string wallDistText = "Wall Dist: F:" + std::to_string(wallDistAhead).substr(0, 4) + 
                                      " L:" + std::to_string(wallDistLeft).substr(0, 4) +
                                      " R:" + std::to_string(wallDistRight).substr(0, 4) + 
                                      " B:" + std::to_string(wallDistBehind).substr(0, 4);
            cv::putText(image, wallDistText, cv::Point(10, 150), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
            
            // Obstacle distances
            std::string obsDistText = "Obs Dist: F:" + std::to_string(obstacleDistAhead).substr(0, 4) + 
                                     " L:" + std::to_string(obstacleDistLeft).substr(0, 4) +
                                     " R:" + std::to_string(obstacleDistRight).substr(0, 4) + 
                                     " B:" + std::to_string(obstacleDistBehind).substr(0, 4);
            cv::putText(image, obsDistText, cv::Point(10, 170), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
            
            // Productive direction display
            auto productiveDirs = router.getProductiveDirections(currentCell, goalCell);
            std::string prodDirText = "Prod Dir: ";
            if (!productiveDirs.empty()) {
                switch (productiveDirs[0]) {
                    case MazeRouter100x100::Direction::NORTH: prodDirText += "NORTH"; break;
                    case MazeRouter100x100::Direction::SOUTH: prodDirText += "SOUTH"; break;
                    case MazeRouter100x100::Direction::EAST: prodDirText += "EAST"; break;
                    case MazeRouter100x100::Direction::WEST: prodDirText += "WEST"; break;
                    default: prodDirText += "UNKNOWN"; break;
                }
            } else {
                prodDirText += "NONE";
            }
            cv::putText(image, prodDirText, cv::Point(10, 190), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

            std::string steerText = "Steer: " + std::to_string(steerCmd).substr(0, 5);
            cv::putText(image, steerText, cv::Point(10, 220), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
            
            std::string desiredPhiText = "Desired Phi: " + std::to_string(desiredPhi).substr(0, 5);
            cv::putText(image, desiredPhiText, cv::Point(10, 250), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);

            std::string currentPhiText = "Current Phi: " + std::to_string(phi).substr(0, 5);
            cv::putText(image, currentPhiText, cv::Point(10, 280), cv::FONT_HERSHEY_SIMPLEX, 
                        0.6, cv::Scalar(0, 0, 0), 2);

            std::string currentCarPositionText = "Current Car Position: (" + std::to_string(x) + ", " + std::to_string(y) + ")";
            cv::putText(image, currentCarPositionText, cv::Point(10, 310), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
            
            // Capture frame every 100 frames for MP4
            if (stepCount % 100 == 0) {
                frames.push_back(image.clone());
            }
            
            // Show frame
            cv::imshow("Bicycle Navigation - Wall Following (360 degree Sensing)", image);
            
            // Handle keyboard
            int key = cv::waitKey(paused ? 0 : 1);  // 1ms delay for faster execution
            
            if (key == 27) {  // ESC
                std::cout << "\nUser interrupted at t=" << t << "s" << std::endl;
                break;
            }
            else if (key == 32) {  // SPACE
                paused = !paused;
                std::cout << (paused ? "PAUSED" : "RESUMED") << std::endl;
            }
        }
        
        t += dt;
    }
    
    // 6. Final Summary
    std::cout << "\n=== Simulation Complete ===" << std::endl;
    std::cout << "Total time: " << t << "s" << std::endl;
    std::cout << "Goal reached: " << (goalReached ? "YES" : "NO") << std::endl;
    
    // show final frame
    auto finalPose = car.pose();
    double finalX = std::get<0>(finalPose);
    double finalY = std::get<1>(finalPose);
    double finalPhi = std::get<2>(finalPose);
    double finalDist = std::sqrt((finalX - goalX) * (finalX - goalX) + (finalY - goalY) * (finalY - goalY));
    
    std::cout << "Final position: (" << finalX << ", " << finalY << ")" << std::endl;
    std::cout << "Final distance from goal: " << finalDist << std::endl;
    
    cv::Mat finalImage = maze.visualizeWithSubcells(cellPixels, 10);
    
    // draw goal (red) - using major cell coordinates
    int goalPixX = goalMajorX * cellPixels + cellPixels / 2;  // x = horizontal
    int goalPixY = goalMajorY * cellPixels + cellPixels / 2;  // y = vertical
    cv::circle(finalImage, cv::Point(goalPixX, goalPixY), 10, cv::Scalar(0, 0, 255), -1);
    
    // draw start (green) - using major cell coordinates
    int startPixX = startMajorX * cellPixels + cellPixels / 2;  // x = horizontal
    int startPixY = startMajorY * cellPixels + cellPixels / 2;  // y = vertical
    cv::circle(finalImage, cv::Point(startPixX, startPixY), 8, cv::Scalar(0, 255, 0), -1);
    
    // draw bicycle final position (blue) - using world coordinates
    int carPixX = static_cast<int>(finalX * cellPixels / 10.0);  // x = horizontal
    int carPixY = static_cast<int>(finalY * cellPixels / 10.0);  // y = vertical
    cv::circle(finalImage, cv::Point(carPixX, carPixY), 6, cv::Scalar(255, 0, 0), -1);
    
    // final status
    std::string finalText = goalReached ? "GOAL REACHED!" : "Timeout";
    cv::putText(finalImage, finalText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                0.8, goalReached ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
    
    cv::imshow("Bicycle Navigation - Wall Following (360° Sensing)", finalImage);
    
    // save frames as MP4 video
    if (!frames.empty()) {
        std::string videoPath = "bicycle_navigation.mp4";
        int codec = cv::VideoWriter::fourcc('h', '2', '6', '4');  // H.264 codec
        double fps = 5.0;  // 5 frames per second
        cv::Size frameSize = frames[0].size();
        
        std::cout << "\nSaving " << frames.size() << " frames to " << videoPath << "..." << std::endl;
        
        cv::VideoWriter writer;
        if (writer.open(videoPath, codec, fps, frameSize)) {
            for (size_t i = 0; i < frames.size(); i++) {
                writer.write(frames[i]);
                if ((i + 1) % 50 == 0) {
                    std::cout << "Saved " << (i + 1) << "/" << frames.size() << " frames..." << std::endl;
                }
            }
            
            writer.release();
            std::cout << "Video saved successfully to " << videoPath << std::endl;
        } else {
            std::cerr << "Error: Could not open video file for writing" << std::endl;
        }
    }
    
    std::cout << "\nPress any key to exit..." << std::endl;
    cv::waitKey(0);
    
    return 0;
}

