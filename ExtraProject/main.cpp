//
// Created by andoro on 2025-10-13.
//


#include "GenMaze.hpp"
#include <opencv2/opencv.hpp>
#include "BicycleTemp.hpp"
#include "MazeRouter.hpp"

int main ()
{
    int mazeSize =100;
    const int cellPixels = 12;
    MazeGenerator gen(mazeSize);
    gen.generateRandomMaze();
    gen.setRandomStartGoal();
    gen.printStatistics();

    // === 2. Maze Router Setup ===
    MazeRouter router(gen.getMaze());
    
    // Find start and goal positions
    int startX = -1, startY = -1, goalX = -1, goalY = -1;
    for (int i = 0; i < mazeSize; i++) {
        for (int j = 0; j < mazeSize; j++) {
            if (gen.getMaze()[i][j] == 2) { // START
                startX = i; startY = j;
            }
            if (gen.getMaze()[i][j] == 3) { // GOAL
                goalX = i; goalY = j;
            }
        }
    }
    
    if (startX == -1 || goalX == -1) {
        std::cout << "Could not find start or goal position!" << std::endl;
        return -1;
    }
    
    // Initialize routing
    MazeRouter::Cell start(startX, startY);
    MazeRouter::Cell goal(goalX, goalY);
    router.initializeRouting(start, goal);
    
    std::cout << "Maze Router initialized:" << std::endl;
    std::cout << "Start: (" << startX << ", " << startY << ")" << std::endl;
    std::cout << "Goal: (" << goalX << ", " << goalY << ")" << std::endl;
    std::cout << "Goal reachable: " << (router.isGoalReachable() ? "Yes" : "No") << std::endl;

    Bicycle car(startX * cellPixels + cellPixels/2.0, startY * cellPixels + cellPixels/2.0, -M_PI/2.0);
    car.setVelocity(1.0);          // 1 m/s

    // === 3. Visualization Setup ===
    cv::Mat image = gen.visualize(cellPixels);

    double t = 0.0;
    MazeRouter::Cell currentPos = start;
    MazeRouter::Direction lastDirection = MazeRouter::Direction::NORTH;
    
    // === 4. Main Simulation Loop ===
    while (t < 200.0)
    {
        const double dt = 0.05;
        
        // === 5. Maze Routing Decision ===
        if (currentPos.x != goal.x || currentPos.y != goal.y) {
            auto decision = router.routeStep(currentPos, goal, lastDirection);
            
            if (decision.unreachable) {
                std::cout << "Goal is unreachable!" << std::endl;
                break;
            }
            
            // Move to next cell
            if (decision.dir != MazeRouter::Direction::LOCAL) {
                currentPos = router.step(currentPos, decision.dir);
                lastDirection = decision.dir;
                
                // Convert grid position to world coordinates for car
                double worldX = currentPos.x + 0.5; // Convert grid (i,j) to world (x,y) in cell space
                double worldY = currentPos.y + 0.5; // Convert to mathematical coordinates (Y up)
                
                // Set bicycle position to match the route
                auto pose = car.pose();
                double currentPhi = std::get<2>(pose);
                car.setPosition(worldX, worldY, currentPhi); // Keep current heading
                
                // Simple steering based on direction
                double steerCmd = 0.0;
                switch (decision.dir) {
                    case MazeRouter::Direction::NORTH: steerCmd = 0.0; break;
                    case MazeRouter::Direction::EAST: steerCmd = 0.1; break;
                    case MazeRouter::Direction::SOUTH: steerCmd = 0.0; break;
                    case MazeRouter::Direction::WEST: steerCmd = -0.1; break;
                    default: break;
                }
                
                car.setSteerCmd(steerCmd);
            }
        }
        
        // Physics
        car.step(dt);

        // === 6. Draw Maze ===
        image = gen.visualize(cellPixels);
        
        // Draw current path
        cv::Point currentPosPix(int((currentPos.x * cellPixels) + (cellPixels/2)), 
                               int(currentPos.y * cellPixels + cellPixels/2));
        circle(image, currentPosPix, 5, cv::Scalar(0, 255, 255), -1); // Yellow for current position

        // === 7. Draw Car ===
        auto pose = car.pose();
        double x = std::get<0>(pose);
        double y = std::get<1>(pose);
        double phi = std::get<2>(pose);
        cv::Point carPos(int(x * cellPixels), int(y * cellPixels));
        circle(image, carPos, 3, cv::Scalar(255, 0, 0), -1);

        // === 8. Display ===
        cv::imshow("Maze + Bicycle Simulation + Maze Router", image);
        cv::waitKey(10);

        t += dt;
        
        // Check if goal reached
        if (currentPos.x == goal.x && currentPos.y == goal.y) {
            std::cout << "Goal reached!" << std::endl;
            break;
        }
    }
    cv::waitKey(0);
    return 0;
}
