#include <iostream>
#include <vector>
#include <random>
#include <opencv2/opencv.hpp>
#include "GenMaze.hpp"



MazeGenerator::MazeGenerator(int mazeSize)
        : size(mazeSize), gen(rd())
{
    maze.resize(size, std::vector<int>(size, FREE));
}

void MazeGenerator::generateRandomMaze(double obstacleRatio)
{
    // Create border walls
    for (int i = 0; i < size; i++) {
        maze[0][i] = WALL;           // Top border
        maze[size-1][i] = WALL;      // Bottom border
        maze[i][0] = WALL;           // Left border
        maze[i][size-1] = WALL;      // Right border
    }

    // Add random obstacles
    std::uniform_int_distribution<> dis(1, size-2);  // Avoid borders
    int numObstacles = static_cast<int>(size * size * obstacleRatio);

    for (int i = 0; i < numObstacles; i++) {
        int x = dis(gen);
        int y = dis(gen);
        if (maze[x][y] == FREE) {
            maze[x][y] = WALL;
        }
    }
}
// Set random start and goal positions
    void MazeGenerator::setRandomStartGoal() {
        std::uniform_int_distribution<> dis(1, size-2);  // Avoid borders

        // Find random start position
        int startX, startY;
        do {
            startX = dis(gen);
            startY = dis(gen);
        } while (maze[startX][startY] != FREE);

        // Find random goal position (different from start)
        int goalX, goalY;
        do {
            goalX = dis(gen);
            goalY = dis(gen);
        } while (maze[goalX][goalY] != FREE || (goalX == startX && goalY == startY));

        maze[startX][startY] = START;
        maze[goalX][goalY] = GOAL;

        std::cout << "Start position: (" << startX << ", " << startY << ")" << std::endl;
        std::cout << "Goal position: (" << goalX << ", " << goalY << ")" << std::endl;
    }

    // Print the maze
    void MazeGenerator::printMaze() {
        std::cout << "\nGenerated Maze (" << size << "x" << size << "):" << std::endl;
        std::cout << "Legend: ' ' = Free, '#' = Wall, 'S' = Start, 'G' = Goal" << std::endl;
        std::cout << std::string(size + 2, '-') << std::endl;

        for (int i = 0; i < size; i++) {
            std::cout << "|";
            for (int j = 0; j < size; j++) {
                switch (maze[i][j]) {
                    case FREE:  std::cout << " "; break;
                    case WALL:  std::cout << "#"; break;
                    case START: std::cout << "S"; break;
                    case GOAL:  std::cout << "G"; break;
                    default: break;
                }
            }
            std::cout << "|" << std::endl;
        }
        std::cout << std::string(size + 2, '-') << std::endl;
    }

    // Count different cell types
    void MazeGenerator::printStatistics() {
        int freeCount = 0, wallCount = 0, startCount = 0, goalCount = 0;

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                switch (maze[i][j]) {
                    case FREE:  freeCount++; break;
                    case WALL:  wallCount++; break;
                    case START: startCount++; break;
                    case GOAL:  goalCount++; break;
                    default: break;
                }
            }
        }

        std::cout << "\nMaze Statistics:" << std::endl;
        std::cout << "Total cells: " << size * size << std::endl;
        std::cout << "Free cells: " << freeCount << std::endl;
        std::cout << "Wall cells: " << wallCount << std::endl;
        std::cout << "Start cells: " << startCount << std::endl;
        std::cout << "Goal cells: " << goalCount << std::endl;
        std::cout << "Obstacle ratio: " << static_cast<double>(wallCount) / (size * size) * 100 << "%" << std::endl;
    }

    // Get maze data (for future use with pathfinding)
    std::vector<std::vector<int>> MazeGenerator::getMaze() const {
        return maze;
    }

    // Get maze size
    int MazeGenerator::getSize() const {
        return size;
    }

    // Visualize the maze using OpenCV; each cell is cellPixels x cellPixels
    cv::Mat MazeGenerator::visualize(int cellPixels) const {
        const int height = size * cellPixels;
        const int width  = size * cellPixels;
        cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                cv::Rect cell(i * cellPixels, j * cellPixels, cellPixels, cellPixels);
                switch (maze[i][j]) {
                    case FREE:  rectangle(image, cell, cv::Scalar(255, 255, 255), -1); break; // white
                    case WALL:  rectangle(image, cell, cv::Scalar(0, 0, 0), -1);       break; // black
                    case START: rectangle(image, cell, cv::Scalar(0, 255, 0), -1);     break; // green
                    case GOAL:  rectangle(image, cell, cv::Scalar(0, 0, 255), -1);     break; // red
                    default:    rectangle(image, cell, cv::Scalar(200, 200, 200), -1); break;
                }
            }
        }

        // Optional grid lines
        for (int r = 0; r <= size; ++r) {
            line(image, cv::Point(0, r * cellPixels), cv::Point(width, r * cellPixels), cv::Scalar(180, 180, 180), 1);
        }
        for (int c = 0; c <= size; ++c) {
            line(image, cv::Point(c * cellPixels, 0), cv::Point(c * cellPixels, height), cv::Scalar(180, 180, 180), 1);
        }

    return image;
        // imshow("Maze (OpenCV)", image);
        // cv::waitKey(0);
    }

//#ifdef GENMAZE_STANDALONE
//int main() {
//    cout << "Random Maze Generator" << endl;
//    cout << "====================" << endl;
//
//    // Create maze generator for 100x100 maze
//    int mazeSize = 100;
//    MazeGenerator generator(mazeSize);
//
//    cout << "Generating " << mazeSize << "x" << mazeSize << " maze..." << endl;
//    cout << "Each cell represents 1m x 1m" << endl;
//    cout << "Total area: " << mazeSize << "m x " << mazeSize << "m" << endl;
//
//    // Generate maze with 20% obstacles
//    generator.generateRandomMaze(0.1);
//
//    // Set random start and goal
//    generator.setRandomStartGoal();
//
//    // Print statistics
//    generator.printStatistics();
//
//    // Print maze (only for small mazes)
//    // if (mazeSize <= 20) {
//    generator.printMaze();
//    // } else {
//    //     cout << "\nMaze too large to display in console." << endl;
//    //     cout << "Maze data is available for pathfinding algorithms." << endl;
//    // }
//
//    cout << "\nMaze generation complete!" << endl;
//    cout << "Ready for pathfinding algorithm implementation." << endl;
//
//    return 0;
//}
//#endif
