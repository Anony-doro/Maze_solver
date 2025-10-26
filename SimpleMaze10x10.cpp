#include "SimpleMaze10x10.hpp"
#include <iostream>
#include <algorithm>

SimpleMaze10x10::SimpleMaze10x10() : size(10), gen(rd())
{
    // Initialize all cells as FREE
    cells.resize(size, std::vector<int>(size, FREE));
    
    // Initialize walls - all walls present initially
    // Horizontal walls: (size+1) rows × size columns
    horizontalWalls.resize(size + 1, std::vector<bool>(size, true));
    
    // Vertical walls: size rows × (size+1) columns
    verticalWalls.resize(size, std::vector<bool>(size + 1, true));
    
    // Initialize obstacles grid (100×100) - all free initially
    obstacles.resize(100, std::vector<bool>(100, false));
    
    // Initialize start/goal positions to invalid values
    startX = startY = -1;
    goalX = goalY = -1;
}

void SimpleMaze10x10::carveMazeDFS(int x, int y, std::vector<std::vector<bool>>& visited)
{
    visited[x][y] = true;
    
    // Directions: North, East, South, West
    int dx[] = {-1, 0, 1, 0};
    int dy[] = {0, 1, 0, -1};
    
    // Shuffle directions for randomness
    std::vector<int> dirs = {0, 1, 2, 3};
    std::shuffle(dirs.begin(), dirs.end(), gen);
    
    for (int dir : dirs) {
        int nx = x + dx[dir]; //nx is the next x coordinate
        int ny = y + dy[dir]; //ny is the next y coordinate
        
        // Check if next cell is valid and not visited
        if (nx >= 0 && nx < size && ny >= 0 && ny < size && !visited[nx][ny]) {
            // Remove wall between current cell (x,y) and next cell (nx,ny)
            if (dir == 0) {  // North - remove horizontal wall above current cell
                horizontalWalls[x][y] = false;
            }
            else if (dir == 1) {  // East - remove vertical wall to right of current cell
                verticalWalls[x][y + 1] = false;
            }
            else if (dir == 2) {  // South - remove horizontal wall below current cell
                horizontalWalls[x + 1][y] = false;
            }
            else if (dir == 3) {  // West - remove vertical wall to left of current cell
                verticalWalls[x][y] = false;
            }
            
            // Recursively carve to next cell
            carveMazeDFS(nx, ny, visited);
        }
    }
}

void SimpleMaze10x10::generateMaze()
{
    
    // All cells start as WALL (already initialized in constructor)
    std::vector<std::vector<bool>> visited(size, std::vector<bool>(size, false));
    
    // Start carving from a random position
    std::uniform_int_distribution<> dis(0, size - 1);
    int startX = dis(gen);
    int startY = dis(gen);
     
    // Carve maze using DFS
    carveMazeDFS(startX, startY, visited);
}

void SimpleMaze10x10::setRandomStartGoal()
{
    std::uniform_int_distribution<> dis(0, size - 1);
    
    // Find random cell for start
    int startX = dis(gen);
    int startY = dis(gen);
    
    // Find random cell for goal (different from start)
    int goalX, goalY;
    do {
        goalX = dis(gen);
        goalY = dis(gen);
    } while (goalX == startX && goalY == startY);
    
    cells[startX][startY] = START;
    cells[goalX][goalY] = GOAL;
    
    // Store positions for efficient access
    this->startX = startX;
    this->startY = startY;
    this->goalX = goalX;
    this->goalY = goalY;
}

std::pair<int, int> SimpleMaze10x10::getStartPosition() const
{
    return std::make_pair(startX, startY);
}

std::pair<int, int> SimpleMaze10x10::getGoalPosition() const
{
    return std::make_pair(goalX, goalY);
}

void SimpleMaze10x10::generateObstacles(double density)
{
    std::cout << "\n=== Generating Obstacles (density: " << (density * 100) << "%) ===" << std::endl;
    
    std::uniform_real_distribution<> prob(0.0, 1.0);
    int obstacleCount = 0;
    
    // Get start and goal major cells efficiently
    int startMajorX = startX;
    int startMajorY = startY;
    int goalMajorX = goalX;
    int goalMajorY = goalY;
    
    // Generate obstacles in 100×100 grid
    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 100; j++) {
            // Don't place obstacles in start/goal major cells (10×10 regions)
            int majorX = i / 10;
            int majorY = j / 10;
            
            bool inStartCell = (majorX == startMajorX && majorY == startMajorY);
            bool inGoalCell = (majorX == goalMajorX && majorY == goalMajorY);
            
            if (!inStartCell && !inGoalCell && prob(gen) < density) {
                obstacles[i][j] = true;
                obstacleCount++;
            }
        }
    }
    
    std::cout << "Generated " << obstacleCount << " obstacles in 100x100 grid" << std::endl;
}

bool SimpleMaze10x10::hasObstacle(int gridX, int gridY) const
{
    if (gridX < 0 || gridX >= 100 || gridY < 0 || gridY >= 100) {
        return false;  // Out of bounds = no obstacle
    }
    return obstacles[gridX][gridY];
}

bool SimpleMaze10x10::isObstacleFree(double worldX, double worldY) const
{
    // Check if a circular region around (worldX, worldY) is free of obstacles
    // World coordinates: 0-100 (matching 100×100 grid)
    
    // Check center
    int centerX = static_cast<int>(worldX);
    int centerY = static_cast<int>(worldY);
    if (hasObstacle(centerY, centerX)) {
        return false;
    }
    
    // // Check surrounding cells based on radius
    // int radiusCells = static_cast<int>(std::ceil(radius));
    // for (int dx = -radiusCells; dx <= radiusCells; dx++) {
    //     for (int dy = -radiusCells; dy <= radiusCells; dy++) {
    //         double dist = std::sqrt(dx*dx + dy*dy);
    //         if (dist <= radius) {
    //             int checkX = centerX + dx;
    //             int checkY = centerY + dy;
    //             if (hasObstacle(checkX, checkY)) {
    //                 return false;
    //             }
    //         }
    //     }
    // }
    
    return true;
}

cv::Mat SimpleMaze10x10::visualizeWithSubcells(int cellPixels, int subCellsPerMajor) const
{
    const int height = size * cellPixels;
    const int width = size * cellPixels;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));  // White background
    
    int subCellPixels = cellPixels / subCellsPerMajor;  // Pixels per subcell (e.g., 120/10 = 12)
    
    // Draw cell colors (start/goal)
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            cv::Rect cell(j * cellPixels, i * cellPixels, cellPixels, cellPixels);
            
            switch (cells[i][j]) {
                case START:
                    cv::rectangle(image, cell, cv::Scalar(150, 255, 150), -1);  // Light green
                    break;
                case GOAL:
                    cv::rectangle(image, cell, cv::Scalar(150, 150, 255), -1);  // Light red
                    break;
                case FREE:
                    // Already white, do nothing
                    break;
            }
        }
    }
    
    // Draw fine subcell grid (100×100 total)
    int totalSubCells = size * subCellsPerMajor;  // 10 * 10 = 100
    for (int i = 0; i <= totalSubCells; ++i) {
        int pos = i * subCellPixels;
        // Horizontal subcell lines
        cv::line(image, cv::Point(0, pos), cv::Point(width, pos), 
                 cv::Scalar(230, 230, 230), 1);
        // Vertical subcell lines
        cv::line(image, cv::Point(pos, 0), cv::Point(pos, height), 
                 cv::Scalar(230, 230, 230), 1);
    }
    
    // Draw obstacles (small dark gray squares)
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            if (obstacles[i][j]) {
                int pixelX = j * subCellPixels;
                int pixelY = i * subCellPixels;
                cv::rectangle(image, 
                             cv::Point(pixelX, pixelY), 
                             cv::Point(pixelX + subCellPixels, pixelY + subCellPixels),
                             cv::Scalar(80, 80, 80), -1);  // Dark gray
            }
        }
    }
    
    // Draw maze walls (thicker, on top of grid and obstacles)
    // Draw horizontal walls (lines between rows)
    for (int i = 0; i <= size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (horizontalWalls[i][j]) {
                int y = i * cellPixels;
                int x1 = j * cellPixels;
                int x2 = (j + 1) * cellPixels;
                cv::line(image, cv::Point(x1, y), cv::Point(x2, y), cv::Scalar(0, 0, 0), 4);
            }
        }
    }
    
    // Draw vertical walls (lines between columns)
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j <= size; ++j) {
            if (verticalWalls[i][j]) {
                int x = j * cellPixels;
                int y1 = i * cellPixels;
                int y2 = (i + 1) * cellPixels;
                cv::line(image, cv::Point(x, y1), cv::Point(x, y2), cv::Scalar(0, 0, 0), 4);
            }
        }
    }

    return image;
}

bool SimpleMaze10x10::hasWallBetween(int majorX1, int majorY1, int majorX2, int majorY2) const
{
    // Check if two adjacent major cells have a wall between them
    if (majorX2 == majorX1 + 1 && majorY2 == majorY1) {
        // moving east - check vertical wall
        return verticalWalls[majorY1][majorX2];
    }
    else if (majorX2 == majorX1 - 1 && majorY2 == majorY1) {
        // moving west - check vertical wall
        return verticalWalls[majorY1][majorX1];
    }
    else if (majorY2 == majorY1 + 1 && majorX2 == majorX1) {
        // moving south - check horizontal wall
        return horizontalWalls[majorY2][majorX1];
    }
    else if (majorY2 == majorY1 - 1 && majorX2 == majorX1) {
        // moving north - check horizontal wall
        return horizontalWalls[majorY1][majorX1];
    }
    
    // Not adjacent or same cell
    return false;
}

