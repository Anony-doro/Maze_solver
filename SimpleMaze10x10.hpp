#ifndef SIMPLEMAZE10X10_HPP
#define SIMPLEMAZE10X10_HPP

#include <random>
#include <vector>
#include <opencv2/opencv.hpp>

class SimpleMaze10x10 {
private:
    std::vector<std::vector<int>> cells;  // Cell contents (FREE, START, GOAL)
    std::vector<std::vector<bool>> horizontalWalls;  // Walls between rows (size+1 × size)
    std::vector<std::vector<bool>> verticalWalls;    // Walls between columns (size × size+1)
    std::vector<std::vector<bool>> obstacles;  // Obstacles in 100×100 grid
    int size;  // 10×10
    std::random_device rd;
    std::mt19937 gen;
    
    // Store start and goal positions for efficient access
    int startX, startY;
    int goalX, goalY;
    
    // Cell types
    static constexpr int FREE = 0;
    static constexpr int OBSTACLE = 1;
    static constexpr int START = 2;
    static constexpr int GOAL = 3;
    
    // DFS helper
    void carveMazeDFS(int x, int y, std::vector<std::vector<bool>>& visited);
    
public:
    SimpleMaze10x10();
    
    void generateMaze();  // Create 10×10 maze using DFS
    void setRandomStartGoal();
    void generateObstacles(double density = 0.15);  // Add random obstacles (15% density)
    cv::Mat visualizeWithSubcells(int cellPixels = 120, int subCellsPerMajor = 10) const;  // With 100×100 grid
    
    std::vector<std::vector<int>> getMaze() const { return cells; }
    int getSize() const { return size; }
    
    // Get start and goal positions directly
    // Returns (row, col) where row=vertical, col=horizontal
    // Note: For consistent (x,y) usage, use getStartPosition().second as x, getStartPosition().first as y
    std::pair<int, int> getStartPosition() const;
    std::pair<int, int> getGoalPosition() const;
    
    // For MazeRouter - check if there's a wall between two adjacent major cells
    // Parameters: (majorX, majorY) where majorX=horizontal, majorY=vertical (OpenCV convention)
    bool hasWallBetween(int majorX1, int majorY1, int majorX2, int majorY2) const;
    bool hasHorizontalWall(int row, int col) const { return horizontalWalls[row][col]; }
    bool hasVerticalWall(int row, int col) const { return verticalWalls[row][col]; }
    
    // For obstacle detection (100×100 grid)
    // Parameters: (gridX, gridY) where gridX=horizontal, gridY=vertical (OpenCV convention)
    bool hasObstacle(int gridX, int gridY) const;
    bool isObstacleFree(double worldX, double worldY) const;
};

#endif // SIMPLEMAZE10X10_HPP

