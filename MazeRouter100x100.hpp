#ifndef MAZEROUTER100X100_HPP
#define MAZEROUTER100X100_HPP

#include "SimpleMaze10x10.hpp"
#include <vector>
#include <queue>
#include <iostream>
#include <string>
#include <climits>

class MazeRouter100x100 {
public:
    struct Cell {
        int x, y;
        Cell() : x(0), y(0) {}
        Cell(int x, int y) : x(x), y(y) {}
        bool operator==(const Cell& other) const { return x == other.x && y == other.y; }
    };
    
    enum class Direction {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3,
        LOCAL = 4
    };
    
    enum class Mode {
        NORMAL,
        TRAVERSAL_LEFT,
        TRAVERSAL_RIGHT
    };
    
    struct Decision {
        Direction dir;
        bool unreachable;
        Mode mode;
    };
    
private:
    const SimpleMaze10x10* maze;  // Reference to the maze
    int gridSize;  // 100 (subcells)
    int majorCellSize;  // 10 (major cells)
    int subCellsPerMajor;  // 10
    
    Cell currentPos;
    Cell goalPos;
    
    // Routing state (like original MazeRouter)
    struct RoutingState {
        Mode mode = Mode::NORMAL;
        int MDbest = INT_MAX;
        bool leftHand = true;
        Direction traversalHeading = Direction::NORTH;
        
        struct Mark {
            Cell start;
            Direction firstDir;
            bool set = false;
        } mark;
    } state;
    
    // Helper methods
    bool isFree(const Cell& cell) const;
    int manhattanDistance(const Cell& a, const Cell& b) const;
    bool canMoveBetween(const Cell& from, const Cell& to) const;
    
    // Wall-following helpers
    Direction turnLeft(Direction dir) const;
    Direction turnRight(Direction dir) const;
    Direction getDirectionFromDelta(int dx, int dy) const;
    Direction getLeftHandRuleDir(const Cell& cur, const Cell& dst) const;
    Direction getRightHandRuleDir(const Cell& cur, const Cell& dst) const;
    
public:
    MazeRouter100x100(const SimpleMaze10x10* mazePtr);
    
    void reset();
    void initializeRouting(const Cell& startCell, const Cell& goalCell);
    bool isGoalReachable() const;
    
    Decision routeStep(const Cell& current, const Cell& goal, Direction incomingDir);
    Cell step(const Cell& current, Direction dir) const;
    
    // Public helper for productive directions
    std::vector<Direction> getProductiveDirections(const Cell& cur, const Cell& dst) const;
    
    // Debug helpers
    std::string getModeString() const;
    std::string getDirectionString(Direction dir) const;
};

#endif // MAZEROUTER100X100_HPP

