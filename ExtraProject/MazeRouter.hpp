//
// Created by andoro on 2025-10-14.
//

#ifndef MAZEROUTER_HPP
#define MAZEROUTER_HPP

#include <vector>
#include <tuple>
#include <cmath>
#include <string>

class MazeRouter {
public:
    // Direction enum (4-connected)
    enum class Direction {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3,
        LOCAL = 4
    };
    
    // Cell structure
    struct Cell {
        int x, y;
        Cell(int x = 0, int y = 0) 
            : x(x), y(y) 
            {}
        bool operator==(const Cell& other) const {
            return x == other.x && y == other.y;
        }
    };
    
    // Mode enum
    enum class Mode {
        NORMAL,
        TRAVERSAL_LEFT,
        TRAVERSAL_RIGHT
    };
    
    // Traversal mark for unreachable detection
    struct TravMark {
        Cell start;   //stores the last tried direction and cell when entering traversal mode
        Direction firstDir;
        bool set = false;
    };
    
    // Routing state
    struct RoutingState {
        Mode mode = Mode::NORMAL;
        int MDbest = 10000;  // Best Manhattan distance reached
        TravMark mark;
        bool leftHand = true;  // Choose once when entering traversal
        Direction traversalHeading;  // Current heading direction in traversal mode
    };
    
    // Decision result
    struct Decision {
        Direction dir;
        bool unreachable = false;
        Mode newMode = Mode::NORMAL;
    };
    
    // Constructor
    MazeRouter(const std::vector<std::vector<int>>& mazeGrid);
    
    // Main routing function - implements Algorithm 1 from the paper
    Decision routeStep(const Cell& current, const Cell& goal, Direction incomingDir);
    
    // Initialize routing for new path
    void initializeRouting(const Cell& start, const Cell& goal);
    
    // Helper function for stepping
    Cell step(const Cell& cell, Direction dir) const;
    
    // Get current routing state
    Mode getCurrentMode() const { return state.mode; }
    int getMDbest() const { return state.MDbest; }
    
    // Check if goal is reachable
    bool isGoalReachable() const;
    
    // Reset routing state
    void reset();
    
    // Helper methods for integration
    Cell getCurrentPosition() const { return currentPos; }
    Cell getGoalPosition() const { return goalPos; }
    std::string getModeString() const;
    std::string getDirectionString(Direction dir) const;

private:
    // Cell types (matching GenMaze.hpp)
    static constexpr int FREE = 0;
    static constexpr int WALL = 1;
    static constexpr int START = 2;
    static constexpr int GOAL = 3;
    
    // Member variables
    std::vector<std::vector<int>> maze;
    int mazeSize;
    Cell currentPos;
    Cell goalPos;
    RoutingState state;
    
    // Helper functions
    int manhattanDistance(const Cell& from, const Cell& to) const;
    bool isFree(const Cell& cell) const;
    std::vector<Direction> getProductiveDirections(const Cell& cur, const Cell& dst) const;
    Direction turnLeft(Direction dir) const;
    Direction turnRight(Direction dir) const;
    Direction getLeftHandRuleDir(const Cell& cur, const Cell& dst) const;
    Direction getRightHandRuleDir(const Cell& cur, const Cell& dst) const;
    Direction getDirectionFromDelta(int dx, int dy) const;
};

#endif //MAZEROUTER_HPP
