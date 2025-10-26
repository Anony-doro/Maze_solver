#include "MazeRouter100x100.hpp"
#include <algorithm>
#include <climits>
#include <cmath>

MazeRouter100x100::MazeRouter100x100(const SimpleMaze10x10* mazePtr)
    : maze(mazePtr), majorCellSize(10), subCellsPerMajor(10)
{
    gridSize = majorCellSize * subCellsPerMajor;  // 10 * 10 = 100
    reset();
    std::cout << "MazeRouter100x100 initialized for " << gridSize << "x" << gridSize << " grid" << std::endl;
}

void MazeRouter100x100::reset()
{
    state = RoutingState();
    currentPos = Cell(0, 0);
    goalPos = Cell(0, 0);
}

void MazeRouter100x100::initializeRouting(const Cell& startCell, const Cell& goalCell)
{
    currentPos = startCell;
    goalPos = goalCell;
    state = RoutingState();
    state.MDbest = manhattanDistance(startCell, goalCell);
    
    std::cout << "\nInitializing routing:" << std::endl;
    std::cout << "Start: (" << startCell.x << ", " << startCell.y << ")" << std::endl;
    std::cout << "Goal: (" << goalCell.x << ", " << goalCell.y << ")" << std::endl;
    std::cout << "Initial Manhattan Distance: " << state.MDbest << std::endl;
}

int MazeRouter100x100::manhattanDistance(const Cell& a, const Cell& b) const
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

MazeRouter100x100::Cell MazeRouter100x100::step(const Cell& cell, Direction dir) const
{
    static const int dx[4] = {0, 1, 0, -1};  // N, E, S, W
    static const int dy[4] = {-1, 0, 1, 0};
    
    if (dir == Direction::LOCAL) {
        return cell;
    }
    
    int dirIdx = static_cast<int>(dir);
    return Cell(cell.x + dx[dirIdx], cell.y + dy[dirIdx]);
}

bool MazeRouter100x100::isFree(const Cell& cell) const
{
    // Check bounds
    if (cell.x < 0 || cell.x >= gridSize || cell.y < 0 || cell.y >= gridSize) {
        return false;
    }
    
    // Check for obstacles in the 100x100 grid
    if (maze->hasObstacle(cell.x, cell.y)) {
        return false;
    }
    
    // Note: Wall checking between major cells is handled by canMoveBetween() when stepping
    // This function only checks if the current cell itself is free
    return true;
}

bool MazeRouter100x100::canMoveBetween(const Cell& from, const Cell& to) const
{
    // Check if movement from 'from' to 'to' is allowed
    // This only checks walls between major cells, not obstacles (handled by isFree)
    
    int fromMajorX = from.x / subCellsPerMajor;
    int fromMajorY = from.y / subCellsPerMajor;
    int toMajorX = to.x / subCellsPerMajor;
    int toMajorY = to.y / subCellsPerMajor;
    
    // If moving within the same major cell, no wall check needed
    if (fromMajorX == toMajorX && fromMajorY == toMajorY) {
        return true;
    }
    
    // Moving between major cells - check for wall
    return !maze->hasWallBetween(fromMajorX, fromMajorY, toMajorX, toMajorY);
}

std::vector<MazeRouter100x100::Direction> MazeRouter100x100::getProductiveDirections(
    const Cell& cur, const Cell& dst) const
{
    std::vector<Direction> productiveDirs;
    int currentMD = manhattanDistance(cur, dst);
    
    for (int i = 0; i < 4; ++i) {
        Direction dir = static_cast<Direction>(i);
        Cell next = step(cur, dir);
        
        // Check if this move is valid (respects walls and obstacles)
        if (canMoveBetween(cur, next) && isFree(next) && manhattanDistance(next, dst) == currentMD - 1) {
            productiveDirs.push_back(dir);
        }
    }
    
    return productiveDirs;
}

MazeRouter100x100::Direction MazeRouter100x100::turnLeft(Direction dir) const
{
    return static_cast<Direction>((static_cast<int>(dir) + 3) % 4);
}

MazeRouter100x100::Direction MazeRouter100x100::turnRight(Direction dir) const
{
    return static_cast<Direction>((static_cast<int>(dir) + 1) % 4);
}

MazeRouter100x100::Direction MazeRouter100x100::getDirectionFromDelta(int dx, int dy) const
{
    if (std::abs(dx) >= std::abs(dy)) {
        return (dx > 0) ? Direction::EAST : Direction::WEST;
    } else {
        return (dy > 0) ? Direction::SOUTH : Direction::NORTH;
    }
}

MazeRouter100x100::Direction MazeRouter100x100::getLeftHandRuleDir(const Cell& cur, const Cell& dst) const
{
    int dx = dst.x - cur.x;
    int dy = dst.y - cur.y;
    Direction base = getDirectionFromDelta(dx, dy);
    
    // Sweep left-first: left, then backward
    Direction backward = static_cast<Direction>((static_cast<int>(base) + 2) % 4);
    Direction order[2] = {turnLeft(base), backward};
    
    for (auto dir : order) {
        Cell next = step(cur, dir);
        
        // Check if can move
        if (canMoveBetween(cur, next) && isFree(next)) {
            return dir;
        }
    }
    return base;  // fallback
}

MazeRouter100x100::Direction MazeRouter100x100::getRightHandRuleDir(const Cell& cur, const Cell& dst) const
{
    int dx = dst.x - cur.x;
    int dy = dst.y - cur.y;
    Direction base = getDirectionFromDelta(dx, dy);
    
    // Sweep right-first: right, then backward
    Direction backward = static_cast<Direction>((static_cast<int>(base) + 2) % 4);
    Direction order[2] = {turnRight(base), backward};
    
    for (auto dir : order) {
        Cell next = step(cur, dir);
        
        // Check if can move
        if (canMoveBetween(cur, next) && isFree(next)) {
            return dir;
        }
    }
    return base;  // fallback
}

MazeRouter100x100::Decision MazeRouter100x100::routeStep(const Cell& current, const Cell& goal, Direction incomingDir)
{
    // Update current position
    currentPos = current;
    
    // Line 1-2: Destination reached
    if (current.x == goal.x && current.y == goal.y) {
        return {Direction::LOCAL, false, Mode::NORMAL};
    }
    
    int currentMD = manhattanDistance(current, goal);
    auto productiveDirs = getProductiveDirections(current, goal);
    
    // Line 3-6: Normal mode - move productively if possible
    if (state.mode == Mode::NORMAL && !productiveDirs.empty() && state.MDbest == currentMD) {
        state.MDbest = currentMD - 1;
        state.mode = Mode::NORMAL;
        return {productiveDirs.front(), false, Mode::NORMAL};
    }
    
    // Line 7-8: Traversal mode - simple wall following
    if (state.mode == Mode::TRAVERSAL_LEFT || state.mode == Mode::TRAVERSAL_RIGHT) {
        // Get current heading
        Direction heading = state.traversalHeading;
        
        // Calculate hand-turn direction
        Direction handTurn = (state.mode == Mode::TRAVERSAL_LEFT) ? 
                            turnLeft(heading) : turnRight(heading);
        
        // Calculate back-turn direction
        Direction backTurn = (state.mode == Mode::TRAVERSAL_LEFT) ? 
                            turnLeft(handTurn) : turnRight(handTurn);
        
        // Calculate hand-turn of back-turn
        Direction handTurnOfBackTurn = (state.mode == Mode::TRAVERSAL_LEFT) ? 
                                     turnLeft(backTurn) : turnRight(backTurn);
        
        // Try heading first, then hand-turn, then back-turn, then hand-turn of back-turn
        Direction order[4] = {heading, handTurn, backTurn, handTurnOfBackTurn};
        
        for (auto dir : order) {
            Cell next = step(current, dir);
            
            // Check if can move (wall check)
            if (canMoveBetween(current, next) && isFree(next)) {
                // Update heading based on which direction worked
                if (dir == heading) {
                    state.traversalHeading = state.leftHand ? turnRight(heading) : turnLeft(heading);
                }
                if (dir == backTurn) {
                    state.traversalHeading = state.leftHand ? turnLeft(heading) : turnRight(heading);
                }
                if (dir == handTurnOfBackTurn) {
                    state.traversalHeading = state.leftHand ? backTurn : backTurn;
                }
                
                // Exit condition: if at same MD as best and productive exists, exit to NORMAL next hop
                int nextMD = manhattanDistance(next, goal);
                
                // Unreachable test: revisiting start with same outgoing dir
                if (state.mark.set && next.x == state.mark.start.x && 
                    next.y == state.mark.start.y && dir == state.mark.firstDir &&
                    nextMD >= state.MDbest) {
                    return {dir, true, state.mode};
                }
                if (nextMD == state.MDbest && !getProductiveDirections(next, goal).empty()) {
                    state.mode = Mode::NORMAL;
                    state.mark.set = false;
                }
                return {dir, false, state.mode};
            }
        }
        // If truly boxed: dead end
        Direction backDir = static_cast<Direction>((static_cast<int>(incomingDir) + 2) % 4);
        return {backDir, false, state.mode};
    }
    
    // Line 9-11: Enter traversal mode
    Direction goalDir = getDirectionFromDelta(goal.x - current.x, goal.y - current.y);
    Direction incomingDirLeft = turnLeft(incomingDir);
    Direction incomingDirRight = turnRight(incomingDir);
    
    // Choose LHR if goal is to the left of incoming direction, RHR if to the right
    state.leftHand = (goalDir == incomingDirLeft);
    
    state.mode = state.leftHand ? Mode::TRAVERSAL_LEFT : Mode::TRAVERSAL_RIGHT;
    Direction firstDir = state.leftHand ? getLeftHandRuleDir(current, goal) : getRightHandRuleDir(current, goal);
    
    // Set traversal heading based on hand rule
    state.traversalHeading = state.leftHand ? turnRight(firstDir) : turnLeft(firstDir);
    state.mark = {current, firstDir, true};
    
    return {firstDir, false, state.mode};
}

bool MazeRouter100x100::isGoalReachable() const
{
    // Simple check: if we can find any path using BFS
    std::vector<std::vector<bool>> visited(gridSize, std::vector<bool>(gridSize, false));
    std::vector<Cell> queue;
    
    queue.push_back(currentPos);
    visited[currentPos.x][currentPos.y] = true;
    
    while (!queue.empty()) {
        Cell current = queue.front();
        queue.erase(queue.begin());
        
        if (current.x == goalPos.x && current.y == goalPos.y) {
            return true;
        }
        
        for (int i = 0; i < 4; ++i) {
            Direction dir = static_cast<Direction>(i);
            Cell next = step(current, dir);
            
            if (next.x >= 0 && next.x < gridSize && next.y >= 0 && next.y < gridSize &&
                !visited[next.x][next.y]) {
                
                // Check if can move (wall check)
                if (canMoveBetween(current, next) && isFree(next)) {
                    visited[next.x][next.y] = true;
                    queue.push_back(next);
                }
            }
        }
    }
    
    return false;
}

std::string MazeRouter100x100::getModeString() const
{
    switch (state.mode) {
        case Mode::NORMAL: return "NORMAL";
        case Mode::TRAVERSAL_LEFT: return "TRAVERSAL_LEFT";
        case Mode::TRAVERSAL_RIGHT: return "TRAVERSAL_RIGHT";
        default: return "UNKNOWN";
    }
}

std::string MazeRouter100x100::getDirectionString(Direction dir) const
{
    switch (dir) {
        case Direction::NORTH: return "NORTH";
        case Direction::EAST: return "EAST";
        case Direction::SOUTH: return "SOUTH";
        case Direction::WEST: return "WEST";
        case Direction::LOCAL: return "LOCAL";
        default: return "UNKNOWN";
    }
}
