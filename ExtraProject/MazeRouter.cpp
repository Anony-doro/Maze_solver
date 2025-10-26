#include "MazeRouter.hpp"
#include <algorithm>
#include <climits>
#include <iostream>

MazeRouter::MazeRouter(const std::vector<std::vector<int>>& mazeGrid)
    : maze(mazeGrid), mazeSize(static_cast<int>(mazeGrid.size())) {
    reset();
}

void MazeRouter::reset() {
    state = RoutingState();
    currentPos = Cell(0, 0);
    goalPos = Cell(0, 0);
}

void MazeRouter::initializeRouting(const Cell& start, const Cell& goal) {
    currentPos = start;
    goalPos = goal;
    state = RoutingState();
    state.MDbest = manhattanDistance(start, goal);
}

int MazeRouter::manhattanDistance(const Cell& from, const Cell& to) const {
    return std::abs(from.x - to.x) + std::abs(from.y - to.y);
}

MazeRouter::Cell MazeRouter::step(const Cell& cell, Direction dir) const {
    static const int dx[4] = {0, 1, 0, -1};  // N, E, S, W
    static const int dy[4] = {-1, 0, 1, 0};
    
    if (dir == Direction::LOCAL) {
        return cell;
    }
    
    int dirIdx = static_cast<int>(dir);
    return Cell(cell.x + dx[dirIdx], cell.y + dy[dirIdx]);
}

bool MazeRouter::isFree(const Cell& cell) const {
    if (cell.x < 0 || cell.x >= mazeSize || cell.y < 0 || cell.y >= mazeSize) {
        return false;
    }
    return maze[cell.x][cell.y] == FREE || maze[cell.x][cell.y] == GOAL;
}

std::vector<MazeRouter::Direction> MazeRouter::getProductiveDirections(const Cell& cur, const Cell& dst) const {
    std::vector<Direction> productiveDirs;
    int currentMD = manhattanDistance(cur, dst);
    
    for (int i = 0; i < 4; ++i) {
        Direction dir = static_cast<Direction>(i);
        Cell next = step(cur, dir);
        if (isFree(next) && manhattanDistance(next, dst) == currentMD - 1) {
            productiveDirs.push_back(dir);
        }
    }
    
    return productiveDirs;
}

MazeRouter::Direction MazeRouter::turnLeft(Direction dir) const {
    return static_cast<Direction>((static_cast<int>(dir) + 3) % 4);
}

MazeRouter::Direction MazeRouter::turnRight(Direction dir) const {
    return static_cast<Direction>((static_cast<int>(dir) + 1) % 4);
}

// MazeRouter::Direction MazeRouter::turnBack(Direction dir) const {
//     return static_cast<Direction>((static_cast<int>(dir) + 2) % 4);
// }   for now over engineering

MazeRouter::Direction MazeRouter::getDirectionFromDelta(int dx, int dy) const {
    if (std::abs(dx) >= std::abs(dy)) {
        return (dx > 0) ? Direction::EAST : Direction::WEST;
    } else {
        return (dy > 0) ? Direction::SOUTH : Direction::NORTH;
    }
}

MazeRouter::Direction MazeRouter::getLeftHandRuleDir(const Cell& cur, const Cell& dst) const {
    int dx = dst.x - cur.x;
    int dy = dst.y - cur.y;
    Direction base = getDirectionFromDelta(dx, dy); //always the same direction forward from normal mode
    // Sweep left-first: left, then backward
    Direction backward = static_cast<Direction>((static_cast<int>(base) + 2) % 4);
    Direction order[2] = {turnLeft(base), backward};
    
    for (auto dir : order) {
        if (isFree(step(cur, dir))) {
            return dir;
        }
    }
    return base; // fallback
}

MazeRouter::Direction MazeRouter::getRightHandRuleDir(const Cell& cur, const Cell& dst) const {
    int dx = dst.x - cur.x;
    int dy = dst.y - cur.y;
    Direction base = getDirectionFromDelta(dx, dy);
    
    // Sweep right-first: right, then backward
    Direction backward = static_cast<Direction>((static_cast<int>(base) + 2) % 4);
    Direction order[2] = {turnRight(base), backward};
    
    for (auto dir : order) {
        if (isFree(step(cur, dir))) {
            return dir;
        }
    }
    return base; // fallback
}

MazeRouter::Decision MazeRouter::routeStep(const Cell& current, const Cell& goal, Direction incomingDir) {
    // Update current position
    currentPos = current;
    
    // Line 1-2: Destination reached
    if (current.x == goal.x && current.y == goal.y) {
        return {Direction::LOCAL, false, Mode::NORMAL};
    }
    
    int currentMD = manhattanDistance(current, goal);
    auto productiveDirs = getProductiveDirections(current, goal); //type vector<Direction>
    
    // Line 3-6: Normal mode - move productively if possible
    if (state.mode == Mode::NORMAL && !productiveDirs.empty() && state.MDbest == currentMD) { //The robot should only try to move productively when it's at the best distance it has ever reached. This prevents:
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
            if (isFree(next)) {
                // Update heading based on which direction worked and hand rule      // importatnt! this makes it possible to tackle anykind of structure for wall following by actively switching the traversalheading dir
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
                // Only declare unreachable if we haven't improved our position
                if (state.mark.set && next.x == state.mark.start.x && 
                    next.y == state.mark.start.y && dir == state.mark.firstDir &&
                    nextMD >= state.MDbest) {  // Only if we haven't improved our position
                    return {dir, true, state.mode};
                }
                if (nextMD == state.MDbest && !getProductiveDirections(next, goal).empty()) {
                    state.mode = Mode::NORMAL;
                    state.mark.set = false;  // Reset the traversal mark, debug for infinite loop
                }
                return {dir, false, state.mode};
            }
        }
        // If truly boxed: dead end
        Direction backDir = static_cast<Direction>((static_cast<int>(incomingDir) + 2) % 4);
        return {backDir, false, state.mode};
    }
    
    // Line 9-11: Enter traversal mode
    // Choose hand rule based on goal direction
    Direction goalDir = getDirectionFromDelta(goal.x - current.x, goal.y - current.y);
    Direction incomingDirLeft = turnLeft(incomingDir);
    Direction incomingDirRight = turnRight(incomingDir);
    
    // Choose LHR if goal is to the left of incoming direction, RHR if to the right
    state.leftHand = (goalDir == incomingDirLeft);
    
    state.mode = state.leftHand ? Mode::TRAVERSAL_LEFT : Mode::TRAVERSAL_RIGHT;
    Direction firstDir = state.leftHand ? getLeftHandRuleDir(current, goal) : getRightHandRuleDir(current, goal);
    
    // Set traversal heading based on hand rule
    state.traversalHeading = state.leftHand ? turnRight(firstDir) : turnLeft(firstDir);
    state.mark = {current, firstDir, true}; // remember Ntrav & DIRtrav
    
    return {firstDir, false, state.mode};
}

bool MazeRouter::isGoalReachable() const {
    // Simple check: if we can find any path using BFS
    std::vector<std::vector<bool>> visited(mazeSize, std::vector<bool>(mazeSize, false));
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
            
            if (next.x >= 0 && next.x < mazeSize && next.y >= 0 && next.y < mazeSize &&
                !visited[next.x][next.y] && isFree(next)) {
                visited[next.x][next.y] = true;
                queue.push_back(next);
            }
        }
    }
    
    return false;
}

std::string MazeRouter::getModeString() const {
    switch (state.mode) {
        case Mode::NORMAL: return "NORMAL";
        case Mode::TRAVERSAL_LEFT: return "TRAVERSAL_LEFT";
        case Mode::TRAVERSAL_RIGHT: return "TRAVERSAL_RIGHT";
        default: return "UNKNOWN";
    }
}

std::string MazeRouter::getDirectionString(Direction dir) const {
    switch (dir) {
        case Direction::NORTH: return "NORTH";
        case Direction::EAST: return "EAST";
        case Direction::SOUTH: return "SOUTH";
        case Direction::WEST: return "WEST";
        case Direction::LOCAL: return "LOCAL";
        default: return "UNKNOWN";
    }
}