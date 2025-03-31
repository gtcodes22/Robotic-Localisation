#include "AStarPlanner.hpp"

AStarPlanner::AStarPlanner(int width, int height) : width(width), height(height) {}

float AStarPlanner::calculateH(int x, int y, int goalX, int goalY) {
    return std::sqrt(std::pow(x - goalX, 2) + std::pow(y - goalY, 2));
}

std::vector<Node*> AStarPlanner::getNeighbors(Node* node, const std::vector<std::vector<bool>>& grid) {
    std::vector<Node*> neighbors;
    
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = node->x + dx;
            int ny = node->y + dy;
            
            if (nx >= 0 && nx < width && ny >= 0 && ny < height && !grid[nx][ny]) {
                neighbors.push_back(new Node(nx, ny));
            }
        }
    }
    
    return neighbors;
}

std::vector<Node*> AStarPlanner::reconstructPath(Node* current) {
    std::vector<Node*> path;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node*> AStarPlanner::findPath(int startX, int startY, int goalX, int goalY, const std::vector<std::vector<bool>>& grid) {
    std::vector<Node*> openSet;
    std::vector<std::vector<bool>> closedSet(width, std::vector<bool>(height, false));
    
    Node* startNode = new Node(startX, startY);
    startNode->h = calculateH(startX, startY, goalX, goalY);
    startNode->f = startNode->h;
    openSet.push_back(startNode);
    
    while (!openSet.empty()) {
        auto it = std::min_element(openSet.begin(), openSet.end(), 
            [](Node* a, Node* b) { return a->f < b->f; });
        Node* current = *it;
        
        if (current->x == goalX && current->y == goalY) {
            return reconstructPath(current);
        }
        
        openSet.erase(it);
        closedSet[current->x][current->y] = true;
        
        for (Node* neighbor : getNeighbors(current, grid)) {
            if (closedSet[neighbor->x][neighbor->y]) {
                delete neighbor;
                continue;
            }
            
            float tentativeG = current->g + 1;
            
            bool inOpenSet = std::find_if(openSet.begin(), openSet.end(), 
                [neighbor](Node* n) { return n->x == neighbor->x && n->y == neighbor->y; }) != openSet.end();
            
            if (!inOpenSet || tentativeG < neighbor->g) {
                neighbor->parent = current;
                neighbor->g = tentativeG;
                neighbor->h = calculateH(neighbor->x, neighbor->y, goalX, goalY);
                neighbor->f = neighbor->g + neighbor->h;
                
                if (!inOpenSet) {
                    openSet.push_back(neighbor);
                }
            } else {
                delete neighbor;
            }
        }
    }
    
    return {}; // Return empty path if no path found
}

std::vector<PathSegment> AStarPlanner::convertPathToCommands(const std::vector<Node*>& path, float resolution) {
    std::vector<PathSegment> commands;
    if (path.size() < 2) return commands;

    Node* prev = path[0];
    Node* current = path[1];
    
    // Initial orientation (facing positive x-axis)
    float current_angle = 0;
    
    for (size_t i = 1; i < path.size(); i++) {
        current = path[i];
        
        // Calculate direction vector
        int dx = current->x - prev->x;
        int dy = current->y - prev->y;
        
        // Calculate desired angle (in radians)
        float desired_angle = atan2(dy, dx);
        
        // Calculate angle difference
        float angle_diff = desired_angle - current_angle;
        
        // Normalize angle to [-π, π]
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;
        
        // Add turn command if needed
        if (fabs(angle_diff) > 0.1) { // 0.1 rad (~5.7°) threshold
            PathSegment turn;
            turn.type = (angle_diff > 0) ? PathSegment::TURN_LEFT : PathSegment::TURN_RIGHT;
            turn.distance = fabs(angle_diff);
            commands.push_back(turn);
            current_angle = desired_angle;
        }
        
        // Add forward command
        PathSegment forward;
        forward.type = PathSegment::FORWARD;
        forward.distance = resolution * sqrt(dx*dx + dy*dy);
        commands.push_back(forward);
        
        prev = current;
    }
    
    return commands;
}