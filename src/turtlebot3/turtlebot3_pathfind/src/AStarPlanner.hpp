#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Node {
    int x, y;
    float g, h, f;
    Node* parent;
    
    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}
};

struct PathSegment {
    enum Type { FORWARD, TURN_LEFT, TURN_RIGHT } type;
    float distance;  // in meters for FORWARD, radians for TURN
};

class AStarPlanner {
public:
    AStarPlanner(int width, int height);
    std::vector<Node*> findPath(int startX, int startY, int goalX, int goalY, const std::vector<std::vector<bool>>& grid);
    std::vector<PathSegment> convertPathToCommands(const std::vector<Node*>& path, float resolution);
    
private:
    int width, height;
    float calculateH(int x, int y, int goalX, int goalY);
    std::vector<Node*> getNeighbors(Node* node, const std::vector<std::vector<bool>>& grid);
    std::vector<Node*> reconstructPath(Node* current);
};