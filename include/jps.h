#pragma once

#include "grid_map.h"
#include "heuristic.h"
#include "a_star.h"
#include <vector>

class JPS {
public:
    struct Direction {
        int dx;
        int dy;
        Direction(int dx_ = 0, int dy_ = 0) : dx(dx_), dy(dy_) {}
        bool operator==(const Direction& other) const {
            return dx == other.dx && dy == other.dy;
        }
    };

    JPS(GridMap& grid, HeuristicType heuristic = HeuristicType::Octile);

    SearchResult search(int startX, int startY, int goalX, int goalY);

private:
    GridMap& grid_;
    HeuristicType heuristicType_;

    Node* jump(int x, int y, int dx, int dy, int goalX, int goalY);

    bool hasForcedNeighbor(int x, int y, int dx, int dy);

    std::vector<std::pair<Direction, Node*>> identifySuccessors(Node* current, int goalX, int goalY);

    std::vector<Node*> reconstructPath(Node* goalNode);
};
