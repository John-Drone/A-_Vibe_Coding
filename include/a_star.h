#pragma once

#include "grid_map.h"
#include "heuristic.h"
#include <vector>
#include <chrono>

struct SearchResult {
    std::vector<Node*> path;
    bool found;
    double pathCost;
    int nodesExpanded;
    double searchTimeMs;
    std::string algorithmName;

    SearchResult()
        : found(false), pathCost(0.0),
          nodesExpanded(0), searchTimeMs(0.0) {}
};

class AStar {
public:
    AStar(GridMap& grid, HeuristicType heuristic = HeuristicType::Octile,
          bool allowDiagonal = true);

    SearchResult search(int startX, int startY, int goalX, int goalY);

private:
    GridMap& grid_;
    HeuristicType heuristicType_;
    bool allowDiagonal_;

    std::vector<Node*> reconstructPath(Node* goalNode);
};
