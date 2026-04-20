#include "a_star.h"
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <iostream>

AStar::AStar(GridMap& grid, HeuristicType heuristic, bool allowDiagonal)
    : grid_(grid), heuristicType_(heuristic), allowDiagonal_(allowDiagonal) {}

SearchResult AStar::search(int startX, int startY, int goalX, int goalY) {
    SearchResult result;
    result.algorithmName = "A*";

    auto startTime = std::chrono::high_resolution_clock::now();

    grid_.resetNodes();

    Node* startNode = grid_.getNode(startX, startY);
    Node* goalNode = grid_.getNode(goalX, goalY);

    if (!startNode || !goalNode) {
        result.found = false;
        return result;
    }

    if (!grid_.isWalkable(startX, startY) || !grid_.isWalkable(goalX, goalY)) {
        result.found = false;
        return result;
    }

    startNode->g = 0.0;
    startNode->h = Heuristic::calculate(startX, startY, goalX, goalY, heuristicType_);
    startNode->updateF();
    startNode->state = NodeState::Open;

    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> openList;

    // TODO: 为什么我们需要一个额外的数据结构来检查节点是否在 Open List 中？
    // 提示：priority_queue 不支持遍历和查找操作
    // 请你自己考虑是否需要添加，以及添加什么数据结构
    openList.push(startNode);

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        if (current->state == NodeState::Closed) {
            continue;
        }

        current->state = NodeState::Closed;
        result.nodesExpanded++;

        if (current->x == goalX && current->y == goalY) {
            result.path = reconstructPath(goalNode);
            result.found = true;
            result.pathCost = goalNode->g;
            break;
        }

        std::vector<Node*> neighbors = grid_.getNeighbors(current, allowDiagonal_);

        for (Node* neighbor : neighbors) {
            if (neighbor->state == NodeState::Closed) {
                continue;
            }

            double tentative_g = current->g + grid_.getMoveCost(current, neighbor);

            // TODO: 这里有一个关键的判断条件需要你思考
            // 什么时候应该更新邻居节点？
            // 选项A：tentative_g < neighbor->g
            // 选项B：tentative_g <= neighbor->g
            // 选项C：neighbor->state == NodeState::Unvisited || tentative_g < neighbor->g
            // 请思考每种选项的影响，然后选择正确的条件！
            if (neighbor->state == NodeState::Unvisited || tentative_g < neighbor->g) {
                neighbor->parent = current;
                neighbor->g = tentative_g;
                neighbor->h = Heuristic::calculate(neighbor->x, neighbor->y, goalX, goalY, heuristicType_);
                neighbor->updateF();

                if (neighbor->state == NodeState::Unvisited) {
                    neighbor->state = NodeState::Open;
                    openList.push(neighbor);
                } else {
                    // TODO: 当节点已经在 Open List 中但找到了更优路径时，
                    // 我们需要做什么？priority_queue 不支持修改操作！
                    // 提示：一种常见的做法是直接再 push 一次，
                    // 然后在取出时通过 state 检查跳过重复节点
                    openList.push(neighbor);
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.searchTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    return result;
}

std::vector<Node*> AStar::reconstructPath(Node* goalNode) {
    std::vector<Node*> path;
    Node* current = goalNode;

    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
