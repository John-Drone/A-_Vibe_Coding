#include "jps.h"
#include <queue>
#include <chrono>
#include <algorithm>
#include <iostream>

JPS::JPS(GridMap& grid, HeuristicType heuristic)
    : grid_(grid), heuristicType_(heuristic) {}

SearchResult JPS::search(int startX, int startY, int goalX, int goalY) {
    SearchResult result;
    result.algorithmName = "JPS";

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

        auto successors = identifySuccessors(current, goalX, goalY);

        for (auto& [dir, successor] : successors) {
            if (successor->state == NodeState::Closed) {
                continue;
            }

            double moveCost = Heuristic::calculate(
                current->x, current->y, successor->x, successor->y, heuristicType_);

            // TODO: JPS 的代价计算和 A* 有什么不同？
            // 提示：跳点之间可能跨越多个格子，所以代价不是简单的 1 或 sqrt(2)
            // 这里用启发函数近似直线距离，但更精确的做法是什么？
            double tentative_g = current->g + moveCost;

            if (successor->state == NodeState::Unvisited || tentative_g < successor->g) {
                successor->parent = current;
                successor->g = tentative_g;
                successor->h = Heuristic::calculate(
                    successor->x, successor->y, goalX, goalY, heuristicType_);
                successor->updateF();

                if (successor->state == NodeState::Unvisited) {
                    successor->state = NodeState::Open;
                }
                openList.push(successor);
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.searchTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    return result;
}

Node* JPS::jump(int x, int y, int dx, int dy, int goalX, int goalY) {
    int nx = x + dx;
    int ny = y + dy;

    if (!grid_.isWalkable(nx, ny)) {
        return nullptr;
    }

    if (nx == goalX && ny == goalY) {
        return grid_.getNode(nx, ny);
    }

    if (hasForcedNeighbor(nx, ny, dx, dy)) {
        return grid_.getNode(nx, ny);
    }

    // 对角线移动时，需要同时检查水平和垂直方向
    if (dx != 0 && dy != 0) {
        // TODO: 对角线跳跃时，为什么要分别检查水平方向和垂直方向？
        // 提示：如果水平方向存在跳点，那么当前点也是一个跳点
        // 因为从当前点需要改变方向才能到达那个跳点
        if (jump(nx, ny, dx, 0, goalX, goalY) != nullptr ||
            jump(nx, ny, 0, dy, goalX, goalY) != nullptr) {
            return grid_.getNode(nx, ny);
        }
    }

    return jump(nx, ny, dx, dy, goalX, goalY);
}

bool JPS::hasForcedNeighbor(int x, int y, int dx, int dy) {
    if (dx != 0 && dy != 0) {
        // 对角线移动时的被迫邻居检查
        // TODO: 对角线移动时，什么情况下会出现被迫邻居？
        // 提示：如果 (x-dx, y) 是障碍且 (x-dx, y+dy) 可通行，
        // 那么 (x-dx, y+dy) 是被迫邻居，当前点是跳点
        // 同理检查另一个方向
        // 请你自己完成这个逻辑！
        bool neighbor1 = grid_.isObstacle(x - dx, y) && grid_.isWalkable(x - dx, y + dy);
        bool neighbor2 = grid_.isObstacle(x, y - dy) && grid_.isWalkable(x + dx, y - dy);
        return neighbor1 || neighbor2;
    } else if (dx != 0) {
        // 水平移动
        // TODO: 水平移动时，被迫邻居在哪个方向？
        // 提示：如果上方是障碍且斜上方可通行，那就是被迫邻居
        bool top = grid_.isObstacle(x, y - 1) && grid_.isWalkable(x + dx, y - 1);
        bool bottom = grid_.isObstacle(x, y + 1) && grid_.isWalkable(x + dx, y + 1);
        return top || bottom;
    } else {
        // 垂直移动
        // TODO: 垂直移动时，被迫邻居在哪个方向？
        bool left = grid_.isObstacle(x - 1, y) && grid_.isWalkable(x - 1, y + dy);
        bool right = grid_.isObstacle(x + 1, y) && grid_.isWalkable(x + 1, y + dy);
        return left || right;
    }
}

std::vector<std::pair<JPS::Direction, Node*>> JPS::identifySuccessors(Node* current, int goalX, int goalY) {
    std::vector<std::pair<Direction, Node*>> successors;

    std::vector<Direction> directions;

    if (current->parent == nullptr) {
        // 起点：检查所有8个方向
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                directions.emplace_back(dx, dy);
            }
        }
    } else {
        // 非起点：只检查当前移动方向的自然邻居和被迫邻居方向
        int pdx = current->x - current->parent->x;
        int pdy = current->y - current->parent->y;

        // 归一化方向
        int dx = (pdx > 0) ? 1 : ((pdx < 0) ? -1 : 0);
        int dy = (pdy > 0) ? 1 : ((pdy < 0) ? -1 : 0);

        directions.emplace_back(dx, dy);

        // 对角线移动时添加分方向
        if (dx != 0 && dy != 0) {
            directions.emplace_back(dx, 0);
            directions.emplace_back(0, dy);
        } else if (dx != 0) {
            // TODO: 水平移动时，什么方向需要添加？
            // 提示：检查上下是否有被迫邻居
            if (grid_.isObstacle(current->x, current->y - 1)) {
                directions.emplace_back(dx, -1);
            }
            if (grid_.isObstacle(current->x, current->y + 1)) {
                directions.emplace_back(dx, 1);
            }
        } else {
            // TODO: 垂直移动时，什么方向需要添加？
            if (grid_.isObstacle(current->x - 1, current->y)) {
                directions.emplace_back(-1, dy);
            }
            if (grid_.isObstacle(current->x + 1, current->y)) {
                directions.emplace_back(1, dy);
            }
        }
    }

    for (const auto& dir : directions) {
        Node* jumpNode = jump(current->x, current->y, dir.dx, dir.dy, goalX, goalY);
        if (jumpNode != nullptr) {
            successors.emplace_back(dir, jumpNode);
        }
    }

    return successors;
}

std::vector<Node*> JPS::reconstructPath(Node* goalNode) {
    std::vector<Node*> path;
    Node* current = goalNode;

    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }

    std::reverse(path.begin(), path.end());

    // TODO: JPS 返回的路径是跳点序列，中间有间隔
    // 如果需要完整的逐格路径，需要在这里进行插值
    // 即在相邻跳点之间填充中间的所有格子
    // 你可以自己实现这个功能，或者直接使用跳点路径
    // 提示：从 path[i] 到 path[i+1]，沿直线方向逐步填充

    return path;
}
