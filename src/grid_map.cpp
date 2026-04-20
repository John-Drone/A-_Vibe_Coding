#include "grid_map.h"
#include <fstream>
#include <iostream>
#include <algorithm>

GridMap::GridMap(int width, int height)
    : width_(width), height_(height),
      grid_(height, std::vector<CellType>(width, CellType::Empty)),
      nodes_(height, std::vector<Node>(width, Node()))
{
    initNodes();
}

void GridMap::initNodes() {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            nodes_[y][x] = Node(x, y);
        }
    }
}

bool GridMap::isValid(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridMap::isWalkable(int x, int y) const {
    if (!isValid(x, y)) return false;
    return grid_[y][x] == CellType::Empty;
}

bool GridMap::isObstacle(int x, int y) const {
    if (!isValid(x, y)) return true;
    return grid_[y][x] == CellType::Obstacle;
}

void GridMap::setObstacle(int x, int y) {
    if (isValid(x, y)) {
        grid_[y][x] = CellType::Obstacle;
    }
}

void GridMap::clearObstacle(int x, int y) {
    if (isValid(x, y)) {
        grid_[y][x] = CellType::Empty;
    }
}

Node* GridMap::getNode(int x, int y) {
    if (!isValid(x, y)) return nullptr;
    return &nodes_[y][x];
}

const Node* GridMap::getNode(int x, int y) const {
    if (!isValid(x, y)) return nullptr;
    return &nodes_[y][x];
}

void GridMap::resetNodes() {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            nodes_[y][x].reset();
        }
    }
}

std::vector<Node*> GridMap::getNeighbors(const Node* node, bool allowDiagonal) {
    std::vector<Node*> neighbors;

    static const int dx4[] = {0, 1, 0, -1};
    static const int dy4[] = {-1, 0, 1, 0};

    static const int dx8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    static const int dy8[] = {-1, 0, 1, -1, 1, -1, 0, 1};

    if (allowDiagonal) {
        for (int i = 0; i < 8; ++i) {
            int nx = node->x + dx8[i];
            int ny = node->y + dy8[i];
            if (isWalkable(nx, ny)) {
                // TODO: 对角线移动时，需要检查是否可以"穿过"角落
                // 提示：如果从 (x,y) 对角移动到 (x+dx, y+dy)，
                // 那么 (x+dx, y) 和 (x, y+dy) 都不能是障碍物
                // 请你自己实现这个检查逻辑！
                neighbors.push_back(getNode(nx, ny));
            }
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            int nx = node->x + dx4[i];
            int ny = node->y + dy4[i];
            if (isWalkable(nx, ny)) {
                neighbors.push_back(getNode(nx, ny));
            }
        }
    }

    return neighbors;
}

double GridMap::getMoveCost(const Node* from, const Node* to) const {
    int dx = std::abs(to->x - from->x);
    int dy = std::abs(to->y - from->y);

    if (dx + dy == 2) {
        return std::sqrt(2.0);
    }
    return 1.0;
}

void GridMap::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open map file: " << filename << std::endl;
        return;
    }

    for (int y = 0; y < height_; ++y) {
        std::string line;
        if (!std::getline(file, line)) break;
        for (int x = 0; x < width_ && x < static_cast<int>(line.size()); ++x) {
            if (line[x] == '#' || line[x] == '1') {
                setObstacle(x, y);
            }
        }
    }
    file.close();
}

void GridMap::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to save map file: " << filename << std::endl;
        return;
    }

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            file << (grid_[y][x] == CellType::Obstacle ? '#' : '.');
        }
        file << '\n';
    }
    file.close();
}

void GridMap::print() const {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            std::cout << (grid_[y][x] == CellType::Obstacle ? "#" : ".");
        }
        std::cout << '\n';
    }
}
