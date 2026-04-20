#pragma once

#include "node.h"
#include <vector>
#include <string>

enum class CellType {
    Empty,
    Obstacle
};

class GridMap {
public:
    GridMap(int width, int height);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    bool isValid(int x, int y) const;
    bool isWalkable(int x, int y) const;
    bool isObstacle(int x, int y) const;

    void setObstacle(int x, int y);
    void clearObstacle(int x, int y);

    Node* getNode(int x, int y);
    const Node* getNode(int x, int y) const;

    void resetNodes();

    std::vector<Node*> getNeighbors(const Node* node, bool allowDiagonal = true);

    double getMoveCost(const Node* from, const Node* to) const;

    void loadFromFile(const std::string& filename);
    void saveToFile(const std::string& filename) const;

    void print() const;

private:
    int width_;
    int height_;
    std::vector<std::vector<CellType>> grid_;
    std::vector<std::vector<Node>> nodes_;

    void initNodes();
};
