#pragma once

#include <cmath>
#include <limits>

enum class NodeState {
    Unvisited,
    Open,
    Closed
};

struct Node {
    int x;
    int y;
    double g;
    double h;
    double f;
    Node* parent;
    NodeState state;

    Node(int x_ = 0, int y_ = 0)
        : x(x_), y(y_),
          g(std::numeric_limits<double>::infinity()),
          h(0.0),
          f(std::numeric_limits<double>::infinity()),
          parent(nullptr),
          state(NodeState::Unvisited) {}

    void updateF() {
        f = g + h;
    }

    void reset() {
        g = std::numeric_limits<double>::infinity();
        h = 0.0;
        f = std::numeric_limits<double>::infinity();
        parent = nullptr;
        state = NodeState::Unvisited;
    }

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Node& other) const {
        return !(*this == other);
    }
};

struct NodeCompare {
    bool operator()(const Node* a, const Node* b) const {
        if (a->f != b->f) return a->f > b->f;
        if (a->h != b->h) return a->h > b->h;
        return (a->x + a->y) > (b->x + b->y);
    }
};
