#include "visualizer.h"
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <cmath>

void Visualizer::printGrid(const GridMap& grid,
                            const std::vector<Node*>& path,
                            const Node* start,
                            const Node* goal) {
    std::unordered_set<int> pathSet;
    for (Node* n : path) {
        pathSet.insert(n->y * grid.getWidth() + n->x);
    }

    std::cout << "  ";
    for (int x = 0; x < grid.getWidth(); ++x) {
        std::cout << x % 10;
    }
    std::cout << "\n";

    for (int y = 0; y < grid.getHeight(); ++y) {
        std::cout << y % 10 << " ";
        for (int x = 0; x < grid.getWidth(); ++x) {
            if (start && x == start->x && y == start->y) {
                std::cout << "S";
            } else if (goal && x == goal->x && y == goal->y) {
                std::cout << "G";
            } else if (pathSet.count(y * grid.getWidth() + x)) {
                std::cout << "*";
            } else if (grid.isObstacle(x, y)) {
                std::cout << "#";
            } else {
                std::cout << ".";
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

void Visualizer::printSearchResult(const SearchResult& result) {
    std::cout << "=== " << result.algorithmName << " Search Result ===\n";
    std::cout << "Found: " << (result.found ? "Yes" : "No") << "\n";
    if (result.found) {
        std::cout << "Path length: " << result.path.size() << " nodes\n";
        std::cout << "Path cost: " << std::fixed << std::setprecision(2) << result.pathCost << "\n";
    }
    std::cout << "Nodes expanded: " << result.nodesExpanded << "\n";
    std::cout << "Search time: " << std::fixed << std::setprecision(3) << result.searchTimeMs << " ms\n";
    std::cout << "\n";
}

void Visualizer::compareResults(const SearchResult& astarResult,
                                 const SearchResult& jpsResult) {
    std::cout << "=== Performance Comparison ===\n";
    std::cout << std::left << std::setw(25) << "Metric"
              << std::setw(15) << "A*" << std::setw(15) << "JPS" << "\n";
    std::cout << std::string(55, '-') << "\n";

    std::cout << std::left << std::setw(25) << "Path found"
              << std::setw(15) << (astarResult.found ? "Yes" : "No")
              << std::setw(15) << (jpsResult.found ? "Yes" : "No") << "\n";

    std::cout << std::left << std::setw(25) << "Path cost"
              << std::setw(15) << std::fixed << std::setprecision(2) << astarResult.pathCost
              << std::setw(15) << jpsResult.pathCost << "\n";

    std::cout << std::left << std::setw(25) << "Nodes expanded"
              << std::setw(15) << astarResult.nodesExpanded
              << std::setw(15) << jpsResult.nodesExpanded << "\n";

    double ratio = (astarResult.nodesExpanded > 0)
                       ? (double)jpsResult.nodesExpanded / astarResult.nodesExpanded * 100.0
                       : 0.0;
    std::cout << std::left << std::setw(25) << "JPS/A* expand ratio"
              << std::setw(15) << "100%"
              << std::setw(15) << (std::to_string((int)ratio) + "%") << "\n";

    std::cout << std::left << std::setw(25) << "Search time (ms)"
              << std::setw(15) << std::fixed << std::setprecision(3) << astarResult.searchTimeMs
              << std::setw(15) << jpsResult.searchTimeMs << "\n";

    double timeRatio = (astarResult.searchTimeMs > 0)
                           ? jpsResult.searchTimeMs / astarResult.searchTimeMs * 100.0
                           : 0.0;
    std::cout << std::left << std::setw(25) << "JPS/A* time ratio"
              << std::setw(15) << "100%"
              << std::setw(15) << (std::to_string((int)timeRatio) + "%") << "\n";

    std::cout << "\n";
}

void Visualizer::printComparisonTable(
    const std::vector<std::string>& mapNames,
    const std::vector<SearchResult>& astarResults,
    const std::vector<SearchResult>& jpsResults) {

    std::cout << "\n=== Comprehensive Comparison Table ===\n\n";
    std::cout << std::left
              << std::setw(15) << "Map"
              << std::setw(10) << "Algo"
              << std::setw(10) << "Found"
              << std::setw(12) << "Cost"
              << std::setw(12) << "Expanded"
              << std::setw(12) << "Time(ms)"
              << "\n";
    std::cout << std::string(71, '-') << "\n";

    for (size_t i = 0; i < mapNames.size(); ++i) {
        const auto& a = astarResults[i];
        const auto& j = jpsResults[i];

        std::cout << std::left
                  << std::setw(15) << mapNames[i]
                  << std::setw(10) << "A*"
                  << std::setw(10) << (a.found ? "Y" : "N")
                  << std::setw(12) << std::fixed << std::setprecision(2) << a.pathCost
                  << std::setw(12) << a.nodesExpanded
                  << std::setw(12) << std::fixed << std::setprecision(3) << a.searchTimeMs
                  << "\n";

        std::cout << std::left
                  << std::setw(15) << ""
                  << std::setw(10) << "JPS"
                  << std::setw(10) << (j.found ? "Y" : "N")
                  << std::setw(12) << std::fixed << std::setprecision(2) << j.pathCost
                  << std::setw(12) << j.nodesExpanded
                  << std::setw(12) << std::fixed << std::setprecision(3) << j.searchTimeMs
                  << "\n";

        std::cout << std::string(71, '-') << "\n";
    }
}
