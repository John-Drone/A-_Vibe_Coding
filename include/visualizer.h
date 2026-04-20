#pragma once

#include "grid_map.h"
#include "a_star.h"
#include <vector>
#include <string>

class Visualizer {
public:
    static void printGrid(const GridMap& grid,
                          const std::vector<Node*>& path = {},
                          const Node* start = nullptr,
                          const Node* goal = nullptr);

    static void printSearchResult(const SearchResult& result);

    static void compareResults(const SearchResult& astarResult,
                               const SearchResult& jpsResult);

    static void printComparisonTable(
        const std::vector<std::string>& mapNames,
        const std::vector<SearchResult>& astarResults,
        const std::vector<SearchResult>& jpsResults);
};
