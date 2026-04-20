#include "grid_map.h"
#include "a_star.h"
#include "jps.h"
#include "heuristic.h"
#include "visualizer.h"

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

void runSingleTest(GridMap& grid, int startX, int startY, int goalX, int goalY) {
    std::cout << "\n--- Running A* ---\n";
    AStar astar(grid, HeuristicType::Octile, true);
    SearchResult astarResult = astar.search(startX, startY, goalX, goalY);
    Visualizer::printSearchResult(astarResult);

    std::cout << "--- Running JPS ---\n";
    JPS jps(grid, HeuristicType::Octile);
    SearchResult jpsResult = jps.search(startX, startY, goalX, goalY);
    Visualizer::printSearchResult(jpsResult);

    Visualizer::compareResults(astarResult, jpsResult);

    std::cout << "A* Path:\n";
    Visualizer::printGrid(grid, astarResult.path,
                          grid.getNode(startX, startY), grid.getNode(goalX, goalY));

    std::cout << "JPS Path:\n";
    Visualizer::printGrid(grid, jpsResult.path,
                          grid.getNode(startX, startY), grid.getNode(goalX, goalY));
}

void runBenchmark() {
    struct BenchmarkCase {
        std::string name;
        std::string filename;
        int width;
        int height;
        int startX, startY, goalX, goalY;
    };

    std::vector<BenchmarkCase> cases = {
        {"Simple",    "maps/simple.txt",    10, 10, 0, 0, 9, 9},
        {"Open",      "maps/open.txt",      10, 10, 0, 0, 9, 9},
        {"Corridor",  "maps/corridor.txt",  10, 10, 0, 0, 9, 9},
        {"Complex",   "maps/complex.txt",   24, 24, 0, 0, 23, 23},
        {"Large",     "maps/large.txt",     40, 40, 0, 0, 39, 39},
    };

    std::vector<std::string> names;
    std::vector<SearchResult> astarResults;
    std::vector<SearchResult> jpsResults;

    for (auto& tc : cases) {
        GridMap grid(tc.width, tc.height);
        grid.loadFromFile(tc.filename);

        AStar astar(grid, HeuristicType::Octile, true);
        SearchResult ar = astar.search(tc.startX, tc.startY, tc.goalX, tc.goalY);

        JPS jps(grid, HeuristicType::Octile);
        SearchResult jr = jps.search(tc.startX, tc.startY, tc.goalX, tc.goalY);

        names.push_back(tc.name);
        astarResults.push_back(ar);
        jpsResults.push_back(jr);

        std::cout << "Completed: " << tc.name << "\n";
    }

    Visualizer::printComparisonTable(names, astarResults, jpsResults);
}

void interactiveMode() {
    std::cout << "=== Interactive Pathfinding ===\n";
    std::cout << "Enter grid size (width height): ";
    int width, height;
    std::cin >> width >> height;

    GridMap grid(width, height);

    std::cout << "Enter number of obstacles: ";
    int numObstacles;
    std::cin >> numObstacles;

    std::srand(static_cast<unsigned>(std::time(nullptr)));
    for (int i = 0; i < numObstacles; ++i) {
        int ox = std::rand() % width;
        int oy = std::rand() % height;
        grid.setObstacle(ox, oy);
    }

    std::cout << "\nGenerated Map:\n";
    grid.print();

    std::cout << "\nEnter start (x y): ";
    int sx, sy;
    std::cin >> sx >> sy;

    std::cout << "Enter goal (x y): ";
    int gx, gy;
    std::cin >> gx >> gy;

    runSingleTest(grid, sx, sy, gx, gy);
}

void testHeuristics() {
    std::cout << "\n=== Heuristic Comparison on Simple Map ===\n";

    GridMap grid(10, 10);
    grid.loadFromFile("maps/simple.txt");

    HeuristicType types[] = {HeuristicType::Manhattan, HeuristicType::Euclidean, HeuristicType::Octile};

    for (auto type : types) {
        std::cout << "\n--- " << Heuristic::toString(type) << " ---\n";
        AStar astar(grid, type, true);
        SearchResult result = astar.search(0, 0, 9, 9);
        Visualizer::printSearchResult(result);
    }
}

int main() {
    while (true) {
        std::cout << "\n========================================\n";
        std::cout << "   Pathfinding Algorithm Project\n";
        std::cout << "   A* & Jump Point Search (JPS)\n";
        std::cout << "========================================\n";
        std::cout << "1. Run single test (simple map)\n";
        std::cout << "2. Run single test (corridor map)\n";
        std::cout << "3. Run single test (complex map)\n";
        std::cout << "4. Run benchmark (all maps)\n";
        std::cout << "5. Interactive mode\n";
        std::cout << "6. Compare heuristics\n";
        std::cout << "0. Exit\n";
        std::cout << "========================================\n";
        std::cout << "Choose: ";

        int choice;
        std::cin >> choice;

        switch (choice) {
            case 1: {
                GridMap grid(10, 10);
                grid.loadFromFile("maps/simple.txt");
                runSingleTest(grid, 0, 0, 9, 9);
                break;
            }
            case 2: {
                GridMap grid(10, 10);
                grid.loadFromFile("maps/corridor.txt");
                runSingleTest(grid, 0, 0, 9, 9);
                break;
            }
            case 3: {
                GridMap grid(24, 24);
                grid.loadFromFile("maps/complex.txt");
                runSingleTest(grid, 0, 0, 23, 23);
                break;
            }
            case 4: {
                runBenchmark();
                break;
            }
            case 5: {
                interactiveMode();
                break;
            }
            case 6: {
                testHeuristics();
                break;
            }
            case 0: {
                std::cout << "Goodbye!\n";
                return 0;
            }
            default: {
                std::cout << "Invalid choice!\n";
                break;
            }
        }
    }

    return 0;
}
