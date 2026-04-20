#pragma once

#include <cmath>
#include <string>

enum class HeuristicType {
    Manhattan,
    Euclidean,
    Octile
};

class Heuristic {
public:
    static double calculate(int x1, int y1, int x2, int y2, HeuristicType type) {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);

        switch (type) {
            case HeuristicType::Manhattan:
                return static_cast<double>(dx + dy);

            case HeuristicType::Euclidean:
                return std::sqrt(static_cast<double>(dx * dx + dy * dy));

            case HeuristicType::Octile:
                // Octile距离：8方向移动的最优启发函数
                // 公式：max(dx,dy) + (sqrt(2)-1) * min(dx,dy)
                // 这个公式是怎么来的？提示：对角线走 min(dx,dy) 步，
                // 直线走 max(dx,dy)-min(dx,dy) 步
                {
                    double D = 1.0;
                    double D2 = std::sqrt(2.0);
                    return D * std::max(dx, dy) + (D2 - D) * std::min(dx, dy);
                }

            default:
                return 0.0;
        }
    }

    static std::string toString(HeuristicType type) {
        switch (type) {
            case HeuristicType::Manhattan: return "Manhattan";
            case HeuristicType::Euclidean: return "Euclidean";
            case HeuristicType::Octile: return "Octile";
            default: return "Unknown";
        }
    }
};
