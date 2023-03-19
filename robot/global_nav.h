#ifndef GLOBAL_NAV_H
#define GLOBAL_NAV_H
#include <vector>
#include <utility.h>
#include <queue>
#include <algorithm>
namespace diff_drive{
    class GlobalNav
    {
    public:
        static constexpr int ROW = 115;
        static constexpr int COL = 115;
        static constexpr int GRID_SIZE = 5;
        static constexpr double X_OFFSET = 50;
        static constexpr double Y_OFFSET = 50;
    public:
        GlobalNav() = default;
        GlobalNav(const std::vector<std::vector<int>>& map, double s_x, double s_y, double g_x, double g_y);
        void floodFill();
        std::vector<Point<int>> getPath();
        std::vector<Point<double>> generateWaypoints();
    private:
        std::vector<std::vector<int>> map_;
        std::vector<std::vector<bool>> visited_;
        std::vector<Point<int>> path_;
        std::vector<diff_drive::Point<double>> waypoints_;
        Point<int> start_;
        Point<int> goal_;
        double g_x_;
        double g_y_;
    };
}

#endif // GLOBAL_NAV_H
