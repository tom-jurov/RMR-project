#include "global_nav.h"
#include <iostream>
diff_drive::GlobalNav::GlobalNav(const std::vector<std::vector<int>>& map, double s_x, double s_y, double g_x, double g_y)
    : map_(map)
    , visited_(std::vector<std::vector<bool>>(115, std::vector<bool>(115,false)))
    , start_({static_cast<int>((s_x+X_OFFSET)/GRID_SIZE),static_cast<int>((s_y+Y_OFFSET))/GRID_SIZE})
    , goal_({static_cast<int>((g_x+X_OFFSET)/GRID_SIZE),static_cast<int>((g_y+Y_OFFSET))/GRID_SIZE})
{
}

void diff_drive::GlobalNav::floodFill()
{
    std::queue<Point<int>> elemet_queue;
    visited_[start_.x][start_.y] = true;
    map_[start_.x][start_.y] = 2;
    elemet_queue.push(start_);
    while(!elemet_queue.empty())
    {
        Point<int> actual_element = elemet_queue.front();
        elemet_queue.pop();
        if (actual_element.x == goal_.x && actual_element.y == goal_.y)
        {
            return;
        }

        std::vector<Point<int>> neighbors = {
            {actual_element.x+1, actual_element.y}, {actual_element.x-1, actual_element.y},
            {actual_element.x, actual_element.y+1}, {actual_element.x, actual_element.y-1}
        };

        for (const Point<int>& point : neighbors)
        {
            if (point.x >= 0 && point.x < ROW && point.y >= 0 && point.y < COL && map_[point.x][point.y] != 1)
            {
                if (!visited_[point.x][point.y])
                {
                    visited_[point.x][point.y] = true;
                    elemet_queue.push(point);
                    map_[point.x][point.y] = map_[actual_element.x][actual_element.y] + 1;
                }
            }
        }
    }
}

std::vector<diff_drive::Point<int> > diff_drive::GlobalNav::getPath()
{
    std::vector<Point<int>> path;
    if (!visited_[goal_.x][goal_.y])
    {
        return path;
    }

    Point<int> p = goal_;
    int cost = map_[goal_.x][goal_.y];
    while (p.x != start_.x || p.y != start_.y)
    {
        path.push_back(p);

        std::vector<Point<int>> neighbors = {
            {p.x+1, p.y}, {p.x-1, p.y},
            {p.x, p.y+1}, {p.x, p.y-1}
        };

        Point<int> next = p;
        for (const Point<int>& n : neighbors)
        {
            if (n.x >= 0 && n.x < ROW && n.y >= 0 && n.y < COL && visited_[n.x][n.y])
            {
                if (map_[n.x][n.y] == cost - 1)
                {
                    cost = map_[n.x][n.y];
                    next = n;
                }
            }
        }
        p = next;
    }
    path.push_back(start_);
    std::reverse(path.begin(), path.end());
    return path;
}


