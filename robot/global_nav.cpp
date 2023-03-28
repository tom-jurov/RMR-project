#include "global_nav.h"
#include <iostream>
diff_drive::GlobalNav::GlobalNav(const std::vector<std::vector<int>>& map, double s_x, double s_y, double g_x, double g_y)
    : map_(map)
    , visited_(std::vector<std::vector<bool>>(115, std::vector<bool>(115,false)))
    , start_({static_cast<int>((s_x+X_OFFSET)/GRID_SIZE),static_cast<int>((s_y+Y_OFFSET))/GRID_SIZE})
    , goal_({static_cast<int>((g_x+X_OFFSET)/GRID_SIZE),static_cast<int>((g_y+Y_OFFSET))/GRID_SIZE})
    , g_x_(g_x)
    , g_y_(g_y)
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
    if (!visited_[goal_.x][goal_.y])
    {
        return path_;
    }

    Point<int> p = goal_;
    int cost = map_[goal_.x][goal_.y];
    while (p.x != start_.x || p.y != start_.y)
    {
        path_.push_back(p);

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
    path_.push_back(start_);
    std::reverse(path_.begin(), path_.end());
    return path_;
}

std::vector<diff_drive::Point<double> > diff_drive::GlobalNav::generateWaypoints()
{
    auto local_path = path_;
    local_path.pop_back();
    local_path.erase( local_path.begin());
    waypoints_.push_back(Point<double>{0,0});
    prev_point_ = local_path[0];
    for (const auto& p :  local_path)
    {
        std::cout << p.x << " " << p.y << std::endl;
       diff_drive::Point<double> way_point;
       if (p.x == prev_point_.x && p.y != prev_point_.y)
       {
            way_point.x = (p.x * diff_drive::GlobalNav::GRID_SIZE - diff_drive::GlobalNav::X_OFFSET) / 100;
            way_point.y = (p.y * diff_drive::GlobalNav::GRID_SIZE - (double)diff_drive::GlobalNav::GRID_SIZE/2 - diff_drive::GlobalNav::Y_OFFSET) / 100;
       }
       else if (p.y == prev_point_.y && p.x != prev_point_.x)
       {
            way_point.x = (p.x * diff_drive::GlobalNav::GRID_SIZE - (double)diff_drive::GlobalNav::GRID_SIZE/2 - diff_drive::GlobalNav::X_OFFSET) / 100;
            way_point.y = (p.y * diff_drive::GlobalNav::GRID_SIZE - diff_drive::GlobalNav::Y_OFFSET) / 100;
       }
       else
       {
           way_point.x = (p.x * diff_drive::GlobalNav::GRID_SIZE - (double)diff_drive::GlobalNav::GRID_SIZE/2 - diff_drive::GlobalNav::X_OFFSET) / 100;
           way_point.y = (p.y * diff_drive::GlobalNav::GRID_SIZE - (double)diff_drive::GlobalNav::GRID_SIZE/2 - diff_drive::GlobalNav::Y_OFFSET) / 100;
       }
        waypoints_.push_back(way_point);
        prev_point_ = p;
    }
    waypoints_.push_back(diff_drive::Point<double>{g_x_/100, g_y_/100});
    return  waypoints_;
}


