#include "map.h"

diff_drive::Map::Map()
: map_size_x_(0)
, map_size_y_(0)
, zero_offset_x_(0)
, zero_offset_y_(0)
{
}

void diff_drive::Map::update(const LaserMeasurement& laser_measurement, const diff_drive::Odometry& odom)
{
    std::vector<diff_drive::Point<int>> new_points(laser_measurement.numberOfScans);

    new_points = getNewPoints(laser_measurement, odom.getX(), odom.getY());
    resizeMap(new_points);
    fillMap(new_points);
    std::cout << gyro_angle_ << std::endl;
}

std::vector<diff_drive::Point<int>> diff_drive::Map::getNewPoints(const LaserMeasurement& laser_measurement, const double xr, const double yr)
{
    std::vector<diff_drive::Point<int>> points;
    diff_drive::Point<int> point;

    for(int i = 0; i < laser_measurement.numberOfScans; i++)
    {
        double laser_dis = laser_measurement.Data[i].scanDistance;

        // xr, yr [m], laser distance [mm]
        if((laser_dis > 150 && laser_dis < 650) || (laser_dis > 700 && laser_dis < 2700)){
            point.x = (int)((xr*1000 + laser_dis*cos(gyro_angle_ + deg2rad(-laser_measurement.Data[i].scanAngle)))/GRID_RESOLUTION);
            point.y = (int)((yr*1000 + laser_dis*sin(gyro_angle_ + deg2rad(-laser_measurement.Data[i].scanAngle)))/GRID_RESOLUTION);
            points.emplace_back(point);
        }
    }

    return points;
}

void diff_drive::Map::resizeMap(const std::vector<diff_drive::Point<int>>& new_points)
{
    for(int i = 0; i < new_points.size(); i++)
    {
        if((new_points[i].x < 0) && ((int)new_points[i].x + zero_offset_x_ < 0))
        {
            int delta_x = (int)abs(new_points[i].x) - zero_offset_x_;
            int old_size_x =  map_size_x_;

            map_size_x_ = map_size_x_ + delta_x;
            map_.resize(map_size_x_, std::vector<int>(map_size_y_, 0));
            zero_offset_x_ = zero_offset_x_ + delta_x;

            // Offsetting map by new offset
            for(int y = map_size_y_ - 1; y >= 0; y--)
            {
                for(int x = map_size_x_ - 1; x >= map_size_x_ - old_size_x; x--)
                {
                    map_[x][y] = map_[x - delta_x][y];
                    map_[x - delta_x][y] = 0;
                }
            }
        }

        if((new_points[i].x + zero_offset_x_ + 1) > map_size_x_)
        {
            int delta_x = (int)new_points[i].x - map_size_x_ + zero_offset_x_ + 1;

            map_size_x_ = map_size_x_ + delta_x;
            map_.resize(map_size_x_, std::vector<int>(map_size_y_, 0));

        }

        if((new_points[i].y < 0) && ((int)new_points[i].y + zero_offset_y_ < 0))
        {
            int delta_y = (int)abs(new_points[i].y) - zero_offset_y_;
            int old_size_y =  map_size_y_;

            map_size_y_ = map_size_y_ + delta_y;
            for (auto &row: map_) row.resize(map_size_y_);
            zero_offset_y_ = zero_offset_y_ + delta_y;

            // Offsetting map by new offset
            for(int y = map_size_y_ - 1; y >= map_size_y_ - old_size_y; y--)
            {
                for(int x = map_size_x_ - 1; x >= 0; x--)
                {
                    map_[x][y] = map_[x][y - delta_y];
                    map_[x][y - delta_y] = 0;
                }
            }
        }

        if((new_points[i].y + zero_offset_y_ + 1) > map_size_y_)
        {
            int delta_y = (int)new_points[i].y - map_size_y_ + zero_offset_y_ + 1;

            map_size_y_ = map_size_y_ + delta_y;
            for (auto &row: map_) row.resize(map_size_y_);
        }
    }
}

void diff_drive::Map::fillMap(const std::vector<diff_drive::Point<int>>& new_points)
{
    int x, y;

    for(int i = 0; i < new_points.size(); i++)
    {
        x = (int)new_points[i].x + zero_offset_x_;
        y = (int)new_points[i].y + zero_offset_y_;
        map_[x][y] = 1;
    }

    map_[0 + zero_offset_x_][0 + zero_offset_y_] = 3; // Dont forget to delete later !!!!
}

void diff_drive::Map::printMap()
{
    for(int y = map_size_y_ - 1; y >= 0; y--)
    {
        for(int x = 0;x < map_size_x_; x++)
        {
            std::cout << map_[x][y] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "Mx:" << map_size_x_ << " , My:" << map_size_y_;
    std::cout << " , Ox:" << zero_offset_x_ << " , Oy:" << zero_offset_y_;
    std::cout << std::endl << std::endl;
}

void diff_drive::Map::setGyroAngle(int angle)
{
    gyro_angle_ = deg2rad((angle/100.0)) - gyro_start_angle_;
}

void diff_drive::Map::setGyroStartAngle(int angle)
{
    gyro_start_angle_ = deg2rad((angle/100.0));
}

