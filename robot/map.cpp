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
    std::vector<diff_drive::Point> new_points(NUM_OF_LASER_READINGS);

    new_points = laserToMapPoints(laser_measurement, odom.getX(), odom.getY(), odom.getHeading());
    for(int i = 0; i < NUM_OF_LASER_READINGS; i++)
    {
        if((new_points[i].x + zero_offset_x_) > map_size_x_ + zero_offset_x_){
            map_size_x_ = (int)new_points[i].x + zero_offset_x_;
            map_.resize(map_size_x_, std::vector<int>(map_size_y_, 0));

        }
        if((new_points[i].y + zero_offset_y_) > map_size_y_ + zero_offset_y_){
            map_size_y_ = (int)new_points[i].y + zero_offset_y_;
            for (auto &row: map_) row.resize(map_size_y_);
        }

        //if((new_points[i].y + zero_offset_y_) > pos_map_size_y_){
    }

    //print_map();
}

std::vector<diff_drive::Point> diff_drive::Map::laserToMapPoints(const LaserMeasurement& laser_measurement, const double& xr, const double& yr, const double& fir)
{
    std::vector<diff_drive::Point> points;
    diff_drive::Point point;

    for(int i = 0; i < NUM_OF_LASER_READINGS; i++)
    {
        point.x = (int)((xr + laser_measurement.Data[i].scanDistance*cos(fir + laser_measurement.Data[i].scanAngle))/GRID_RESOLUTION);
        point.y = (int)((yr + laser_measurement.Data[i].scanDistance*sin(fir + laser_measurement.Data[i].scanAngle))/GRID_RESOLUTION);
        points.emplace_back(point);
    }

    return points;
}

void diff_drive::Map::print_map()
{
    for(int j = 0; j < map_size_x_ - 1; j++)
    {
        for(int i = 0;i < map_size_y_ - 1; i++)
        {
            std::cout << map_[j][i];
        }
        std::cout << std::endl;
    }
    std::cout << map_size_x_ << " , " <<map_size_y_;
    std::cout << std::endl << std::endl;
}
