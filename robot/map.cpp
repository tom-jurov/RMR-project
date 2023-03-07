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
    std::vector<diff_drive::Point> new_points(laser_measurement.numberOfScans);

    new_points = laserToMapPoints(laser_measurement, odom.getX(), odom.getY(), odom.getHeading());
    resize_map(new_points);

}

std::vector<diff_drive::Point> diff_drive::Map::laserToMapPoints(const LaserMeasurement& laser_measurement, const double& xr, const double& yr, const double& fir)
{
    std::vector<diff_drive::Point> points;
    diff_drive::Point point;

    for(int i = 0; i < laser_measurement.numberOfScans; i++)
    {
        double laser_dis = laser_measurement.Data[i].scanDistance;

        // xr, yr [m], laser distance [mm]
        if((laser_dis > 200 && laser_dis < 600) || (laser_dis > 750 && laser_dis < 2700)){
            point.x = (int)((xr*1000 + laser_dis*cos(fir + deg2rad(-laser_measurement.Data[i].scanAngle)))/GRID_RESOLUTION);
            point.y = (int)((yr*1000 + laser_dis*sin(fir + deg2rad(-laser_measurement.Data[i].scanAngle)))/GRID_RESOLUTION);
            points.emplace_back(point);
        }
    }

    return points;
}

void diff_drive::Map::resize_map(const std::vector<diff_drive::Point>& new_points)
{
    for(int i = 0; i < new_points.size(); i++)
    {
        if((new_points[i].x + zero_offset_x_) > map_size_x_)
        {
            map_size_x_ = (int)new_points[i].x + zero_offset_x_;
            map_.resize(map_size_x_, std::vector<int>(map_size_y_, 0));

        }
        if((new_points[i].y + zero_offset_y_) > map_size_y_)
        {
            map_size_y_ = (int)new_points[i].y + zero_offset_y_;
            for (auto &row: map_) row.resize(map_size_y_);
        }

        if((new_points[i].x < 0) && ((int)new_points[i].x + zero_offset_x_ < 0))
        {
            int offset_delta = (int)abs(new_points[i].x) - zero_offset_x_;
            int old_size_x =  map_size_x_;

            std::cout << offset_delta << " , Px:" << new_points[i].x << " , Ox:"<< zero_offset_x_ << std::endl;
            map_size_x_ = map_size_x_ + offset_delta;
            map_.resize(map_size_x_, std::vector<int>(map_size_y_, 0));
            zero_offset_x_ = zero_offset_x_ + offset_delta;

            for(int y = map_size_y_ - 1; y >= 0; y--)
            {
                for(int x = map_size_x_ - 1; x >= map_size_x_ - old_size_x; x--)
                {
                    map_[x][y] = map_[x - offset_delta][y];
                }
            }
        }

        if((new_points[i].y < 0) && ((int)new_points[i].y + zero_offset_y_ < 0))
        {
            int offset_delta = (int)abs(new_points[i].y) - zero_offset_y_;
            int old_size_y =  map_size_y_;

            std::cout << offset_delta << " , Py:" << new_points[i].y << " , Oy:"<< zero_offset_y_ << std::endl;
            map_size_y_ = map_size_y_ + offset_delta;
            for (auto &row: map_) row.resize(map_size_y_);
            zero_offset_y_ = zero_offset_y_ + offset_delta;

            for(int y = map_size_y_ - 1; y >= map_size_y_ - old_size_y; y--)
            {
                for(int x = map_size_x_ - 1; x >= 0; x--)
                {
                    map_[x][y] = map_[x][y - offset_delta];
                }
            }
        }
    }
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
    std::cout << "Mx:" << map_size_x_ << " , My:" << map_size_y_;
    std::cout << " , Ox:" << zero_offset_x_ << " , Oy:" << zero_offset_y_;
    std::cout << std::endl << std::endl;
}

double diff_drive::Map::deg2rad(double degrees){
    return degrees * (M_PI/180);
}
