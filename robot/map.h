#ifndef MAP_H
#define MAP_H
#include "odometry.h"
#include "rplidar.h"
#include "robot_global.h"
#include <vector>
#define GRID_RESOLUTION 50 // [mm]

namespace diff_drive
{
    class ROBOT_EXPORT Map
    {
    public:
        Map();
        ~Map() = default;
        void update(const LaserMeasurement& laser_measurement, const diff_drive::Odometry& odom);
        void print_map();
    private:
        std::vector<diff_drive::Point> laserToMapPoints(const LaserMeasurement& laser_measurement, const double& xr, const double& yr, const double& fir);
        void resize_map(const std::vector<diff_drive::Point>& new_points);
        double deg2rad(double degrees);
    private:
        int map_size_x_;
        int map_size_y_;
        int zero_offset_x_; // offset for x = 0 in map_
        int zero_offset_y_; // offset for y = 0 in map_
        std::vector<std::vector<int>> map_;
    };
}

#endif // MAP_H
