#ifndef MAP_H
#define MAP_H
#include "odometry.h"
#include "rplidar.h"
#include "robot_global.h"
#include <vector>
#define GRID_RESOLUTION 50 // [mm]
#define NUM_OF_LASER_READINGS 277 // number of useful data from laser_data

namespace diff_drive
{
    class ROBOT_EXPORT Map
    {
    public:
        Map();
        ~Map() = default;
        void update(const LaserMeasurement& laser_measurement, const diff_drive::Odometry& odom);
    private:
        std::vector<diff_drive::Point> laserToMapPoints(const LaserMeasurement& laser_measurement, const double& xr, const double& yr, const double& fir);
        void print_map();
    private:
        int map_size_x_;
        int map_size_y_;
        int zero_offset_x_; // offset for x = 0 in xy_
        int zero_offset_y_; // offset for y = 0 in xy_
        std::vector<std::vector<int>> map_;
    };
}

#endif // MAP_H
