#ifndef MAP_H
#define MAP_H
#include "odometry.h"
#include "rplidar.h"
#include "robot_global.h"
#include <vector>
#define GRID_RESOLUTION 5 // [cm]
#define NUM_OF_LASER_READINGS 276 // number of useful data from laser_data

namespace diff_drive
{
    class ROBOT_EXPORT Map
    {
    public:
        Map();
        ~Map() = default;
        void update(LaserMeasurement laser_data);
    private:
        void laserToMapPoints(LaserMeasurement laser_data);
    private:
        int size_x_;
        int size_y_;
        int zero_offset_x_; // offset for x = 0 in xy_
        int zero_offset_y_; // offset for y = 0 in xy_
        int start_offset_y_; // offset for start of y in xy_
        std::vector<bool> xy_;
    };
}

#endif // MAP_H
