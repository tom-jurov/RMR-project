#ifndef MAP_H
#define MAP_H
#include "odometry.h"
#include "rplidar.h"
#include "robot_global.h"
#include <vector>
#include "utility.h"

namespace diff_drive
{
    class ROBOT_EXPORT Map
    {
    public:
        Map();
        ~Map() = default;
        void update(const LaserMeasurement& laser_measurement, const diff_drive::Odometry& odom);
        void printMap();
        void setGyroAngle(int angle);
        void setGyroStartAngle(int angle);
    private:
        std::vector<diff_drive::Point<int>> getNewPoints(const LaserMeasurement& laser_measurement, const double xr, const double yr);
        void resizeMap(const std::vector<diff_drive::Point<int>>& new_points);
        void fillMap(const std::vector<diff_drive::Point<int>>& new_points);
    private:
        static constexpr int GRID_RESOLUTION = 50; // [mm]
    private:
        int map_size_x_;
        int map_size_y_;
        int zero_offset_x_; // offset for x = 0 in map_
        int zero_offset_y_; // offset for y = 0 in map_
        double gyro_angle_;
        double gyro_start_angle_;
        std::vector<std::vector<int>> map_;
    };
}

#endif // MAP_H
