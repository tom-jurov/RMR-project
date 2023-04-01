#ifndef LOCAL_NAV_H
#define LOCAL_NAV_H
#include "robot_global.h"
#include "utility.h"
#include <vector>
#include "rplidar.h"
namespace diff_drive{
    class ROBOT_EXPORT LocalNav
    {
    public:
        LocalNav();
        Point<double> getNormalVector(const Robot& robot_pos, const Point<double>& lidar_pos);
        std::vector<Point<double>> generateWaypoints(const Robot& robot_pos, const LaserMeasurement& laser_measurement, double wall_distance);
        std::vector<Point<double>> processLidar(const LaserMeasurement& laser_measurement, const Robot &robot_pos);
    };
}

#endif // LOCAL_NAV_H
