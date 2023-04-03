#ifndef LOCAL_NAV_H
#define LOCAL_NAV_H
#include "robot_global.h"
#include <vector>
#include "utility.h"
#include "rplidar.h"
namespace diff_drive{
    class ROBOT_EXPORT LocalNav
    {
    public:
        LocalNav();
        std::vector<Point<double>> generateWaypoints(const Robot &robot_pos, const LaserMeasurement& laser_measurement, double wall_distance);
    private:
         Point<double> getNormalVector(const Point<double>& lidar_pos, const Point<double>& lidar_consequent_pos);
         std::vector<Point<double>> processLidar(const LaserMeasurement& laser_measurement, const Robot &robot_pos);
    };
}

#endif // LOCAL_NAV_H
