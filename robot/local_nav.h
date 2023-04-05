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
        std::vector<Point<double>> generateWaypoints(const Robot &robot_pos, const LaserMeasurement& laser_measurement);
    private:
         Point<double> findTargetPoint(const LaserMeasurement& laser_measurement, const Robot &robot_pos);
         double desired_distance_ = 0.5;
    };
}

#endif // LOCAL_NAV_H
