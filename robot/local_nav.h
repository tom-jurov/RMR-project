#ifndef LOCAL_NAV_H
#define LOCAL_NAV_H
#include "robot_global.h"
#include <algorithm>
#include <vector>
#include "utility.h"
#include "rplidar.h"
namespace diff_drive{
    class ROBOT_EXPORT LocalNav
    {
    public:
        LocalNav();
        bool isPathClear(const Point<double>& goal, const Robot &robot_pos, const LaserMeasurement& laser_measurement, double safe_zone);
        std::vector<diff_drive::Point<double>> findObstacleEdges(const Robot &robot_pos, const LaserMeasurement& laser_measurement);
        std::vector<diff_drive::Point<double>> findEdgeNormals(const Point<double>& goal, const Robot &robot_pos,const std::vector<Point<double>>& edges, double safe_zone);
        std::vector<Point<double>> generateWaypoints(const Robot &robot_pos, const LaserMeasurement& laser_measurement);
    private:
        std::vector<Point<double>> processLidar(const LaserMeasurement& laser_measurement, const Robot &robot_pos);
        Point<double> findTargetPoint(const LaserMeasurement& laser_measurement, const Robot &robot_pos);
    private:
        std::vector<bool> is_norm_size_left_;
        double desired_distance_ = 0.5;
    };

}

#endif // LOCAL_NAV_H
