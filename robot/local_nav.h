#ifndef LOCAL_NAV_H
#define LOCAL_NAV_H
#include "robot_global.h"
#include <algorithm>
#include <vector>
#include "utility.h"
#include "rplidar.h"
namespace diff_drive{
    enum Direction{
        LEFT = 0,
        RIGHT = 1
    };

    class ROBOT_EXPORT LocalNav
    {
    public:
        LocalNav();
        bool isPathClear(const Point<double>& goal, const Robot &robot_pos, const LaserMeasurement& laser_measurement, double safe_zone);
        std::vector<diff_drive::Point<double>> findObstacleEdges(const Robot &robot_pos, const LaserMeasurement& laser_measurement);
        std::vector<diff_drive::Point<double>> findEdgeNormals(const Robot &robot_pos,const std::vector<Point<double>>& edges, double edge_dis);
        diff_drive::Point<double> findClosestAccessiblePoint(const Point<double>& goal, const Robot &robot_pos, const LaserMeasurement& laser_measurement, const std::vector<Point<double>>& normals);
        std::vector<Point<double>> generateWaypoints(const Point<double>& goal, const Robot &robot_pos, const LaserMeasurement& laser_measurement);
    private:
        std::vector<Point<double>> processLidar(const LaserMeasurement& laser_measurement, const Robot &robot_pos);
        Point<double> findTargetPoint(const LaserMeasurement& laser_measurement, const Robot &robot_pos);
        double getHeruisticDistance(const Point<double> &normal, const Point<double> &goal, const Robot &robot_pos) const;
    private:
        std::vector<bool> normals_switch_;
        double desired_distance_ = 0.5;
        Point<double> temp_followed_point_;
        bool first_edge_detected_ = true;
        double smallest_heruistic_distance_, current_heruistic_distance_;
        bool is_wall_following_ = false;
        bool direction_wall_following_flag_ = LEFT;
    };

}

#endif // LOCAL_NAV_H
