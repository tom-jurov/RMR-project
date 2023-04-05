#include "local_nav.h"

diff_drive::LocalNav::LocalNav()
{
}

std::vector<diff_drive::Point<double> >
diff_drive::LocalNav::generateWaypoints(const diff_drive::Robot &robot_pos, const LaserMeasurement& laser_measurement)
{
    Point<double> target_point = std::move(findTargetPoint(laser_measurement, robot_pos));
    std::vector<Point<double>> waypoints;
    waypoints.emplace_back(Point<double>{robot_pos.x, robot_pos.y});
    waypoints.emplace_back(std::move(target_point));
    return waypoints;
}

diff_drive::Point<double>
diff_drive::LocalNav::findTargetPoint(const LaserMeasurement& laser_measurement, const diff_drive::Robot &robot_pos)
{
    diff_drive::Point<double> nearest_point;
    double nearest_distance = 2.700;
    double angle = 0;
    for(int i = 0; i < laser_measurement.numberOfScans; i++)
    {
        double laser_dis = laser_measurement.Data[i].scanDistance / 1000;
        // xr, yr [m], laser distance [mm]
        if (((laser_dis > 0.150 && laser_dis < 0.650) || (laser_dis > 0.700 && laser_dis < 2.700)) && (laser_measurement.Data[i].scanAngle > 0 && laser_measurement.Data[i].scanAngle < 180))
        {
            if (laser_dis < nearest_distance) {
                nearest_point.x = (robot_pos.x + laser_dis*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                nearest_point.y = (robot_pos.y + laser_dis*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                angle = deg2rad(-laser_measurement.Data[i].scanAngle);
                nearest_distance = laser_dis;
            }
        }
    }
    Point<double> d_vector = {nearest_point.x - robot_pos.x, nearest_point.y - robot_pos.y};
    Point<double> n_vector = {-d_vector.y, d_vector.x};
    double norm = std::sqrt(std::pow(n_vector.x, 2) + std::pow(n_vector.y, 2));
    n_vector = {n_vector.x/norm, n_vector.y/norm};
    if (fabs(n_vector.x) > fabs(n_vector.y))
    {
        if (sgn(n_vector.x) > 0)
        {
            n_vector = {0.3*n_vector.x, n_vector.y+desired_distance_};
        }
        else
        {
            n_vector = {0.3*n_vector.x, n_vector.y-desired_distance_};
        }
        std::cout << n_vector.x << " " << n_vector.y << " " << angle <<std::endl;
    }
    else
    {
        n_vector = {n_vector.x-desired_distance_, 0.3*n_vector.y};
    }
    Point<double> target_point = {nearest_point.x + n_vector.x, nearest_point.y + n_vector.y};

    return target_point;
}

