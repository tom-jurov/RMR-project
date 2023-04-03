#include "local_nav.h"

diff_drive::LocalNav::LocalNav()
{
}

diff_drive::Point<double> diff_drive::LocalNav::getNormalVector(const Point<double> &lidar_pos, const Point<double> &lidar_consequent_pos)
{
    Point<double> normal;
    if ((lidar_consequent_pos.x - lidar_pos.x) == 0)
    {
        if (lidar_consequent_pos.y > lidar_pos.y)
        {
            normal.x = -1;
            normal.y = 0;
        }
        else
        {
            normal.x = 1;
            normal.y = 0;
        }
    }
    else if ((lidar_consequent_pos.y - lidar_pos.y) == 0)
    {
        if (lidar_consequent_pos.x > lidar_pos.x)
        {
            normal.x = 0;
            normal.y = 1;
        }
        else
        {
            normal.x = 0;
            normal.y = -1;
        }
    }
    else
    {
        double slope = (lidar_consequent_pos.y - lidar_pos.y) / (lidar_consequent_pos.x - lidar_pos.x);
        double angle_normal = atan2(-1, slope);
        normal.x = cos(angle_normal);
        normal.y = sin(angle_normal);
    }
    double length = sqrt(pow(normal.x, 2) + pow(normal.y, 2));
    normal.x = normal.x / length;
    normal.y = normal.y / length;
    return normal;
}

std::vector<diff_drive::Point<double> >
diff_drive::LocalNav::generateWaypoints(const diff_drive::Robot &robot_pos, const LaserMeasurement& laser_measurement, double wall_distance)
{
    std::vector<Point<double>> lidar_data = std::move(processLidar(laser_measurement, robot_pos));
    std::vector<Point<double>> waypoints;
    for (int i = 0; i < lidar_data.size()-1; i++)
    {
        Point<double> normal = getNormalVector(lidar_data[i], lidar_data[i+1]);
        Point<double> scaled_normal;
        scaled_normal.x = normal.x * wall_distance;
        scaled_normal.y = normal.y * wall_distance;

        Point<double> waypoint;
        waypoint.x = lidar_data[i].x + scaled_normal.x;
        waypoint.y = lidar_data[i].y + scaled_normal.y;

        waypoints.emplace_back(std::move(waypoint));
    }
    return waypoints;
}

std::vector<diff_drive::Point<double>>
diff_drive::LocalNav::processLidar(const LaserMeasurement& laser_measurement, const diff_drive::Robot &robot_pos)
{
    std::vector<diff_drive::Point<double>> lidar_data;
    lidar_data.reserve(laser_measurement.numberOfScans);
    diff_drive::Point<double> point;
    for(int i = 0; i < laser_measurement.numberOfScans; i++)
    {
        double laser_dis = laser_measurement.Data[i].scanDistance / 1000;
        // xr, yr [m], laser distance [mm]
        if (((laser_dis > 0.150 && laser_dis < 0.650) || (laser_dis > 0.700 && laser_dis < 2.700)) && (laser_measurement.Data[i].scanAngle >= 0 && laser_measurement.Data[i].scanAngle <= 180))
        {
            point.x = (robot_pos.x + laser_dis*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
            point.y = (robot_pos.y + laser_dis*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
            //std::cout << point.x << " " << point.y << " " << laser_measurement.Data[i].scanAngle <<std::endl;
            lidar_data.emplace_back(point);
        }
    }
    return lidar_data;
}

