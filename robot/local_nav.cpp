#include "local_nav.h"

diff_drive::LocalNav::LocalNav()
{
}

diff_drive::Point<double> diff_drive::LocalNav::getNormalVector(const Point<double> &lidar_pos, const Point<double> &lidar_consequent_pos)
{
    double dx = lidar_consequent_pos.x - lidar_pos.x;
    double dy = lidar_consequent_pos.y - lidar_pos.y;
    double length = sqrt(pow(dx, 2) + pow(dy, 2));
    Point<double> normal;
    //std::cout << "Lidar data: " << lidar_pos.x << " " << lidar_pos.y << " " << lidar_consequent_pos.x << " " << lidar_consequent_pos.y << std::endl;
    if (lidar_consequent_pos.x > lidar_pos.x || lidar_consequent_pos.y > lidar_pos.y)
    {
        normal.x = -dy / length;
        normal.y = dx / length;
    }
    else
    {
        normal.x = dy / length;
        normal.y = -dx / length;
    }
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
        //std::cout << "Way point: " << waypoint.x << " " << waypoint.y;

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
        if (((laser_dis > 0.150 && laser_dis < 0.650) || (laser_dis > 0.700 && laser_dis < 2.700)) && (laser_measurement.Data[i].scanAngle >= 45 && laser_measurement.Data[i].scanAngle <= 135))
        {
            point.x = (robot_pos.x + laser_dis*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
            point.y = (robot_pos.y + laser_dis*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
            //std::cout << point.x << " " << point.y << " " << laser_measurement.Data[i].scanAngle <<std::endl;
            lidar_data.emplace_back(point);
        }
    }
    return lidar_data;
}

