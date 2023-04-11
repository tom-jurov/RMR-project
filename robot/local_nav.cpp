#include "local_nav.h"
#define EPS 5e-3
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
    for(int i = laser_measurement.numberOfScans - 1; i > -1; i--)
    {
        double laser_dis = laser_measurement.Data[i].scanDistance / 1000;
        // xr, yr [m], laser distance [mm]
        if (((laser_dis > 0.150 && laser_dis < 0.650) || (laser_dis > 0.700 && laser_dis < 2.700)) && (laser_measurement.Data[i].scanAngle > 0 && laser_measurement.Data[i].scanAngle < 180))
        {
            if (laser_dis + EPS < nearest_distance) {

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
        if (n_vector.x > 0)
        {
            n_vector = {0.3*n_vector.x, n_vector.y+desired_distance_};
        }
        else
        {
            n_vector = {0.3*n_vector.x, n_vector.y-desired_distance_};
        }
        //std::cout << n_vector.x << " " << n_vector.y << " " << angle << " " << nearest_distance <<std::endl;
    }
    else
    {
        if (n_vector.y > 0)
        {
            n_vector = {n_vector.x-desired_distance_, 0.3*n_vector.y};
        }
        else
        {
            n_vector = {n_vector.x+desired_distance_, 0.3*n_vector.y};
        }
        //std::cout << n_vector.x << " " << n_vector.y << " " << angle << " " << nearest_distance << std::endl;
    }
    Point<double> target_point = {nearest_point.x + n_vector.x, nearest_point.y + n_vector.y};

    return target_point;
}

bool diff_drive::LocalNav::isPathClear(const Point<double>& goal, const diff_drive::Robot &robot_pos, const LaserMeasurement& laser_measurement, double safe_zone)
{
    double robot_goal_angle = wrapAngle(atan2((goal.y - robot_pos.y),(goal.x - robot_pos.x)) - robot_pos.heading);
    double robot_goal_distance = sqrt(pow(goal.y - robot_pos.y,2) + pow(goal.x - robot_pos.x,2));
    double laser_normalized_angle, laser_normalized_dis;
    double laser_dis_crit, laser_dis;

    for(int i = 0; i < laser_measurement.numberOfScans; i++)
    {
        laser_dis = laser_measurement.Data[i].scanDistance / 1000;
        if ((laser_dis > 0.150 && laser_dis < 0.650) || (laser_dis > 0.700 && laser_dis < 2.700))
        {
            laser_normalized_angle = wrapAngle(robot_goal_angle + deg2rad(laser_measurement.Data[i].scanAngle));
            laser_dis_crit = fabs(safe_zone/sin(laser_normalized_angle));
            laser_normalized_dis = fabs(laser_dis*cos(laser_normalized_angle));

            if((laser_dis < laser_dis_crit) && (laser_normalized_dis < robot_goal_distance) && ((laser_normalized_angle < 0.5*M_PI) || (laser_normalized_angle > 1.5*M_PI)))
            {
                 return false;
            }
        }
    }

    return true;
}

std::vector<diff_drive::Point<double>>
diff_drive::LocalNav::findObstacleEdges(const Point<double>& goal, const Robot &robot_pos, const LaserMeasurement& laser_measurement, double safe_zone)
{
    int last_read_index = 0, first_valid_index = 0;
    double laser_dis, last_laser_dis ;
    double last_laser_angle, laser_angle ;
    bool first_valid_reading = true;
    bool validity_of_reading = false;
    bool validity_of_last_reading = false;
    bool stop_flag = false;
    bool next_cycle = false;
    std::vector<diff_drive::Point<double>> points;
    diff_drive::Point<double> point;

    for(int i = 0; i < laser_measurement.numberOfScans && stop_flag == false; i++)
    {
        laser_dis = laser_measurement.Data[i].scanDistance/1000;
        laser_angle = deg2rad(laser_measurement.Data[i].scanAngle);

        if (laser_dis > 0.150 && laser_dis < 2.7)
        {
            validity_of_reading = true;
            validity_of_last_reading = validity_of_reading;

            if(first_valid_reading)
            {
                last_laser_dis = laser_measurement.Data[i].scanDistance/1000;
                last_laser_angle = deg2rad(laser_measurement.Data[i].scanAngle);
                first_valid_reading = false;
                first_valid_index = i;
            }

            if(fabs(laser_dis - last_laser_dis) > 0.25 || (fabs(laser_angle - last_laser_angle) > 0.25 && fabs(laser_angle - last_laser_angle) < 6.28))
            {
                point.x = (robot_pos.x + laser_measurement.Data[last_read_index].scanDistance/1000*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[last_read_index].scanAngle)));
                point.y = (robot_pos.y + laser_measurement.Data[last_read_index].scanDistance/1000*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[last_read_index].scanAngle)));
                points.emplace_back(point);

                point.x = (robot_pos.x + laser_dis*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                point.y = (robot_pos.y + laser_dis*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                points.emplace_back(point);
            }

            last_laser_dis = laser_dis;
            last_laser_angle = laser_angle;
            last_read_index = i;
        }
        else
        {
            validity_of_reading = false;
        }

        if(validity_of_last_reading != validity_of_reading)
        {
            point.x = (robot_pos.x + laser_measurement.Data[last_read_index].scanDistance/1000*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[last_read_index].scanAngle)));
            point.y = (robot_pos.y + laser_measurement.Data[last_read_index].scanDistance/1000*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[last_read_index].scanAngle)));
            points.emplace_back(point);
            validity_of_last_reading = validity_of_reading;
        }

        if(i == laser_measurement.numberOfScans - 1)
        {
           i = 0;
           next_cycle = true;
        }

        if(i == first_valid_index && next_cycle == true)
        {
            stop_flag = true;
        }

    }

    return points;
}
