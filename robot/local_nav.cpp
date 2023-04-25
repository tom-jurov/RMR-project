#include "local_nav.h"

diff_drive::LocalNav::LocalNav()
{
}

std::vector<diff_drive::Point<double> >
diff_drive::LocalNav::generateWaypoints(const Point<double>& goal, const diff_drive::Robot &robot_pos, const LaserMeasurement& laser_measurement)
{
    std::vector<Point<double>> waypoints;
    Point<double> robot = {robot_pos.x, robot_pos.y};

    waypoints.emplace_back(Point<double>{robot_pos.x, robot_pos.y});

    if(isPathClear(goal, robot_pos, laser_measurement, 0.2))
    {
        std::cout << "Straight to goal" << std::endl;
        waypoints.emplace_back(goal);
    }
    else
    {
        int index;
        auto edges = findObstacleEdges(robot_pos, laser_measurement);
        auto normals = findEdgeNormals(robot_pos, edges, 0.45);
        auto temp_followed_point = findClosestAccessiblePoint(goal, robot_pos, laser_measurement, normals, &index);

        if(first_edge_detected_)
        {
            current_followed_point_ = temp_followed_point;
            last_followed_point_ = current_followed_point_;
            smallest_heruistic_distance_ = getHeruisticDistance(temp_followed_point, goal, robot_pos);
            current_heruistic_distance_ = smallest_heruistic_distance_;
            current_followed_edge_ = edges[index];
            last_followed_edge_ = current_followed_edge_;
            first_edge_detected_ = false;
        }

        // Wall following, followed until leaving condiotion satsfied
        if(getHeruisticDistance(current_followed_point_, goal, robot_pos) > smallest_heruistic_distance_)
        {
            auto first_vec = last_followed_edge_ + -1*reinterpret_cast<const Point<double>&>(robot_pos);
            Point<double> second_vec = {sin(robot_pos.heading), cos(robot_pos.heading)};
            double dot_product = first_vec.x * second_vec.x + first_vec.y * second_vec.y;
            double mult_mag = norm(first_vec) * norm(second_vec);
            double angle_of_vec = std::acos(dot_product/mult_mag);
            //std::cout << angle_of_vec << std::endl;

            if(is_wall_following_ == false)
            {
                if (angle_of_vec > 0 && angle_of_vec < M_PI/2)
                {
                    direction_wall_following_flag_ = RIGHT;
                }
                else
                {
                    direction_wall_following_flag_ = LEFT;
                }
            }

            is_wall_following_ = true;
        }

        // Leaving condition
        if(magnitude(reinterpret_cast<const Point<double>&>(robot_pos), goal) < magnitude(last_followed_point_ , goal) && is_wall_following_)
        {
            is_wall_following_ = false;
            current_followed_point_ = temp_followed_point;
        }

        if(is_wall_following_)
        {
            current_followed_point_ = findTargetPoint(laser_measurement,robot_pos);
            if (direction_wall_following_flag_ == LEFT)
            {
                std::cout << "Wall following left" << std::endl;
            }
            else
            {
                std::cout << "Wall following right" << std::endl;
            }
        }
        else
        {
            std::cout << "Standart following" << std::endl;
        }


        // Standart following, locked until point is reached
        if(magnitude(robot, current_followed_point_) < 0.3 && !is_wall_following_)
        {
            last_followed_point_ = current_followed_point_;
            current_followed_point_ = temp_followed_point;
            current_heruistic_distance_ = getHeruisticDistance(temp_followed_point, goal, robot_pos);
            last_followed_edge_ = current_followed_edge_;
            current_followed_edge_ = edges[index];

            if(current_heruistic_distance_ < smallest_heruistic_distance_)
            {
                smallest_heruistic_distance_ = current_heruistic_distance_;
            }  
        }

        waypoints.emplace_back(current_followed_point_);
    }

    //std::cout <<magnitude(reinterpret_cast<const Point<double>&>(robot_pos), goal)  << std::endl;

    return waypoints;
}

diff_drive::Point<double>
diff_drive::LocalNav::findTargetPoint(const LaserMeasurement& laser_measurement, const diff_drive::Robot &robot_pos)
{
    diff_drive::Point<double> nearest_point;
    double nearest_distance = 2.700;
    double angle = 0;
    bool condition_for_range = 0;

    for(int i = laser_measurement.numberOfScans - 1; i > -1; i--)
    {
        double laser_dis = laser_measurement.Data[i].scanDistance / 1000;
        bool condition_for_range = 0;
        // xr, yr [m], laser distance [mm]
        if (direction_wall_following_flag_ == LEFT)
        {
            condition_for_range = (laser_measurement.Data[i].scanAngle > 180 && laser_measurement.Data[i].scanAngle < 360);
        }
        else if (direction_wall_following_flag_ == RIGHT)
        {
            condition_for_range = (laser_measurement.Data[i].scanAngle > 0 && laser_measurement.Data[i].scanAngle < 180);
        }
        if (((laser_dis > 0.150 && laser_dis < 0.650) || (laser_dis > 0.700 && laser_dis < 2.700)) && condition_for_range)
        {
            double eps_range = 0;
            if (direction_wall_following_flag_ == LEFT)
            {
                eps_range = laser_dis + EPS;
            }
            else if (direction_wall_following_flag_ == RIGHT)
            {
                eps_range = laser_dis - EPS;
            }
            if (eps_range < nearest_distance) {

                nearest_point.x = (robot_pos.x + laser_dis*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                nearest_point.y = (robot_pos.y + laser_dis*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                angle = deg2rad(-laser_measurement.Data[i].scanAngle);
                nearest_distance = laser_dis;
            }
        }
    }
    Point<double> d_vector = {nearest_point.x - robot_pos.x, nearest_point.y - robot_pos.y};
    Point<double> d_vector_normed = diff_drive::normalized(d_vector);
    Point<double> n_vector_normed;

    if (direction_wall_following_flag_ == LEFT){
        n_vector_normed = {d_vector_normed.y, -d_vector_normed.x};
    }
    else if (direction_wall_following_flag_ == RIGHT){
        n_vector_normed = {-d_vector_normed.y, d_vector_normed.x};
    }

    Point<double> scaled_d_vector = -desired_distance_*d_vector_normed;
    Point<double> scaled_n_vector = 0.3*n_vector_normed;
    Point<double> target_vec = scaled_d_vector + scaled_n_vector;

    Point<double> target_point = {nearest_point.x + target_vec.x, nearest_point.y + target_vec.y};

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

            if((laser_dis < laser_dis_crit) && ((laser_normalized_dis + 0.2) < robot_goal_distance) && ((laser_normalized_angle < 0.5*M_PI) || (laser_normalized_angle > 1.5*M_PI)))
            {
                 return false;
            }
        }
    }

    return true;
}

std::vector<diff_drive::Point<double>>
diff_drive::LocalNav::findObstacleEdges(const Robot &robot_pos, const LaserMeasurement& laser_measurement)
{
    int last_read_index = 0, first_valid_index = 0;
    double laser_dis, last_laser_dis ;
    double last_laser_angle, laser_angle ;
    bool first_valid_reading = true;
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
            if(first_valid_reading)
            {
                last_laser_dis = laser_measurement.Data[i].scanDistance/1000;
                last_laser_angle = deg2rad(laser_measurement.Data[i].scanAngle);
                first_valid_reading = false;
                first_valid_index = i;
            }

            if(fabs(laser_dis - last_laser_dis) > 0.40 || (fabs(laser_angle - last_laser_angle) > 0.25 && fabs(laser_angle - last_laser_angle) < 6))
            {
                point.x = (robot_pos.x + laser_measurement.Data[last_read_index].scanDistance/1000*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[last_read_index].scanAngle)));
                point.y = (robot_pos.y + laser_measurement.Data[last_read_index].scanDistance/1000*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[last_read_index].scanAngle)));
                points.emplace_back(point);
                normals_switch_.emplace_back(true);

                point.x = (robot_pos.x + laser_dis*cos(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                point.y = (robot_pos.y + laser_dis*sin(robot_pos.heading + deg2rad(-laser_measurement.Data[i].scanAngle)));
                points.emplace_back(point);
                normals_switch_.emplace_back(false);
            }

            last_laser_dis = laser_dis;
            last_laser_angle = laser_angle;
            last_read_index = i;
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

std::vector<diff_drive::Point<double>>
diff_drive::LocalNav::findEdgeNormals(const Robot &robot_pos, const std::vector<Point<double>>& edges, double edge_dis)
{
    diff_drive::Point<double> d_vector_norm;
    diff_drive::Point<double> n_vector_norm;
    diff_drive::Point<double> d_vector;
    diff_drive::Point<double> n_vector;
    std::vector<diff_drive::Point<double>> points;
    diff_drive::Point<double> point;

    for(int i = 0; i < edges.size(); i++)
    {
        d_vector = {edges[i].x - robot_pos.x, edges[i].y - robot_pos.y};
        n_vector = {-d_vector.y, d_vector.x};
        d_vector_norm = diff_drive::normalized(d_vector);
        n_vector_norm = diff_drive::normalized(n_vector);

        if(normals_switch_[i] == true)
        {
            point = edges[i] + -edge_dis*n_vector_norm;
        }
        else
        {
           point = edges[i] + edge_dis*n_vector_norm;
        }
        points.emplace_back(point);
    }

    return points;
}
diff_drive::Point<double>
diff_drive::LocalNav::findClosestAccessiblePoint(const Point<double>& goal, const Robot &robot_pos, const LaserMeasurement& laser_measurement, const std::vector<Point<double>>& normals, int* index)
{
    diff_drive::Point<double> point;
    double robot_to_goal_dis;
    double smallest_dis = 30000;
    bool is_path_clear;

    for(int i = 0; i < normals.size(); i++)
    {
        is_path_clear = isPathClear(normals[i], robot_pos, laser_measurement, 0.2);

        if(is_path_clear)
        {
            robot_to_goal_dis = getHeruisticDistance(normals[i],goal,robot_pos);

            if(robot_to_goal_dis < smallest_dis)
            {
                *index = i;
                point = normals[i];
                smallest_dis = robot_to_goal_dis;
            }
        }
    }

    return point;
}

double diff_drive::LocalNav::getHeruisticDistance(const Point<double> &normal, const Point<double> &goal, const Robot &robot_pos) const
{
    return  magnitude(reinterpret_cast<const Point<double>&>(robot_pos),normal) + magnitude(normal,goal);
}
