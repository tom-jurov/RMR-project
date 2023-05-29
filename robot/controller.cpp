#include "controller.h"
#include <cmath>
diff_drive::Controller::Controller()
:   path_(std::vector<Point<double>>{})
,   current_state_(Robot{0,0,0})
,   look_ahead_dist_(0.3)
,   last_found_index_(0)
,   linear_velocity_(0)
,   goal_velocity_(700)
,   treshold_(0.3)
{
}

void diff_drive::Controller::setCurrentPosition(const diff_drive::Odometry &odom)
{
    current_state_ = {odom.getX(), odom.getY(), odom.getHeading()};
}

void diff_drive::Controller::setPath(const std::vector<Point<double>> &path)
{
    path_ = path;
}

void diff_drive::Controller::setGoal(const diff_drive::Point<double> &goal)
{
    goal_ = goal;
}

diff_drive::CTRL_Output diff_drive::Controller::controlStep()
{
    /*bool intersection_found = false;
    int starting_index = last_found_index_;

    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    double dx = 0;
    double dy = 0;
    double dr = 0;

    double D = 0;
    double discriminant = 0;
    double solution_x1 = 0;
    double solution_x2 = 0;
    double solution_y1 = 0;
    double solution_y2 = 0;

    double min_x = 0;
    double min_y = 0;
    double max_x = 0;
    double max_y = 0;

    Point<double> solution_point_1{};
    Point<double> solution_point_2{};

    Point<double> goal_point{};
    for (std::size_t i = starting_index; i < path_.size() - 1; ++i)
    {
        x1 = path_[i].x - current_state_.x;
        y1 = path_[i].y - current_state_.y;
        x2 = path_[i+1].x - current_state_.x;
        y2 = path_[i+1].y - current_state_.y;
        dx = x2 - x1;
        dy = y2 - y1;
        dr = sqrt (dx*dx + dy*dy);

        D = x1*y2 - x2*y1;
        discriminant = (look_ahead_dist_ * look_ahead_dist_) * (dr * dr) - (D * D);
        if (discriminant >= 0)
        {
            solution_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            solution_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            solution_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / (dr * dr);
            solution_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / (dr * dr);

            solution_point_1 = {solution_x1 + current_state_.x, solution_y1 + current_state_.y};
            solution_point_2 = {solution_x2 + current_state_.x, solution_y2 + current_state_.y};
            min_x = std::min(path_[i].x, path_[i+1].x);
            min_y = std::min(path_[i].y, path_[i+1].y);
            max_x = std::max(path_[i].x, path_[i+1].x);
            max_y = std::max(path_[i].y, path_[i+1].y);
            if (((min_x <= solution_point_1.x && solution_point_1.x <= max_x) && (min_y <= solution_point_1.y && solution_point_1.y <= max_y)) ||
                ((min_x <= solution_point_2.x && solution_point_2.x <= max_x) && (min_y <= solution_point_2.y && solution_point_2.y <= max_y)))
            {
                intersection_found = true;
                if (((min_x <= solution_point_1.x && solution_point_1.x <= max_x) && (min_y <= solution_point_1.y && solution_point_1.y <= max_y)) &&
                    ((min_x <= solution_point_2.x && solution_point_2.x <= max_x) && (min_y <= solution_point_2.y && solution_point_2.y <= max_y)))
                {
                    if (magnitude(solution_point_1, path_[i+1]) < magnitude(solution_point_2, path_[i+1]))
                    {
                        goal_point = solution_point_1;
                    }
                    else
                    {
                        goal_point = solution_point_2;
                    }
                }
                else
                {
                    if ((min_x <= solution_point_1.x && solution_point_1.x <= max_x) && (min_y <= solution_point_1.y && solution_point_1.y <= max_y))
                    {
                        goal_point = solution_point_1;
                    }
                    else
                    {
                        goal_point = solution_point_2;
                    }
                }
                if (magnitude(goal_point, path_[i+1]) < magnitude(Point<double>{current_state_.x, current_state_.y},path_[i+1]))
                {
                    last_found_index_ = i;
                    break;
                }
                else
                {
                    last_found_index_ = i+1;
                }
            }
        }
        else
        {
            intersection_found = false;
            goal_point = {path_[last_found_index_].x, path_[last_found_index_].y};
        }
    }*/
    diff_drive::Point<double> goal_point = path_[1];

    double target_angle = atan2( (goal_point.y - current_state_.y), (goal_point.x - current_state_.x) );

    if (target_angle < 0)
    {
        target_angle += 2*M_PI;
    }

    double turn_error = target_angle - current_state_.heading;

    if (turn_error > M_PI || turn_error < -M_PI)
    {
        turn_error = -1 * sgn(turn_error) * (2*M_PI - fabs(turn_error));
    }
    double distance_to_go = magnitude(goal_, reinterpret_cast<Point<double>&>(current_state_));
    if (distance_to_go < treshold_)
    {
        if (distance_to_go < 0.05)
        {
            linear_velocity_ = 0;
            last_found_index_ = 0;
            current_state_ = Robot{0,0,0};
            return {0,0};
        }
        int speed_down = 0.5*linear_velocity_/treshold_*distance_to_go;
        int local_look_ahead = look_ahead_dist_/treshold_*distance_to_go;
        double r = 1000*local_look_ahead/(2*sin(turn_error));
        return {speed_down,static_cast<int>(r)};
    }

    if (linear_velocity_ < goal_velocity_)
        linear_velocity_ += static_cast<int>(goal_velocity_/70);
    double r = 1000*look_ahead_dist_/(2*sin(turn_error));
    int vel = static_cast<int>(linear_velocity_*fabs(r)*0.001);
    if (vel>700)
    {
        vel = 700;
    }
    return {vel, static_cast<int>(r)};
}
