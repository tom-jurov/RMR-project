#include "controller.h"
#include <cmath>
diff_drive::Controller::Controller()
:   path_(std::vector<Point>{})
,   current_state_(Robot{0,0,0})
,   look_ahead_dist_(0.1)
,   last_found_index_(0)
,   linear_velocity_(200)
{
}

void diff_drive::Controller::setCurrentPosition(const diff_drive::Odometry &odom)
{
    current_state_ = {odom.getX(), odom.getY(), odom.getHeading()};
}

void diff_drive::Controller::setPath(const std::vector<Point> &path)
{
    path_ = path;
}

diff_drive::CTRL_Output diff_drive::Controller::controlStep()
{
    bool intersection_found = false;
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
    double sol_x1 = 0;
    double sol_x2 = 0;
    double sol_y1 = 0;
    double sol_y2 = 0;

    double minX = 0;
    double minY = 0;
    double maxX = 0;
    double maxY = 0;

    Point sol_pt1{};
    Point sol_pt2{};

    Point goal_pt{};
    for (std::size_t i = starting_index; i<path_.size(); ++i)
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
            sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / (dr * dr);
            sol_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / (dr * dr);
            sol_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / (dr * dr);

            sol_pt1 = {sol_x1 + current_state_.x, sol_y1 + current_state_.y};
            sol_pt2 = {sol_x2 + current_state_.x, sol_y2 + current_state_.y};
            minX = std::min(path_[i].x, path_[i+1].x);
            minY = std::min(path_[i].y, path_[i+1].y);
            maxX = std::max(path_[i].x, path_[i+1].x);
            maxY = std::max(path_[i].y, path_[i+1].y);
            if (((minX <= sol_pt1.x && sol_pt1.x <= maxX) && (minY <= sol_pt1.y && sol_pt1.y <= maxY)) ||
                ((minX <= sol_pt2.x && sol_pt2.x <= maxX) && (minY <= sol_pt2.y && sol_pt2.y <= maxY)))
            {
                intersection_found = true;
                if (((minX <= sol_pt1.x && sol_pt1.x <= maxX) && (minY <= sol_pt1.y && sol_pt1.y <= maxY)) &&
                    ((minX <= sol_pt2.x && sol_pt2.x <= maxX) && (minY <= sol_pt2.y && sol_pt2.y <= maxY)))
                {
                    if (magnitude(sol_pt1, path_[i+1]) < magnitude(sol_pt2, path_[i+1]))
                    {
                        goal_pt = sol_pt1;
                    }
                    else
                    {
                        goal_pt = sol_pt2;
                    }
                }
                else
                {
                    if ((minX <= sol_pt1.x && sol_pt1.x <= maxX) && (minY <= sol_pt1.y && sol_pt1.y <= maxY))
                    {
                        goal_pt = sol_pt1;
                    }
                    else
                    {
                        goal_pt = sol_pt2;
                    }
                }
                if (magnitude(goal_pt, path_[i+1]) < magnitude(Point{current_state_.x, current_state_.y},path_[i+1]))
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
            goal_pt = {path_[last_found_index_].x, path_[last_found_index_].y};
        }
    }

    if ( last_found_index_ == path_.size()-1)
    {
        if ( fabs(path_.back().x - current_state_.x) < 0.04 && fabs(path_.back().y - current_state_.y) < 0.04)
        {
            return {0,0};
        }
    }
    double Kp = 0.2;
    double target_angle = atan2( (goal_pt.y - current_state_.y), (goal_pt.x - current_state_.x) );

    if (target_angle < 0)
    {
        target_angle += 2*M_PI;
    }

    double turn_error = target_angle - current_state_.heading;

    if (turn_error > M_PI || turn_error < -M_PI)
    {
        turn_error = -1 * sgn(turn_error) * (2*M_PI - fabs(turn_error));
    }

    double turn_vel = Kp * turn_error;
    return {linear_velocity_, static_cast<int>(linear_velocity_ / turn_vel)};
}

int diff_drive::Controller::sgn(int num)
{
    if (num>=0)
    {
        return 1;
    }
    return -1;
}

double diff_drive::Controller::magnitude(const Point &pt1, const Point &pt2)
{
    return sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y));
}
