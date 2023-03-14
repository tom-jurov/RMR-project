#include "odometry.h"
#include <cmath>
#include <climits>
diff_drive::Odometry::Odometry()
: x_(0.0)
, y_(0.0)
, heading_(0.0)
, wheel_separation_(0.0)
, left_wheel_old_pos_(0)
, right_wheel_old_pos_(0)
, angular_change_(0)
{
}

bool diff_drive::Odometry::update(const unsigned short left_wheel_current_pos, const unsigned short right_wheel_current_pos)
{
    short left_diff, right_diff;

    if (left_wheel_old_pos_ - left_wheel_current_pos > (USHRT_MAX+1)/2)
    {
        left_diff = left_wheel_current_pos - left_wheel_old_pos_ + USHRT_MAX;
    }
    else if (left_wheel_current_pos - left_wheel_old_pos_> (USHRT_MAX+1)/2)
    {
        left_diff = left_wheel_current_pos - left_wheel_old_pos_ - USHRT_MAX;
    }
    else
    {
        left_diff = left_wheel_current_pos - left_wheel_old_pos_;
    }

    if (right_wheel_old_pos_ - right_wheel_current_pos > (USHRT_MAX+1)/2)
    {
        right_diff = right_wheel_current_pos - right_wheel_old_pos_ + USHRT_MAX;
    }
    else if (right_wheel_current_pos - right_wheel_old_pos_ > (USHRT_MAX+1)/2)
    {
        right_diff = right_wheel_current_pos - right_wheel_old_pos_ - USHRT_MAX;
    }
    else
    {
        right_diff = right_wheel_current_pos - right_wheel_old_pos_;
    }

    auto [dleft_wheel_pos, dright_wheel_pos] = ticksToMeters(left_diff, right_diff);

    left_wheel_old_pos_ = left_wheel_current_pos;
    right_wheel_old_pos_ = right_wheel_current_pos;

    double linear_change = (dright_wheel_pos + dleft_wheel_pos) * 0.5;
    angular_change_ = (dright_wheel_pos - dleft_wheel_pos) / wheel_separation_;
    exactIntegration(linear_change, angular_change_);

    return true;
}

void diff_drive::Odometry::RungeKutta2(double dx_centroid, double dphi_centroid)
{
    double direction = heading_ /*+ dphi_centroid * 0.5*/;
    x_ += dx_centroid * cos(direction);
    y_ += dx_centroid * sin(direction);
    heading_ += dphi_centroid;
}

void diff_drive::Odometry::exactIntegration(double dx_centroid, double dphi_centroid)
{
    if (fabs(dphi_centroid) < 15)
    {
        RungeKutta2(dx_centroid, dphi_centroid);
    }
    else
    {
        double heading_old = heading_;
        double r = dx_centroid / dphi_centroid;
        heading_ += dphi_centroid;
        x_ += r*(sin(heading_) - sin(heading_old));
        y_ += -r*(cos(heading_) - cos(heading_old));
    }
    if (heading_ > 2*M_PI)
    {
        heading_ = heading_ - 2*M_PI;
    }
    if (heading_ < 0)
    {
        heading_ = 2*M_PI - heading_;
    }
}

double diff_drive::Odometry::getHeading() const
{
    return heading_;
}

double diff_drive::Odometry::getX() const
{
    return x_;
}

double diff_drive::Odometry::getY() const
{
    return y_;
}

double diff_drive::Odometry::getAngularSpeed() const
{
    return angular_change_;
}

std::pair<double, double> diff_drive::Odometry::ticksToMeters(short left_encoder, short right_encoder)
{
    return std::make_pair(TICK_TO_METER*left_encoder, TICK_TO_METER*right_encoder);
}

void diff_drive::Odometry::setWheelSeparation(double value)
{
    wheel_separation_ = value;
}

void diff_drive::Odometry::setInitState(const unsigned short left_wheel_current_pos, const unsigned short right_wheel_current_pos)
{
    left_wheel_old_pos_ = left_wheel_current_pos;
    right_wheel_old_pos_ = right_wheel_current_pos;
}

void diff_drive::Odometry::setX(double value)
{
    x_ = value;
}

void diff_drive::Odometry::setY(double value)
{
    y_ = value;
}

void diff_drive::Odometry::setHeading(double value)
{
    heading_ = value;
}
