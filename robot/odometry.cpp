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
, left_wheel_overflow_count_(0)
, right_wheel_overflow_count_(0)
{
}

bool diff_drive::Odometry::update(const unsigned short left_wheel_current_pos, const unsigned short right_wheel_current_pos)
{
    auto [double_left_wheel_old_pos, double_right_wheel_old_pos] = ticksToMeters(left_wheel_old_pos_, right_wheel_old_pos_);
    if (left_wheel_old_pos_ - left_wheel_current_pos > (USHRT_MAX+1)/2)
    {
        left_wheel_overflow_count_++;
        std::cout << " Count of overflows " << left_wheel_overflow_count_ << std::endl;
    }
    if (right_wheel_old_pos_ - right_wheel_current_pos > (USHRT_MAX+1)/2)
    {
        right_wheel_overflow_count_++;
    }
    if (left_wheel_current_pos - left_wheel_old_pos_> (USHRT_MAX+1)/2)
    {
        left_wheel_overflow_count_--;
        std::cout << " Count of overflows " << left_wheel_overflow_count_ << std::endl;
    }
    if (right_wheel_current_pos - right_wheel_old_pos_ > (USHRT_MAX+1)/2)
    {
        right_wheel_overflow_count_--;
    }

    auto [double_left_wheel_current_pos, double_right_wheel_current_pos] = ticksToMeters(left_wheel_current_pos, right_wheel_current_pos);
    double dleft_wheel_pos = double_left_wheel_current_pos - double_left_wheel_old_pos;
    double dright_wheel_pos = double_right_wheel_current_pos - double_right_wheel_old_pos;

    left_wheel_old_pos_ = left_wheel_current_pos;
    right_wheel_old_pos_ = right_wheel_current_pos;

    double linear_change = (dright_wheel_pos + dleft_wheel_pos) * 0.5;
    double angular_change = (dright_wheel_pos - dleft_wheel_pos) / wheel_separation_;

    exactIntegration(linear_change, angular_change);

    return true;
}

void diff_drive::Odometry::RungeKutta2(double dx_centroid, double dphi_centroid)
{
    double direction = heading_ + dphi_centroid * 0.5;

    x_ += dx_centroid * cos(direction);
    y_ += dx_centroid * sin(direction);
    heading_ += dphi_centroid;
}

void diff_drive::Odometry::exactIntegration(double dx_centroid, double dphi_centroid)
{
    if (fabs(dphi_centroid) < 1e-6)
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

std::pair<double, double> diff_drive::Odometry::ticksToMeters(const unsigned short left_encoder, const unsigned short right_encoder)
{
    return std::make_pair(TICK_TO_METER*(left_encoder + left_wheel_overflow_count_*USHRT_MAX), TICK_TO_METER*(right_encoder + right_wheel_overflow_count_*USHRT_MAX));
}

void diff_drive::Odometry::setWheelSeparation(double value)
{
    wheel_separation_ = value;
}
