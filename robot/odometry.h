#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <iostream>
#include "robot_global.h"
#include "utility.h"
#define TICK_TO_METER 0.000085292090497737556558
namespace diff_drive
{
    /*struct Point {
        double x, y;
    };*/

    class ROBOT_EXPORT Odometry
    {
    public:
        Odometry();
        ~Odometry() = default;
        bool update(const unsigned short left_wheel_current_pos, const unsigned short right_wheel_current_pos);
        double getHeading() const;
        double getX() const;
        double getY() const;
        double getAngularSpeed() const;
        std::pair<double, double> ticksToMeters(short left_encoder, short right_encoder);
        void setWheelSeparation(double value);
        void setInitState(const unsigned short left_wheel_current_pos, const unsigned short right_wheel_current_pos);
        void setX(double value);
        void setY(double value);
        void setHeading(double value);

    private:
        void RungeKutta2(double dx_centroid, double dphi_centroid);
        void exactIntegration(double dx_centroid, double dphi_centroid);

    private:
        // Current pose:
        double x_;         // [m]
        double y_;         // [m]
        double heading_;   // [rad]

        // Centroid speeds:
        double angular_change_; // [rad/s]

        //Wheel parameters [m]:
        double wheel_separation_;

        //Old position of wheels [rad]:
        unsigned short left_wheel_old_pos_;
        unsigned short right_wheel_old_pos_;
    };
}

#endif // ODOMETRY_H
