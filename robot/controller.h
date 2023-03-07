#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "odometry.h"
#include <vector>
namespace diff_drive{
    struct Point {
        double x, y;
    };

    struct Robot {
        double x, y, heading;
    };
    struct CTRL_Output{
        int speed, radius;
    };

    class Controller
    {
    public:
        Controller();
        ~Controller() = default;
        void setCurrentPosition(const diff_drive::Odometry& odom);
        void setPath(const std::vector<Point>& path);
        CTRL_Output controlStep();
    private:
        static int sgn(int num);
        static double magnitude(const Point& pt1, const Point& pt2);
    private:
        std::vector<Point> path_;
        Robot current_state_;
        double look_ahead_dist_;
        unsigned int last_found_index_;
        int linear_velocity_;
        double treshold_;
    };
}

#endif // CONTROLLER_H
