#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "odometry.h"
#include "robot_global.h"
#include <vector>
#include "utility.h"
namespace diff_drive{
    struct CTRL_Output{
        int speed, radius;
    };

    class ROBOT_EXPORT Controller
    {
    public:
        Controller();
        ~Controller() = default;
        void setCurrentPosition(const diff_drive::Odometry& odom);
        void setPath(const std::vector<Point<double>>& path);
        void setGoal(const diff_drive::Point<double>& goal);
        CTRL_Output controlStep();

    private:
        std::vector<Point<double>> path_;
        Robot current_state_;
        double look_ahead_dist_;
        unsigned int last_found_index_;
        int linear_velocity_;
        double goal_velocity_;
        double treshold_;
        diff_drive::Point<double> goal_;
    };
}

#endif // CONTROLLER_H
