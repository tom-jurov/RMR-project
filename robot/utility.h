#ifndef UTILITY_H
#define UTILITY_H
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <Eigen/Core>
#include <robot_global.h>
#include <rplidar.h>

using Vec3b = Eigen::Matrix<char, 3, 1>;
using Vec3d = Eigen::Vector3d;
using Vec2d = Eigen::Vector2d;
using Vec2i = Eigen::Vector2i;

using Mat3d = Eigen::Matrix3d;

template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

struct SE2 {
    Eigen::Vector2d t;  // translation
    double theta;       // rotation angle in radians

    SE2() : t(Eigen::Vector2d::Zero()), theta(0) {}
    SE2(const Eigen::Vector2d& trans, double rot) : t(trans), theta(rot) {}
    SE2(double x, double y, double rot) : t(x, y), theta(rot) {}

    // Compose two SE2 transformations
    SE2 operator*(const SE2& other) const {
        double s = sin(theta);
        double c = cos(theta);
        Eigen::Vector2d new_t(c * other.t.x() - s * other.t.y() + t.x(),
                              s * other.t.x() + c * other.t.y() + t.y());
        double new_theta = wrapAngle(theta + other.theta);
        return SE2(new_t, new_theta);
    }

    // Transform a point
    Eigen::Vector2d operator*(const Eigen::Vector2d& pt) const {
        double s = sin(theta);
        double c = cos(theta);
        return Eigen::Vector2d(c * pt.x() - s * pt.y() + t.x(),
                               s * pt.x() + c * pt.y() + t.y());
    }

    // Inverse transformation
    SE2 inverse() const {
        double s = sin(theta);
        double c = cos(theta);
        Eigen::Vector2d inv_t(-c * t.x() - s * t.y(),
                              s * t.x() - c * t.y());
        double inv_theta = wrapAngle(-theta);
        return SE2(inv_t, inv_theta);
    }

    Eigen::Vector2d translation() const { return t; }
    void setTranslation(const Eigen::Vector2d& trans) { t = trans; }

    double rotation() const { return theta; }
    void setRotation(double rot) { theta = rot; }

private:
    static inline double wrapAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI); // shift to [0, 2π)
        if (angle < 0) angle += 2.0 * M_PI;         // make sure positive
        return angle - M_PI;                         // shift to [-π, π]
    }
};

namespace diff_drive{
struct Robot {
    double x, y, heading;
};

static constexpr inline int sgn(int num)
{
    if (num >= 0)
    {
        return 1;
    }
    return -1;
}

static constexpr inline double deg2rad(double degrees){
    return degrees * (M_PI/180);
}

static inline double wrapAngle(double angle)
{
    return angle - 2*M_PI * floor(angle / (2*M_PI));
}
}

#endif // UTILITY_H
