#ifndef UTILITY_H
#define UTILITY_H
#define _USE_MATH_DEFINES
#include <math.h>

namespace diff_drive{
    template<class T>
    struct Point {
        T x, y;
    };

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

    template<class T>
    static constexpr inline double magnitude(const Point<T> &pt1, const Point<T> &pt2)
    {
        return sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y));
    }

    static constexpr inline double deg2rad(double degrees){
        return degrees * (M_PI/180);
    }

    static inline double wrapAngle(double angle)
    {
        return angle - 2*M_PI * floor(angle / (2*M_PI));
    }

    static inline double norm(const Point<double> &vec)
    {
        return std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2));
    }

    static inline Point<double> normalized(const Point<double> &vec)
    {
        Point<double> normalized;
        double norm  = diff_drive::norm(vec);
        normalized = {vec.x / norm, vec.y / norm};
        return normalized;
    }

    static Point<double> operator+ (const Point<double>& v1, const Point<double>& v2)
    {
        return {v1.x + v2.x, v1.y + v2.y};
    }
    static Point<double> operator* (double scalar, const Point<double>& v)
    {
        return {scalar*v.x, scalar*v.y};
    }
}

#endif // UTILITY_H
