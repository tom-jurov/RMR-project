#ifndef UTILITY_H
#define UTILITY_H
#include <cmath>

namespace diff_drive{
    template<class T>
    struct Point {
        T x, y;
    };

    static int sgn(int num)
    {
        if (num>=0)
        {
            return 1;
        }
        return -1;
    }

    template<class T>
    static double magnitude(const Point<T> &pt1, const Point<T> &pt2)
    {
        return sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y));
    }
}

#endif // UTILITY_H
