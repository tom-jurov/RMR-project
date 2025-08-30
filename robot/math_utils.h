#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <opencv2/core.hpp>
namespace math{
    inline void KeepAngleInPI(double& angle) {
        while (angle < -M_PI) {
        angle = angle + 2 * M_PI;
        }
        while (angle > M_PI) {
        angle = angle - 2 * M_PI;
        }
    }

    // bilinear interpolation
    template <typename T>
    inline float GetPixelValue(const cv::Mat& img, float x, float y) {
        // boundary check
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x >= img.cols) x = img.cols - 1;
        if (y >= img.rows) y = img.rows - 1;
        const T* data = &img.at<T>(floor(y), floor(x));
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] + (1 - xx) * yy * data[img.step / sizeof(T)] +
             xx * yy * data[img.step / sizeof(T) + 1]);
    }
}
