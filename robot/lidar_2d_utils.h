//
// Created by xiang on 2022/3/18.
//
#pragma once
#include <utility.h>
#include <opencv2/core/core.hpp>

void Visualize2DScan(std::shared_ptr<LaserMeasurement> scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size = 800,
                     float resolution = 20.0, const SE2& pose_submap = SE2());
