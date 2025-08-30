//
// Created by xiang on 2022/3/18.
//
#pragma once
#include <utility.h>
#include <rplidar.h>

struct ROBOT_EXPORT Frame {
    Frame() {}
    Frame(std::shared_ptr<LaserMeasurement> scan) : scan_(scan) {}

    size_t id_ = 0;
    size_t keyframe_id_ = 0;
    double timestamp_ = 0;
    std::shared_ptr<LaserMeasurement> scan_ = nullptr;
    SE2 pose_;
    SE2 pose_submap_;
};
