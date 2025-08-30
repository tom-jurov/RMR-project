#ifndef SLAM_IN_AUTO_DRIVING_FRAME_H
#define SLAM_IN_AUTO_DRIVING_FRAME_H

#include <eigen_types.h>
#include <rplidar.h>

struct ROBOT_EXPORT Frame {
    Frame() {}
    Frame(std::shared_ptr<LaserMeasurement> scan) : scan_(scan) {}

    size_t id_ = 0;             // scan id
    size_t keyframe_id_ = 0;    // 关键帧 id
    double timestamp_ = 0;      // 时间戳，一般不用
    std::shared_ptr<LaserMeasurement> scan_ = nullptr; // 激光扫描数据
    SE2 pose_;                  // 位姿，world to scan, T_w_c
    SE2 pose_submap_;           // 位姿，submap to scan, T_s_c
};

#endif  // SLAM_IN_AUTO_DRIVING_FRAME_H
