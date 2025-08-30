//
// Created by xiang on 2022/3/23.
//
#pragma once
#include "frame.h"
#include <utility.h>
#include <memory>
#include <opencv2/core.hpp>
#include <deque>
class Submap;

// class OdometryBuffer {
// public:
//     void AddOdometry(double timestamp, const Sophus::SE2d &pose) {
//         std::lock_guard<std::mutex> lock(mutex_);
//         buffer_.emplace_back(timestamp, pose);
//         while (buffer_.size() > 1000) buffer_.pop_front();
//     }

//     bool Interpolate(double timestamp, Sophus::SE2d &interp_pose) const {
//         std::lock_guard<std::mutex> lock(mutex_);
//         if (buffer_.size() < 2) return false;

//         auto it = std::upper_bound(buffer_.begin(), buffer_.end(), timestamp,
//                                    [](double t, const std::pair<double, Sophus::SE2d> &p){ return t < p.first; });

//         if (it == buffer_.begin() || it == buffer_.end()) return false;

//         auto prev = it - 1;
//         auto next = it;

//         double t0 = prev->first;
//         double t1 = next->first;
//         const Sophus::SE2d &pose0 = prev->second;
//         const Sophus::SE2d &pose1 = next->second;

//         double alpha = (timestamp - t0) / (t1 - t0);
//         Sophus::SE2d rel = pose0.inverse() * pose1;
//         interp_pose = pose0 * Sophus::SE2d::exp(alpha * rel.log());
//         return true;
//     }

// private:
//     mutable std::mutex mutex_;
//     std::deque<std::pair<double, Sophus::SE2d>> buffer_;
// };
class OdometryBuffer {
public:
    void AddOdometry(const SE2 &pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        prev_pose_ = curr_pose_;
        curr_pose_ = pose;
    }

    void GetDeskewPoses(SE2 &prev, SE2 &curr) const {
        std::lock_guard<std::mutex> lock(mutex_);
        prev = prev_pose_;
        curr = curr_pose_;
    }

private:
    mutable std::mutex mutex_;
    SE2 prev_pose_ = SE2();  // identity (t = [0,0], theta = 0)
    SE2 curr_pose_ = SE2();  // identity
};

class ROBOT_EXPORT Mapping2D {
public:
    bool Init(bool with_loop_closing = true);

    bool ProcessScan(std::shared_ptr<LaserMeasurement> scan);

    OdometryBuffer odom_buffer;

private:
    bool IsKeyFrame();

    void AddKeyFrame();

    void ExpandSubmap();
    void UndistortScan(LaserMeasurement &measurement);

    size_t frame_id_ = 0;
    size_t keyframe_id_ = 0;
    size_t submap_id_ = 0;

    bool first_scan_ = true;
    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> last_frame_ = nullptr;
    SE2 motion_guess_;
    std::shared_ptr<Frame> last_keyframe_ = nullptr;
    std::shared_ptr<Submap> current_submap_ = nullptr;

    std::vector<std::shared_ptr<Submap>> all_submaps_;

    inline static constexpr double keyframe_pos_th_ = 0.3;              // 关键帧位移量
    inline static constexpr double keyframe_ang_th_ = 15 * M_PI / 180;  // 关键帧角度量
};
