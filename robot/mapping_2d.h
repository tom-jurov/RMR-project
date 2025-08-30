//
// Created by xiang on 2022/3/23.
//

#ifndef SLAM_IN_AUTO_DRIVING_MAPPING_2D_H
#define SLAM_IN_AUTO_DRIVING_MAPPING_2D_H

#include "frame.h"
#include "eigen_types.h"

#include <memory>
#include <opencv2/core.hpp>
#include <deque>

class Submap;
class LoopClosing;


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
    void AddOdometry(const Sophus::SE2d &pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        prev_pose_ = curr_pose_;
        curr_pose_ = pose;
    }

    void GetDeskewPoses(Sophus::SE2d &prev, Sophus::SE2d &curr) const {
        std::lock_guard<std::mutex> lock(mutex_);
        prev = prev_pose_;
        curr = curr_pose_;
    }

private:
    mutable std::mutex mutex_;
    Sophus::SE2d prev_pose_ = Sophus::SE2d();  // identity
    Sophus::SE2d curr_pose_ = Sophus::SE2d();  // identity
};

/**
 * 2D 激光建图的主要类
 */
class ROBOT_EXPORT Mapping2D {
public:
    bool Init(bool with_loop_closing = true);

    /// 单回波的scan
    bool ProcessScan(std::shared_ptr<LaserMeasurement> scan);

    /**
     * 显示全局地图
     * @param max_size 全局地图最大长宽
     * @return 全局地图图像
     */
    cv::Mat ShowGlobalMap(int max_size = 500);
    OdometryBuffer odom_buffer;

private:
    /// 判定当前帧是否为关键帧
    bool IsKeyFrame();

    /// 增加一个关键帧
    void AddKeyFrame();

    /// 扩展新的submap
    void ExpandSubmap();
    void UndistortScan(LaserMeasurement &measurement);

    /// 数据成员
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

    std::shared_ptr<LoopClosing> loop_closing_ = nullptr;  // 回环检测

    // 参数
    inline static constexpr double keyframe_pos_th_ = 0.3;              // 关键帧位移量
    inline static constexpr double keyframe_ang_th_ = 15 * M_PI / 180;  // 关键帧角度量
};


#endif  // SLAM_IN_AUTO_DRIVING_MAPPING_2D_H
