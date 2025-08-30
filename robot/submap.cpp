//
// Created by xiang on 2022/3/23.
//
#include "submap.h"

void Submap::SetOccuFromOtherSubmap(std::shared_ptr<Submap> other) {
    auto frames_in_other = other->GetFrames();
    for (size_t i = frames_in_other.size() - 10; i < frames_in_other.size(); ++i) {
        if (i > 0) {
            occu_map_.AddLidarFrame(frames_in_other[i]);
        }
    }
    field_.SetFieldImageFromOccuMap(occu_map_.GetOccupancyGrid());
}

bool Submap::MatchScan(std::shared_ptr<Frame> frame) {
    field_.SetSourceScan(frame->scan_);
    field_.AlignGaussNewton(frame->pose_submap_);
    frame->pose_ = pose_ * frame->pose_submap_;  // T_w_c = T_w_s * T_s_c

    return true;
}

void Submap::AddScanInOccupancyMap(std::shared_ptr<Frame> frame) {
    occu_map_.AddLidarFrame(frame, OccupancyMap::GridMethod::MODEL_POINTS);
    field_.SetFieldImageFromOccuMap(occu_map_.GetOccupancyGrid());
}

bool Submap::HasOutsidePoints() const { return occu_map_.HasOutsidePoints(); }

void Submap::SetPose(const SE2& pose) {
    pose_ = pose;
    occu_map_.SetPose(pose);
    field_.SetPose(pose);
}

void Submap::UpdateFramePoseWorld() {
    for (auto& frame : frames_) {
        frame->pose_ = pose_ * frame->pose_submap_;
    }
}
