//
// Created by xiang on 2022/3/23.
//
#pragma once
#include "frame.h"
#include "likelihood_filed.h"
#include "occupancy_map.h"

class Submap {
public:
    Submap(const SE2& pose) : pose_(pose) {
        occu_map_.SetPose(pose_);
        field_.SetPose(pose_);
    }

    void SetOccuFromOtherSubmap(std::shared_ptr<Submap> other);

    bool MatchScan(std::shared_ptr<Frame> frame);

    bool HasOutsidePoints() const;

    void AddScanInOccupancyMap(std::shared_ptr<Frame> frame);

    void AddKeyFrame(std::shared_ptr<Frame> frame) { frames_.emplace_back(frame); }

    void UpdateFramePoseWorld();

    OccupancyMap& GetOccuMap() { return occu_map_; }
    LikelihoodField& GetLikelihood() { return field_; }

    std::vector<std::shared_ptr<Frame>>& GetFrames() { return frames_; }
    size_t NumFrames() const { return frames_.size(); }

    void SetId(size_t id) { id_ = id; }
    size_t GetId() const { return id_; }

    void SetPose(const SE2& pose);
    SE2 GetPose() const { return pose_; }

private:
    SE2 pose_;
    size_t id_ = 0;

    std::vector<std::shared_ptr<Frame>> frames_;
    LikelihoodField field_;
    OccupancyMap occu_map_;
};
