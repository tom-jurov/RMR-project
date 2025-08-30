//
// Created by xiang on 2022/3/18.
//
#pragma once
#include <opencv2/core.hpp>
#include <utility.h>

class ROBOT_EXPORT LikelihoodField {
public:
    struct ModelPoint {
        ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
        int dx_ = 0;
        int dy_ = 0;
        float residual_ = 0;
    };

    LikelihoodField() { BuildModel(); }

    void SetTargetScan(std::shared_ptr<LaserMeasurement> scan);

    void SetSourceScan(std::shared_ptr<LaserMeasurement> scan);

    void SetFieldImageFromOccuMap(const cv::Mat& occu_map);

    bool AlignGaussNewton(SE2& init_pose);

    cv::Mat GetFieldImage();

    bool HasOutsidePoints() const { return has_outside_pts_; }

    void SetPose(const SE2& pose) { pose_ = pose; }

private:
    void BuildModel();

    SE2 pose_;  // T_W_S
    std::shared_ptr<LaserMeasurement> target_ = nullptr;
    std::shared_ptr<LaserMeasurement> source_ = nullptr;

    std::vector<ModelPoint> model_;
    cv::Mat field_;
    bool has_outside_pts_ = false;

    inline static const float resolution_ = 20;
};
