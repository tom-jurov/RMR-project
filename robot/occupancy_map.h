//
// Created by xiang on 2022/3/23.
//
#pragma once
#include <opencv2/core.hpp>
#include "frame.h"
#include <math_utils.h>

class ROBOT_EXPORT OccupancyMap {
public:
    struct Model2DPoint {
        int dx_ = 0;
        int dy_ = 0;
        double angle_ = 0;  // in rad
        float range_ = 0;   // in meters
    };

    enum class GridMethod {
        MODEL_POINTS,
        BRESENHAM,
    };

    OccupancyMap();

    void AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method = GridMethod::BRESENHAM);

    cv::Mat GetOccupancyGrid() const { return occupancy_grid_; }

    cv::Mat GetOccupancyGridBlackWhite() const;

    void SetPose(const SE2& pose) { pose_ = pose; }

    bool HasOutsidePoints() const { return has_outside_pts_; }

    double Resolution() const { return resolution_; }

    void SetPoint(const Vec2i& pt, bool occupy);

private:
    void BuildModel();

    template <class T>
    inline Vec2i World2Image(const Eigen::Matrix<T, 2, 1>& pt) {
        Vec2d pt_map = (pose_.inverse() * pt) * resolution_ + center_image_;
        int x = int(pt_map[0]);
        int y = int(pt_map[1]);
        return Vec2i(x, y);
    }

    double FindRangeInAngle(double angle, std::shared_ptr<LaserMeasurement> scan);

    void BresenhamFilling(const Vec2i& p1, const Vec2i& p2);

    cv::Mat occupancy_grid_;  // 8bit

    SE2 pose_;  // T_W_S
    Vec2d center_image_ = Vec2d(image_size_ / 2, image_size_ / 2);

    bool has_outside_pts_ = false;

    std::vector<Model2DPoint> model_;

    inline static constexpr double closest_th_ = 0.2;
    inline static constexpr double endpoint_close_th_ = 0.1;
    inline static constexpr double resolution_ = 20.0;
    inline static constexpr float inv_resolution_ = 0.05;
    inline static constexpr int image_size_ = 1000;
    inline static constexpr int model_size_ = 400;
};
