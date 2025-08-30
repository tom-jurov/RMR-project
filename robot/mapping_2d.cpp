//
// Created by xiang on 2022/3/23.
//
#include "mapping_2d.h"
#include "lidar_2d_utils.h"
#include "submap.h"
#include <opencv2/opencv.hpp>

bool Mapping2D::Init(bool with_loop_closing) {
    keyframe_id_ = 0;
    current_submap_ = std::make_shared<Submap>(SE2());
    all_submaps_.emplace_back(current_submap_);

    return true;
}

// void Mapping2D::UndistortScan(LaserMeasurement &measurement){
//     if (measurement.numberOfScans == 0) return;

//     for (int i = 0; i < measurement.numberOfScans; ++i) {
//         double t = static_cast<double>(measurement.Data[i].timestamp) * 1e-6;
//         std::cout << "Cas: " << t << std::endl;

//         Sophus::SE2d interp_pose;
//         if (!odom_buffer.Interpolate(t, interp_pose)) continue;

//         double r = measurement.Data[i].scanDistance;
//         double angle_rad = measurement.Data[i].scanAngle * M_PI / 180.0;
//         Eigen::Vector2d point(std::cos(angle_rad) * r, std::sin(angle_rad) * r);

//         Eigen::Vector2d deskewed = interp_pose * point;

//         measurement.Data[i].scanDistance = deskewed.norm();
//         measurement.Data[i].scanAngle = std::atan2(deskewed.y(), deskewed.x()) * RAD2DEG;
//     }
// }

void Mapping2D::UndistortScan(LaserMeasurement &measurement) {
    if (measurement.numberOfScans == 0) return;

    // Get previous and current poses
    SE2 pose_prev, pose_curr;
    odom_buffer.GetDeskewPoses(pose_prev, pose_curr);

    // relative transformation from previous to current pose
    SE2 relative = pose_prev.inverse() * pose_curr;

    // timestamps of first and last point in the scan (microseconds)
    double t_start = static_cast<double>(measurement.Data[0].timestamp);
    double t_end   = static_cast<double>(measurement.Data[measurement.numberOfScans - 1].timestamp);
    double dt = t_end - t_start;
    if (dt <= 0) dt = 1.0; // avoid division by zero

    for (int i = 0; i < measurement.numberOfScans; ++i) {
        // interpolation factor alpha
        double alpha = (static_cast<double>(measurement.Data[i].timestamp) - t_start) / dt;
        alpha = std::clamp(alpha, 0.0, 1.0);

        // interpolate translation and rotation linearly
        Eigen::Vector2d interp_t = alpha * relative.translation();
        double interp_theta = alpha * relative.rotation();

        SE2 interp_relative(interp_t, interp_theta);

        // convert polar coordinates to cartesian
        double r = measurement.Data[i].scanDistance;
        double angle_rad = measurement.Data[i].scanAngle * M_PI / 180.0;
        Eigen::Vector2d point(std::cos(angle_rad) * r, std::sin(angle_rad) * r);

        // apply interpolated relative motion
        Eigen::Vector2d deskewed = interp_relative * point;

        // convert back to polar
        measurement.Data[i].scanDistance = deskewed.norm();
        measurement.Data[i].scanAngle = std::atan2(deskewed.y(), deskewed.x()) * 180.0 / M_PI;
    }
}

bool Mapping2D::ProcessScan(std::shared_ptr<LaserMeasurement> scan) {
    current_frame_ = std::make_shared<Frame>(scan);
    current_frame_->id_ = frame_id_++;

    if (last_frame_) {
        current_frame_->pose_ = last_frame_->pose_ * motion_guess_;
        current_frame_->pose_submap_ = last_frame_->pose_submap_;
        UndistortScan(*current_frame_->scan_);
    }

    if (!first_scan_) {
        current_submap_->MatchScan(current_frame_);
    }

    first_scan_ = false;
    bool is_kf = IsKeyFrame();

    if (is_kf) {
        AddKeyFrame();
        std::cout << "Pridal keyframe" << std::endl;
        current_submap_->AddScanInOccupancyMap(current_frame_);
    }
    std::cout << "Aktualna pozicia: " << current_frame_->pose_.translation().transpose() << std::endl;

    auto occu_image = current_submap_->GetOccuMap().GetOccupancyGridBlackWhite();
    Visualize2DScan(current_frame_->scan_, current_frame_->pose_, occu_image, Vec3b(0, 0, 255), 1000, 20.0,
                    current_submap_->GetPose());
    cv::putText(occu_image, "submap " + std::to_string(current_submap_->GetId()), cv::Point2f(20, 20),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
    cv::putText(occu_image, "keyframes " + std::to_string(current_submap_->NumFrames()), cv::Point2f(20, 50),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
    cv::imshow("occupancy map", occu_image);

    auto field_image = current_submap_->GetLikelihood().GetFieldImage();
    Visualize2DScan(current_frame_->scan_, current_frame_->pose_, field_image, Vec3b(0, 0, 255), 1000, 20.0,
                    current_submap_->GetPose());
    cv::imshow("likelihood", field_image);

    cv::waitKey(10);

    if (last_frame_) {
        motion_guess_ = last_frame_->pose_.inverse() * current_frame_->pose_;
    }

    last_frame_ = current_frame_;

    return true;
}

bool Mapping2D::IsKeyFrame() {
    if (last_keyframe_ == nullptr) {
        return true;
    }

    SE2 delta_pose = last_keyframe_->pose_.inverse() * current_frame_->pose_;
    if (delta_pose.translation().norm() > keyframe_pos_th_ || fabs(delta_pose.rotation()) > keyframe_ang_th_) {
        return true;
    }

    return false;
}

void Mapping2D::AddKeyFrame() {
    std::cout << "add keyframe " << keyframe_id_ << std::endl;
    current_frame_->keyframe_id_ = keyframe_id_++;
    current_submap_->AddKeyFrame(current_frame_);
    last_keyframe_ = current_frame_;
}
