//
// Created by xiang on 2022/3/18.
//
#include "likelihood_filed.h"
#include <Eigen/Dense>
#include <cmath>

void LikelihoodField::SetTargetScan(std::shared_ptr<LaserMeasurement> scan) {
    target_ = scan;

    field_ = cv::Mat(1000, 1000, CV_32F, 30.0);

    for (int i = 0; i < scan->numberOfScans; ++i) {
        float dist_m = scan->Data[i].scanDistance * 0.001f;
        if (dist_m <= 0) {
            continue;
        }

        float real_angle = scan->Data[i].scanAngle * static_cast<float>(M_PI / 180.0f);
        double x = dist_m * std::cos(real_angle) * resolution_ + 500;
        double y = dist_m * std::sin(real_angle) * resolution_ + 500;

        for (auto& model_pt : model_) {
            int xx = int(x + model_pt.dx_);
            int yy = int(y + model_pt.dy_);
            if (xx >= 0 && xx < field_.cols && yy >= 0 && yy < field_.rows &&
                field_.at<float>(yy, xx) > model_pt.residual_) {
                field_.at<float>(yy, xx) = model_pt.residual_;
            }
        }
    }
}

void LikelihoodField::BuildModel() {
    const int range = 20;
    for (int x = -range; x <= range; ++x) {
        for (int y = -range; y <= range; ++y) {
            model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
        }
    }
}

void LikelihoodField::SetSourceScan(std::shared_ptr<LaserMeasurement> scan) { source_ = scan; }

bool LikelihoodField::AlignGaussNewton(SE2& init_pose) {
    const int iterations = 10;
    const double range_th = 15.0;    // max scan range [m]
    const double huber_delta = 0.8;  // Huber threshold
    const int min_effect_pts = 20;
    const int image_border = 20;

    SE2 current_pose = init_pose;
    has_outside_pts_ = false;

    for (int iter = 0; iter < iterations; ++iter) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        double cost = 0;
        int effective_num = 0;

        double theta = current_pose.rotation();
        for (int i = 0; i < source_->numberOfScans; ++i) {
            float r = source_->Data[i].scanDistance * 0.001f;  // mm -> m
            if (r <= 0 || r > range_th) continue;

            float angle = source_->Data[i].scanAngle * static_cast<float>(M_PI / 180.0f);

            // Transform point to world frame
            Eigen::Vector2d pt_local(r * cos(angle), r * sin(angle));
            Eigen::Vector2d pw = current_pose * pt_local;

            // Convert to pixel coordinates
            Eigen::Vector2d pf_d = pw * resolution_ + Eigen::Vector2d(500, 500);
            Eigen::Vector2i pf = pf_d.cast<int>();

            if (pf[0] >= image_border && pf[0] < field_.cols - image_border &&
                pf[1] >= image_border && pf[1] < field_.rows - image_border) {

                effective_num++;

                // Compute finite differences for gradient
                float dx = 0.5f * (field_.at<float>(pf[1], pf[0] + 1) - field_.at<float>(pf[1], pf[0] - 1));
                float dy = 0.5f * (field_.at<float>(pf[1] + 1, pf[0]) - field_.at<float>(pf[1] - 1, pf[0]));

                // Jacobian w.r.t [x, y, theta]
                Eigen::Vector3d J;
                J << resolution_ * dx,
                    resolution_ * dy,
                    -resolution_ * dx * r * sin(angle + theta) + resolution_ * dy * r * cos(angle + theta);

                float e = field_.at<float>(pf[1], pf[0]);

                // Huber weighting
                float weight = 1.0f;
                if (std::abs(e) > huber_delta) weight = huber_delta / std::abs(e);

                H += weight * J * J.transpose();
                b += -weight * J * e;

                cost += weight * e * e;
            } else {
                has_outside_pts_ = true;
            }
        }

        if (effective_num < min_effect_pts) return false;

        Eigen::Vector3d dx = H.ldlt().solve(b);
        if (std::isnan(dx[0])) break;

        // Update pose using Euler angles
        Eigen::Vector2d t_new = current_pose.translation() + dx.head<2>();
        double theta_new = current_pose.rotation() + dx[2];
        current_pose.setTranslation(t_new);
        current_pose.setRotation(theta_new);

        cost /= effective_num;
    }

    init_pose = current_pose;
    return true;
}


cv::Mat LikelihoodField::GetFieldImage() {
    cv::Mat image(field_.rows, field_.cols, CV_8UC3);
    for (int x = 0; x < field_.cols; ++x) {
        for (int y = 0; y < field_.rows; ++y) {
            float r = field_.at<float>(y, x) * 255.0 / 30.0;
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
        }
    }

    return image;
}

void LikelihoodField::SetFieldImageFromOccuMap(const cv::Mat& occu_map) {
    const int boarder = 25;
    field_ = cv::Mat(1000, 1000, CV_32F, 30.0);

    for (int x = boarder; x < occu_map.cols - boarder; ++x) {
        for (int y = boarder; y < occu_map.rows - boarder; ++y) {
            if (occu_map.at<uchar>(y, x) < 127) {
                for (auto& model_pt : model_) {
                    int xx = int(x + model_pt.dx_);
                    int yy = int(y + model_pt.dy_);
                    if (xx >= 0 && xx < field_.cols && yy >= 0 && yy < field_.rows &&
                        field_.at<float>(yy, xx) > model_pt.residual_) {
                        field_.at<float>(yy, xx) = model_pt.residual_;
                    }
                }
            }
        }
    }
}
