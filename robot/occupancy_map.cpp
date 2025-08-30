//
// Created by xiang on 2022/3/23.
//
#define NOMINMAX
#include "occupancy_map.h"
#include <utility.h>
#include <execution>
#include <set>
#include <cmath>

OccupancyMap::OccupancyMap() {
    BuildModel();
    occupancy_grid_ = cv::Mat(image_size_, image_size_, CV_8U, 127);
}

void OccupancyMap::BuildModel() {
    for (int x = -model_size_; x <= model_size_; x++) {
        for (int y = -model_size_; y <= model_size_; y++) {
            Model2DPoint pt;
            pt.dx_ = x;
            pt.dy_ = y;
            pt.range_ = sqrt(x * x + y * y) * inv_resolution_;
            pt.angle_ = std::atan2(y, x);
            pt.angle_ = pt.angle_ > M_PI ? pt.angle_ - 2 * M_PI : pt.angle_;  // limit in 2pi
            model_.push_back(pt);
        }
    }
}

double OccupancyMap::FindRangeInAngle(double angle, std::shared_ptr<LaserMeasurement> scan) {
    math::KeepAngleInPI(angle);

    // Find the closest angle in the laser measurement
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    for (int i = 0; i < scan->numberOfScans; ++i) {
        double current_angle = scan->Data[i].scanAngle * static_cast<float>(M_PI / 180.0f);
        double diff = std::abs(current_angle - angle);
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = i;
        }
    }

    if (closest_index != -1) {
        float dist_m = scan->Data[closest_index].scanDistance * 0.001f;
        if (dist_m > 0) {
            return dist_m;
        }
    }
    return 0.0;
}

void OccupancyMap::AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method) {
    auto& scan = frame->scan_;

    SE2 pose_in_submap = pose_.inverse() * frame->pose_;
    float theta = pose_in_submap.theta;
    has_outside_pts_ = false;

    std::set<Vec2i, less_vec<2>> endpoints;

    for (int i = 0; i < scan->numberOfScans; ++i) {
        float dist_m = scan->Data[i].scanDistance * 0.001f;
        if (dist_m <= 0) {
            continue;
        }

        double real_angle = scan->Data[i].scanAngle * static_cast<float>(M_PI / 180.0f);
        double x = dist_m * std::cos(real_angle);
        double y = dist_m * std::sin(real_angle);

        endpoints.emplace(World2Image(frame->pose_ * Vec2d(x, y)));
    }

    if (method == GridMethod::MODEL_POINTS) {
        std::for_each(std::execution::par_unseq, model_.begin(), model_.end(), [&](const Model2DPoint& pt) {
            Vec2i pos_in_image = World2Image(frame->pose_.translation());
            Vec2i pw = pos_in_image + Vec2i(pt.dx_, pt.dy_);  // submap下

            if (pt.range_ < closest_th_) {
                SetPoint(pw, false);
                return;
            }

            double angle = pt.angle_ - theta;
            double range = FindRangeInAngle(angle, scan);

            if (range == 0.0) {
                if (pt.range_ < endpoint_close_th_) {
                    SetPoint(pw, false);
                }
                return;
            }

            if (range > pt.range_ && endpoints.find(pw) == endpoints.end()) {
                SetPoint(pw, false);
            }
        });
    } else {
        Vec2i start = World2Image(frame->pose_.translation());
        std::for_each(std::execution::par_unseq, endpoints.begin(), endpoints.end(),
                      [this, &start](const auto& pt) { BresenhamFilling(start, pt); });
    }

    std::for_each(endpoints.begin(), endpoints.end(), [this](const auto& pt) { SetPoint(pt, true); });
}

void OccupancyMap::SetPoint(const Vec2i& pt, bool occupy) {
    int x = pt[0], y = pt[1];
    if (x < 0 || y < 0 || x >= occupancy_grid_.cols || y >= occupancy_grid_.rows) {
        if (occupy) {
            has_outside_pts_ = true;
        }

        return;
    }

    uchar value = occupancy_grid_.at<uchar>(y, x);
    if (occupy) {
        if (value > 117) {
            occupancy_grid_.ptr<uchar>(y)[x] -= 1;
        }
    } else {
        if (value < 137) {
            occupancy_grid_.ptr<uchar>(y)[x] += 1;
        }
    }
}

cv::Mat OccupancyMap::GetOccupancyGridBlackWhite() const {
    cv::Mat image(image_size_, image_size_, CV_8UC3);
    for (int x = 0; x < occupancy_grid_.cols; ++x) {
        for (int y = 0; y < occupancy_grid_.rows; ++y) {
            if (occupancy_grid_.at<uchar>(y, x) == 127) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(127, 127, 127);
            } else if (occupancy_grid_.at<uchar>(y, x) < 127) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            } else if (occupancy_grid_.at<uchar>(y, x) > 127) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            }
        }
    }

    return image;
}

void OccupancyMap::BresenhamFilling(const Vec2i& p1, const Vec2i& p2) {
    int dx = p2.x() - p1.x();
    int dy = p2.y() - p1.y();
    int ux = dx > 0 ? 1 : -1;
    int uy = dy > 0 ? 1 : -1;

    dx = abs(dx);
    dy = abs(dy);
    int x = p1.x();
    int y = p1.y();

    if (dx > dy) {
        int e = -dx;
        for (int i = 0; i < dx; ++i) {
            x += ux;
            e += 2 * dy;
            if (e >= 0) {
                y += uy;
                e -= 2 * dx;
            }

            if (Vec2i(x, y) != p2) {
                SetPoint(Vec2i(x, y), false);
            }
        }
    } else {
        int e = -dy;
        for (int i = 0; i < dy; ++i) {
            y += uy;
            e += 2 * dx;
            if (e >= 0) {
                x += ux;
                e -= 2 * dy;
            }
            if (Vec2i(x, y) != p2) {
                SetPoint(Vec2i(x, y), false);
            }
        }
    }
}
