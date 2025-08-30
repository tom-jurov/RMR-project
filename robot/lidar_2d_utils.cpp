#include "lidar_2d_utils.h"
#include <opencv2/imgproc.hpp>

void Visualize2DScan(std::shared_ptr<LaserMeasurement> scan, const SE2& pose, cv::Mat& image,
                     const Vec3b& color, int image_size, float resolution, const SE2& pose_submap) {
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    // Draw scan points
    for (int i = 0; i < scan->numberOfScans; ++i) {
        float dist_m = scan->Data[i].scanDistance * 0.001f;
        float real_angle = scan->Data[i].scanAngle * static_cast<float>(M_PI / 180.0f);

        if (dist_m > 0) {
            double x = dist_m * std::cos(real_angle);
            double y = dist_m * std::sin(real_angle);

            Eigen::Vector2d psubmap = pose_submap.inverse() * (pose * Eigen::Vector2d(x, y));

            int image_x = int(psubmap[0] * resolution + image_size / 2);
            int image_y = int(psubmap[1] * resolution + image_size / 2);
            if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
                image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }
    }

    // Draw robot center
    Eigen::Vector2d pose_in_image = pose_submap.inverse() * pose.translation() * resolution
                                    + Eigen::Vector2d(image_size / 2, image_size / 2);
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5,
               cv::Scalar(color[0], color[1], color[2]), 2);

    // Draw heading
    double heading_len = 10.0; // pixels
    Eigen::Vector2d heading_end = pose_in_image + heading_len * Eigen::Vector2d(std::cos(pose.rotation()),
                                                                                std::sin(pose.rotation()));
    cv::line(image, cv::Point2f(pose_in_image[0], pose_in_image[1]),
             cv::Point2f(heading_end[0], heading_end[1]),
             cv::Scalar(color[0], color[1], color[2]), 2);
}
