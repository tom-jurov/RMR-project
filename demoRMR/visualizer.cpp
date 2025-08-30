#include "visualizer.h"
#include <QPainter>
#include <opencv2/opencv.hpp>
#include <cmath>

Visualizer::Visualizer(QWidget *parent) : QFrame(parent)  // inherit from QFrame
{
    setMinimumSize(200, 200);
}

void Visualizer::setLaserData(const  LaserMeasurement &data)
{
    laserData = data;
    updateLaserPicture = true;
    update(); // trigger repaint
}
void Visualizer::setOccupancyGrid(const cv::Mat& grid) {
    mapImage = grid.clone();
    update();
}

void Visualizer::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);

    cv::Mat canvas;
    if (!mapImage.empty()) {
        canvas = mapImage.clone();
    } else {
        canvas = cv::Mat(height(), width(), CV_8UC3, cv::Scalar(50, 50, 50));
    }

    // Flip vertically (around x-axis)
    cv::flip(canvas, canvas, 0);

    QImage img((uchar*)canvas.data, canvas.cols, canvas.rows, canvas.step, QImage::Format_BGR888);

    QPainter painter(this);

    // Calculate top-left corner for centering
    int x = (width()  - img.width())  / 2;
    int y = (height() - img.height()) / 2;

    painter.drawImage(x, y, img);
}



// void Visualizer::paintEvent(QPaintEvent *event)
// {
//     Q_UNUSED(event);

//     // Create canvas
//     cv::Mat canvas(height(), width(), CV_8UC3, cv::Scalar(0, 0, 0));

//     if (updateLaserPicture) {
//         updateLaserPicture = false;

//         for (int k = 0; k < laserData.numberOfScans; k++) {
//             int dist = laserData.Data[k].scanDistance / 20;
//             int xp = canvas.cols / 2 - dist * 2 * sin((-laserData.Data[k].scanAngle) * CV_PI / 180.0);
//             int yp = canvas.rows / 2 - dist * 2 * cos((-laserData.Data[k].scanAngle) * CV_PI / 180.0);

//             if (xp >= 0 && yp >= 0 && xp < canvas.cols && yp < canvas.rows)
//                 cv::circle(canvas, cv::Point(xp, yp), 2, cv::Scalar(0, 255, 0), -1); // green LIDAR dot
//         }
//     }

//     // Draw robot at center
//     cv::circle(canvas, cv::Point(canvas.cols / 2, canvas.rows / 2), 15, cv::Scalar(255, 0, 255), -1);

//     // Convert cv::Mat -> QImage
//     QImage img((uchar*)canvas.data, canvas.cols, canvas.rows, canvas.step, QImage::Format_BGR888);

//     // Draw in widget
//     QPainter painter(this);
//     painter.drawImage(0, 0, img);
// }
