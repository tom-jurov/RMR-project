#ifndef VISUALIZER_H
#define VISUALIZER_H
#define NOMINMAX
#include <QFrame>
#include <vector>
#include <rplidar.h>
#include <opencv2/core.hpp>

class Visualizer : public QFrame
{
    Q_OBJECT
public:
    explicit Visualizer(QWidget *parent = nullptr);

    void setLaserData(const LaserMeasurement &data);
    void setOccupancyGrid(const cv::Mat& grid);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    LaserMeasurement laserData;
    cv::Mat mapImage;
    bool updateLaserPicture = false;
};

#endif // VISUALIZER_H
