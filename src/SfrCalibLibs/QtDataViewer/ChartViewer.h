#ifndef CHARTVIEWER_H
#define CHARTVIEWER_H

#include <iostream>
#include <algorithm>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include <opencv2/opencv.hpp>

QT_CHARTS_USE_NAMESPACE

class ChartViewer : public QChartView
{
public:
    ChartViewer(QWidget *parent = nullptr);

    void setPointCloud(const std::vector<cv::Point2f> &pointCloud);

    void autoFit();

protected:
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    QValueAxis *axisX;
    QValueAxis *axisY;
    QLineSeries *series;
    QScatterSeries *pointSeries;
    QPoint lastMousePosition;
};

#endif // CHARTVIEWER_H