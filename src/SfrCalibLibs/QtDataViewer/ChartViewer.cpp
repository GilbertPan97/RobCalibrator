#include "ChartViewer.h"

#include <QScrollBar>

ChartViewer::ChartViewer(QWidget *parent) : QChartView(parent)
{
    // create X and Y axis
    axisX = new QValueAxis;
    axisY = new QValueAxis;
    axisX->setTitleText("X");
    axisY->setTitleText("Z");       // 3d scanner coordination Y = 0

    chart()->addAxis(axisX, Qt::AlignBottom);       // add X axis to chart
    chart()->addAxis(axisY, Qt::AlignLeft);
    chart()->setTheme(QChart::ChartThemeBlueNcs);

    QPen pen(Qt::darkGray);
    pen.setWidth(2);
    chart()->setBackgroundPen(pen);

    // Set up chart viewer
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::RubberBandDrag);
    setOptimizationFlag(QGraphicsView::DontAdjustForAntialiasing);
    setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setInteractive(true);

    // Set up scroll bars range and step
    setHorizontalScrollBar(new QScrollBar(Qt::Horizontal, this));
    setVerticalScrollBar(new QScrollBar(Qt::Vertical, this));
    horizontalScrollBar()->setRange(0, chart()->plotArea().width());
    verticalScrollBar()->setRange(0, chart()->plotArea().height());
    horizontalScrollBar()->setSingleStep(10);
    verticalScrollBar()->setSingleStep(10);

    // create point cloud series
    pointSeries = new QScatterSeries();

    chart()->addSeries(pointSeries);      // add to chart
    pointSeries->attachAxis(axisX);
    pointSeries->attachAxis(axisY);
}

void ChartViewer::setPointCloud(const std::vector<cv::Point2f> &pointCloud)
{
    pointSeries->clear();    // clear point series data

    // add new point cloud data
    for (const auto &point : pointCloud)
        pointSeries->append(point.x, point.y);

    QPen pen(Qt::gray);      // Create a pen object with a transparent color
    pen.setWidth(1);
    QColor skyBlue(0x87, 0xCE, 0xEB);
    pointSeries->setPen(pen);       // Set the pen object for the point series
    pointSeries->setBrush(QBrush(skyBlue));
    pointSeries->setMarkerSize(7);

    autoFit();
}

void ChartViewer::autoFit()
{
    if(pointSeries->count() == 0)
        return;
    
    QVector<QPointF> pointCloud = pointSeries->pointsVector();
    QPointF center;
    for(const auto& point : pointCloud) {
        center += point;
    }
    center /= pointCloud.size();

    // find the max X and Y from point to center
    double maxXDist = 0.0, maxYDist = 0.0, maxDistSquared = 0.0;
    for (const auto& point : pointCloud) {
        double dx = point.x() - center.x();
        double dy = point.y() - center.y();
        if (std::abs(dx) > std::abs(maxXDist)) 
            maxXDist = std::abs(dx);
        
        if (std::abs(dy) > std::abs(maxYDist))
            maxYDist = std::abs(dy);
    }
    
    // Set the axis ranges
    float maxAxisDist = std::max(maxXDist, maxYDist);
    this->chart()->axisX()->setRange(center.x() - maxAxisDist - 10, center.x() + maxAxisDist + 10);
    this->chart()->axisY()->setRange(center.y() - maxAxisDist - 10, center.y() + maxAxisDist + 10);
    
    // connect(this, &QWidget::resizeEvent, this, [=](){
    //     float radio = this->size().height() / this->size().width();
    //     float x_range = maxAxisDist;
    //     float y_range = maxAxisDist * radio;
    // }); 
}

void ChartViewer::wheelEvent(QWheelEvent *event)
{
    qreal factor = 1.2;
    if (event->delta() > 0) {  // Scroll up - Zoom in
        this->chart()->zoom(factor);
    } else {  // Scroll down - Zoom out
        this->chart()->zoom(1 / factor);
    }
    event->accept();
}

void ChartViewer::mousePressEvent(QMouseEvent *event)
{
    // Record last mouse position on press
    lastMousePosition = event->pos();
    event->accept();
}

void ChartViewer::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() == Qt::RightButton) {
        // Calculate offset and update chart view
        QPoint offset = event->pos() - lastMousePosition;
        chart()->scroll(-offset.x(), offset.y());
        lastMousePosition = event->pos();
    }
    else {
        // Calculate offset and update scroll bars position
        QPoint offset = event->pos() - lastMousePosition;
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - offset.x());
        verticalScrollBar()->setValue(verticalScrollBar()->value() - offset.y());
        lastMousePosition = event->pos();
    }
    event->accept();
}

