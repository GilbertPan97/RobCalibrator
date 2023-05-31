#include "ChartViewer.h"

#include <QScrollBar>

ChartViewer::ChartViewer(QWidget *parent) : QChartView(parent)
{
    // create X and Y axis
    axisX = new QValueAxis;
    axisX->setTitleText("X");
    axisY = new QValueAxis;
    axisY->setTitleText("Z");       // 3d scanner coordination Y = 0

    chart()->addAxis(axisX, Qt::AlignBottom);       // add X axis to chart
    chart()->addAxis(axisY, Qt::AlignLeft);

    chart()->setTheme(QChart::ChartThemeBlueNcs);
    QPen pen(QRgb(0xd18952));
    pen.setWidth(3);

    // Set pen on chart object
    chart()->legend()->setPen(pen);
    chart()->legend()->setBrush(QBrush(QColor(255, 255, 255, 200)));
    chart()->legend()->setAlignment(Qt::AlignTop);

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
    pointSeries = new QScatterSeries;
    pointSeries->setMarkerSize(8.0);      // set points size

    chart()->addSeries(pointSeries);      // add to chart
    pointSeries->attachAxis(axisX);
    pointSeries->attachAxis(axisY);
}

void ChartViewer::setPointCloud(const std::vector<cv::Point2f> &pointCloud)
{
    pointSeries->clear();    // clear point series data

    // add new point cloud data
    for (const auto &point : pointCloud)
    {
        pointSeries->append(point.x, point.y);
    }

    autoFit();
}

void ChartViewer::autoFit()
{
    if(pointSeries->count() == 0)
        return;
    
    double minX = FLT_MAX, maxX = -FLT_MAX, minY = FLT_MAX, maxY = -FLT_MAX;
    QVector<QPointF> pointCloud = pointSeries->pointsVector();
    for (auto &point : pointCloud)
    {
        minX = std::min(minX, point.x());
        maxX = std::max(maxX, point.x());
        minY = std::min(minY, point.y());
        maxY = std::max(maxY, point.y());
    }

    // Set the axis ranges
    this->chart()->axisX()->setRange(minX - 10, maxX + 10);
    this->chart()->axisY()->setRange(minY - 10, maxY + 10);
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

