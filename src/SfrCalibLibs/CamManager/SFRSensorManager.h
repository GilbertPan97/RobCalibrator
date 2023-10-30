#pragma once

#include <QDialog>
#include <QWidget>
#include <QMap>
#include "camera.h"

#include <QString>
#include <qttreepropertybrowser.h>
#include <QtCharts>
#include <QThread>
#include <grabThread.h>
#include <QVector>
#include <QPointF>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class SFRSensorManager; }
QT_END_NAMESPACE

class SFRSensorManager : public QDialog
{
    Q_OBJECT

private:
    SFRSensorManager(QDialog *parent = nullptr);
public:
    static SFRSensorManager* getInstance();
    ~SFRSensorManager();
    QString save();
    void load(QString strConfig);
    QStringList getCamerasName();
    Camera* getCamera(QString name);
    void closeEvent(QCloseEvent *) override;
private:
    static SFRSensorManager* instance;
    Ui::SFRSensorManager *ui;
    QMap<QString, Camera*> cameras;
    Camera* currentCamera=nullptr;
    QtTreePropertyBrowser* propertyBrowser;
    GrabThread *thread;

    QLabel* imageLabel;
    QLabel* pointsLabel;
    QChartView* profileChart;
    QChart* chart;
    QLineSeries* series;
    QValueAxis* xaxis;
    QValueAxis* yaxis;
    QVector<QPointF> showPoints;
public slots:
    void updateGraph(int type,void* data);
private:
    void initUI();
    void initData();
    void setButtonClickedHandler();
};
