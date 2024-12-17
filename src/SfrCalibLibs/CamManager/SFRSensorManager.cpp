#include "SFRSensorManager.h"
#include "ui_SFRSensorManager.h"
#include <QMessageBox>
#include <QMenu>
#include <QInputDialog>
#include <QString>
#include <QThread>
#include <QDebug>
#include <qtreewidget.h>
#include <QRadioButton>
#include <list>
#include <nlohmann/json.hpp>

SFRSensorManager* SFRSensorManager::instance=nullptr;

SFRSensorManager::SFRSensorManager(QDialog *parent)
    : QDialog(parent)
    , ui(new Ui::SFRSensorManager)
{
    ui->setupUi(this);
    initUI();
    initData();
    setButtonClickedHandler();
}

SFRSensorManager::~SFRSensorManager()
{
    delete ui;
}

void SFRSensorManager::initUI()
{
    imageLabel=new QLabel(this);
    imageLabel->setText("Image Display");
    pointsLabel=new QLabel(this);
    pointsLabel->setText("Points Display");
    profileChart=new QChartView (this);
    QLayout* imageLayout=new QHBoxLayout(this);
    QLayout* pointsLayout=new QHBoxLayout(this);
    QLayout* profileLayout=new QHBoxLayout(this);
    imageLayout->addWidget(imageLabel);
    pointsLayout->addWidget(pointsLabel);
    profileLayout->addWidget(profileChart);

    ui->imageDisplay->setLayout(imageLayout);
    ui->profileDisplay->setLayout(profileLayout);
    ui->pointsDisplay->setLayout(pointsLayout);

    propertyBrowser=new QtTreePropertyBrowser(this);
    QVBoxLayout* layout=new QVBoxLayout(this);
    layout->addWidget(propertyBrowser);
    ui->propertyWidget->setContentsMargins(0,0,0,0);
    ui->propertyWidget->setLayout(layout);
    ui->treeWidget->expandAll();
    ui->treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->treeWidget->setColumnWidth(0,200);
    ui->treeWidget->setHeaderItem(new QTreeWidgetItem(QStringList()<<"Camera"));
    QTreeWidgetItem* sszn=new QTreeWidgetItem(QStringList()<<"SSZN");
    ui->treeWidget->addTopLevelItem(sszn);
    connect(ui->treeWidget,&QTreeWidget::itemDoubleClicked,[this](QTreeWidgetItem* item,int column){
        if (item->parent())
        {
            currentCamera=cameras[item->text(0)];
            ui->label->setText(item->text(0));
            currentCamera->setProperties(propertyBrowser);
            ui->displayWidget->setCurrentIndex(currentCamera->cameraType());
        }
    });
    connect(ui->treeWidget,&QTreeWidget::customContextMenuRequested,[this](QPoint pos){
        QTreeWidgetItem* curItem=ui->treeWidget->itemAt(pos);
        if(!curItem){
            return;
        }else if(!curItem->parent()){
            QMenu* popAddMenu=new QMenu(this);
            QAction* newAction=new QAction("Search",this);
            popAddMenu->addAction(newAction);
            connect(newAction,&QAction::triggered,[this,curItem](){
                QString typeName=curItem->text(0);
                for(auto item:curItem->takeChildren())
                {
                    curItem->removeChild(item);
                    delete cameras[item->text(0)];
                    cameras[item->text(0)]=nullptr;
                    cameras.remove(item->text(0));
                }
                QStringList list=Camera::search(typeName);
                for(auto str:list)
                {
                    QTreeWidgetItem* item=new QTreeWidgetItem(QStringList()<<str);
                    Camera* camera=Camera::createCamera(curItem->text(0),str);
                    cameras.insert(str,camera);
                    curItem->addChild(item);
                }
                ui->treeWidget->expandAll();
            });
            popAddMenu->exec(QCursor::pos());
        }else
        {
            QMenu* popChildMenu=new QMenu(this);

            QAction* renameAction=new QAction("Rename",this);
            connect(renameAction,&QAction::triggered,[this,curItem](){
                QString text = QInputDialog::getText(this,"Camera Name","Please input the camera name",
                 QLineEdit::Normal, curItem->text(0), nullptr);
                if(text.isEmpty())
                {
                    return;
                }
                if(cameras.keys().contains(text))
                {
                    QMessageBox::information(this,"Information","Camera name already exists");
                }else{
                    QString oldText = curItem->text(0);
                    curItem->setText(0,text);
                    cameras.insert(text,cameras[oldText]);
                    cameras.remove(oldText);
                }
            });
            popChildMenu->addAction(renameAction);            
            popChildMenu->exec(QCursor::pos());
        }
    });
}

void SFRSensorManager::initData()
{
    series=new QLineSeries();
    chart=new QChart();
    chart->addSeries(series);
    xaxis=new QValueAxis();
    xaxis->setRange(0,100);
    yaxis=new QValueAxis();
    yaxis->setRange(-50,50);
    chart->addAxis(xaxis,Qt::AlignBottom);
    chart->addAxis(yaxis,Qt::AlignLeft);
    series->attachAxis(xaxis);
    series->attachAxis(yaxis);
    thread=new GrabThread();
    connect(thread,&GrabThread::updateGraph,this,&SFRSensorManager::updateGraph,Qt::QueuedConnection);
}

void SFRSensorManager::setButtonClickedHandler()
{
    connect(ui->btnConnect,&QPushButton::clicked,[this](){
       if(currentCamera==nullptr)
       {
           return;
       }
       bool res=currentCamera->open();
       if (res)
       {
           //连接成功
       }else{
           QMessageBox::information(this,"Information","Failed to connect");
       }
    });
    connect(ui->btnDisconnect,&QPushButton::clicked,[this](){
        if(currentCamera==nullptr)
        {
            return;
        }
        if(thread->isRunning())
        {
            thread->terminate();
        }
        int res=currentCamera->close();
        if (res)
        {
            //断开成功
        }else{
            
            QMessageBox::information(this,"Information","Failed to disconnect");
        }
    });
    connect(ui->btnGrab,&QPushButton::clicked,[this](){
        if(currentCamera==nullptr)
        {
            return;
        }
        if(thread->isRunning())
        {
            thread->terminate();
        }
        if(currentCamera->cameraType()==2)
        {
            thread->setCamera(currentCamera);
            thread->start();
        }
    });
    connect(ui->btnSave, &QPushButton::released, [this](){
        std::vector<cv::Point2f> scanData;

        if(this->getCamera(this->getCamerasName()[0]))
            scanData = this->getCamera(this->getCamerasName()[0])->getGrabData();
        else{
            QMessageBox::warning(this, "Warning", "No camera detected.");
            return;
        }
        
        if(!scanData.empty()){
            QString filePath = QFileDialog::getSaveFileName(this, 
                "Save YAML File", QDir::homePath(), "YAML Files (*.yml)");
            std::string fileName = filePath.toStdString();

            if (filePath.isEmpty())
                return;
                
            // Read scan data
			std::vector<cv::Point3f> scan_line;
            for(auto point: scanData){
                if(point.x <= -10000.0 || point.y <= -10000.0)
                    continue;           // filter invalid points
                cv::Point3f point3d(point.x, 0, point.y);
                scan_line.push_back(point3d);
            }
			cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
			fs << "scan_line" << scan_line;
			fs.release();
        }
        else
            QMessageBox::warning(this, "Warning", "Data is empty. Saving failed.");
    });
    connect(ui->btnStop,&QPushButton::clicked,[this](){
        thread->running=false;
        thread->terminate();
    });
}

void SFRSensorManager::updateGraph(int type,void* data)
{
    if(type==3)
    {
        if(ui->displayWidget->currentIndex()!=2)
        {
            ui->displayWidget->setCurrentIndex(2);
        }
        std::vector<cv::Point2f>* list = (std::vector<cv::Point2f>*)data;
        showPoints.clear();
        for(auto item: *list)
        {   
            if(item.y>-1000 && item.y<1000)
            {
                QPointF p(item.x, item.y);
                showPoints.append(p);
            }
           
        }
        series->replace(showPoints);
        profileChart->setChart(chart);
        profileChart->update();
    }
}

QStringList SFRSensorManager::getCamerasName()
{
    return cameras.keys();
}

Camera* SFRSensorManager::getCamera(QString name)
{
    return cameras[name];
}

QString SFRSensorManager::save()
{
    //xuliehua
    using nlohmann::json;
    json config;
    json jCameraTypes;
    for(int i=0;i<ui->treeWidget->topLevelItemCount();i++)
    {
        json jCameraType;
        QTreeWidgetItem* topItem=ui->treeWidget->topLevelItem(i);
        jCameraType["TypeName"]=topItem->text(0).toStdString();
        json jCameras;
        for(int j=0; j<topItem->childCount(); j++)
        {
            json jCamera;
            QTreeWidgetItem* item=topItem->child(j);
            jCamera["Name"]=item->text(0).toStdString();
            jCamera["IP"]=cameras[item->text(0)]->getProperty("IP").toStdString();
            jCamera["Status"]=cameras[item->text(0)]->isOpened();
            jCameras.push_back(jCamera);
        }
        jCameraType["Cameras"]=jCameras;
        jCameraTypes.push_back(jCameraType);
    }
    config["CameraTypes"]=jCameraTypes;
    return QString::fromStdString(config.dump());
}

void SFRSensorManager::load(QString strConfig)
{
    ui->treeWidget->clear();
    using nlohmann::json;
    json config=json::parse(strConfig.toStdString());
    json jCameraTypes=config["CameraTypes"];
    for(auto jCameraType:jCameraTypes)
    {
        QString typeName=QString::fromStdString(jCameraType["TypeName"]);
        ui->treeWidget->addTopLevelItem(new QTreeWidgetItem(QStringList()<<typeName));
        json jCameras=jCameraType["Cameras"];
        for(auto jCamera:jCameras)
        {
            QString name=QString::fromStdString(jCamera["Name"]);
            QString ip=QString::fromStdString(jCamera["IP"]);
            bool status=jCamera["Status"];
            QTreeWidgetItem* item=new QTreeWidgetItem(QStringList()<<name);
            Camera* camera=Camera::createCamera(typeName,ip);
            cameras.insert(name,camera);
            if(status)
            {
                camera->open();
            }
            ui->treeWidget->topLevelItem(ui->treeWidget->topLevelItemCount()-1)->addChild(item);
        }
    }
    ui->treeWidget->expandAll();
}

void SFRSensorManager::closeEvent(QCloseEvent* event)
{
    if(thread!=nullptr)
    {
        if(thread->isRunning())
        {
            thread->terminate();
        }
    }
}

SFRSensorManager* SFRSensorManager::getInstance()
{
    if(instance==nullptr)
    {
        instance=new SFRSensorManager();
    }
    return instance;
}