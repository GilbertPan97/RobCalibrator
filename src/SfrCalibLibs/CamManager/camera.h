#pragma once
#include <QStringList>
#include <qttreepropertybrowser.h>
#include <opencv2/opencv.hpp>

class Camera
{
public:
    static Camera* createCamera(QString name,QString ip);
    static QStringList search(QString CameraType);
    /**
     * @brief cameraType
     * @return the camera type
     * 0:RBG Camera; 
     * 1:Points Camera; 
     * 2:Profile Camera; 
    */
    virtual int cameraType()=0;
    virtual bool open()=0;
    virtual bool isOpened()=0;
    virtual bool close()=0;
    /**
     * @brief capture the image
     * @param type the image type 0:Image;1:Profile
     * @return true if success
    */
    virtual void* capture(int type=0)=0;
    virtual bool setProperties(QtTreePropertyBrowser* browser)=0;
    virtual QString getProperty(QString name)=0;
    virtual std::vector<cv::Point2f> getGrabData() = 0;
};