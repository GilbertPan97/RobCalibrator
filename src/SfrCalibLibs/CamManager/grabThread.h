#pragma once
#include <QThread>
#include "camera.h"

class GrabThread:public QThread
{
    Q_OBJECT
public:
    GrabThread();
    void setCamera(Camera* camera);
    Camera* getCamera();
    void run();
    bool running;
signals:
    void updateGraph(int type,void* data);
private:    
    Camera* camera;
};