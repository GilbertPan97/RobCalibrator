#include "grabThread.h"

GrabThread::GrabThread()
{
}

void GrabThread::run()
{
    if(camera==nullptr)
    {
        running=false;
        return;
    }
    if(!camera->isOpened())
    {
        running=false;
        return;
    }
    running=true;
    while (running)
    {
        if(camera->cameraType()==2)
        {
            auto data=camera->capture(1);
            updateGraph(3,data);
        }
        else{
            break;
        }
        msleep(100);
    }
}

void GrabThread::setCamera(Camera* camera)
{
    this->camera=camera;
}

Camera* GrabThread::getCamera()
{
    return this->camera;
}