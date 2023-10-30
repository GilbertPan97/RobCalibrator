#include "camera.h"
#include "sszn.h"

Camera* Camera::createCamera(QString name,QString ip)
{
    if(name=="SSZN")
    {
        static int ssznDeviceId=0;
        return new SSZN(ssznDeviceId++,ip);
    }
    else{
        return nullptr;
    }
}
QStringList Camera::search(QString CameraType)
{
    if(CameraType=="SSZN")
    {
        return SSZN::search();
    }
}