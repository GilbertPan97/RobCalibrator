#include "sszn.h"
#include <QDebug>
// #include <nlohmann/json.hpp>

SSZN::SSZN(int deviceId,QString ip)
{
    points = new std::vector<cv::Point2f>();
    intManager = new QtIntPropertyManager();
    doubleManager = new QtDoublePropertyManager();
    stringManager = new QtStringPropertyManager();
    boolManager = new QtBoolPropertyManager();
    enumManager=new QtEnumPropertyManager();


    spinBoxFactory = new QtSpinBoxFactory();
    lineEditFactory = new QtLineEditFactory();
    doubleSpinBoxFactory = new QtDoubleSpinBoxFactory();
    checkBoxFactory = new QtCheckBoxFactory();
    enumEditorFactory = new QtEnumEditorFactory();

    //设备ID
    intDeivceID=intManager->addProperty("DeviceID");
    intManager->setValue(intDeivceID,deviceId);
    intDeivceID->setEnabled(false);
    //设备IP
    strIp=stringManager->addProperty("IP");
    stringManager->setValue(strIp,ip);
    QStringList strs=ip.split(".");
    cfg.abyIpAddress[0]=strs[0].toInt();
    cfg.abyIpAddress[1]=strs[1].toInt();
    cfg.abyIpAddress[2]=strs[2].toInt();
    cfg.abyIpAddress[3]=strs[3].toInt();
    strIp->setEnabled(false);
    //设备是否打开
    bOpened=boolManager->addProperty("Opened");
    bOpened->setEnabled(false);
    boolManager->setValue(bOpened,false);
    strVersion = stringManager->addProperty("Version");
    strVersion->setEnabled(false);
    strSensorType=stringManager->addProperty("Sensor Type");
    strSensorType->setEnabled(false);
    strSensorSerialNumber=stringManager->addProperty("Serial Number");
    strSensorSerialNumber->setEnabled(false);
    strLicenseKey = stringManager->addProperty("License Key");
    strLicenseKey->setEnabled(false);
    strCameraTemperature = stringManager->addProperty("Camera Temperature");
    strCameraTemperature->setEnabled(false);

    intSampleRate=intManager->addProperty("Sample Rate");
    intManager->setMaximum(intSampleRate,67000);
    intManager->setMinimum(intSampleRate,10);
    enumBatchSwitch=enumManager->addProperty("Batch Switch");
    enumManager->setEnumNames(enumBatchSwitch,QStringList()<<"OFF"<<"ON");
    enumExposureSensitive=enumManager->addProperty("Exposure Sensitive");
    enumManager->setEnumNames(enumExposureSensitive,QStringList()<<"High Accuracy"<<"High Dynamic Range 1"
    <<"High Dynamic Range 2"<<"High Dynamic Range 3"<<"High Dynamic Range 4"<<"Custom High Dynamic");
    enumExposureTime=enumManager->addProperty("Exposure Time");
    enumManager->setEnumNames(enumExposureTime,QStringList()<<"10μs"<<"15μs"<<"30μs"<<"60μs"<<"120μs"<<"240μs"<<"480μs"
    <<"960μs"<<"1920μs"<<"2400μs"<<"4900μs"<<"9800μs");
    enumLight=enumManager->addProperty("Light");
    enumManager->setEnumNames(enumLight,QStringList()<<"Auto"<<"Manual");
    intlaserLightMax=intManager->addProperty("Laser Light Max");
    intManager->setMaximum(intlaserLightMax,99);
    intManager->setMinimum(intlaserLightMax,1);
    intLaserLightMin=intManager->addProperty("Laser Light Min");
    intManager->setMaximum(intLaserLightMin,99);
    intManager->setMinimum(intLaserLightMin,1);
    intMaxSensivtie=intManager->addProperty("Max Sensivtie");
    intManager->setMaximum(intMaxSensivtie,5);
    intManager->setMinimum(intMaxSensivtie,1);
    enumMaxType=enumManager->addProperty("Max Type");
    enumManager->setEnumNames(enumMaxType,QStringList()<<"Standard"<<"Near"<<"Far"<<"To Not Valid"<<"Continous");
    enumGrabMode=enumManager->addProperty("Grab Mode");
    enumManager->setEnumNames(enumGrabMode,QStringList()<<"3D"<<"2.5D");
    //数据变换信号
    QObject::connect(stringManager,&QtStringPropertyManager::valueChanged, [this](QtProperty* property,const QString& val){
   
    });
    QObject::connect(intManager,&QtIntPropertyManager::valueChanged, [this](QtProperty* property,int val){
        if(property==intSampleRate)
        {
            SR7IF_SetSetting(intManager->value(intDeivceID),2,-1,0,2,0,&val,4);
        }
    });
    
}

bool SSZN::open()
{
    if(stringManager->value(strIp)=="")
    {
        return false;
    }
    if(boolManager->value(bOpened))
    {
        return true;
    }
    int res = SR7IF_EthernetOpen(intManager->value(intDeivceID),&cfg);
    if(res!=0)
    {
        return false;
    }else
    {
        boolManager->setValue(bOpened,true);
        readParameter();
        return true;
    }
}

void SSZN::readParameter()
{
    xWidth=SR7IF_ProfileDataWidth(intManager->value(intDeivceID),nullptr);
    xGap=SR7IF_ProfileData_XPitch(intManager->value(intDeivceID),nullptr);
    QString version = SR7IF_GetVersion();
    stringManager->setValue(strVersion,version);
    QString sensorType=SR7IF_GetModels(intManager->value(intDeivceID));
    stringManager->setValue(strSensorType,sensorType);
    QString sensorSerialNumber=SR7IF_GetHeaderSerial(intManager->value(intDeivceID),0);
    stringManager->setValue(strSensorSerialNumber,sensorSerialNumber);
    unsigned short day=0;
    SR7IF_GetLicenseKey(intManager->value(intDeivceID),&day); 
    stringManager->setValue(strLicenseKey,QString::number(day));
    int temp=0;
    SR7IF_GetCameraTemperature(intManager->value(intDeivceID), &temp,nullptr);
    stringManager->setValue(strCameraTemperature,QString::number((double)temp/100.0));
    int iData=0;
    char cData=0;
    SR7IF_GetSetting(intManager->value(intDeivceID),-1,0,2,0,&iData,4);
    intManager->setValue(intSampleRate,iData);
    SR7IF_GetSetting(intManager->value(intDeivceID),-1,0,3,0,&cData,1);
    enumManager->setValue(enumBatchSwitch,cData);
    // cData=0;
    // SR7IF_GetSetting(intManager->value(intDeivceID),-1,1,5,0,&cData,1);
    // enumManager->setValue(enumExposureSensitive,cData);
    // cData=0;
    // SR7IF_GetSetting(intManager->value(intDeivceID),-1,1,6,0,&cData,1);
    // enumManager->setValue(enumExposureTime,cData);
    // cData=0;
    // SR7IF_GetSetting(intManager->value(intDeivceID),-1,1,0xB,0,&cData,1);
    // enumManager->setValue(enumLight,cData);
    // cData=0;
    // SR7IF_GetSetting(intManager->value(intDeivceID),-1,1,0xC,0,&cData,1);
    // intManager->setValue(intlaserLightMax,cData);
    // cData=0;
    // SR7IF_GetSetting(intManager->value(intDeivceID),-1,1,0xD,0,&cData,1);
    // intManager->setValue(intLaserLightMin,cData);
    // cData=0;
    // SR7IF_GetSetting(intManager->value(intDeivceID),-1,1,0xF,0,&cData,1);
    // enumManager->setValue(intMaxSensivtie,cData);
    // cData=0;
    // SR7IF_GetSetting(intManager->value(intDeivceID),-1,1,0x11,0,&cData,1);
    // enumManager->setValue(enumMaxType,cData);
    cData=0;
    SR7IF_GetSetting(intManager->value(intDeivceID),-1,0x30,0,0,&cData,1);
    enumManager->setValue(enumGrabMode,cData);
    // if(cData==0)
    // {
    //     intManager->setValue(intBatchSwitch,0);
    // }else{
    //     intManager->setValue(intBatchSwitch,1);
    // }
    // browser->addProperty(intBatchSwitch);
    // browser->addProperty(intExposureSensitive);
    // browser->addProperty(intExposureTime);
    // browser->addProperty(intLight);
    // browser->addProperty(intlaserLightMax);
    // browser->addProperty(intLaserLightMin);
    // browser->addProperty(intMaxSensivtie);
    // browser->addProperty(intMaxType);
    // browser->addProperty(intGrabMode);
}

bool SSZN::isOpened()
{
    return boolManager->value(bOpened);
}

bool SSZN::close()
{
    if(!isOpened())
    {
        return true;
    }
    int res=SR7IF_CommClose(intManager->value(intDeivceID));
    if(res!=0)
    {
        return false;
    }else
    {
        boolManager->setValue(bOpened,false);
        return true;
    }
}

void* SSZN::capture(int type)
{
    if(isOpened())
    {
        if(type==1)     //profile
        {
            int* d = new int[xWidth];
            int res = SR7IF_GetSingleProfile(intManager->value(intDeivceID),d,nullptr);
            if(res!=0)
            {
                delete[] d;
                return nullptr;
            }
            else{
                points->clear();
                for(int i=0;i<xWidth;i++)
                {
                    points->push_back(cv::Point2f(i*xGap, d[i]/100000.0));
                }
                delete[] d;
                return points;
            }
        }
        return nullptr;
    }
    return nullptr;
}



int SSZN::cameraType()
{
    return 2;
}

QStringList SSZN::search()
{
    int countNumber=0;
    SR7IF_ETHERNET_CONFIG* cfg=SR7IF_SearchOnline(&countNumber,5000);
    QStringList list;
    for(int i=0;i<countNumber;i++)
    {
        int i1=cfg[i].abyIpAddress[0];
        int i2=cfg[i].abyIpAddress[1];
        int i3=cfg[i].abyIpAddress[2];
        int i4=cfg[i].abyIpAddress[3];
        QString ip=QString("%1.%2.%3.%4").arg(i1).arg(i2).arg(i3).arg(i4);
        list.append(ip);
    }
    return list;
}

bool SSZN::setProperties(QtTreePropertyBrowser* browser)
{
    browser->clear();
    browser->setFactoryForManager(intManager,spinBoxFactory);
    browser->setFactoryForManager(doubleManager,doubleSpinBoxFactory);
    browser->setFactoryForManager(stringManager,lineEditFactory);
    browser->setFactoryForManager(enumManager,enumEditorFactory);
    browser->addProperty(intDeivceID);
    browser->addProperty(strIp);
    browser->addProperty(bOpened);
    browser->addProperty(strVersion);
    browser->addProperty(strSensorType);
    browser->addProperty(strSensorSerialNumber);
    browser->addProperty(strLicenseKey);
    browser->addProperty(strCameraTemperature);
    browser->addProperty(intSampleRate);
    browser->addProperty(enumBatchSwitch);
    browser->addProperty(enumExposureSensitive);
    browser->addProperty(enumExposureTime);
    browser->addProperty(enumLight);
    browser->addProperty(intlaserLightMax);
    browser->addProperty(intLaserLightMin);
    browser->addProperty(intMaxSensivtie);
    browser->addProperty(enumMaxType);
    browser->addProperty(enumGrabMode);
    
    return true;
}

QString SSZN::getProperty(QString name)
{
    if(name=="IP")
    {
        return stringManager->value(strIp);
    }
}
std::vector<cv::Point2f> SSZN::getGrabData()
{
    // create new vector vector
    std::vector<cv::Point2f> data;

    // Iterate over the original vector and copy the data
    if(!points->empty()){
        for (const auto& point : *points)
            data.push_back(point);
    }
    return data;
}

SSZN::~SSZN()
{
    delete points;
}