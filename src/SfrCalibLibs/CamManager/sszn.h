#include "camera.h"
#include <qtpropertymanager.h>
#include <qteditorfactory.h>
#include <SR7Link.h>
#include <vector>
#include <opencv2/opencv.hpp>

class SSZN : public Camera
{
public:
    static QStringList search();
    SSZN(int deviceId,QString ip);
    ~SSZN();
    bool open() override;
    bool isOpened() override;
    bool close()override;
    void* capture(int type=0)override;
    int cameraType()override;
    bool setProperties(QtTreePropertyBrowser* browser)override;
    QString getProperty(QString name)override;
    std::vector<cv::Point2f> getGrabData()override;
    void readParameter();
private:
    std::vector<cv::Point2f>* points;
    int xWidth;
    double xGap;
    SR7IF_ETHERNET_CONFIG cfg;
    QtIntPropertyManager *intManager;
    QtDoublePropertyManager *doubleManager;
    QtStringPropertyManager *stringManager;
    QtBoolPropertyManager* boolManager;
    QtEnumPropertyManager *enumManager;


    QtSpinBoxFactory *spinBoxFactory;
    QtLineEditFactory *lineEditFactory;
    QtDoubleSpinBoxFactory *doubleSpinBoxFactory;
    QtCheckBoxFactory *checkBoxFactory;
    QtEnumEditorFactory *enumEditorFactory;



    QtProperty* intDeivceID;
    QtProperty* strIp;
    QtProperty* bOpened;
    QtProperty* bGrabbing;

    QtProperty* strVersion;
    QtProperty* strSensorType;
    QtProperty* strSensorSerialNumber;
    QtProperty* strLicenseKey;
    QtProperty* strCameraTemperature;
    //QtProperty* intTriggerMode;//0:连续触发；1：外部触发；2：编码器触发
    QtProperty* intSampleRate;//帧率：from 10 to 67000
    QtProperty* enumBatchSwitch;//批处理开关，0：off；1：on
    //QtProperty* intEncoderType;//编码器类型，0:1相1递增，1:2相1递增，2:2相2递增，3:2相4递增
    //QtProperty* intTiggerGap;//触发间隔，1-10000
    //QtProperty* intBatchCount;//批处理点数 50-15000
    //QtProperty* intCircleMode;//循环模式：0：关闭；1：打开
    //QtProperty* intBatchOutput;//批处理输出，0：轮廓+亮度；1：轮廓
    QtProperty* enumExposureSensitive;//曝光灵敏度，0：高精度，1：高动态范围1；2：高动态范围2；3：高动态范围3；4：高动态范围4；5：自定义
    QtProperty* enumExposureTime;//曝光时间：0:10μs；1:15μs；2:30μs；3:60μs；4:120μs；5:240μs；6:480μs；7:960μs；8:1920μs；9:2400μs；10:4900μs；11:9800μs
    QtProperty* enumLight;//光亮控制0：自动；1手动
    QtProperty* intlaserLightMax;//激光亮度上限 1-99
    QtProperty* intLaserLightMin;//激光亮度下限1-99
    QtProperty* intMaxSensivtie;//峰值灵敏度 1-5
    QtProperty* enumMaxType;//峰值选择0：标准；1：near；2：far；3：是指转为无效数据；4：连续
    //QtProperty* intXaxisZipType;//X轴压缩设定；1：off，2:2；4:4；8:8；16:16；2.5D不支持
    //QtProperty* intXaxisMedianFilterSize;//X轴滤波宽度1:OFF;3:3;5:5;7:7;9:9
    //QtProperty* intYaxisMedianFilterSize;//Y轴滤波宽度1:OFF;3:3;5:5;7:7;9:9
    //QtProperty* intSmoothFilter;//平滑滤波1；2；4；8；16；32；64次
    //QtProperty* intAverageFilter;//平均滤波1；2；4；8；16；32；64；128；256次
    QtProperty* enumGrabMode;//采集模式0:3D；1:2.5D
};