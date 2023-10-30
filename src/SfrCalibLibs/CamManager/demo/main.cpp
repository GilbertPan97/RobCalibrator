#include "SFRSensorManager.h"
#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <SR7Link.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SFRSensorManager* manager = SFRSensorManager::getInstance();
    QFile file("config.txt");
    if(file.exists())
    {
        if(file.open(QIODevice::ReadOnly))
        {
            QTextStream in(&file);
            QString config;
            in>>config;
            manager->load(config);
            file.close();
        }
    }
    manager->exec();
    QFile file2("config.txt");
    if(file2.open(QIODevice::WriteOnly))
    {
        QTextStream out(&file2);
        out<<manager->save();
        file2.close();
    }
    return 0;
}