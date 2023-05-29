#include "mainwindow.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QDebug>

//重定向qdebug的打印
void log_out_put(QtMsgType type, const QMessageLogContext& context, const QString& msg);

/**
 * @brief 重定向qdebug的打印
 * @param type
 * @param context
 * @param msg
 */
void log_out_put(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    QByteArray localMsg = msg.toLocal8Bit();

    switch (type) {
    case QtDebugMsg:
        fprintf(stdout, "[Debug] %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        break;

        //    case QtInfoMsg:
        //        fprintf(stdout, "[Info] %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line,
        //        context.function); break;

    case QtWarningMsg:
        fprintf(stdout, "[Warning] %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        break;

    case QtCriticalMsg:
        fprintf(stdout, "[Critical] %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        break;

    case QtFatalMsg:
        fprintf(stdout, "Fatal: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        abort();
        break;

    default:
        fprintf(stdout, "[Debug] %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    }
#ifndef QT_NO_DEBUG_OUTPUT
    fflush(stdout);
#endif
}

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    qInstallMessageHandler(log_out_put);
    QFont f = a.font();

    f.setFamily("微软雅黑");
    a.setFont(f);
    QElapsedTimer cost;

    cost.start();
    MainWindow w;

    qDebug() << "window build cost:" << cost.elapsed() << " ms";
    w.show();

    return (a.exec());
}


// int main(int argc, char* argv[]){
    
    // QApplication app(argc, argv);

    // // 创建 QLabel 控件用于显示动画
    // QLabel label;

    // // 创建 QMovie 对象并加载 GIF 文件
    // QMovie movie("C:\\Users\\GilbertPan\\Desktop\\RobLab\\src\\resource\\loading.gif");

    // // 设置动画到 QLabel 控件
    // label.setMovie(&movie);

    // // 开始播放动画
    // movie.start();

    // // 设置窗口属性
    // label.setWindowTitle("GIF Animation");
    // label.resize(400, 300);
    // label.show();

    // return app.exec();
// }