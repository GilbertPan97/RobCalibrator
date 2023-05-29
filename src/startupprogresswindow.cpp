#include <iostream>
#include <QLabel>
#include <QMovie>
#include <QWidget>
#include <QThread>
#include <QApplication>
#include <QTimer>

#include "startupprogresswindow.h"

StartupProgressWindow::StartupProgressWindow(QMainWindow *parent)
    : QMainWindow(parent)
{
    m_progressLabel = new QLabel(this);
    m_progressMovie = new QMovie(":/animations/resource/loading1.jfif");

    // check progress movie load status
    if(!m_progressMovie->isValid())
        std::cout << "ERROR: Progress movie load fail.\n";

    m_progressLabel->setMovie(m_progressMovie);
    m_progressLabel->setAlignment(Qt::AlignCenter);

    setCentralWidget(m_progressLabel);
    setFixedSize(1000, 600);
    setWindowTitle("Startup Progress");
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);

    m_progressMovie->start();
}

void StartupProgressWindow::showEvent(QShowEvent *event)
{
    this->show();
    m_progressLabel->show();

    QApplication::processEvents();      // Refresh gui
}

void StartupProgressWindow::hideEvent(QHideEvent *event)
{
    m_progressLabel->hide();
}

void StartupProgressWindow::wait(int milliseconds)
{
    QEventLoop loop;
    QTimer::singleShot(milliseconds, &loop, &QEventLoop::quit);
    loop.exec();
}

StartupProgressWindow::~StartupProgressWindow()
{
    m_progressMovie->stop();
    delete m_progressMovie;
    delete m_progressLabel;
}
