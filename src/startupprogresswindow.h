#ifndef STARTUPPROGRESSWINDOW_H
#define STARTUPPROGRESSWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QMovie>
#include <QShowEvent>
#include <QHideEvent>

class StartupProgressWindow : public QMainWindow
{
    Q_OBJECT

public:
    StartupProgressWindow(QMainWindow *parent = nullptr);

    void wait(int milliseconds);
    
    ~StartupProgressWindow();

protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

private:
    QLabel* m_progressLabel;
    QMovie* m_progressMovie;
};

#endif // STARTUPPROGRESSWINDOW_H
