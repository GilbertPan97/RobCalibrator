﻿#ifdef _WIN32
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#endif
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "SARibbonMainWindow.h"
#include "QVtkViewer.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "DockAreaWidget.h"
#include "startupprogresswindow.h"
#include "ChartViewer.h"

#include <QVTKOpenGLNativeWidget.h>
#include <nlohmann/json.hpp>

class SARibbonCategory;
class SARibbonContextCategory;
class SARibbonCustomizeWidget;
class SARibbonActionsManager;
class SARibbonQuickAccessBar;
class SARibbonButtonGroupWidget;
class QTextEdit;
class QVTKOpenGLNativeWidget;
class CImageViewer;
class QListWidgetItem;
class QListWidget;

class MainWindow : public SARibbonMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget* par = nullptr);

private:
    ads::CDockWidget* createQtVTKviewerDockWidget();
    ads::CDockWidget* createQtImgviewerDockWidget();
    ads::CDockWidget* createQtChartviewerDockWidget();
    ads::CDockWidget* createPropertyBrowser();
    QWidget* createControlPanel();
    ads::CDockWidget* createDataBrowserDockWidget();
    void createCategoryCalib(SARibbonCategory* page);
    void createQuickAccessBar(SARibbonQuickAccessBar* quickAccessBar);
    void createRightButtonGroup(SARibbonButtonGroupWidget* rightBar);
    void createStatusBar();
    void addSomeOtherAction();
    QAction* createAction(const QString& text, const QString& iconurl, const QString& objName);
    QAction* createAction(const QString& text, const QString& iconurl);

    void sectionDispWidget(bool showWithSection = false);

    void loadQssStyle(SARibbonBar* ribbon);
    bool view3DLoadStl(std::string stl_path);

signals:
    void signalUpdateBrowser();
    void signalUpdateStatusBar(QString message);

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    bool onCalibTriggered();
    void onCalibToolTriggered();
    void onActionCustomizeAndSaveTriggered(bool b);
    void onActionHelpTriggered();
    void onActionRemoveAppBtnTriggered(bool b);
    void showScanData(QListWidgetItem* item);
    void saveProjectSection();
    void savePerspective();
    void restorePerspective();
    void saveLayoutConfig();
    void restoreLayoutConfig();

private:
    StartupProgressWindow* m_startupProgressWindow;
    SARibbonCustomizeWidget* m_customizeWidget;

    QVtkViewer* m_viewer3d;
    CImageViewer* m_ImgViewer;
    ChartViewer* m_ChartViewer;
    QListWidget* m_dataBrowser;

    ads::CDockManager* m_dockManager;

    QString m_section_path;
    nlohmann::json m_section;       // calibration section
    SARibbonActionsManager* m_actMgr;
    int m_actionTagText;
    int m_actionTagWithIcon;
};

#endif  // MAINWINDOW_H
