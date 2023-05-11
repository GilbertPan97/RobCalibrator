#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "SARibbonMainWindow.h"
#include "thQVTKOpenGLNativeWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "DockAreaWidget.h"

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

class MainWindow : public SARibbonMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget* par = nullptr);

private:
    ads::CDockWidget* createQtVTKviewerDockWidget();
    ads::CDockWidget* createPropertyBrowser();
    void createCategoryCalib(SARibbonCategory* page);
    void createQuickAccessBar(SARibbonQuickAccessBar* quickAccessBar);
    void createRightButtonGroup(SARibbonButtonGroupWidget* rightBar);
    void addSomeOtherAction();
    QAction* createAction(const QString& text, const QString& iconurl, const QString& objName);
    QAction* createAction(const QString& text, const QString& iconurl);

    bool view3DLoadYML(std::string img_path);

private slots:
    bool onCalibTriggered();
    void onNewSectionTriggered();
    void onActionCustomizeAndSaveTriggered(bool b);
    void onActionHelpTriggered();
    void onActionRemoveAppBtnTriggered(bool b);

    // void onStyleClicked(int id);
    // void onActionUseQssTriggered();
    // void onActionLoadCustomizeXmlFileTriggered();
    // void onActionWindowFlagNormalButtonTriggered(bool b);

private:
    SARibbonContextCategory* m_contextCategory;
    SARibbonContextCategory* m_contextCategory2;
    SARibbonCustomizeWidget* m_customizeWidget;
    QTextEdit* m_edit;
    thQVTKOpenGLNativeWidget* m_viewer3d;
    nlohmann::json m_section;
    SARibbonActionsManager* m_actMgr;
    int m_actionTagText;
    int m_actionTagWithIcon;
};

#endif  // MAINWINDOW_H
