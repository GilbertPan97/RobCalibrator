﻿#include "mainwindow.h"
#include "SARibbonBar.h"
#include "SARibbonCategory.h"
#include "SARibbonPannel.h"
#include "SARibbonToolButton.h"
#include "SARibbonTabBar.h"
#include "SARibbonMenu.h"
#include "SARibbonComboBox.h"
#include "SARibbonLineEdit.h"
#include "SARibbonGallery.h"
#include "SARibbonCheckBox.h"
#include "SARibbonQuickAccessBar.h"
#include "SARibbonButtonGroupWidget.h"
#include "SARibbonApplicationButton.h"
#include "SARibbonCustomizeWidget.h"
#include "SARibbonElementManager.h"
#include "SARibbonCustomizeDialog.h"
#include "SAFramelessHelper.h"
#include "data_processor.h"
#include "calibrator.h"
#include "algorithm.h"
#include "SFRSensorManager.h"

#include "DockManager.h"
#include "DockWidget.h"
#include "DockAreaWidget.h"
#include <qtpropertymanager.h>
#include <qteditorfactory.h>
#include <qttreepropertybrowser.h>
#include <qtbuttonpropertybrowser.h>
#include <qtgroupboxpropertybrowser.h>
#include "ImageViewer.h"

#include <QVTKOpenGLNativeWidget.h>     // QVTK... is vtk GUI support
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkContourFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkCaptionActor2D.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkNamedColors.h>
#include <vtkTextProperty.h>
#include <vtkNew.h>
#include <vtkAxesActor.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <QDockWidget>
#include <QApplication>
#include <QFile>
#include <QThread>
#include <QTextEdit>
#include <QToolBar>
#include <QTreeView>
#include <QScrollBar>
#include <QResource>
#include <QListWidget>
#include <QFileSystemModel>
#include <QTextBrowser>
#include <QAbstractButton>
#include <QAbstractItemView>
#include <QFileDialog>
#include <QProgressDialog>
#include <QInputDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QPropertyAnimation>
#include <QMessageBox>
#include <QSettings>
#include <QAction>
#include <QMenu>
#include <QSlider>
#include <QDateTime>
#include <QStatusBar>
#include <QDebug>
#include <QElapsedTimer>
#include <QRadioButton>
#include <QButtonGroup>
#include <QSpinBox>
#include <QLineEdit>
#include <QCalendarWidget>
#include <QXmlStreamWriter>
#include <QTextStream>
#include <QFontComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QScreen>
#include <QCloseEvent>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct MainWindowPrivate
{

};

static QIcon svgIcon(const QString& File)
{
	// This is a workaround, because in item views SVG icons are not
	// properly scaled and look blurry or pixelate
	QIcon SvgIcon(File);
	SvgIcon.addPixmap(SvgIcon.pixmap(92));
	return SvgIcon;
}

nlohmann::json jsonSolXYZQUA(Eigen::VectorXf xyzqua){
    nlohmann::json js_xyzqua;
    if(xyzqua.size() != 7){
        std::cerr << "ERROR: xyzqua size is-" << xyzqua.size() << std::endl;
        exit(-1);
    }
    js_xyzqua["xyz"]["x"] = xyzqua[0];
    js_xyzqua["xyz"]["y"] = xyzqua[1];
    js_xyzqua["xyz"]["z"] = xyzqua[2];
    js_xyzqua["quanternion"]["w"] = xyzqua[3];
    js_xyzqua["quanternion"]["x"] = xyzqua[4];
    js_xyzqua["quanternion"]["y"] = xyzqua[5];
    js_xyzqua["quanternion"]["z"] = xyzqua[6];

    return js_xyzqua;
}

MainWindow::MainWindow(QWidget* par) : 
    SARibbonMainWindow(par), 
    m_customizeWidget(nullptr)
{
    // startup Progress Window
    m_startupProgressWindow = new StartupProgressWindow();

    m_startupProgressWindow->show();
    QApplication::processEvents();          // Refresh gui
    m_startupProgressWindow->wait(3000);

    // get screen size
    QScreen *screen = QGuiApplication::primaryScreen();
    QSize scrnSize = screen->size();

    // create helper widget
    SAFramelessHelper* helper = framelessHelper();
    helper->setRubberBandOnResize(false);
    setWindowTitle(("SfrCalib"));
    setStatusBar(new QStatusBar());

    // construct ribbon bar
    SARibbonBar* ribbon = ribbonBar();
    ribbon->setContentsMargins(5, 0, 5, 0);
    // this->loadQssStyle(ribbon);

    // set robot connect checker
    ribbon->applicationButton()->setText(("   HardWare   "));
    SFRSensorManager* camManager = SFRSensorManager::getInstance();
    connect(ribbon->applicationButton(), &QAbstractButton::released, this, [=](){
        QFile file("config.txt");
        if(file.exists()){
            if(file.open(QIODevice::ReadOnly))
            {
                QTextStream in(&file);
                QString config;
                in>>config;
                camManager->load(config);
                file.close();
            }
        }
        camManager->setWindowTitle("Camera Manager");
        camManager->resize(800, 400);
        camManager->showNormal();
        QApplication::processEvents();

        // QFile file2("config.txt");
        // if(file2.open(QIODevice::WriteOnly)){
        //     QTextStream out(&file2);
        //     out << camManager->save();
        //     file2.close();
        // }
    });
    
    // add 3D line scanner calib ribbon
    SARibbonCategory* cat_3dScanner = new SARibbonCategory();
    cat_3dScanner->setCategoryName(tr("   ScanCalibrator   "));
    cat_3dScanner->setObjectName(("ScanCalibrator"));
    createCategoryCalib(cat_3dScanner);
    ribbon->addCategoryPage(cat_3dScanner);

//     // TODO: add camera hand-eye calibrator
// #ifdef CAMERA_PLUGIN
//     SARibbonCategory* cat_2dCamera = new SARibbonCategory();
//     cat_2dCamera->setCategoryName(tr("   CamCalibrator   "));
//     cat_2dCamera->setObjectName(("CamCalibrator"));
//     ribbon->addCategoryPage(cat_2dCamera);
// #endif

    // add 2D & 3D viewer, and QtPropertyBrowser
    m_dockManager = new ads::CDockManager(this);
    auto DockWidget1 = createQtVTKviewerDockWidget();
    auto DockWidget2 = createQtChartviewerDockWidget();
    auto DockWidget4 = createDataBrowserDockWidget();    // DataBrowser should build before PropertyBrowser
    auto DockWidget3 = createPropertyBrowser();

    // set DockWidget area and initial size
    auto TopDockArea = m_dockManager->addDockWidget(ads::TopDockWidgetArea, DockWidget2);
    auto ViewerDockArea = m_dockManager->addDockWidget(ads::CenterDockWidgetArea, DockWidget1, TopDockArea);
    auto CenterDockArea = m_dockManager->addDockWidget(ads::BottomDockWidgetArea, DockWidget4, ViewerDockArea);
    auto RightDockArea = m_dockManager->addDockWidget(ads::RightDockWidgetArea, DockWidget3);

    m_dockManager->setSplitterSizes(CenterDockArea, {800/10 * 8, 800/10 * 3});
    m_dockManager->setSplitterSizes(RightDockArea, {1280/10 * 7, 1280/10 * 3});
    
    this->setCentralWidget(m_dockManager);

    // add quickAccessBar and rightBar
    SARibbonQuickAccessBar* quickAccessBar = ribbon->quickAccessBar();
    createQuickAccessBar(quickAccessBar);
    SARibbonButtonGroupWidget* rightBar = ribbon->rightButtonGroup();
    createRightButtonGroup(rightBar);
    addSomeOtherAction();

    // create status bar and add to main window
    createStatusBar();
    
    // set main window minimum size, and create a window shrink animation
    QSize minSize = {scrnSize.width()/2, scrnSize.height()/2};
    this->setMinimumSize(minSize);

    // startup Progress Window hide
    m_startupProgressWindow->hide();

    // Maximize show main window & set window Icon
    this->showMaximized();
    this->setWindowIcon(QIcon(":/icon/resource/RobHand.svg"));

    // Test stl model load
    // this->view3DLoadStl("../../data/ScanData/SR7140.STL");
}

ads::CDockWidget* MainWindow::createQtVTKviewerDockWidget()
{
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("3D Viewer"));
    m_viewer3d = new QVtkViewer();
    DockWidget->setWidget(m_viewer3d);
    DockWidget->setIcon(svgIcon(":/icon/resource/color_lens.svg"));
    DockWidget->setToggleViewActionMode(ads::CDockWidget::ActionModeShow);

    return DockWidget;
}

ads::CDockWidget* MainWindow::createQtImgviewerDockWidget()
{
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("2D Viewer"));
    m_ImgViewer = new CImageViewer();
    DockWidget->setWidget(m_ImgViewer, ads::CDockWidget::ForceNoScrollArea);
    DockWidget->setIcon(svgIcon(":/icon/resource/photo.svg"));
    auto ToolBar = DockWidget->createDefaultToolBar();
    ToolBar->addActions(m_ImgViewer->actions());

    m_ImgViewer->loadFile("./RobHand.jpeg");

    return DockWidget;
}

ads::CDockWidget* MainWindow::createQtChartviewerDockWidget()
{
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("2D Viewer"));
    m_ChartViewer = new ChartViewer();
    DockWidget->setWidget(m_ChartViewer, ads::CDockWidget::ForceNoScrollArea);
    DockWidget->setIcon(svgIcon(":/icon/resource/photo.svg"));
    auto ToolBar = DockWidget->createDefaultToolBar();
    ToolBar->addActions(m_ChartViewer->actions());

    return DockWidget;
}

ads::CDockWidget* MainWindow::createPropertyBrowser()
{
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("Property Browser"));
    
    QWidget* w = new QWidget();

    // BUG: createPropertyBrowser should createDataBrowserDockWidget first
    QtStringPropertyManager* fileName = new QtStringPropertyManager(w);
    QtPathPropertyManager* filePath = new QtPathPropertyManager(w);
    QtStringPropertyManager* objName = new QtStringPropertyManager(w);
    QtIntPropertyManager* pntsNumber = new QtIntPropertyManager(w);
    QtGroupPropertyManager* filter = new QtGroupPropertyManager(w);
    QtBoolPropertyManager* filterEnable = new QtBoolPropertyManager(w);
    QtEnumPropertyManager* filterAlgor = new QtEnumPropertyManager(w);
    QtEnumPropertyManager* color = new QtEnumPropertyManager(w);

    QtProperty* item0 = fileName->addProperty("File Name");
    QtProperty* item1 = filePath->addProperty("Path");
    QtProperty* item2 = objName->addProperty("Objection");
    QtProperty* item3 = pntsNumber->addProperty("Points Number");
    QtProperty* item4 = filter->addProperty("Filter");
    QtProperty* item5 = filterEnable->addProperty("Filter Enable");
    QtProperty* item6 = filterAlgor->addProperty("Method");
    QtProperty* item7 = color->addProperty("Color");

    item4->addSubProperty(item5);
    item4->addSubProperty(item6);

    QtAbstractPropertyBrowser *editor = new QtTreePropertyBrowser(w);
    editor->addProperty(item0);
    editor->addProperty(item1);
    editor->addProperty(item2);
    editor->addProperty(item3);
    editor->addProperty(item4);
    editor->addProperty(item7);

    // connect data browser and property browser
    if(m_dataBrowser == nullptr){
        std::cout << "ERROR: Data browser can not connect to property browser. "
                  << "Please create property browser first.\n";
        exit(-1);
    }
    connect(m_dataBrowser, &QListWidget::itemActivated, this, [=](QListWidgetItem* item){
        QString file_name = item->text();
        fileName->setValue(item0, file_name);

        auto ImgSaveDir = m_section["Dataset"]["ImagesDir"];
        QString ImgPath = QString::fromStdString(ImgSaveDir) + "/" + file_name;
        filePath->setValue(item1, ImgPath);

        objName->setValue(item2, "Sphere");

        std::vector<cv::Point3f> scan_line;
		cv::FileStorage fs(ImgPath.toStdString(), cv::FileStorage::READ);
		fs["scan_line"] >> scan_line;
		fs.release();

        pntsNumber->setValue(item3, scan_line.size());
    });

    // build layout to DockWidget
    QVBoxLayout* v_layout = new QVBoxLayout(w);
    QWidget* w_panel = createControlPanel();         // control panel widget

    v_layout->addWidget(w_panel);
    v_layout->addWidget(editor);

    DockWidget->setWidget(w);
    DockWidget->setIcon(svgIcon(":/icon/resource/date_range.svg"));
    DockWidget->setToggleViewActionMode(ads::CDockWidget::ActionModeShow);

    return DockWidget;
}

QWidget* MainWindow::createControlPanel()
{
    QWidget *w = new QWidget();

    // Noise filter
    QGroupBox *gBoxNoise = new QGroupBox("Noise Filter", w);
    QGridLayout *grid_layout = new QGridLayout(w);
    grid_layout->setAlignment(Qt::AlignCenter);
    gBoxNoise->setLayout(grid_layout);

    QLabel *lab00 = new QLabel(w);
    lab00->setText("Eps:");
    QSlider *slider00 = new QSlider(Qt::Horizontal, w);
    QLineEdit *lineEdit00 = new QLineEdit(w);
    slider00->setRange(0, 10);
    lineEdit00->setFixedWidth(120);
    grid_layout->addWidget(lab00, 0, 0);
    grid_layout->addWidget(slider00, 0, 1);
    grid_layout->addWidget(lineEdit00, 0, 2);

    QLabel *lab01 = new QLabel(w);
    lab01->setText("Minimum Points:");
    QSlider *slider01 = new QSlider(Qt::Horizontal, w);
    QLineEdit *lineEdit01 = new QLineEdit(w);
    lineEdit01->setFixedWidth(120);
    grid_layout->addWidget(lab01, 1, 0);
    grid_layout->addWidget(slider01, 1, 1);
    grid_layout->addWidget(lineEdit01, 1, 2);

    // Background filter
    QGroupBox *gBoxBackg = new QGroupBox("Background Filter", w);
    QGridLayout *grid_layout1 = new QGridLayout(w);
    grid_layout1->setAlignment(Qt::AlignCenter);
    gBoxBackg->setLayout(grid_layout1);

    QLabel *lab10 = new QLabel(w);
    lab10->setText("Radius:");
    QSlider *slider10 = new QSlider(Qt::Horizontal, w);
    QLineEdit *lineEdit10 = new QLineEdit(w);
    slider10->setRange(0, 10);
    lineEdit10->setFixedWidth(120);
    grid_layout1->addWidget(lab10, 0, 0);
    grid_layout1->addWidget(slider10, 0, 1);
    grid_layout1->addWidget(lineEdit10, 0, 2);

    QLabel *lab11 = new QLabel(w);
    lab11->setText("Minimum Points:");
    QSlider *slider11 = new QSlider(Qt::Horizontal, w);
    QLineEdit *lineEdit11 = new QLineEdit(w);
    lineEdit11->setFixedWidth(120);
    grid_layout1->addWidget(lab11, 1, 0);
    grid_layout1->addWidget(slider11, 1, 1);
    grid_layout1->addWidget(lineEdit11, 1, 2);

    // Feature point extractor
    QGroupBox *gBoxFeature = new QGroupBox("Feature extractor", w);
    QGridLayout *grid_layout2 = new QGridLayout(w);
    grid_layout2->setAlignment(Qt::AlignCenter);
    gBoxFeature->setLayout(grid_layout2);

    QLabel *lab20 = new QLabel(w);
    lab20->setText("Point Index:");
    QSpinBox *spinBox20 = new QSpinBox(w);
    spinBox20->setRange(0, 10);
    spinBox20->setSingleStep(1);
    grid_layout2->addWidget(lab20, 0, 0);
    grid_layout2->addWidget(spinBox20, 0, 1, 1, 2);

    QLabel *lab21 = new QLabel(w);
    QLineEdit *lineEdit21 = new QLineEdit(w);
    QSlider *slider21 = new QSlider(Qt::Horizontal, w);
    lab21->setText("Tolerance:");
    slider21->setRange(0, 10);
    lineEdit21->setFixedWidth(120);
    grid_layout2->addWidget(lab21, 1, 0);
    grid_layout2->addWidget(slider21, 1, 1);
    grid_layout2->addWidget(lineEdit21, 1, 2);

    // Add panels to widget
    QVBoxLayout *v_layout = new QVBoxLayout(w);
    v_layout->addWidget(gBoxNoise);
    v_layout->addWidget(gBoxBackg);
    v_layout->addWidget(gBoxFeature);
    v_layout->setAlignment(Qt::AlignCenter);

    w->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    w->setLayout(v_layout);

    return w;
}

/**
 * Creates a dock widget with a file system tree view
 */
ads::CDockWidget* MainWindow::createDataBrowserDockWidget()
{

    QWidget *w = new QWidget();
    w->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    w->setStyleSheet("border: 2px solid darkGray; border-radius: 10px;");

    QVBoxLayout *layout_v = new QVBoxLayout(w);
    layout_v->setAlignment(Qt::AlignCenter);
    w->setLayout(layout_v);

    // create m_dataBrowser, and add it to widget
    m_dataBrowser = new QListWidget(w);
    layout_v->addWidget(m_dataBrowser);
    m_dataBrowser->setViewMode(QListView::IconMode);
    m_dataBrowser->setGridSize(QSize(180,200));
    m_dataBrowser->setIconSize(QSize(156,156));
    m_dataBrowser->setResizeMode(QListWidget::Adjust);
    m_dataBrowser->setStyleSheet("QListWidget::item:hover{background-color:rgba(208,206,206,255);border-radius:8px; }"
                                 "QListWidget::item:selected{background-color:rgba(208,206,206,255);border-radius:8px; }"
                                 "QScrollBar:horizontal{width:6px}");
    
    m_dataBrowser->setFlow(QListView::LeftToRight);
    m_dataBrowser->setWrapping(false);
    m_dataBrowser->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    m_dataBrowser->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_dataBrowser->setFocusPolicy(Qt::NoFocus);

    QScrollBar* horizontalScrollBar = new QScrollBar(Qt::Horizontal, m_dataBrowser);
    m_dataBrowser->setHorizontalScrollBar(horizontalScrollBar);

    // create DockWidget, and add widget to it
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("Data Browser"));
    DockWidget->setWidget(w);
    DockWidget->setIcon(svgIcon(":/icon/resource/folder.svg"));
    
    // read files from image directory
    connect(this, &MainWindow::signalUpdateBrowser, this, [this](){
        // check m_section
        if(m_section["Dataset"]["ImagesDir"].empty()){
            std::cerr << "ERROR: Images directory is empty.\n";
            return;
        }

        // set filter extension
        QString folderPath = QString::fromStdString(m_section["Dataset"]["ImagesDir"]);
        QString fileExtension = "*.yml";

        QDir folder(folderPath);
        if (!folder.exists()) {
            std::cerr << "ERROR: YML folder is not exist!\n";
            return;
        }

        // only yml files can be browser display
        QStringList nameFilters;
        nameFilters << fileExtension;
        folder.setNameFilters(nameFilters);
        QStringList files = folder.entryList(QDir::Files | QDir::NoDotAndDotDot);
        files.sort();

        // clear listWidget and reset data browser
        m_dataBrowser->clear();
        m_dataBrowser->setStyleSheet("QListWidget{background-color: transparent;}");
        QFont font("Arial", 7);
        for(int i = 0; i < files.count(); i++){
            QListWidgetItem* item = new QListWidgetItem();
            item->setFont(font);
            item->setText(files[i]);
            item->setIcon(QIcon(":/icon/resource/photo.svg"));
            m_dataBrowser->addItem(item);
        }

        connect(m_dataBrowser, &QListWidget::itemActivated, this, &MainWindow::showScanData);
    });
    
    return DockWidget;
}

void MainWindow::createCategoryCalib(SARibbonCategory* page)
{
    SARibbonPannel* pannel_section = new SARibbonPannel(("Project Section"));
    SARibbonPannel* pannel_data = new SARibbonPannel(("Calibration Data"));
    SARibbonPannel* pannel_mode = new SARibbonPannel(("Mode Settings"));
    SARibbonPannel* pannel_calib = new SARibbonPannel(("Calibration"));
    SARibbonPannel* pannel_result = new SARibbonPannel(("Result"));

    pannel_section->setPannelTitleHeight(30);

    /* calibration section */
    QAction* actNewSection = createAction(tr("New Section"), ":/icon/resource/new.svg");
    connect(actNewSection, &QAction::triggered, this, [this](){ sectionDispWidget(false); });

    QAction* actOpenSection = createAction(tr("Open Section"), ":/icon/resource/open.svg");
    connect(actOpenSection, &QAction::triggered, this, [this](){
        m_section_path = QFileDialog::getOpenFileName(
            this,
            QStringLiteral("Open Section."),
            QCoreApplication::applicationDirPath(),
            QStringLiteral("Calibration Section (*.sec)")
        );

        if(m_section_path.isEmpty()){
            std::cout << "WARNING: No section file load.\n";
            return;
        }

        // parse json file to m_section
        std::ifstream js_file_in(m_section_path.toStdString());
        m_section = nlohmann::json::parse(js_file_in);
        js_file_in.close();

        // display m_section with sectionWidget
        sectionDispWidget(true);

        // emit update signal to refersh other widget, such as 3D viewer and property browser.
        emit signalUpdateBrowser();
    });

    QAction* actSaveSection = createAction(tr("Save Section"), ":/icon/resource/save1.svg");
    connect(actSaveSection, &QAction::triggered, this, &MainWindow::saveProjectSection);

    pannel_section->addLargeAction(actNewSection);
    pannel_section->addLargeAction(actOpenSection);
    pannel_section->addLargeAction(actSaveSection);

    /* calibration data */
    QAction* actCollect = createAction(tr("Auto Collection"), ":/icon/resource/camera.svg");

    QAction* actSetImgs = createAction(tr("Set Images"), ":/icon/resource/images.svg");
    connect(actSetImgs, &QAction::triggered, this, [this](){
        QString scan_data_dir = QFileDialog::getExistingDirectory(
            this, 
            QStringLiteral("Images (Scan data) Directory."),
            QCoreApplication::applicationDirPath(), 
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
        );
        m_section["Dataset"]["ImagesDir"] = scan_data_dir.toStdString();
    });

    QAction* actSetRobs = createAction(tr("Set RobPoses"), ":/icon/resource/robot.svg");
    connect(actSetRobs, &QAction::triggered, this, [this](){
        QString rob_pose_path = QFileDialog::getOpenFileName(
            this,
            QStringLiteral("Robot Poses File Path"),
            QCoreApplication::applicationDirPath(),
            QStringLiteral("Robot poses(*.rob)")
        );
        m_section["RobPosesFile"] = rob_pose_path.toStdString();
    });

    pannel_data->addLargeAction(actCollect);
    pannel_data->addLargeAction(actSetImgs);
    pannel_data->addLargeAction(actSetRobs);

    /* calibration mode */
    QAction *actType = createAction(tr("Calibrate Type"), ":/icon/resource/type.svg");
    QMenu *optionsType = new QMenu();
    actType->setMenu(optionsType);
    QActionGroup *optionGroup = new QActionGroup(actType);
    QAction *option_EIH = new QAction("Eye In Hand", optionGroup);
    QAction *option_ETH = new QAction("Eye To Hand", optionGroup);
    
    option_EIH->setCheckable(true);
    option_ETH->setCheckable(true);
    optionsType->addAction(option_EIH);
    optionsType->addAction(option_ETH);
    connect(option_EIH, &QAction::triggered, this, [this](){ m_section["CalibType"] = "EyeInHand";});
    connect(option_ETH, &QAction::triggered, this, [this](){ m_section["CalibType"] = "EyeToHand";});

    QAction* actTool = createAction(tr("Calibrate Tools"), ":/icon/resource/tools.svg");
    connect(actTool, &QAction::triggered, this, &MainWindow::onCalibToolTriggered);

    QAction* actAlgors = createAction(tr("Algorithms"), ":/icon/resource/algorithms.svg");
    QMenu *optionsAlgor = new QMenu();
    actAlgors->setMenu(optionsAlgor);
    QActionGroup *optionGroup1 = new QActionGroup(actAlgors);
    QAction *optionRegr = new QAction("Regression", optionGroup1);
    QAction *optionIter = new QAction("Iterative", optionGroup1);

    optionRegr->setCheckable(true);
    optionIter->setCheckable(true);
    optionsAlgor->addAction(optionRegr);
    optionsAlgor->addAction(optionIter);
    connect(optionRegr, &QAction::triggered, this, [this]()
    { 
        m_section["Config"]["CalibAlgor"] = "Regression";
    });
    connect(optionIter, &QAction::triggered, this, [this]()
    { 
        m_section["Config"]["CalibAlgor"] = "Iterative";
    });

    pannel_mode->addLargeAction(actType);
    pannel_mode->addLargeAction(actTool);
    pannel_mode->addLargeAction(actAlgors);

    /* calibration run */
    QAction* actCalib = createAction(tr("Calibration"), ":/icon/resource/calibration.svg");
    pannel_calib->addLargeAction(actCalib);

    connect(actCalib, &QAction::triggered, this, &MainWindow::onCalibTriggered);

    /* calib result */
    QAction* actExport = createAction(tr("Export"), ":/icon/resource/export.svg");
    QAction* actDisp = createAction(tr("Display"), ":/icon/resource/display.svg");
    pannel_result->addLargeAction(actExport);
    pannel_result->addLargeAction(actDisp);

    page->addPannel(pannel_section);
    page->addPannel(pannel_data);
    page->addPannel(pannel_mode);
    page->addPannel(pannel_calib);
    page->addPannel(pannel_result);
}

void MainWindow::closeEvent(QCloseEvent *event) {
    // if m_section is empty, close window directly
    bool projectNeedsSaving = false;
    if(!m_section.empty() && !m_section.is_null())
        projectNeedsSaving = true;  // check if the project needs saving

    if (projectNeedsSaving) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Save project?", "Do you want to save the project before closing?",
                                      QMessageBox::Yes|QMessageBox::No|QMessageBox::Cancel);
        if (reply == QMessageBox::Cancel) {
            event->ignore();        // ignore the close event
            return;
        } else if (reply == QMessageBox::Yes) {
            saveProjectSection();       // save the project
        }
    }
    QMainWindow::closeEvent(event);
}

bool MainWindow::onCalibTriggered()
{
    if(m_section.empty()){
        QMessageBox::warning(this, "Error", " Fail to load section",
			QMessageBox::Ok);
        return false;
    }

    QDialog dlg(this);
    dlg.resize(1080, 600);
    QVBoxLayout* layout = new QVBoxLayout(&dlg);
    QProgressBar* prog = new QProgressBar(&dlg);
    prog->setRange(0, 100);
    prog->setValue(0);
    QLabel* lab = new QLabel("Calibration result:", &dlg);
    QTextBrowser* textbrowser = new QTextBrowser(&dlg);

    layout->addWidget(prog);
    layout->addWidget(lab);
    layout->addWidget(textbrowser);

    dlg.show();
    QApplication::processEvents();      // Refresh gui

    std::string ImgSaveDir = m_section["Dataset"]["ImagesDir"];
    // Read scan lines data
	int snap_cnt = m_section["Dataset"]["NbrData"];      // @FIXME: the calibration data should be writen to section
	std::vector<std::vector<cv::Point3f>> scan_lines;
	for (size_t i = 0; i < snap_cnt; i++){
		// read from yml file, @TODO: what is the name sequence of files
        std::string file_name = "p" + std::to_string(i + 1) + ".yml";
		std::string file_path = ImgSaveDir + "/" + file_name;

		std::vector<cv::Point3f> scan_line;
		cv::FileStorage fs(file_path, cv::FileStorage::READ);
		fs["scan_line"] >> scan_line;

		scan_lines.push_back(scan_line);
		fs.release();
	}
    prog->setValue(10);
    QThread::msleep(800);       // wait 800ms 

	// Process scan data, @TODO: expose sphere center direction interface
	DataProc proc(scan_lines, CalibObj::SPHERE);
	float rad_sphere = m_section["CalibObj"]["Radius"];
    std::string dirc_sphCtr = m_section["CalibObj"]["SphCtrDirc"];
	std::vector<cv::Point3f> ctr_pnts = proc.CalcSphereCtrs(rad_sphere, dirc_sphCtr);
    prog->setValue(30);
    QThread::msleep(800);

	// Read robot pose data
	std::string rob_js_path = m_section["Dataset"]["RobPosesFile"];
	std::vector<Eigen::Vector<float, 6>> vec_rob_pose;
    
    std::ifstream js_file_in(rob_js_path);
    auto js_data = nlohmann::json::parse(js_file_in)["RobPoses"];
    js_file_in.close();
    for (auto& pose : js_data) {
        Eigen::Vector<float, 6> rob_pose;
        for (int i = 0; i < 6; ++i)
            rob_pose[i] = pose[i];
        vec_rob_pose.push_back(rob_pose);
    }
    prog->setValue(50);
    QThread::msleep(800);

	// Create calibration handle
	LineScanner::HandEyeCalib hec;

	hec.SetRobPose(vec_rob_pose);

	hec.SetObjData(ctr_pnts, CalibObj::SPHERE);
    prog->setValue(70);
    QThread::msleep(800);

    // @TODO: expose solver method optional 
	hec.run(LineScanner::SolveMethod::ITERATION);
    prog->setValue(100);

	std::vector<float> dis_ctr;
    float err = 0;
    bool check = hec.CalcCalibError(dis_ctr, err);

    Eigen::Matrix4f sol_mtr = hec.GetCalcResult();
    auto sol_xyzwpr = hec.GetCalcResultXYZWPR();
    auto sol_xyzQua = hec.GetCalcResultXYZQUA();

    // save result to section
    m_section["Solution"] = {
        { "xyzwpr", sol_xyzwpr },
        { "HTM", {
            {sol_mtr(0,0), sol_mtr(0,1), sol_mtr(0,2), sol_mtr(0,3)},
            {sol_mtr(1,0), sol_mtr(1,1), sol_mtr(1,2), sol_mtr(1,3)},
            {sol_mtr(2,0), sol_mtr(2,1), sol_mtr(2,2), sol_mtr(2,3)},
            {sol_mtr(3,0), sol_mtr(3,1), sol_mtr(3,2), sol_mtr(3,3)} }
        },
        { "xyzQua", jsonSolXYZQUA(sol_xyzQua) }
    };

    emit signalUpdateStatusBar("Calibration finished.");

    // Print calibration result
    Eigen::IOFormat fmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    std::ostringstream stream;
    stream << sol_mtr.format(fmt);
    std::string msg = stream.str() + "\n";
    textbrowser->append("INFO: Calibration result in matrix format:");
    textbrowser->append(QString::fromStdString(msg));

    std::string msg1;
    msg1 += "[";
    for (const auto& elem : sol_xyzwpr)
        msg1 += std::to_string(elem) + " ";
    msg1 += "]\n";
    textbrowser->append("INFO: Calibration result in XYZWPR format:");
    textbrowser->append(QString::fromStdString(msg1));

    std::string msg2;
    msg2 += "[";
    for (const auto& elem : dis_ctr)
        msg2 += std::to_string(elem) + " ";
    msg2 += "]\n";
    textbrowser->append("INFO: The distance of center point in robot base frame is:");
    textbrowser->append(QString::fromStdString(msg2));

    std::string msg3;
    msg3 += "INFO: Calibration error is: [ ";
    msg3 += std::to_string(err);
    msg3 += " ]\n";
    textbrowser->append(QString::fromStdString(msg3));
    
    dlg.exec();

    return true;
}

void MainWindow::onCalibToolTriggered()
{
    QWidget* w = new QWidget();
    QtTreePropertyBrowser* tool_browser = new QtTreePropertyBrowser();

    QtEnumPropertyManager* toolName = new QtEnumPropertyManager(tool_browser);
    QtEnumEditorFactory* toolNameFactory = new QtEnumEditorFactory(toolName);
    QtProperty* item0 = toolName->addProperty("Tool Name");
    toolName->setEnumNames(item0, QStringList() << "Sphere");
    tool_browser->setFactoryForManager(toolName, toolNameFactory);
    tool_browser->addProperty(item0);

    QtDoublePropertyManager* sphereRadius = new QtDoublePropertyManager(tool_browser);
    QtDoubleSpinBoxFactory * sphereRadiusFactory = new QtDoubleSpinBoxFactory(tool_browser);
    QtProperty* item1 = sphereRadius->addProperty("Sphere Radius");
    sphereRadius->setSingleStep(item1, 0.1);
    sphereRadius->setRange(item1, 0, 100);
    tool_browser->setFactoryForManager(sphereRadius, sphereRadiusFactory);
    tool_browser->addProperty(item1);

    QVBoxLayout* layout = new QVBoxLayout();
    QPushButton* setbtn = new QPushButton("OK");
    connect(setbtn, &QPushButton::released, this, [w](){ delete w; });
    
    layout->addWidget(tool_browser);
    layout->addWidget(setbtn);
    w->setLayout(layout);
    w->resize(800, 400);
    w->show();
    QApplication::processEvents();
}

void MainWindow::onActionCustomizeAndSaveTriggered(bool b)
{
    Q_UNUSED(b);
    SARibbonCustomizeDialog dlg(this);
    dlg.setupActionsManager(m_actMgr);
    dlg.fromXml("customize.xml");
    if (SARibbonCustomizeDialog::Accepted == dlg.exec()) {
        dlg.applys();
        QByteArray str;
        QXmlStreamWriter xml(&str);
        xml.setAutoFormatting(true);
        xml.setAutoFormattingIndent(2);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)  // QXmlStreamWriter always encodes XML in UTF-8.
        xml.setCodec("utf-8");
#endif
        xml.writeStartDocument();
        bool isok = dlg.toXml(&xml);
        xml.writeEndDocument();
        if (isok) {
            QFile f("customize.xml");
            if (f.open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Truncate)) {
                QTextStream s(&f);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)  // QTextStream always encodes XML in UTF-8.
                s.setCodec("utf-8");
#endif
                s << str;
                s.flush();
            }
        }
    }
}

void MainWindow::onActionHelpTriggered()
{
    QMessageBox::information(this,
                             tr("infomation"),
                             tr("\n ==============="
                                "\n SfrCalib version:%1"
                                "\n Author:Jiabin Pan"
                                "\n Email:panjiabin@shanghai-fanuc.com.cn"
                                "\n ===============")
                                     .arg(SARibbonBar::versionString()));
}

void MainWindow::onActionRemoveAppBtnTriggered(bool b)
{
    if (b) {
        ribbonBar()->setApplicationButton(nullptr);
    } else {
        SARibbonApplicationButton* actionRemoveAppBtn = new SARibbonApplicationButton();
        actionRemoveAppBtn->setText(tr("File"));
        this->ribbonBar()->setApplicationButton(actionRemoveAppBtn);
    }
}

void MainWindow::showScanData(QListWidgetItem* item)
{
    QString qstr = item->text();
    std::string ImgSaveDir = m_section["Dataset"]["ImagesDir"];
    std::string ImgPath = ImgSaveDir + "/" + qstr.toUtf8().constData();
    
    std::vector<cv::Point3f> pointCloud;
    cv::FileStorage fs(ImgPath, cv::FileStorage::READ);
    fs["scan_line"] >> pointCloud;
    fs.release();

    m_viewer3d->RenderWinReset();
    m_viewer3d->pointsDisplay(pointCloud, 4);

    std::vector<cv::Point2f> pointCloud_2d;
    for(const cv::Point3f pnt3d: pointCloud)
        pointCloud_2d.push_back({pnt3d.x, pnt3d.z});

    m_ChartViewer->setPointCloud(pointCloud_2d);
}

void MainWindow::saveProjectSection()
{
    // get save data and add to section
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString currentTimeString = currentDateTime.toString("yyyy-MM-dd hh:mm:ss");
    m_section["Description"]["SaveTime"] = currentTimeString.toStdString();

    if(m_section_path.isEmpty())
        m_section_path = QFileDialog::getSaveFileName(
            this, 
            QStringLiteral("Section Saveing Path."),
            QCoreApplication::applicationDirPath(), 
            QStringLiteral("Calibration Section (*.sec)")
        );
    
    std::ofstream json_file_out(m_section_path.toStdString());
    json_file_out << std::setw(4) << m_section << std::endl;
    json_file_out.close();
}

void MainWindow::createQuickAccessBar(SARibbonQuickAccessBar* quickAccessBar)
{
    quickAccessBar->addAction(createAction("save", ":/icon/resource/save.svg", "save-quickbar"));
    quickAccessBar->addSeparator();
    quickAccessBar->addAction(createAction("undo", ":/icon/resource/undo.svg"));
    quickAccessBar->addAction(createAction("redo", ":/icon/resource/redo.svg"));
    quickAccessBar->addSeparator();
    
    // add view setting menu
    QMenu* menu_view = new QMenu("View", this);
    menu_view->setIcon(QIcon(":/icon/resource/show.svg"));
    int dockWidgetsCnt = m_dockManager->dockWidgetsMap().size();
    std::cout << "The number of dockWidgets is:" << dockWidgetsCnt << std::endl;
    QMap<QString, ads::CDockWidget *> dockWidgetMap = m_dockManager->dockWidgetsMap();
    QMap<QString, ads::CDockWidget *>::const_iterator iter;
    for (iter = dockWidgetMap.constBegin(); iter != dockWidgetMap.constEnd(); ++iter)
    {
        QString dockWidgetName = iter.key();
        ads::CDockWidget *dockWidget = iter.value();
        std::cout << "DockWidget " << dockWidgetName.toStdString()  << " address is: "
            << dockWidget << std::endl;

        QAction* action = new QAction(dockWidgetName, menu_view);
        action->setCheckable(true);
        action->setChecked(true);
        menu_view->addAction(dockWidget->toggleViewAction());
    }
    menu_view->addSeparator();
    QAction* act_loadLayout = new QAction("Load Layout", menu_view);
    QAction* act_saveLayout = new QAction("Save Layout", menu_view);
    menu_view->addAction(act_loadLayout);
    menu_view->addAction(act_saveLayout);
    connect(act_loadLayout, &QAction::triggered, this, &MainWindow::restoreLayoutConfig);
    connect(act_saveLayout, &QAction::triggered, this, &MainWindow::saveLayoutConfig);

    quickAccessBar->addMenu(menu_view);

    QAction* actionCustomizeAndSave = createAction("customize and save", ":/icon/resource/customize.svg");
    quickAccessBar->addAction(actionCustomizeAndSave);
    connect(actionCustomizeAndSave, &QAction::triggered, this, &MainWindow::onActionCustomizeAndSaveTriggered);
}

void MainWindow::createRightButtonGroup(SARibbonButtonGroupWidget* rightBar)
{
    QAction* actionHelp = createAction(tr("help"), ":/icon/resource/help.svg");
    connect(actionHelp, &QAction::triggered, this, &MainWindow::onActionHelpTriggered);
    rightBar->addAction(actionHelp);
}

void MainWindow::createStatusBar()
{
    // get screen size
    QScreen *screen = QGuiApplication::primaryScreen();
    QSize scrnSize = screen->size();

    // create status bar
    QStatusBar *statusBar = new QStatusBar(this);
    this->setStatusBar(statusBar);
    statusBar->resize(screen->size().width(), 40);
    statusBar->setStyleSheet("QStatusBar{border: 1px solid black; background-color: #E6E6E6}");

    // add log button
    QPushButton *logBtn = new QPushButton("Log");
    logBtn->setFixedWidth(scrnSize.width() * 0.05);
    logBtn->setFlat(true);
    logBtn->setStyleSheet("QPushButton {border: 1px; background-color: #D9D9D9} QPushButton:hover {background-color: gray;}");
    statusBar->addWidget(logBtn);

    // add status widgets
    QLabel *label = new QLabel("SfrCalib Status:");
    label->setFixedWidth(scrnSize.width() * 0.15);
    statusBar->addWidget(label);
    QLabel *statusLabel = new QLabel();
    label->setAlignment(Qt::AlignCenter);
    statusLabel->setFixedWidth(scrnSize.width() * 0.5);
    statusLabel->setAlignment(Qt::AlignCenter);
    statusLabel->setText("Ready");
    statusBar->addWidget(statusLabel);

    connect(this, &MainWindow::signalUpdateStatusBar, this, [=](QString str_msg){
        statusLabel->setText(str_msg);
    });
}

void MainWindow::addSomeOtherAction()
{
    //添加其他的action，这些action并不在ribbon管理范围，主要用于SARibbonCustomizeWidget自定义用
    QAction* acttext1 = new QAction(("text action1"), this);
    QAction* acttext2 = new QAction(("text action2"), this);
    QAction* acttext3 = new QAction(("text action3"), this);
    QAction* acttext4 = new QAction(("text action4"), this);
    QAction* acttext5 = new QAction(("text action5"), this);

    QAction* actIcon1 = new QAction(QIcon(":/icon/resource/layout.svg"), ("action with icon"), this);

    m_actionTagText     = SARibbonActionsManager::UserDefineActionTag + 1;
    m_actionTagWithIcon = SARibbonActionsManager::UserDefineActionTag + 2;

    m_actMgr = new SARibbonActionsManager(this);  //申明过程已经自动注册所有action

    //以下注册特别的action
    m_actMgr->registeAction(acttext1, SARibbonActionsManager::CommonlyUsedActionTag);
    m_actMgr->registeAction(acttext3, SARibbonActionsManager::CommonlyUsedActionTag);
    m_actMgr->registeAction(acttext5, SARibbonActionsManager::CommonlyUsedActionTag);
    m_actMgr->registeAction(actIcon1, SARibbonActionsManager::CommonlyUsedActionTag);

    m_actMgr->registeAction(acttext1, m_actionTagText);
    m_actMgr->registeAction(acttext2, m_actionTagText);
    m_actMgr->registeAction(acttext3, m_actionTagText);
    m_actMgr->registeAction(acttext4, m_actionTagText);
    m_actMgr->registeAction(acttext5, m_actionTagText);

    m_actMgr->registeAction(actIcon1, m_actionTagWithIcon);

    m_actMgr->setTagName(SARibbonActionsManager::CommonlyUsedActionTag, tr("in common use"));  //
    m_actMgr->setTagName(m_actionTagText, tr("no icon action"));
    m_actMgr->setTagName(m_actionTagWithIcon, tr("have icon action"));
}

QAction* MainWindow::createAction(const QString& text, const QString& iconurl, const QString& objName)
{
    QAction* act = new QAction(this);
    act->setText(text);
    act->setIcon(QIcon(iconurl));
    act->setObjectName(objName);
    return act;
}

QAction* MainWindow::createAction(const QString& text, const QString& iconurl)
{
    QAction* act = new QAction(this);
    act->setText(text);
    act->setIcon(QIcon(iconurl));
    act->setObjectName(text);
    return act;
}

void MainWindow::sectionDispWidget(bool showWithSection)
{
    QWidget* widget_sec = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout();
    QtTreePropertyBrowser* sec_browser = new QtTreePropertyBrowser();

    QString default_path = "C:/Users/Administrator/Desktop";

    // add description widget
    QLabel* lab_desc = new QLabel("Description");
    QTextEdit* description = new QTextEdit();
    description->setFixedHeight(150);
    layout->addWidget(lab_desc);
    layout->addWidget(description);

    // config scan data file directory
    QtPathPropertyManager* imgsDirManager = new QtPathPropertyManager(sec_browser);
    QtPathEditorFactory* imgsDirFactory = new QtPathEditorFactory(sec_browser);
    auto propImgDir = imgsDirManager->addProperty("Images Directory");
    imgsDirManager->setValue(propImgDir, default_path);
    sec_browser->setFactoryForManager(imgsDirManager, imgsDirFactory);
    sec_browser->addProperty(propImgDir);

    // config robot pose file
    QtPathPropertyManager* robPathManager = new QtPathPropertyManager(sec_browser);
    QtPathEditorFactory* robPathFactory = new QtPathEditorFactory(sec_browser);
    auto propRobFilePath = robPathManager->addProperty("Robot Data File");
    robPathManager->setValue(propRobFilePath, default_path);
    sec_browser->setFactoryForManager(robPathManager, robPathFactory);
    sec_browser->addProperty(propRobFilePath);

    // @TODO: add 
    QtIntPropertyManager* nbrDataManager = new QtIntPropertyManager(sec_browser);
    QtSpinBoxFactory* nbrDataFactory = new QtSpinBoxFactory(sec_browser);
    QtProperty* propNbrData = nbrDataManager->addProperty("Number of data");
    nbrDataManager->setSingleStep(propNbrData, 1);
    nbrDataManager->setRange(propNbrData, 0, 100);
    sec_browser->setFactoryForManager(nbrDataManager, nbrDataFactory);
    sec_browser->addProperty(propNbrData);

    // config calibrate algorithm
    QtEnumPropertyManager* algorManager = new QtEnumPropertyManager(sec_browser);
    QtEnumEditorFactory* algorFactory = new QtEnumEditorFactory(sec_browser);
    auto propAlgor = algorManager->addProperty("Algorithm");
    algorManager->setEnumNames(propAlgor, QStringList() << "Iterative" << "Regression");
    sec_browser->setFactoryForManager(algorManager, algorFactory);
    sec_browser->addProperty(propAlgor);

    // config calibrate type
    QtEnumPropertyManager* calibTypeManager = new QtEnumPropertyManager(sec_browser);
    QtEnumEditorFactory* calibTypeFactory = new QtEnumEditorFactory(sec_browser);
    auto propCalibType = calibTypeManager->addProperty("Calibrate type");
    calibTypeManager->setEnumNames(propCalibType, QStringList() << "EyeInHand" << "EyeToHand");
    sec_browser->setFactoryForManager(calibTypeManager, calibTypeFactory);
    sec_browser->addProperty(propCalibType);

    // config calibrate objection 
    QtGroupPropertyManager* calibObjManager = new QtGroupPropertyManager(sec_browser);
    QtProperty* propCalibObj = calibObjManager->addProperty("Calibrate Objection");

    QtEnumPropertyManager* ObjNameManager = new QtEnumPropertyManager(sec_browser);
    QtEnumEditorFactory* ObjNameFactory = new QtEnumEditorFactory(sec_browser);
    QtProperty* propCalibObjName = ObjNameManager->addProperty("Name");
    ObjNameManager->setEnumNames(propCalibObjName, QStringList() << "Sphere" << "Block" << "Board");
    sec_browser->setFactoryForManager(ObjNameManager, ObjNameFactory);
    propCalibObj->addSubProperty(propCalibObjName);

    QtEnumPropertyManager* sphereCtrDircManager = new QtEnumPropertyManager(sec_browser);
    QtEnumEditorFactory* sphereCtrDircFactory = new QtEnumEditorFactory(sec_browser);
    QtProperty* propSphereDircCtr = sphereCtrDircManager->addProperty("Sphere center direction");
    sphereCtrDircManager->setEnumNames(propSphereDircCtr, QStringList() << "+Y" << "-Y");
    sec_browser->setFactoryForManager(sphereCtrDircManager, sphereCtrDircFactory);
    propCalibObj->addSubProperty(propSphereDircCtr);

    QtDoublePropertyManager* sphereRadiusManager = new QtDoublePropertyManager(sec_browser);
    QtDoubleSpinBoxFactory* sphereRadiusFactory = new QtDoubleSpinBoxFactory(sec_browser);
    QtProperty* propCalibObjPara = sphereRadiusManager->addProperty("Sphere Radius");
    sphereRadiusManager->setSingleStep(propCalibObjPara, 0.1);
    sphereRadiusManager->setRange(propCalibObjPara, 0, 100);
    sec_browser->setFactoryForManager(sphereRadiusManager, sphereRadiusFactory);
    propCalibObj->addSubProperty(propCalibObjPara);

    // QtSpinBoxFactory *calibObjFactory = new QtSpinBoxFactory(sec_browser);
    // sec_browser->setFactoryForManager(calibObjManager->subIntPropertyManager(), calibObjFactory);
    sec_browser->addProperty(propCalibObj);
    layout->addWidget(sec_browser);

    // Load section to section Widget
    if(showWithSection && !m_section.empty()){
        std::string desc = m_section["Description"]["Label"];
        description->setText(QString::fromStdString(desc));

        std::string img_dir = m_section["Dataset"]["ImagesDir"];
        imgsDirManager->setValue(propImgDir, QString::fromStdString(img_dir));

        std::string rob_path = m_section["Dataset"]["RobPosesFile"];
        robPathManager->setValue(propRobFilePath, QString::fromStdString(rob_path));

        int nbr_data = m_section["Dataset"]["NbrData"];
        nbrDataManager->setValue(propNbrData, nbr_data);

        std::string calib_type = m_section["Config"]["CalibType"];
        if(calib_type == "EyeInHand")
            calibTypeManager->setValue(propCalibType, 0);
        else
            calibTypeManager->setValue(propCalibType, 1);
        
        std::string calib_algor = m_section["Config"]["CalibAlgor"];
        if(calib_algor == "Iterative")
            algorManager->setValue(propAlgor, 0);
        else
            algorManager->setValue(propAlgor, 1);

        std::string calib_obj = m_section["CalibObj"]["Name"];
        if(calib_obj == "Sphere"){
            std::string dircCtr = m_section["CalibObj"]["SphCtrDirc"];
            ObjNameManager->setValue(propCalibObjName, 0);
            sphereRadiusManager->setValue(propCalibObjPara, m_section["CalibObj"]["Radius"]);
            if (dircCtr == "+Y")
                sphereCtrDircManager->setValue(propSphereDircCtr, 0);
            else if (dircCtr == "-Y"){
                sphereCtrDircManager->setValue(propSphereDircCtr, 1);
            }
            else {
                std::cerr << "ERROR: Wrong sphere center direction in camera frame.\n";
            }
        }
        else if (calib_obj == "Block"){
            ObjNameManager->setValue(propCalibObjName, 1);
        } 
        else if (calib_obj == "Board"){
            ObjNameManager->setValue(propCalibObjName, 2);
        }   
        else{
            std::cerr << "ERROR: Wrong calibration objection name.\n";
        }
    }

    // save calib config to section 
    QHBoxLayout* v_layout = new QHBoxLayout();
    QPushButton* setBtn = new QPushButton("OK");
    connect(setBtn, &QPushButton::released, this, [=]()
    {
        // Clear m_section before save new config
        m_section.clear();

        // section: Description
        QDateTime currentDateTime = QDateTime::currentDateTime();
        QString currentTimeString = currentDateTime.toString("yyyy-MM-dd hh:mm:ss");
        m_section["Description"] = { 
            {"Label", description->toPlainText().toStdString()},
            {"SaveTime", currentTimeString.toStdString()}
        };

        // section: Dataset
        auto imgs_dir = imgsDirManager->value(propImgDir);
        auto nbr_data = nbrDataManager->value(propNbrData);
        auto rob_pose_file = robPathManager->value(propRobFilePath);
        m_section["Dataset"] = {
            {"NbrData", nbr_data},
            {"ImagesDir", imgs_dir.toStdString()},
            {"RobPosesFile", rob_pose_file.toStdString()}
        };

        // section: Config
        auto id_algor = algorManager->value(propAlgor);
        auto id_type = calibTypeManager->value(propCalibType);
        m_section["Config"] = {
            {"CalibAlgor", algorManager->enumNames(propAlgor).at(id_algor).toStdString()},
            {"CalibType", calibTypeManager->enumNames(propCalibType).at(id_type).toStdString()}
        };

        // section: Calib objection
        auto id_calibObj = ObjNameManager->value(propCalibObjName);
        m_section["CalibObj"] = {
            {"Name", ObjNameManager->enumNames(propCalibObjName).at(id_calibObj).toStdString()},
            {"SphCtrDirc", propSphereDircCtr->valueText().toStdString()},
            {"Radius", sphereRadiusManager->value(propCalibObjPara)}
        };

        delete widget_sec; 
    });

    QPushButton* cancelBtn = new QPushButton("Cancel");
    connect(cancelBtn, &QPushButton::released, this, [=](){ delete widget_sec; });
    v_layout->addWidget(setBtn);
    v_layout->addWidget(cancelBtn);
    layout->addLayout(v_layout);

    widget_sec->setLayout(layout);
    widget_sec->setMinimumSize(1000, 600);
    widget_sec->show();
    QApplication::processEvents();
}

void MainWindow::loadQssStyle(SARibbonBar* ribbon)
{
    QFile f(":/theme/resource/default.qss");
    if (!f.exists()) {
        QString fdir = QFileDialog::getOpenFileName(this, tr("select qss file"));
        if (fdir.isEmpty()) {
            return;
        }
        f.setFileName(fdir);
    }

    if (!f.open(QIODevice::ReadOnly)) {
        return;
    }
    QString qss(f.readAll());
    ribbon->setStyleSheet(qss);
}

bool MainWindow::view3DLoadStl(std::string stl_path)
{
    m_viewer3d->RenderWinReset();

    m_viewer3d->stlDiaplay(stl_path);

    return true;
}

void MainWindow::savePerspective()
{
	QString PerspectiveName = QInputDialog::getText(this, "Save Perspective", "Enter unique name:");
	if (PerspectiveName.isEmpty())
		return;

	m_dockManager->addPerspective(PerspectiveName);

    QString fileDir = QFileDialog::getExistingDirectory(this, "Save INI File", "");
    if (!fileDir.isEmpty()){
        QSettings Settings(fileDir + "/Settings.ini", QSettings::IniFormat);

        m_dockManager->savePerspectives(Settings);
    }
}

void MainWindow::restorePerspective()
{
    QString filePath = QFileDialog::getOpenFileName(this, "Open INI File", "", "INI Files (*.ini)");
    qDebug() << filePath;

    if (!filePath.isEmpty()){
        QSettings settings(filePath, QSettings::IniFormat);

        m_dockManager->loadPerspectives(settings);
        m_dockManager->updateGeometry();
    }  
}

//============================================================================
void MainWindow::saveLayoutConfig()
{
	QSettings Settings("Settings.ini", QSettings::IniFormat);
	Settings.setValue("mainWindow/Geometry", this->saveGeometry());
	Settings.setValue("mainWindow/DockingState", m_dockManager->saveState());
}


//============================================================================
void MainWindow::restoreLayoutConfig()
{
	QSettings Settings1("Settings.ini", QSettings::IniFormat);
	this->restoreGeometry(Settings1.value("mainWindow/Geometry").toByteArray());
	m_dockManager->restoreState(Settings1.value("mainWindow/DockingState").toByteArray());
}

