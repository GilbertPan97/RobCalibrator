#include "mainwindow.h"
#include "SARibbonBar.h"
#include "SARibbonCategory.h"
#include "SARibbonPannel.h"
#include "SARibbonToolButton.h"
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
#include <QListWidget>
#include <QFileSystemModel>
#include <QTextBrowser>
#include <QAbstractButton>
#include <QFileDialog>
#include <QProgressDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QMessageBox>
#include <QAction>
#include <QMenu>
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
#include <QVBoxLayout>
#include <QWidget>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

static QIcon svgIcon(const QString& File)
{
	// This is a workaround, because in item views SVG icons are not
	// properly scaled and look blurry or pixelate
	QIcon SvgIcon(File);
	SvgIcon.addPixmap(SvgIcon.pixmap(92));
	return SvgIcon;
}

MainWindow::MainWindow(QWidget* par) : SARibbonMainWindow(par), m_customizeWidget(nullptr)
{
    SAFramelessHelper* helper = framelessHelper();
    helper->setRubberBandOnResize(false);
    setWindowTitle(("SfrCalib"));
    setStatusBar(new QStatusBar());

    // construct ribbon bar
    SARibbonBar* ribbon = ribbonBar();
    ribbon->setContentsMargins(5, 0, 5, 0);
    SARibbonQuickAccessBar* quickAccessBar = ribbon->quickAccessBar();
    createQuickAccessBar(quickAccessBar);
    SARibbonButtonGroupWidget* rightBar = ribbon->rightButtonGroup();
    createRightButtonGroup(rightBar);
    addSomeOtherAction();

    // set robot connect checker
    ribbon->applicationButton()->setText(("Connector"));
    
    // add 3D line scanner calib ribbon
    SARibbonCategory* cat_3dScanner = new SARibbonCategory();
    cat_3dScanner->setCategoryName(tr("Calibrator"));
    cat_3dScanner->setObjectName(("Calibrator"));
    createCategoryCalib(cat_3dScanner);
    ribbon->addCategoryPage(cat_3dScanner);

    // add 2D & 3D viewer, and QtPropertyBrowser
    ads::CDockManager* DockManager = new ads::CDockManager(this);
    auto DockWidget1 = createQtVTKviewerDockWidget();
    auto DockWidget2 = createQtImgviewerDockWidget();
    auto DockWidget4 = createDataBrowserDockWidget();    // DataBrowser should build before PropertyBrowser
    auto DockWidget3 = createPropertyBrowser();

    auto TopDockArea = DockManager->addDockWidget(ads::TopDockWidgetArea, DockWidget2);

    auto ViewerDockArea = DockManager->addDockWidget(ads::CenterDockWidgetArea, DockWidget1, TopDockArea);
    
    auto CenterDockArea = DockManager->addDockWidget(ads::BottomDockWidgetArea, DockWidget4, ViewerDockArea);
    DockManager->addDockWidget(ads::RightDockWidgetArea, DockWidget3);
    
    this->setCentralWidget(DockManager);

    setMinimumWidth(1200);
    setMinimumHeight(800);

    // Maximize show main window & set window Icon
    showMaximized();
    setWindowIcon(QIcon(":/icon/icon/RobHand.svg"));
}

ads::CDockWidget* MainWindow::createQtVTKviewerDockWidget()
{
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("3D Viewer"));
    m_viewer3d = new thQVTKOpenGLNativeWidget();
    DockWidget->setWidget(m_viewer3d);
    DockWidget->setIcon(svgIcon(":/icon/icon/color_lens.svg"));
    DockWidget->setToggleViewActionMode(ads::CDockWidget::ActionModeShow);

    return DockWidget;
}

ads::CDockWidget* MainWindow::createQtImgviewerDockWidget()
{
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("2D Viewer"));
    m_viewer2d = new CImageViewer();
    DockWidget->setWidget(m_viewer2d, ads::CDockWidget::ForceNoScrollArea);
    DockWidget->setIcon(svgIcon(":/icon/icon/photo.svg"));
    auto ToolBar = DockWidget->createDefaultToolBar();
    ToolBar->addActions(m_viewer2d->actions());
    m_viewer2d->loadFile("./RobHand.jpeg");

    return DockWidget;
}

ads::CDockWidget* MainWindow::createPropertyBrowser()
{
    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("Property Browser"));
    
    QWidget* w = new QWidget();

    // BUG: createPropertyBrowser before should createDataBrowserDockWidget
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
    connect(m_dataBrowser, &QListWidget::itemActivated, this, [=](QListWidgetItem* item){
        QString file_name = item->text();
        fileName->setValue(item0, file_name);

        auto ImgSaveDir = m_section["ImagesDir"];
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

    // add widget to layout
    v_layout->addWidget(editor, 1);

    DockWidget->setWidget(w);
    DockWidget->setIcon(svgIcon(":/icon/icon/date_range.svg"));
    DockWidget->setToggleViewActionMode(ads::CDockWidget::ActionModeShow);

    return DockWidget;
}

/**
 * Creates a dock widget with a file system tree view
 */
ads::CDockWidget* MainWindow::createDataBrowserDockWidget()
{
    m_dataBrowser = new QListWidget();
    m_dataBrowser->setViewMode(QListView::IconMode);
    m_dataBrowser->setGridSize(QSize(180,200));
    m_dataBrowser->setIconSize(QSize(156,156));
    m_dataBrowser->setResizeMode(QListWidget::Adjust);
    m_dataBrowser->setStyleSheet("QListWidget::item:hover{background-color:rgba(208,206,206,255);border-radius:8px; }"
                                 "QListWidget::item:selected{background-color:rgba(208,206,206,255);border-radius:8px; }"
                                 "QScrollBar:horizontal{width:6px}");
    // m_dataBrowser->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

    ads::CDockWidget* DockWidget = new ads::CDockWidget(QString("Data Browser"));
    DockWidget->setWidget(m_dataBrowser);
    DockWidget->setIcon(svgIcon(":/icon/icon/folder.svg"));
    // We disable focus to test focus highlighting if the dock widget content
    // does not support focus
    m_dataBrowser->setFocusPolicy(Qt::NoFocus);
    
    // read files from image directory
    connect(this, &MainWindow::signalUpdateBrowser, this, [this](){
        // check m_section
        if(m_section["ImagesDir"].empty()){
            std::cout << "ERROR: Images directory is empty.\n";
            return;
        }

        // set filter extension
        QString folderPath = QString::fromStdString(m_section["ImagesDir"]);
        QString fileExtension = "*.yml";

        QDir folder(folderPath);
        if (!folder.exists()) {
            std::cout << "ERROR: The folder is not exist!\n";
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
        for(int i = 0; i < files.count(); i++){
            QListWidgetItem* item = new QListWidgetItem();
            item->setIcon(QIcon(":/icon/icon/photo.svg"));
            item->setText(files[i]);
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

    /* calibration section */
    static QString section_path;
    QAction* actNewSection = createAction(tr("New Section"), ":/icon/icon/new.svg");
    connect(actNewSection, &QAction::triggered, this, &MainWindow::onNewSectionTriggered);

    QAction* actOpenSection = createAction(tr("Open Section"), ":/icon/icon/open.svg");
    connect(actOpenSection, &QAction::triggered, this, [this](){
        QString section_path_in = QFileDialog::getOpenFileName(
            this,
            QStringLiteral("Open Section."),
            QCoreApplication::applicationDirPath(),
            QStringLiteral("Calibration Section (*.json)")
        );

        if(!section_path_in.isEmpty()){
            std::ifstream js_file_in(section_path_in.toStdString());
            m_section = nlohmann::json::parse(js_file_in);
            js_file_in.close();
        }

        emit signalUpdateBrowser();

        section_path = section_path_in;
    });

    QAction* actSaveSection = createAction(tr("Save Section"), ":/icon/icon/save1.svg");
    connect(actSaveSection, &QAction::triggered, this, [this](){
        // get save data and add to section
        QDateTime currentDateTime = QDateTime::currentDateTime();
        QString currentTimeString = currentDateTime.toString("yyyy-MM-dd hh:mm:ss");
        m_section["SaveTime"] = currentTimeString.toStdString();

        if(section_path.isEmpty())
            section_path = QFileDialog::getSaveFileName(
                this, 
                QStringLiteral("Section Saveing Path."),
                QCoreApplication::applicationDirPath(), 
                QStringLiteral("Calibration Section (*.json)")
            );
        std::ofstream json_file_out(section_path.toStdString());
		json_file_out << std::setw(4) << m_section << std::endl;
		json_file_out.close();
    });
    pannel_section->addLargeAction(actNewSection);
    pannel_section->addLargeAction(actOpenSection);
    pannel_section->addLargeAction(actSaveSection);

    /* calibration data */
    QAction* actCollect = createAction(tr("Auto Collection"), ":/icon/icon/camera.svg");

    QAction* actSetImgs = createAction(tr("Set Images"), ":/icon/icon/images.svg");
    connect(actSetImgs, &QAction::triggered, this, [this](){
        QString scan_data_dir = QFileDialog::getExistingDirectory(
            this, 
            QStringLiteral("Images (Scan data) Directory."),
            QCoreApplication::applicationDirPath(), 
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
        );
        m_section["ImagesDir"] = scan_data_dir.toStdString();
    });

    QAction* actSetRobs = createAction(tr("Set RobPoses"), ":/icon/icon/robot.svg");
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
    QAction *actType = createAction(tr("Calibrate Type"), ":/icon/icon/type.svg");
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

    QAction* actTool = createAction(tr("Calibrate Tools"), ":/icon/icon/tools.svg");
    connect(actTool, &QAction::triggered, this, &MainWindow::onCalibToolTriggered);

    QAction* actAlgors = createAction(tr("Algorithms"), ":/icon/icon/algorithms.svg");
    QMenu *optionsAlgor = new QMenu();
    actAlgors->setMenu(optionsAlgor);
    QActionGroup *optionGroup1 = new QActionGroup(actAlgors);
    QAction *option_Regr = new QAction("Regression", optionGroup1);
    QAction *option_Iter = new QAction("Iterative", optionGroup1);

    option_Regr->setCheckable(true);
    option_Iter->setCheckable(true);
    optionsAlgor->addAction(option_Regr);
    optionsAlgor->addAction(option_Iter);
    connect(option_Regr, &QAction::triggered, this, [this](){ m_section["CalibAlgor"] = "Regression";});
    connect(option_Iter, &QAction::triggered, this, [this](){ m_section["CalibAlgor"] = "Iterative";});

    pannel_mode->addLargeAction(actType);
    pannel_mode->addLargeAction(actTool);
    pannel_mode->addLargeAction(actAlgors);

    /* calibration run */
    QAction* actCalib = createAction(tr("Calibration"), ":/icon/icon/calibration.svg");
    pannel_calib->addLargeAction(actCalib);

    connect(actCalib, &QAction::triggered, this, &MainWindow::onCalibTriggered);

    /* calib result */
    QAction* actExport = createAction(tr("Export"), ":/icon/icon/export.svg");
    QAction* actDisp = createAction(tr("Display"), ":/icon/icon/display.svg");
    pannel_result->addLargeAction(actExport);
    pannel_result->addLargeAction(actDisp);

    page->addPannel(pannel_section);
    page->addPannel(pannel_data);
    page->addPannel(pannel_mode);
    page->addPannel(pannel_calib);
    page->addPannel(pannel_result);
}

bool MainWindow::onCalibTriggered()
{
    if(m_section.empty()){
        QMessageBox::warning(this, "Error", " Fail to load section",
			QMessageBox::Ok);
        return false;
    }

    QDialog dlg(this);
    dlg.resize(1080, 400);
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

    std::string ImgSaveDir = m_section["ImagesDir"];
    // Read scan lines data
	int snap_cnt = 10;
	std::vector<std::vector<cv::Point3f>> scan_lines;
	for (size_t i = 0; i < snap_cnt; i++){
		// read from yml file
		std::string file_path = ImgSaveDir + "/" + std::to_string(i + 1) +"_3C_line.yml";

		std::vector<cv::Point3f> scan_line;
		cv::FileStorage fs(file_path, cv::FileStorage::READ);
		fs["scan_line"] >> scan_line;

		scan_lines.push_back(scan_line);
		fs.release();
	}
    prog->setValue(10);
    QThread::msleep(800);       // wait 800ms 

	// process scan data
	DataProc proc(scan_lines, Calibrator::CalibObj::SPHERE);
	float rad_sphere = 25.4 / 2.0;
	std::vector<cv::Point3f> ctr_pnts = proc.CalcSphereCtrs(rad_sphere);
    prog->setValue(30);
    QThread::msleep(800);

	// read robot pose data
	std::string rob_js_path = m_section["RobPosesFile"];
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

	// build calibration
	Calibrator::LineScanner::HandEyeCalib hec;

	hec.SetRobPose(vec_rob_pose);

	hec.SetObjData(ctr_pnts, Calibrator::CalibObj::SPHERE);
    prog->setValue(70);
    QThread::msleep(800);

	hec.run(Calibrator::LineScanner::SolveMethod::ITERATION);
    prog->setValue(100);

	std::vector<float> dis_ctr;
    float err = 0;
    bool check = hec.CalcCalibError(dis_ctr, err);

    Eigen::Matrix4f solution = hec.GetCalcResult();

    Eigen::IOFormat fmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    std::ostringstream stream;
    stream << solution.format(fmt);
    std::string msg = stream.str() + "\n";

    std::string msg1;
    msg1 += "[";
    for (const auto& elem : dis_ctr)
        msg1 += std::to_string(elem) + " ";
    msg1 += "]"; 
    
    textbrowser->append("INFO: Calibration result in matrix format:");
    textbrowser->append(QString::fromStdString(msg));
    textbrowser->append("INFO: The distance of center point in robot base frame is:");
    textbrowser->append(QString::fromStdString(msg1));
    
    dlg.exec();

    return true;
}

void MainWindow::onNewSectionTriggered()
{
    QWidget* w = new QWidget();
    QtTreePropertyBrowser* sec_browser = new QtTreePropertyBrowser();

    // config scan data file directory
    QtPathPropertyManager* imgs_dir_manager = new QtPathPropertyManager(sec_browser);
    QtPathEditorFactory* imgs_dir_editor = new QtPathEditorFactory(sec_browser);
    auto pathValue = imgs_dir_manager->addProperty("Images Directory");
    imgs_dir_manager->setValue(pathValue, "C:\\Users\\Administrator\\Desktop");
    sec_browser->setFactoryForManager(imgs_dir_manager, imgs_dir_editor);
    sec_browser->addProperty(pathValue);

    // config robot pose file
    QtPathPropertyManager* rob_path_manager = new QtPathPropertyManager(sec_browser);
    QtPathEditorFactory* rob_path_editor = new QtPathEditorFactory(sec_browser);
    auto pathValue1 = rob_path_manager->addProperty("Robot Data File");
    rob_path_manager->setValue(pathValue1, "C:\\Users\\Administrator\\Desktop");
    sec_browser->setFactoryForManager(rob_path_manager, rob_path_editor);
    sec_browser->addProperty(pathValue1);

    // config calibrate algorithm
    QtEnumPropertyManager* algor_manager = new QtEnumPropertyManager(sec_browser);
    QtEnumEditorFactory* algor_editor = new QtEnumEditorFactory(sec_browser);
    auto enumvalue = algor_manager->addProperty("Algorithm");
    algor_manager->setEnumNames(enumvalue, QStringList() << "Iterative" << "Regression");
    sec_browser->setFactoryForManager(algor_manager, algor_editor);
    sec_browser->addProperty(enumvalue);

    // config calibrate type
    QtEnumPropertyManager* calib_type_manager = new QtEnumPropertyManager(sec_browser);
    QtEnumEditorFactory* calib_type_editor = new QtEnumEditorFactory(sec_browser);
    auto enumvalue1 = calib_type_manager->addProperty("Calibrate type");
    calib_type_manager->setEnumNames(enumvalue1, QStringList() << "Eye In Hand" << "Eye To Hand");
    sec_browser->setFactoryForManager(calib_type_manager, calib_type_editor);
    sec_browser->addProperty(enumvalue1);

    // config calibrate objection 
    QtGroupPropertyManager* calibObjManager = new QtGroupPropertyManager(sec_browser);
    QtProperty *item0 = calibObjManager->addProperty("Calibrate Objection");

    QtEnumPropertyManager* ObjNameManager = new QtEnumPropertyManager(sec_browser);
    QtEnumEditorFactory* ObjNameFactory = new QtEnumEditorFactory(sec_browser);
    QtProperty* item1 = ObjNameManager->addProperty("Name");
    ObjNameManager->setEnumNames(item1, QStringList() << "Sphere" << "Block" << "Board");
    sec_browser->setFactoryForManager(ObjNameManager, ObjNameFactory);
    item0->addSubProperty(item1);

    QtIntPropertyManager* sphereRadiusManager = new QtIntPropertyManager(sec_browser);
    QtSpinBoxFactory* sphereRadiusFactory = new QtSpinBoxFactory(sec_browser);
    QtProperty* item2 = sphereRadiusManager->addProperty("Sphere Radius");
    sphereRadiusManager->setRange(item2, 0, 100);
    sec_browser->setFactoryForManager(sphereRadiusManager, sphereRadiusFactory);
    item0->addSubProperty(item2);

    // QtSpinBoxFactory *calibObjFactory = new QtSpinBoxFactory(sec_browser);
    // sec_browser->setFactoryForManager(calibObjManager->subIntPropertyManager(), calibObjFactory);
    sec_browser->addProperty(item0);

    QVBoxLayout* layout = new QVBoxLayout();
    QPushButton* setbtn = new QPushButton("OK");
    connect(setbtn, &QPushButton::released, this, [w](){ delete w; });
    layout->addWidget(sec_browser);
    layout->addWidget(setbtn);

    w->setLayout(layout);
    w->resize(800, 400);
    w->show();
    QApplication::processEvents();
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

    QtIntPropertyManager* sphereRadius = new QtIntPropertyManager(tool_browser);
    QtSpinBoxFactory* sphereRadiusFactory = new QtSpinBoxFactory(tool_browser);
    QtProperty* item1 = sphereRadius->addProperty("Sphere Radius");
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
            m_edit->append("write xml:");
            m_edit->append(str);
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
    std::string ImgSaveDir = m_section["ImagesDir"];
    std::string ImgPath = ImgSaveDir + "/" + qstr.toUtf8().constData();
    view3DLoadYML(ImgPath);
}

void MainWindow::createQuickAccessBar(SARibbonQuickAccessBar* quickAccessBar)
{
    quickAccessBar->addAction(createAction("save", ":/icon/icon/save.svg", "save-quickbar"));
    quickAccessBar->addSeparator();
    quickAccessBar->addAction(createAction("undo", ":/icon/icon/undo.svg"));
    quickAccessBar->addAction(createAction("redo", ":/icon/icon/redo.svg"));
    quickAccessBar->addSeparator();
    QMenu* m = new QMenu("Presentation File", this);
    m->setIcon(QIcon(":/icon/icon/presentationFile.svg"));
    for (int i = 0; i < 10; ++i) {
        m->addAction(createAction(QString("file%1").arg(i + 1), ":/icon/icon/file.svg"));
    }
    quickAccessBar->addMenu(m);

    QAction* actionCustomizeAndSave = createAction("customize and save", ":/icon/icon/customize.svg");
    quickAccessBar->addAction(actionCustomizeAndSave);
    connect(actionCustomizeAndSave, &QAction::triggered, this, &MainWindow::onActionCustomizeAndSaveTriggered);
}

void MainWindow::createRightButtonGroup(SARibbonButtonGroupWidget* rightBar)
{
    QAction* actionHelp = createAction(tr("help"), ":/icon/icon/help.svg");
    connect(actionHelp, &QAction::triggered, this, &MainWindow::onActionHelpTriggered);
    rightBar->addAction(actionHelp);
}

void MainWindow::addSomeOtherAction()
{
    //添加其他的action，这些action并不在ribbon管理范围，主要用于SARibbonCustomizeWidget自定义用
    QAction* acttext1 = new QAction(("text action1"), this);
    QAction* acttext2 = new QAction(("text action2"), this);
    QAction* acttext3 = new QAction(("text action3"), this);
    QAction* acttext4 = new QAction(("text action4"), this);
    QAction* acttext5 = new QAction(("text action5"), this);

    QAction* actIcon1 = new QAction(QIcon(":/icon/icon/layout.svg"), ("action with icon"), this);

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

bool MainWindow::view3DLoadYML(std::string img_path)
{
    m_viewer3d->RenderWinReset();
    std::vector<cv::Point3f> pointCloud;
    cv::FileStorage fs(img_path, cv::FileStorage::READ);
    fs["scan_line"] >> pointCloud;
    fs.release();
    m_viewer3d->displayPoints(pointCloud, 4);

    return true;
}

