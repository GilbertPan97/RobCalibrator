
#include "QVtkViewer.h"
#include "StyleTrackballCamera.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_2_Core>
#include <QOpenGLTexture>
#include <QPointer>
#include <QScopedValueRollback>
#include <QtDebug>

#include <vtkAxesActor.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkTextProperty.h>
#include <vtkCaptionActor2D.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkAnnotatedCubeActor.h>
#include <vtkMapper.h>
#include <vtkPropAssembly.h>
#include <vtkPlaneSource.h>
#include <vtkTransform.h>
#include <vtkPolyDataMapper.h>
#include <vtkCamera.h>

#include <QVTKInteractor.h>
#include <QVTKInteractorAdapter.h>
#include <QVTKRenderWindowAdapter.h>
#include <vtkCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLState.h>
#include <opencv2/opencv.hpp>

//-----------------------------------------------------------------------------
QVtkViewer::QVtkViewer(QWidget* parentWdg, Qt::WindowFlags f)
    : QVtkViewer(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New().GetPointer(), parentWdg, f)
{

}

//-----------------------------------------------------------------------------
QVtkViewer::QVtkViewer(vtkGenericOpenGLRenderWindow* renderWin, QWidget* parentWdg, Qt::WindowFlags f): 
    Superclass(parentWdg, f),
    RenderWindow(nullptr),
    m_axesWidget(nullptr),
    RenderWindowAdapter(nullptr),
    EnableHiDPI(true),
    UnscaledDPI(72),
    DefaultCursor(QCursor(Qt::ArrowCursor))
{
    // default to strong focus
    this->setFocusPolicy(Qt::StrongFocus);
    this->setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);
    this->setMouseTracking(true);

    // we use `QOpenGLWidget::resized` instead of `resizeEvent` or `resizeGL` as
    // an indicator to resize our internal buffer size. This is done, since in
    // addition to widget resize,  `resized` gets fired when screen is changed
    // which causes devicePixelRatio changes.
    this->connect(this, SIGNAL(resized()), SLOT(updateSize()));

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
    renderer->SetBackground(1.0, 1.0, 1.0);              // 设置页面底部颜色值
    renderer->SetBackground2(colors->GetColor3d("SteelBlue").GetData());         // black background
    renderer->SetGradientBackground(1);
    renderWin->AddRenderer(renderer);

    this->setRenderWindow(renderWin);

    m_viewType = zxViewerType::TOP;
    this->setViewerType(zxViewerType::ISO);

    // enable qt gesture events
    this->grabGesture(Qt::PinchGesture);
    this->grabGesture(Qt::PanGesture);
    this->grabGesture(Qt::TapGesture);
    this->grabGesture(Qt::TapAndHoldGesture);
    this->grabGesture(Qt::SwipeGesture);
    
}

bool QVtkViewer::displayPoints(std::vector<cv::Point3f> pntCloud, int pntSize)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    int invalid_pnts = 0;
    for (const auto& point : pntCloud){
        if(!isnan(point.x) && !isnan(point.y) && !isnan(point.z)){
            vtkIdType id = points->InsertNextPoint(point.x, point.y, point.z);
            cells->InsertNextCell(1, &id);
        }
        invalid_pnts++;
    }
    std::cout << "INFO: The number of points is: " << points->GetNumberOfPoints() 
              << ", invalid points have: " << invalid_pnts << std::endl;

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    m_actor_pnts = vtkSmartPointer<vtkActor>::New();
    m_actor_pnts->SetMapper(mapper);
    m_actor_pnts->GetProperty()->SetColor(51/255.0, 63/255.0, 80/255.0);
    m_actor_pnts->GetProperty()->SetPointSize(pntSize);       // 点的大小为5个像素

    this->RenderWindow->SetMapped(true);
    std::cout << "INFO: The render window is " 
              << (this->RenderWindow->GetMapped() ? "mapped" : "not mapped") << std::endl;
    
    int main_ver, minor_ver;
    this->RenderWindow->GetOpenGLVersion(main_ver, minor_ver);
    std::cout << "MSG: Get OpenGL Version: "
              << main_ver << "." << minor_ver << std::endl;

    std::cout << "INFO: The number of renders is: "
              << this->RenderWindow->GetRenderers()->GetNumberOfItems() << std::endl;
    this->RenderWindow->GetRenderers()->GetFirstRenderer()->AddActor(m_actor_pnts);

    this->RenderWindow->Render();

    return true;
}

void QVtkViewer::RenderWinReset()
{
    this->RenderWindow->GetRenderers()->GetFirstRenderer()->RemoveActor(m_actor_pnts);

    this->RenderWindow->Render();
}

//-----------------------------------------------------------------------------
QVtkViewer::~QVtkViewer()
{
    this->makeCurrent();
    this->cleanupContext();

    if (m_axesWidget != NULL)
    {
        m_axesWidget->Off();
        m_axesWidget->Delete();
        m_axesWidget = NULL;
    }
}

//-----------------------------------------------------------------------------
void QVtkViewer::setRenderWindow(vtkRenderWindow* win)
{
    auto gwin = vtkGenericOpenGLRenderWindow::SafeDownCast(win);
    if (win != nullptr && gwin == nullptr)
    {
        qDebug() << "QVTKOpenGLNativeWidget requires a `vtkGenericOpenGLRenderWindow`. `"
            << win->GetClassName() << "` is not supported.";
    }
    this->setRenderWindow(gwin);
}

//-----------------------------------------------------------------------------
void QVtkViewer::setRenderWindow(vtkGenericOpenGLRenderWindow* win)
{
    if (this->RenderWindow == win)
    {
        return;
    }

    // this will release all OpenGL resources associated with the old render
    // window, if any.
    if (this->RenderWindowAdapter)
    {
        this->makeCurrent();
        this->RenderWindowAdapter.reset(nullptr);
    }
    this->RenderWindow = win;
    if (this->RenderWindow)
    {
        this->RenderWindow->SetReadyForRendering(false);

        // if an interactor wasn't provided, we'll make one by default
        if (!this->RenderWindow->GetInteractor())
        {
            // create a default interactor
            vtkNew<QVTKInteractor> iren;
            // iren->SetUseTDx(this->UseTDx);
            this->RenderWindow->SetInteractor(iren);
            iren->Initialize();

            // now set the default style
            vtkNew<StyleTrackballCamera> style;
            iren->SetInteractorStyle(style);

            setAxesSystem(iren);
            setReferenceAxesSystem();
            
        }

        if (this->isValid())
        {
            // this typically means that the render window is being changed after the
            // QVTKOpenGLNativeWidget has initialized itself in a previous update
            // pass, so we emulate the steps to ensure that the new vtkRenderWindow is
            // brought to the same state (minus the actual render).
            this->makeCurrent();
            this->initializeGL();
            this->updateSize();
        }
    }
}

//-----------------------------------------------------------------------------
vtkRenderWindow* QVtkViewer::renderWindow() const
{
    return this->RenderWindow;
}

//-----------------------------------------------------------------------------
QVTKInteractor* QVtkViewer::interactor() const
{
    return this->RenderWindow ? QVTKInteractor::SafeDownCast(this->RenderWindow->GetInteractor())
        : nullptr;
}

//-----------------------------------------------------------------------------
QSurfaceFormat QVtkViewer::defaultFormat(bool stereo_capable)
{
    return QVTKRenderWindowAdapter::defaultFormat(stereo_capable);
}

//-----------------------------------------------------------------------------
void QVtkViewer::setEnableHiDPI(bool enable)
{
    this->EnableHiDPI = enable;
    if (this->RenderWindowAdapter)
    {
        this->RenderWindowAdapter->setEnableHiDPI(enable);
    }
}

//-----------------------------------------------------------------------------
void QVtkViewer::setUnscaledDPI(int dpi)
{
    this->UnscaledDPI = dpi;
    if (this->RenderWindowAdapter)
    {
        this->RenderWindowAdapter->setUnscaledDPI(dpi);
    }
}

//-----------------------------------------------------------------------------
void QVtkViewer::setDefaultCursor(const QCursor& cursor)
{
    this->DefaultCursor = cursor;
    if (this->RenderWindowAdapter)
    {
        this->RenderWindowAdapter->setDefaultCursor(cursor);
    }
}

void QVtkViewer::setAxesSystem(vtkRenderWindowInteractor* iren)
{
    if (!iren)
    {
        return;
    }

    if (this->RenderWindow->GetRenderers()->GetNumberOfItems() == 0) {

        this->RenderWindow->AddRenderer(vtkRenderer::New());
        //m_window->interactor()->Enable();
    }

    if (m_axesWidget == nullptr)
    {
        m_axesWidget = vtkOrientationMarkerWidget::New();

        vtkSmartPointer<vtkAnnotatedCubeActor> cube = vtkSmartPointer<vtkAnnotatedCubeActor>::New();
        cube->SetFaceTextScale(0.65);
        cube->GetCubeProperty()->SetColor(0.9, 0.9, 0.9);
        cube->GetTextEdgesProperty()->SetLineWidth(1);
        cube->GetTextEdgesProperty()->SetDiffuse(0);
        cube->GetTextEdgesProperty()->SetAmbient(1);
        cube->GetTextEdgesProperty()->SetColor(0.2400, 0.2400, 0.2400);
        vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();

        cube->SetXPlusFaceText("L");//����
        cube->SetXMinusFaceText("R");
        cube->SetYPlusFaceText("P");//ǰ����
        cube->SetYMinusFaceText("A");
        cube->SetZPlusFaceText("S");//�µ���
        cube->SetZMinusFaceText("I");

        cube->GetXPlusFaceProperty()->SetColor(1, 0, 0);
        cube->GetXPlusFaceProperty()->SetInterpolationToFlat();

        cube->GetXMinusFaceProperty()->SetColor(1, 0, 0);
        cube->GetXMinusFaceProperty()->SetInterpolationToFlat();

        cube->GetYPlusFaceProperty()->SetColor(0, 1, 0);
        cube->GetYPlusFaceProperty()->SetInterpolationToFlat();

        cube->GetYMinusFaceProperty()->SetColor(0, 1, 0);
        cube->GetYMinusFaceProperty()->SetInterpolationToFlat();

        cube->GetZPlusFaceProperty()->SetColor(0, 0, 1);
        cube->GetZPlusFaceProperty()->SetInterpolationToFlat();

        cube->GetZMinusFaceProperty()->SetColor(0, 0, 1);
        cube->GetZMinusFaceProperty()->SetInterpolationToFlat();

        vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
        axesActor->SetShaftTypeToCylinder();
        axesActor->SetXAxisLabelText("x");
        axesActor->SetYAxisLabelText("y");
        axesActor->SetZAxisLabelText("z");
        axesActor->GetXAxisShaftProperty()->SetColor(1, 0, 0);
        axesActor->GetXAxisTipProperty()->SetColor(1, 0, 0);
        axesActor->GetYAxisShaftProperty()->SetColor(0, 1, 0);
        axesActor->GetYAxisTipProperty()->SetColor(0, 1, 0);
        axesActor->GetZAxisShaftProperty()->SetColor(0, 0, 1);
        axesActor->GetZAxisTipProperty()->SetColor(0, 0, 1);
        axesActor->GetZAxisCaptionActor2D()->GetProperty()->SetColor(1, 1, 1);
        axesActor->SetAxisLabels(1);
        axesActor->SetTotalLength(1.5, 1.5, 1.5);
        axesActor->SetCylinderRadius(0.500 * axesActor->GetCylinderRadius());
        axesActor->SetConeRadius(1.025 * axesActor->GetConeRadius());
        axesActor->SetSphereRadius(1.500 * axesActor->GetSphereRadius());

        vtkTextProperty* tprop = axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty();
        tprop->ItalicOn();
        tprop->ShadowOn();
        tprop->SetFontFamilyToTimes();
        axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy(tprop);
        axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy(tprop);


        vtkSmartPointer<vtkPropAssembly> assembly = vtkSmartPointer<vtkPropAssembly>::New();
        assembly->AddPart(axesActor);
        assembly->AddPart(cube);
        m_axesWidget->SetOutlineColor(1, 1, 1);
        m_axesWidget->SetViewport(0.0, 0.0, 0.3, 0.3);
        m_axesWidget->SetOrientationMarker(assembly);
        m_axesWidget->SetInteractor(iren);
    }
    m_axesWidget->On();
    m_axesWidget->SetInteractive(0);
}

std::vector<vtkSmartPointer<vtkActor>> QVtkViewer::MakePlanesActors(vtkNamedColors* colors) {
    std::vector<vtkSmartPointer<vtkTransformPolyDataFilter>> planes;
    std::vector<vtkSmartPointer<vtkPolyDataMapper>> mappers;
    std::vector<vtkSmartPointer<vtkActor>> actors;

    // Parameters for a plane lying in the x-y plane.
    std::array<int, 2> resolution{ {10, 10} };
    std::array<double, 3> origin{ {0.0, 0.0, 0.0} };
    std::array<double, 3> point1{ {1, 0, 0} };
    std::array<double, 3> point2{ {0, 1, 0} };

    std::array<double, 4> wxyz0{ {0, 0, 0, 0} };
    std::array<double, 3> translate{ {-0.5, -0.5, 0} };
    std::array<double, 4> wxyz1{ {-90, 1, 0, 0} };
    std::array<double, 4> wxyz2{ {-90, 0, 1, 0} };

    /*std::array<double, 4> wxyz0{ {0, 0, 0, 0} };
    std::array<double, 3> translate{ {0, 0, 0} };
    std::array<double, 4> wxyz1{ {90, 1, 0, 0} };
    std::array<double, 4> wxyz2{ {-90, 0, 1, 0} };*/

    planes.push_back(MakePlane(resolution, origin, point1, point2, wxyz0,
        translate)); // x-y plane
    planes.push_back(MakePlane(resolution, origin, point1, point2, wxyz1,
        translate)); // x-z plane
    planes.push_back(MakePlane(resolution, origin, point1, point2, wxyz2,
        translate)); // y-z plane
    for (size_t i = 0; i < planes.size(); ++i) {
        mappers.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
        mappers[i]->SetInputConnection(planes[i]->GetOutputPort());
        actors.push_back(vtkSmartPointer<vtkActor>::New());
        actors[i]->SetMapper(mappers[i]);
    }
    actors[0]->GetProperty()->SetColor(
        colors->GetColor3d("SeaGreen").GetData()); // Transverse plane
    actors[1]->GetProperty()->SetColor(
        colors->GetColor3d("DeepSkyBlue").GetData()); // Coronal plane
    actors[2]->GetProperty()->SetColor(
        colors->GetColor3d("Tomato").GetData()); // Saggital plane
    return actors;
}

vtkSmartPointer<vtkTransformPolyDataFilter> QVtkViewer::MakePlane(
    std::array<int, 2>& resolution, std::array<double, 3>& origin,
    std::array<double, 3>& point1, std::array<double, 3>& point2,
    std::array<double, 4>& wxyz, std::array<double, 3>& translate) {
    vtkSmartPointer<vtkPlaneSource> plane =
        vtkSmartPointer<vtkPlaneSource>::New();
    plane->SetResolution(resolution[0], resolution[1]);
    plane->SetOrigin(origin.data());
    plane->SetPoint1(point1.data());
    plane->SetPoint2(point2.data());
    vtkSmartPointer<vtkTransform> trnf = vtkSmartPointer<vtkTransform>::New();
    trnf->RotateWXYZ(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
    trnf->Translate(translate.data());
    vtkSmartPointer<vtkTransformPolyDataFilter> tpdPlane =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    tpdPlane->SetTransform(trnf);
    tpdPlane->SetInputConnection(plane->GetOutputPort());
    return tpdPlane;
}

void QVtkViewer::setReferenceAxesSystem()
{
    if (this->RenderWindow)
    {
        vtkSmartPointer<vtkNamedColors> colors =
            vtkSmartPointer<vtkNamedColors>::New();

        if (this->RenderWindow->GetRenderers()->GetNumberOfItems() == 0) {

            this->RenderWindow->AddRenderer(vtkRenderer::New());
            //m_window->interactor()->Enable();
        }

        std::vector<vtkSmartPointer<vtkActor>> actors = MakePlanesActors(colors);
        for (auto actor : actors) {
            this->RenderWindow->GetRenderers()->GetFirstRenderer()->AddViewProp(actor);
        }

        vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
        axesActor->SetShaftTypeToCylinder();
        axesActor->SetXAxisLabelText("");
        axesActor->SetYAxisLabelText("");
        axesActor->SetZAxisLabelText("");
        axesActor->GetXAxisShaftProperty()->SetColor(1, 0, 0);
        axesActor->GetXAxisTipProperty()->SetColor(1, 0, 0);
        axesActor->GetYAxisShaftProperty()->SetColor(0, 1, 0);
        axesActor->GetYAxisTipProperty()->SetColor(0, 1, 0);
        axesActor->GetZAxisShaftProperty()->SetColor(0, 0, 1);
        axesActor->GetZAxisTipProperty()->SetColor(0, 0, 1);
        axesActor->GetZAxisCaptionActor2D()->GetProperty()->SetColor(1, 1, 1);
        axesActor->SetAxisLabels(1);
        axesActor->SetTotalLength(1.5, 1.5, 1.5);
        axesActor->SetCylinderRadius(0.500 * axesActor->GetCylinderRadius());
        axesActor->SetConeRadius(1.025 * axesActor->GetConeRadius());
        axesActor->SetSphereRadius(1.500 * axesActor->GetSphereRadius());
        axesActor->SetOrigin(0., 0., 0.);

        vtkTextProperty* tprop = axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty();
        tprop->ItalicOn();
        tprop->ShadowOn();
        tprop->SetFontFamilyToTimes();

        this->RenderWindow->GetRenderers()->GetFirstRenderer()->AddActor(axesActor);
        this->RenderWindow->GetRenderers()->GetFirstRenderer()->ResetCamera();
    }
    
}

void QVtkViewer::setViewerType(zxViewerType type)
{
    if (type == m_viewType) return;

    if (!this->RenderWindow) return;

    m_viewType = type;
    vtkCamera* camera = vtkCamera::New();
    switch (type) {

    case zxViewerType::LEFT:

        camera->SetViewUp(0, 0, 1);
        camera->SetPosition(1, 0, 0);
        camera->SetFocalPoint(0, 0, 0);
        camera->ComputeViewPlaneNormal();
        break;
    case zxViewerType::RIGHT:

        camera->SetViewUp(0, 0, 1);
        camera->SetPosition(-1, 0, 0);
        camera->SetFocalPoint(0, 0, 0);
        camera->ComputeViewPlaneNormal();
        break;
    case zxViewerType::TOP:

        camera->SetViewUp(0, 1, 0);
        camera->SetPosition(0, 0, 1);
        camera->SetFocalPoint(0, 0, 0);
        camera->ComputeViewPlaneNormal();
        break;
    case zxViewerType::BOTTOM:

        camera->SetViewUp(0, -1, 0);
        camera->SetPosition(0, 0, -1);
        camera->SetFocalPoint(0, 0, 0);
        camera->ComputeViewPlaneNormal();
        break;
    case zxViewerType::FRONT:

        camera->SetViewUp(0, 0, 1);
        camera->SetPosition(0, -1, 0);
        camera->SetFocalPoint(0, 0, 0);
        camera->ComputeViewPlaneNormal();
        break;
    case zxViewerType::BACK:

        camera->SetViewUp(0, 0, 1);
        camera->SetPosition(0, 1, 0);
        camera->SetFocalPoint(0, 0, 0);
        camera->ComputeViewPlaneNormal();
        break;
    default:
        camera->SetViewUp(0, 0, 1);
        camera->SetPosition(0, -1, 0);
        camera->SetFocalPoint(0, 0, 0);
        camera->ComputeViewPlaneNormal();
        camera->Azimuth(30.0);
        camera->Elevation(30.0);


    }

    this->RenderWindow->GetRenderers()->GetFirstRenderer()->SetActiveCamera(camera);
    this->RenderWindow->GetRenderers()->GetFirstRenderer()->ResetCamera();
    this->RenderWindow->GetRenderers()->GetFirstRenderer()->ResetCameraClippingRange();
    this->RenderWindow->Render();

    camera->Delete();
    camera = nullptr;
}

//-----------------------------------------------------------------------------
void QVtkViewer::initializeGL()
{
    this->Superclass::initializeGL();
    if (this->RenderWindow)
    {
        Q_ASSERT(this->RenderWindowAdapter.data() == nullptr);

        // When a QOpenGLWidget is told to use a QSurfaceFormat with samples > 0,
        // QOpenGLWidget doesn't actually create a context with multi-samples and
        // internally changes the QSurfaceFormat to be samples=0. Thus, we can't
        // rely on the QSurfaceFormat to indicate to us if multisampling is being
        // used. We should use glGetRenderbufferParameteriv(..) to get
        // GL_RENDERBUFFER_SAMPLES to determine the samples used. This is done by
        // in recreateFBO().
        this->RenderWindowAdapter.reset(
            new QVTKRenderWindowAdapter(this->context(), this->RenderWindow, this));
        this->RenderWindowAdapter->setDefaultCursor(this->defaultCursor());
        this->RenderWindowAdapter->setEnableHiDPI(this->EnableHiDPI);
        this->RenderWindowAdapter->setUnscaledDPI(this->UnscaledDPI);
    }
    this->connect(this->context(), SIGNAL(aboutToBeDestroyed()), SLOT(cleanupContext()),
        static_cast<Qt::ConnectionType>(Qt::UniqueConnection | Qt::DirectConnection));
}

//-----------------------------------------------------------------------------
void QVtkViewer::updateSize()
{
    if (this->RenderWindowAdapter)
    {
        this->RenderWindowAdapter->resize(this->width(), this->height());
    }
}

//-----------------------------------------------------------------------------
void QVtkViewer::paintGL()
{
    this->Superclass::paintGL();
    if (this->RenderWindow)
    {
        Q_ASSERT(this->RenderWindowAdapter);
        this->RenderWindowAdapter->paint();

        // If render was triggered by above calls, that may change the current context
        // due to things like progress events triggering updates on other widgets
        // (e.g. progress bar). Hence we need to make sure to call makeCurrent()
        // before proceeding with blit-ing.
        this->makeCurrent();

        QOpenGLFunctions_3_2_Core* f =
            QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_2_Core>();
        if (f)
        {
            const QSize deviceSize = this->size() * this->devicePixelRatioF();
            this->RenderWindowAdapter->blit(
                this->defaultFramebufferObject(), GL_COLOR_ATTACHMENT0, QRect(QPoint(0, 0), deviceSize));
        }
    }
    else
    {
        // no render window set, just fill with white.
        QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
        f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        f->glClear(GL_COLOR_BUFFER_BIT);
    }
}

//-----------------------------------------------------------------------------
void QVtkViewer::cleanupContext()
{
    this->RenderWindowAdapter.reset(nullptr);
}

//-----------------------------------------------------------------------------
bool QVtkViewer::event(QEvent* evt)
{
    if (this->RenderWindowAdapter)
    {
        this->RenderWindowAdapter->handleEvent(evt);
    }
    return this->Superclass::event(evt);
}

//-----------------------------------------------------------------------------
#if !defined(VTK_LEGACY_REMOVE)
void QVtkViewer::SetRenderWindow(vtkRenderWindow* win)
{
    VTK_LEGACY_REPLACED_BODY(
        QVTKOpenGLNativeWidget::SetRenderWindow, "VTK 9.0", QVTKOpenGLNativeWidget::setRenderWindow);
    vtkGenericOpenGLRenderWindow* gwin = vtkGenericOpenGLRenderWindow::SafeDownCast(win);
    if (gwin == nullptr && win != nullptr)
    {
        qDebug() << "QVTKOpenGLNativeWidget requires a `vtkGenericOpenGLRenderWindow`. `"
            << win->GetClassName() << "` is not supported.";
    }
    this->setRenderWindow(gwin);
}
#endif

//-----------------------------------------------------------------------------
#if !defined(VTK_LEGACY_REMOVE)
void QVtkViewer::SetRenderWindow(vtkGenericOpenGLRenderWindow* win)
{
    VTK_LEGACY_REPLACED_BODY(
        QVTKOpenGLNativeWidget::SetRenderWindow, "VTK 9.0", QVTKOpenGLNativeWidget::setRenderWindow);
    this->setRenderWindow(win);
}
#endif

//-----------------------------------------------------------------------------
#if !defined(VTK_LEGACY_REMOVE)
vtkRenderWindow* QVtkViewer::GetRenderWindow()
{
    VTK_LEGACY_REPLACED_BODY(
        QVTKOpenGLNativeWidget::GetRenderWindow, "VTK 9.0", QVTKOpenGLNativeWidget::renderWindow);
    return this->renderWindow();
}
#endif

//-----------------------------------------------------------------------------
#if !defined(VTK_LEGACY_REMOVE)
QVTKInteractorAdapter* QVtkViewer::GetInteractorAdapter()
{
    VTK_LEGACY_BODY(QVTKOpenGLNativeWidget::GetInteractorAdapter, "VTK 9.0");
    return nullptr;
}
#endif

//-----------------------------------------------------------------------------
#if !defined(VTK_LEGACY_REMOVE)
QVTKInteractor* QVtkViewer::GetInteractor()
{
    VTK_LEGACY_REPLACED_BODY(
        QVTKOpenGLNativeWidget::GetInteractor, "VTK 9.0", QVTKOpenGLNativeWidget::interactor);
    return this->interactor();
}
#endif

//-----------------------------------------------------------------------------
#if !defined(VTK_LEGACY_REMOVE)
void QVtkViewer::setQVTKCursor(const QCursor& cursor)
{
    VTK_LEGACY_REPLACED_BODY(
        QVTKOpenGLNativeWidget::setQVTKCursor, "VTK 9.0", QVTKOpenGLNativeWidget::setCursor);
    this->setCursor(cursor);
}
#endif

//-----------------------------------------------------------------------------
#if !defined(VTK_LEGACY_REMOVE)
void QVtkViewer::setDefaultQVTKCursor(const QCursor& cursor)
{
    VTK_LEGACY_REPLACED_BODY(QVTKOpenGLNativeWidget::setDefaultQVTKCursor, "VTK 9.0",
        QVTKOpenGLNativeWidget::setDefaultCursor);
    this->setDefaultCursor(cursor);
}
#endif
