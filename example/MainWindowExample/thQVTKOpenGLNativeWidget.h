
/**
 * @class thQVTKOpenGLNativeWidget
 * @brief QOpenGLWidget subclass to house a vtkGenericOpenGLRenderWindow in a Qt
 * application.
 *
 * thQVTKOpenGLNativeWidget extends QOpenGLWidget to make it work with a
 * vtkGenericOpenGLRenderWindow. it defined a 3D interactive widget.
 *
 */
#ifndef THQVTKOpenGLNativeWidget_h
#define THQVTKOpenGLNativeWidget_h

#include <iostream>

#include <QOpenGLWidget>
#include <QScopedPointer> // for QScopedPointer.

#include <QVTKInteractor.h>        // needed for QVTKInteractor
#include <vtkGUISupportQtModule.h> // for export macro
#include <vtkNew.h>                // needed for vtkNew
#include <vtkSmartPointer.h>       // needed for vtkSmartPointer
#include <vtkOrientationMarkerWidget.h>
#include <vtkNamedColors.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkActor.h>

#include <opencv2/opencv.hpp>

class QVTKInteractor;
class QVTKInteractorAdapter;
class QVTKRenderWindowAdapter;
class vtkGenericOpenGLRenderWindow;

enum class zxViewerType {
    LEFT = 1,
    RIGHT,
    TOP,
    BOTTOM,
    FRONT,
    BACK,
    ISO,
};

class thQVTKOpenGLNativeWidget : public QOpenGLWidget
{
    Q_OBJECT
        typedef QOpenGLWidget Superclass;

public:
    thQVTKOpenGLNativeWidget(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    thQVTKOpenGLNativeWidget(vtkGenericOpenGLRenderWindow* window, QWidget* parent = nullptr,
        Qt::WindowFlags f = Qt::WindowFlags());
    ~thQVTKOpenGLNativeWidget() override;

    bool displayPoints(std::vector<cv::Point3f> pntCloud);

    //@{
    /**
     * Set a render window to use. It a render window was already set, it will be
     * finalized and all of its OpenGL resource released. If the \c win is
     * non-null and it has no interactor set, then a QVTKInteractor instance will
     * be created as set on the render window as the interactor.
     */
    void setRenderWindow(vtkGenericOpenGLRenderWindow* win);
    void setRenderWindow(vtkRenderWindow* win);
    //@}

    /**
     * Returns the render window that is being shown in this widget.
     */
    vtkRenderWindow* renderWindow() const;

    /**
     * Get the QVTKInteractor that was either created by default or set by the user.
     */
    QVTKInteractor* interactor() const;

    /**
     * @copydoc QVTKRenderWindowAdapter::defaultFormat(bool)
     */
    static QSurfaceFormat defaultFormat(bool stereo_capable = false);

    //@{
    /**
     * Enable or disable support for HiDPI displays. When enabled, this enabled
     * DPI scaling i.e. `vtkWindow::SetDPI` will be called with a DPI value scaled
     * by the device pixel ratio every time the widget is resized. The unscaled
     * DPI value can be specified by using `setUnscaledDPI`.
     */
    void setEnableHiDPI(bool enable);
    bool enableHiDPI() const { return this->EnableHiDPI; }
    //@}

    //@{
    /**
     * Set/Get unscaled DPI value. Defaults to 72, which is also the default value
     * in vtkWindow.
     */
    void setUnscaledDPI(int);
    int unscaledDPI() const { return this->UnscaledDPI; }
    //@}

    //@{
    /**
     * Set/get the default cursor to use for this widget.
     */
    void setDefaultCursor(const QCursor& cursor);
    const QCursor& defaultCursor() const { return this->DefaultCursor; }
    //@}

    //@{
    /**
     * Set the axes system to use for this widget. 
     * by-tianhh-2022.9.27
     */
    void setAxesSystem(vtkRenderWindowInteractor* iren);
    //@}

    //@{
    /**
     * Set the reference axes system to use for this widget.
     * by-tianhh-2022.9.27
     */
    void setReferenceAxesSystem();
    //@}

    //@{
    /**
     * Set viewer type.
     * by-tianhh-2022.9.27
     */
    void setViewerType(zxViewerType type);
    //@}


    //@{
    /**
     * @deprecated in VTK 9.0
     */
    VTK_LEGACY(void SetRenderWindow(vtkGenericOpenGLRenderWindow* win));
    VTK_LEGACY(void SetRenderWindow(vtkRenderWindow* win));
    //@}

    //@{
    /**
     * These methods have be deprecated to fix naming style. Since
     * QVTKOpenGLNativeWidget is QObject subclass, we follow Qt naming conventions
     * rather than VTK's.
     */
    VTK_LEGACY(vtkRenderWindow* GetRenderWindow());
    VTK_LEGACY(QVTKInteractor* GetInteractor());
    //@}

    /**
     * @deprecated in VTK 9.0
     * QVTKInteractorAdapter is an internal helper. Hence the API was removed.
     */
    VTK_LEGACY(QVTKInteractorAdapter* GetInteractorAdapter());

    /**
     * @deprecated in VTK 9.0. Simply use `QWidget::setCursor` API to change
     * cursor.
     */
    VTK_LEGACY(void setQVTKCursor(const QCursor& cursor));

    /**
     * @deprecated in VTK 9.0. Use `setDefaultCursor` instead.
     */
    VTK_LEGACY(void setDefaultQVTKCursor(const QCursor& cursor));

protected slots:
    /**
     * Called as a response to `QOpenGLContext::aboutToBeDestroyed`. This may be
     * called anytime during the widget lifecycle. We need to release any OpenGL
     * resources allocated in VTK work in this method.
     */
    virtual void cleanupContext();

    void updateSize();

protected:
    bool event(QEvent* evt) override;
    void initializeGL() override;
    void paintGL() override;

    std::vector<vtkSmartPointer<vtkActor>> MakePlanesActors(vtkNamedColors* colors);
    vtkSmartPointer<vtkTransformPolyDataFilter> MakePlane(
        std::array<int, 2>& resolution, std::array<double, 3>& origin,
        std::array<double, 3>& point1, std::array<double, 3>& point2,
        std::array<double, 4>& wxyz, std::array<double, 3>& translate);

protected:
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> RenderWindow;
    QScopedPointer<QVTKRenderWindowAdapter> RenderWindowAdapter;

private:
    Q_DISABLE_COPY(thQVTKOpenGLNativeWidget);

    bool EnableHiDPI;
    int UnscaledDPI;
    QCursor DefaultCursor;
    
    vtkOrientationMarkerWidget* m_axesWidget;
    zxViewerType m_viewType;
};

#endif
