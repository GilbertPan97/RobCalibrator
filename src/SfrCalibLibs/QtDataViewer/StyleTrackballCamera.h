#ifndef STYLETRACKBALLCAMERA_H
#define STYLETRACKBALLCAMERA_H

#include <vtkInteractionStyleModule.h> // For export macro
#include <vtkInteractorStyle.h>

class StyleTrackballCamera: public vtkInteractorStyle {
public:
    static StyleTrackballCamera*New();
    vtkTypeMacro(StyleTrackballCamera, vtkInteractorStyle);

    //@{
    /**
     * Event bindings controlling the effects of pressing mouse buttons
     * or moving the mouse.
     */
    void OnMouseMove() override;
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
    void OnMiddleButtonDown() override;
    void OnMiddleButtonUp() override;
    void OnRightButtonDown() override;
    void OnRightButtonUp() override;
    void OnMouseWheelForward() override;
    void OnMouseWheelBackward() override;
    //@}

    // These methods for the different interactions in different modes
    // are overridden in subclasses to perform the correct motion. Since
    // they are called by OnTimer, they do not have mouse coord parameters
    // (use interactor's GetEventPosition and GetLastEventPosition)
    void Rotate() override;
    void Spin() override;
    void Pan() override;
    void Dolly() override;
    void EnvironmentRotate() override;

    //@{
    /**
     * Set the apparent sensitivity of the interactor style to mouse motion.
     */
    vtkSetMacro(MotionFactor, double);
    vtkGetMacro(MotionFactor, double);
    //@}

protected:
    StyleTrackballCamera();
    ~StyleTrackballCamera() override;

    double MotionFactor;

    virtual void Dolly(double factor);

private:
    StyleTrackballCamera(const StyleTrackballCamera&) = delete;
    void operator=(const StyleTrackballCamera&) = delete;
};

#endif // THINTERACTORSTYLETRACKBALLCAMERA_H
