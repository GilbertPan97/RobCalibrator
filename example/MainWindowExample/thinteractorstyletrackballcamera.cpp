#include "thInteractorStyleTrackballCamera.h"
#include "vtkCallbackCommand.h"
#include "vtkCamera.h"
#include "vtkMath.h"
#include "vtkMatrix3x3.h"
#include "vtkObjectFactory.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"

//----------------------------------------------------------------------------
thInteractorStyleTrackballCamera::thInteractorStyleTrackballCamera() {
    this->MotionFactor = 10.0;
}

//----------------------------------------------------------------------------
thInteractorStyleTrackballCamera::~thInteractorStyleTrackballCamera() = default;

//----------------------------------------------------------------------------
thInteractorStyleTrackballCamera* thInteractorStyleTrackballCamera::New() {
    return new thInteractorStyleTrackballCamera;
}

void thInteractorStyleTrackballCamera::OnMouseMove() {
    int x = this->Interactor->GetEventPosition()[0];
    int y = this->Interactor->GetEventPosition()[1];

    switch (this->State) {
        case VTKIS_ENV_ROTATE:
            this->FindPokedRenderer(x, y);
            this->EnvironmentRotate();
            this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
            break;

        case VTKIS_ROTATE:
            this->FindPokedRenderer(x, y);
            this->Rotate();
            this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
            break;

        case VTKIS_PAN:
            this->FindPokedRenderer(x, y);
            this->Pan();
            this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
            break;

        case VTKIS_DOLLY:
            this->FindPokedRenderer(x, y);
            this->Dolly();
            this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
            break;

        case VTKIS_SPIN:
            this->FindPokedRenderer(x, y);
            this->Spin();
            this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
            break;
    }
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnLeftButtonDown() {
    this->FindPokedRenderer(
        this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]);
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    this->GrabFocus(this->EventCallbackCommand);
    if (this->Interactor->GetControlKey()) {
        this->StartPan();
    } else {
        this->StartRotate();
    }
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnLeftButtonUp() {

    switch (this->State) {
        case VTKIS_PAN:
            this->EndPan();
            break;

        case VTKIS_ROTATE:
            this->EndRotate();
            break;
    }

    if (this->Interactor) {
        this->ReleaseFocus();
    }
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnMiddleButtonDown() {
    this->FindPokedRenderer(
        this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]);
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    this->GrabFocus(this->EventCallbackCommand);
    this->StartPan();
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnMiddleButtonUp() {
    switch (this->State) {
        case VTKIS_PAN:
            this->EndPan();
            if (this->Interactor) {
                this->ReleaseFocus();
            }
            break;
    }
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnRightButtonDown() {
    this->FindPokedRenderer(
        this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]);
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    this->GrabFocus(this->EventCallbackCommand);

    if (this->Interactor->GetControlKey()) {
        this->StartGesture();
    } else {
        this->StartSpin();
    }


}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnRightButtonUp() {

    switch (this->State) {
        case VTKIS_GESTURE:
            this->EndGesture();
            break;

        case VTKIS_SPIN:
            this->EndSpin();
            break;
    }

    if (this->Interactor) {
        this->ReleaseFocus();
    }
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnMouseWheelForward() {
    this->FindPokedRenderer(
        this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]);
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    this->GrabFocus(this->EventCallbackCommand);
    this->StartDolly();
    double factor = this->MotionFactor * 0.2 * this->MouseWheelMotionFactor;
    this->Dolly(pow(1.1, factor));
    this->EndDolly();
    this->ReleaseFocus();
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::OnMouseWheelBackward() {
    this->FindPokedRenderer(
        this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]);
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    this->GrabFocus(this->EventCallbackCommand);
    this->StartDolly();
    double factor = this->MotionFactor * -0.2 * this->MouseWheelMotionFactor;
    this->Dolly(pow(1.1, factor));
    this->EndDolly();
    this->ReleaseFocus();
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::Rotate() {
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    vtkRenderWindowInteractor *rwi = this->Interactor;

    int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
    int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

    const int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

    double delta_elevation = -20.0 / size[1];
    double delta_azimuth = -20.0 / size[0];

    double rxf = dx * delta_azimuth * this->MotionFactor;
    double ryf = dy * delta_elevation * this->MotionFactor;

    vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
    camera->Azimuth(rxf);
    camera->Elevation(ryf);
    camera->OrthogonalizeViewUp();

    if (this->AutoAdjustCameraClippingRange) {
        this->CurrentRenderer->ResetCameraClippingRange();
    }

    if (rwi->GetLightFollowCamera()) {
        this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
    }

    rwi->Render();
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::Spin() {
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    vtkRenderWindowInteractor *rwi = this->Interactor;

    double *center = this->CurrentRenderer->GetCenter();

    double newAngle = vtkMath::DegreesFromRadians(
                          atan2(rwi->GetEventPosition()[1] - center[1], rwi->GetEventPosition()[0] - center[0]));

    double oldAngle = vtkMath::DegreesFromRadians(
                          atan2(rwi->GetLastEventPosition()[1] - center[1], rwi->GetLastEventPosition()[0] - center[0]));

    vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
    camera->Roll(newAngle - oldAngle);
    camera->OrthogonalizeViewUp();

    rwi->Render();
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::Pan() {
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    vtkRenderWindowInteractor *rwi = this->Interactor;

    double viewFocus[4], focalDepth, viewPoint[3];
    double newPickPoint[4], oldPickPoint[4], motionVector[3];

    // Calculate the focal depth since we'll be using it a lot

    vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
    camera->GetFocalPoint(viewFocus);
    this->ComputeWorldToDisplay(viewFocus[0], viewFocus[1], viewFocus[2], viewFocus);
    focalDepth = viewFocus[2];

    this->ComputeDisplayToWorld(
        rwi->GetEventPosition()[0], rwi->GetEventPosition()[1], focalDepth, newPickPoint);

    // Has to recalc old mouse point since the viewport has moved,
    // so can't move it outside the loop

    this->ComputeDisplayToWorld(
        rwi->GetLastEventPosition()[0], rwi->GetLastEventPosition()[1], focalDepth, oldPickPoint);

    // Camera motion is reversed

    motionVector[0] = oldPickPoint[0] - newPickPoint[0];
    motionVector[1] = oldPickPoint[1] - newPickPoint[1];
    motionVector[2] = oldPickPoint[2] - newPickPoint[2];

    camera->GetFocalPoint(viewFocus);
    camera->GetPosition(viewPoint);
    camera->SetFocalPoint(
        motionVector[0] + viewFocus[0], motionVector[1] + viewFocus[1], motionVector[2] + viewFocus[2]);

    camera->SetPosition(
        motionVector[0] + viewPoint[0], motionVector[1] + viewPoint[1], motionVector[2] + viewPoint[2]);

    if (rwi->GetLightFollowCamera()) {
        this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
    }

    rwi->Render();
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::Dolly() {
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    vtkRenderWindowInteractor *rwi = this->Interactor;
    double *center = this->CurrentRenderer->GetCenter();
    int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];
    double dyf = this->MotionFactor * dy / center[1];
    this->Dolly(pow(1.1, dyf));
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::Dolly(double factor) {
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
    if (camera->GetParallelProjection()) {
        camera->SetParallelScale(camera->GetParallelScale() / factor);
    } else {
        camera->Dolly(factor);
        if (this->AutoAdjustCameraClippingRange) {
            this->CurrentRenderer->ResetCameraClippingRange();
        }
    }

    if (this->Interactor->GetLightFollowCamera()) {
        this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
    }

    this->Interactor->Render();
}

//----------------------------------------------------------------------------
void thInteractorStyleTrackballCamera::EnvironmentRotate() {
    if (this->CurrentRenderer == nullptr) {
        return;
    }

    vtkRenderWindowInteractor *rwi = this->Interactor;

    int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
    int sizeX = this->CurrentRenderer->GetRenderWindow()->GetSize()[0];

    vtkNew<vtkMatrix3x3> mat;

    double *up = this->CurrentRenderer->GetEnvironmentUp();
    double *right = this->CurrentRenderer->GetEnvironmentRight();

    double front[3];
    vtkMath::Cross(right, up, front);
    for (int i = 0; i < 3; i++) {
        mat->SetElement(i, 0, right[i]);
        mat->SetElement(i, 1, up[i]);
        mat->SetElement(i, 2, front[i]);
    }

    double angle = (dx / static_cast<double>(sizeX)) * this->MotionFactor;

    double c = std::cos(angle);
    double s = std::sin(angle);
    double t = 1.0 - c;

    vtkNew<vtkMatrix3x3> rot;

    rot->SetElement(0, 0, t * up[0] * up[0] + c);
    rot->SetElement(0, 1, t * up[0] * up[1] - up[2] * s);
    rot->SetElement(0, 2, t * up[0] * up[2] + up[1] * s);

    rot->SetElement(1, 0, t * up[0] * up[1] + up[2] * s);
    rot->SetElement(1, 1, t * up[1] * up[1] + c);
    rot->SetElement(1, 2, t * up[1] * up[2] - up[0] * s);

    rot->SetElement(2, 0, t * up[0] * up[2] - up[1] * s);
    rot->SetElement(2, 1, t * up[1] * up[2] + up[0] * s);
    rot->SetElement(2, 2, t * up[2] * up[2] + c);

    vtkMatrix3x3::Multiply3x3(rot, mat, mat);

    // update environment orientation
    this->CurrentRenderer->SetEnvironmentUp(
        mat->GetElement(0, 1), mat->GetElement(1, 1), mat->GetElement(2, 1));
    this->CurrentRenderer->SetEnvironmentRight(
        mat->GetElement(0, 0), mat->GetElement(1, 0), mat->GetElement(2, 0));

    rwi->Render();
}
