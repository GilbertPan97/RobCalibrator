#include <iostream>

#include <QVTKRenderWidget.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkProperty2D.h>
#include <vtkCaptionActor2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkInteractorStyle.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAutoInit.h> 
#include <vtkOrientationMarkerWidget.h>
#include <vtkAnnotatedCubeActor.h>
#include <vtkAxesActor.h>
#include <vtkTextProperty.h>
#include <vtkPropAssembly.h>

#include <opencv2/opencv.hpp>

#include <QLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QApplication>
#include <QPushButton>

VTK_MODULE_INIT(vtkRenderingOpenGL2);   // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);

void setAxesSystem(vtkRenderWindowInteractor* iren)
{
    if (!iren)
    {
        return;
    }

    vtkOrientationMarkerWidget* axesWidget = vtkOrientationMarkerWidget::New();

    vtkSmartPointer<vtkAnnotatedCubeActor> cube = vtkSmartPointer<vtkAnnotatedCubeActor>::New();

    cube->SetFaceTextScale(0.65);
    cube->GetCubeProperty()->SetColor(0.9, 0.9, 0.9);
    cube->GetTextEdgesProperty()->SetLineWidth(1);
    cube->GetTextEdgesProperty()->SetDiffuse(0);
    cube->GetTextEdgesProperty()->SetAmbient(1);
    cube->GetTextEdgesProperty()->SetColor(0.2400, 0.2400, 0.2400);
    vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();

    cube->SetXPlusFaceText("L");    //����
    cube->SetXMinusFaceText("R");
    cube->SetYPlusFaceText("P");    //ǰ����
    cube->SetYMinusFaceText("A");
    cube->SetZPlusFaceText("S");    //�µ���
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
    axesWidget->SetOutlineColor(1, 1, 1);
    axesWidget->SetViewport(0.0, 0.0, 0.2, 0.2);
    axesWidget->SetOrientationMarker(assembly);
    axesWidget->SetInteractor(iren);

    axesWidget->On();
    axesWidget->SetInteractive(0);
}

int main(int argc, char *argv[])
{
    // build QApplication object
    QApplication app(argc, argv); 

    // build widget
    QWidget* w = new QWidget();
    QVTKRenderWidget* renderWidget = new QVTKRenderWidget();
    QVBoxLayout* v_layout = new QVBoxLayout(w);
    QPushButton* show_btn = new QPushButton(w);
    QPushButton* clear_btn = new QPushButton(w);

    show_btn->setText("display");
    clear_btn->setText("clear");

    v_layout->addWidget(show_btn);
    v_layout->addWidget(clear_btn);
    v_layout->addWidget(renderWidget);

    vtkPoints *points = vtkPoints::New();
    vtkCellArray *cells = vtkCellArray::New();

    // read point cloud from yml file
    std::vector<cv::Point3f> pointCloud;
    cv::FileStorage fs1("../../data/ScanData/1_3C_line.yml", cv::FileStorage::READ);
    fs1["scan_line"] >> pointCloud;
    fs1.release();

    // build STL file reader
    vtkSmartPointer<vtkSTLReader> stlReader = vtkSmartPointer<vtkSTLReader>::New();
    stlReader->SetFileName("../../data/ScanData/SR7140.STL");
    stlReader->Update();

    int invalid_pnts = 0;
    for (const auto& point : pointCloud){
        if(!isnan(point.x) && !isnan(point.y) && !isnan(point.z)){
            vtkIdType id = points->InsertNextPoint(point.x, point.y, point.z);
            cells->InsertNextCell(1, &id);
        }
        invalid_pnts++;
    }

    // Set the point coordinates and the verts corresponding to the point coordinates
    vtkPolyData *polyData = vtkPolyData::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);

    // add render window
    vtkRenderer *renderer = vtkRenderer::New();
    renderer->SetBackground(0.1, 0.2, 0.4);
    renderWidget->renderWindow()->AddRenderer(renderer);

    // Set mapper and actor, and set imput data to mapper
    vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
    // mapper->SetInputData(polyData);     // point cloud
    mapper->SetInputConnection(stlReader->GetOutputPort());     // stl model

    vtkActor *actor = vtkActor::New();
    actor->SetMapper(mapper);

    // point cloud display
    QObject::connect(show_btn, &QPushButton::released, renderWidget, [=](){
        // Add actor to renderer, and set color and points size
        actor->GetProperty()->SetColor(1.0, 0.0, 0.0);  
        actor->GetProperty()->SetPointSize(5);

        renderer->AddActor(actor);
        renderer->Render();
    });

    // point cloud clear
    QObject::connect(clear_btn, &QPushButton::released, renderWidget, [=](){
        // Remove actor to clear window
        renderer->RemoveActor(actor);
        renderer->Render();
    });

    // test iren
    vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
    if (!renderWidget->renderWindow()->GetInteractor()){
        // create a default interactor
        vtkNew<QVTKInteractor> iren;
        // iren->SetUseTDx(this->UseTDx);
        renderWidget->renderWindow()->SetInteractor(iren);
        iren->SetInteractorStyle(style);
        setAxesSystem(iren);
    }
    else{
        auto iren = renderWidget->renderWindow()->GetInteractor();
        iren->SetInteractorStyle(style);
        setAxesSystem(iren);
    }

    w->resize(800, 800);
    w->show();

    return app.exec();

}