#include <iostream>

#include <QVTKRenderWidget.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkInteractorStyle.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAutoInit.h> 
#include <opencv2/opencv.hpp>
#include <QLayout>
#include <QWidget>
#include <QApplication>
#include <QPushButton>

VTK_MODULE_INIT(vtkRenderingOpenGL2);   // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);

int main(int argc, char *argv[])
{
    QApplication app(argc, argv); // 创建 QApplication 对象

    QWidget* w = new QWidget();
    QVTKRenderWidget* renderWidget = new QVTKRenderWidget();
    QVBoxLayout* v_layout = new QVBoxLayout(w);
    QPushButton* show_btn = new QPushButton(w);

    show_btn->setText("display");

    v_layout->addWidget(show_btn);
    v_layout->addWidget(renderWidget);

    vtkPoints *points = vtkPoints::New();
    vtkCellArray *cells = vtkCellArray::New();

    // test point cloud display
    // connect(show_btn, &QPushButton::released, renderWidget, [=](){
        std::vector<cv::Point3f> pointCloud;
        cv::FileStorage fs1("../../data/ScanData/1_3C_line.yml", cv::FileStorage::READ);
        fs1["scan_line"] >> pointCloud;
        fs1.release();

        int invalid_pnts = 0;
        for (const auto& point : pointCloud){
            if(!isnan(point.x) && !isnan(point.y) && !isnan(point.z)){
                vtkIdType id = points->InsertNextPoint(point.x, point.y, point.z);
                cells->InsertNextCell(1, &id);
            }
            invalid_pnts++;
        }
    // });

    // 渲染机制未知，需要同时设置点坐标与点坐标对应的verts
    // verts中的id必须与点坐标对应
    vtkPolyData *polyData = vtkPolyData::New();
    polyData->SetPoints(points);
    polyData->SetVerts(cells);

    //下面为正常的可视化流程，可设置的点云颜色、大小等已注释
    vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
    mapper->SetInputData(polyData);

    vtkActor *actor = vtkActor::New();
    actor->SetMapper(mapper);

    //设置颜色与点大小
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0);  
    actor->GetProperty()->SetPointSize(5);

    vtkRenderer *renderer = vtkRenderer::New();
    renderer->AddActor(actor);
    renderer->SetBackground(0.1, 0.2, 0.4);

    // vtkRenderWindow *renderWindow = vtkRenderWindow::New();
    // renderWindow->AddRenderer(renderer);
    renderWidget->renderWindow()->AddRenderer(renderer);

    // vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    // iren->SetRenderWindow(renderWidget->renderWindow());

    // vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
    // iren->SetInteractorStyle(style);

    // iren->Initialize();
    // iren->Start();

    w->resize(800, 800);
    w->show(); // 显示 QWidget

    return app.exec();

}