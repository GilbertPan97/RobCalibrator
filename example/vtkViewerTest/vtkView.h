#include <QVTKRenderWidget.h>

class QVtkViewer : QVTKRenderWidget
{
    Q_OBJECT

public:
    QVtkViewer(QWidget* parent = nullptr);
    
    ~QVtkViewer() override;
};