#ifndef VIEW_H
#define VIEW_H

#include <rviz/panel.h>   //plugin基类的头文件
#include <QGraphicsView>
#include "paintwidget.h"

namespace rviz_calibration
{

class View : public QGraphicsView
{
    Q_OBJECT
public:
    explicit View(QWidget *parent = 0);
protected:
    void mousePressEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
Q_SIGNALS:
    void sendP(QPointF p);
    void sendColorIndex(int index);
    void deleteLastPoint();
    void scaleChanged(double scale);
public Q_SLOTS:
    void changeView();
private:
    int color_count = 0;
    double scale_factor = 1.0;
};

}

#endif // VIEW_H
