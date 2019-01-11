#include "view.h"

namespace rviz_calibration
{
View::View(QWidget *parent) :
    QGraphicsView(parent)
{

}

void View::changeView()
{
    scale(1/scale_factor,1/scale_factor);
    scale_factor = 1;
}

void View::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Plus:
        scale(1.2,1.2);
        scale_factor *= 1.2;
        Q_EMIT scaleChanged(scale_factor);
        Q_EMIT repaintPoints();
        break;
    case Qt::Key_Minus:
        if(scale_factor > 1){
            scale(1.0/1.2,1.0/1.2);
            scale_factor /= 1.2;
            Q_EMIT scaleChanged(scale_factor);
            Q_EMIT repaintPoints();
        }
        break;
    case Qt::Key_R :
        rotate(30);
        break;
    case Qt::Key_L:
        rotate(-30);
        break;
    }
    QGraphicsView::keyPressEvent(event);
}

void View::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        color_count++;
        Q_EMIT sendColorIndex(color_count);

        QGraphicsView::mousePressEvent(event);

        QPoint viewPos = event->pos();
        QPointF scenePos = mapToScene(viewPos);

        Q_EMIT sendP(scenePos);
    }
    else
    {
        Q_EMIT deleteLastPoint();
        Q_EMIT updateListView();
    }
}

}
