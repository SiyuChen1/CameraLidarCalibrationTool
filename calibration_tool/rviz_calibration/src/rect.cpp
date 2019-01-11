#include "rect.h"

namespace rviz_calibration
{
Rect::Rect()
{
}

void Rect::startDraw(QGraphicsSceneMouseEvent * event)
{
        setRect(QRectF(event->scenePos(), QSizeF(0, 0)));
}

void Rect::drawing(QGraphicsSceneMouseEvent * event)
{
        QRectF r(rect().topLeft(),
                         QSizeF(event->scenePos().x() - rect().topLeft().x(), event->scenePos().y() - rect().topLeft().y()));
        setRect(r);
}
}


