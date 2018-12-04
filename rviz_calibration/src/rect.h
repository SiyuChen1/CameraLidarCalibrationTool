#ifndef RECT_H
#define RECT_H

#include <QGraphicsRectItem>
#include "shape.h"

namespace rviz_calibration
{

class Rect : public Shape, public QGraphicsRectItem
{
public:
        Rect();

        void startDraw(QGraphicsSceneMouseEvent * event);
        void drawing(QGraphicsSceneMouseEvent * event);
};

}


#endif // RECT_H