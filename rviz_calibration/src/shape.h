#ifndef SHAPE_H
#define SHAPE_H

#include <QGraphicsSceneMouseEvent>

namespace rviz_calibration
{
class Shape
{
public:
    Shape();
    enum Code {Line,Rect};
    virtual void startDraw(QGraphicsSceneMouseEvent * event) = 0;
    virtual void drawing(QGraphicsSceneMouseEvent * event) = 0;
};
}

#endif // SHAPE_H