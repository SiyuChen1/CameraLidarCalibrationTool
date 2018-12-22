#ifndef LINE_H
#define LINE_H

#include <QGraphicsLineItem>
#include <QPen>
#include <QBrush>
#include <QDebug>

#include "shape.h"

namespace rviz_calibration
{

class Line: public Shape, public QGraphicsLineItem
{
public:
    QPointF start;
    QPointF end;
public:
    Line();
    void startDraw(QGraphicsSceneMouseEvent * event);
    void drawing(QGraphicsSceneMouseEvent * event);
    void setLineColor(int count);
    void setLineScale(double scale);
    void reDraw(QPointF p);

private:
    QColor line_color = Qt::yellow;
    double scale = 1.0;
};

}


#endif // LINE_H

