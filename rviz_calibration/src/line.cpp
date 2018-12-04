#include "line.h"

namespace rviz_calibration
{
    
Line::Line()
{

}

void Line::setLineScale(double scale)
{
    this->scale = scale;
}
// void Line::resetLine(QPointF p,double scale)
// {
//     QBrush brush(line_color,Qt::SolidPattern);
//     QPen pen(brush,10/scale,Qt::SolidLine,Qt::SquareCap,Qt::BevelJoin);
//     setPen(pen);
//     qDebug()<<2/scale<<"2/scale";
//     setLine(QLineF(p,p));
//     start = p;
// }

void Line::startDraw(QGraphicsSceneMouseEvent * event)
{
    setLine(QLineF(event->scenePos(), event->scenePos()));
    start = event->scenePos();
}

void Line::drawing(QGraphicsSceneMouseEvent * event)
{
    QLineF newLine(line().p1(), event->scenePos());
    setLine(newLine);
    end = event->scenePos();
}

void Line::setLineColor(int count)
{
    count = count % 15 ;
    switch(count)
    {
        case 0:
        {
                line_color = Qt::darkGray;
                break;
        }   
        case 1:
        {
                line_color = Qt::white;
                break;
        }
        case 2:
        {
                line_color = Qt::black;
                break;
        }
        case 3:
        {
                line_color = Qt::red;
                break;
        }
        case 4:
        {
                line_color = Qt::darkRed;
                break;
        }
        case 5:
        {
                line_color = Qt::green;
                break;
        }
        case 6:
        {
                line_color = Qt::darkGreen;
                break;
        }
        case 7:
        {
                line_color = Qt::blue;
                break;
        }
        case 8:
        {
                line_color = Qt::darkBlue;
                break;
        }
        case 9:
        {
                line_color = Qt::cyan;
                break;
        }
        case 10:
        {
                line_color = Qt::darkCyan;
                break;
        }
        case 11:
        {
                line_color = Qt::magenta;
                break;
        }
        case 12:
        {
                line_color = Qt::darkMagenta;
                break;
        }
        case 13:
        {
                line_color = Qt::yellow;
                break;
        }
        case 14:
        {
                line_color = Qt::darkYellow;
                break;
        }                
    } 
    qDebug()<<"line";
    QBrush brush(line_color,Qt::SolidPattern);
    QPen pen(brush,1/this->scale,Qt::SolidLine,Qt::SquareCap,Qt::BevelJoin);
    setPen(pen);
}

}

