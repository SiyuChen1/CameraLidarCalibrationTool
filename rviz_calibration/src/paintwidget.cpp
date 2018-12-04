#include "paintwidget.h"

#include <QGraphicsItem>

namespace rviz_calibration
{
PaintWidget::PaintWidget(QObject *parent)
        : QGraphicsScene(parent), currShapeCode(Shape::Line), currItem(NULL), perm(false),shape_select(false)
{
        
}

void PaintWidget::getScaleFactor(double scale)
{
    scale_factor = scale;
}

void PaintWidget::reshowItem(double scale)
{
    QList<QGraphicsItem*> li = this->items();
    QList<Line*> li_temp;
    QGraphicsItem *it;
    Line *s_line = new Line;
    for(int i = 0;i<li.size();i++)
    {
        it = li.at(i);
        s_line = dynamic_cast<Line*>(it);
        li_temp.append(s_line);
        removeItem(it);
    }
    if(it != NULL)
    {
        it = NULL;
    }
}

void PaintWidget::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    switch(currShapeCode)
    {
        case Shape::Line:
                {
                        Line *line = new Line;
                        count ++;
                        qDebug()<<scale_factor;
                        line->setLineScale(scale_factor);
                        line->setLineColor(count);
                        currItem = line;
                        addItem(line);
                        break;
                }
        case Shape::Rect:
                {
                        Rect *rect = new Rect;
                        currItem = rect;
                        addItem(rect);
                        shape_select = true;
                        break;
                }
        }
        if(currItem) {
                currItem->startDraw(event);
                perm = false;
     }
     QGraphicsScene::mousePressEvent(event);
}

void PaintWidget::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
        if(shape_select && !perm) {
                currItem->drawing(event);
        }
        QGraphicsScene::mouseMoveEvent(event);
}

void PaintWidget::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
        perm = true;
        QGraphicsScene::mouseReleaseEvent(event);
}

void PaintWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Plus:
        break;
    case Qt::Key_Minus:
        //scale(1/1.2,1/1.2);
        break;
    case Qt::Key_Right:
        //rotate(30);
        break;
    }
    QGraphicsScene::keyPressEvent(event);
}

void PaintWidget::resetItem()
{
    QList<QGraphicsItem*> li = this->items();
    QGraphicsItem *it;
    Line *s_line = new Line;
    qDebug()<<li.size();
    for(int i = 0;i<li.size();i++)
    {
        it = li.at(i);
        s_line = dynamic_cast<Line*>(it);
        removeItem(it);
    }
    if(s_line != NULL)
    {
        delete s_line;
        s_line = NULL;
    }
    if(it != NULL)
    {
        it = NULL;
    }
}

void PaintWidget::removeLastItem()
{
    QList<QGraphicsItem*> li = this->items();
    if(li.size() != 0)
    {
        QGraphicsItem *it = li.at(0);
        removeItem(it);
        if (it != NULL)
        {
            delete it;
            it = NULL;
        }
    }
}

}

