#ifndef PAINTWIDGET_H
#define PAINTWIDGET_H

#include <QDebug>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QKeyEvent>

#include "shape.h"
#include "line.h"
#include "rect.h"

namespace rviz_calibration
{

class PaintWidget : public QGraphicsScene
{
        Q_OBJECT

public:
        PaintWidget(QObject *parent = 0);

Q_SIGNALS:
        //void sendScale(double scale);
public Q_SLOTS:
        void setCurrentShape(Shape::Code s)
        {
                if(s != currShapeCode) {
                        currShapeCode = s;
                }
        }

        void resetItem();
        void removeLastItem();
        void getScaleFactor(double scale);
        void repaint();

protected:
        void mousePressEvent(QGraphicsSceneMouseEvent *event);
        void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
        void keyPressEvent(QKeyEvent *event);

        void reshowItem(double scale);

private:
        Shape::Code currShapeCode;
        Shape *currItem;
        bool perm;
        bool shape_select;
        int count = 0;
        double scale_factor = 1.0;
};

}

#endif // PAINTWIDGET_H