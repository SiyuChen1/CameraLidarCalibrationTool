#ifndef VIEW_H
#define VIEW_H

#include <QGraphicsView>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QGraphicsItem>
#include <QTime>

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
    void repaintPoints();
    void updateListView();
public Q_SLOTS:
    void changeView();
private:
    int color_count = 0;
    double scale_factor = 1.0;
};

}

#endif // VIEW_H
