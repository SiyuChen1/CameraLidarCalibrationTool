#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "paintwidget.h"
#include "view.h"

#include <QMainWindow>
#include <QLabel>
#include <QTextBrowser>
#include <QPushButton>
#include <QPointer>
#include <QActionGroup>
#include <QToolBar>
#include <QStatusBar>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QListView>
#include <QStringListModel>
#include <QStandardItemModel>
#include <QModelIndex>
#include <QStandardItem>
#include <QAction>
#include <QGraphicsView>
#include <QFile>
#include <QTextStream>



namespace rviz_calibration
{
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
public:
    QPixmap backgroundPic;
    PaintWidget *paintWidget;
    View *view;
    QToolBar *bar;
    QActionGroup *group;
    QLabel* testPointShow;
    QTextBrowser *tb;
    QListView *points_listview;
    QStandardItemModel *points_listview_itemModel;

    QString s;
    QString color_name = "";

Q_SIGNALS:
    void changeCurrentShape(Shape::Code s);
    void signal2ChangePaintWidget();
    void changeViewScale();
public Q_SLOTS:
    void drawLineActionTriggered();
    void drawRectActionTriggered();
    void setBackGroundPic(QPixmap image);
    void changePaintWidget();

    void resetViewScale();

    void showPoints(QPointF p);
    void showColorName(int count);
    void writeFile();
};

}


#endif // MAINWINDOW_H