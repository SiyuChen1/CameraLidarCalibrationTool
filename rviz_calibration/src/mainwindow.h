#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QTextBrowser>
#include <QPushButton>
#include <QPointer>
#include <QActionGroup>
#include <QAction>
#include <QToolBar>
#include <QLabel>
#include <QStatusBar>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QListView>
#include <QStringListModel>
#include <QStandardItemModel>
#include <QModelIndex>
#include <QStandardItem>

#include "paintwidget.h"
#include "view.h"

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
    // QPointer<QToolBar> bar;
    // QPointer<QActionGroup> group;
    // QPointer<QWidget> widget;
    // //QPointer<QLabel> show;
    // QLabel *show;
    // QPointer<QHBoxLayout> ly;
    // QPointer<QVBoxLayout> layout2;
    // QPointer<QAction> drawLineAction;
    // QPointer<QAction> drawRectAction;
    // //QPointer<QLabel> statusMsg;
    // QLabel *statusMsg;

    // //QPointer<QLabel> testPointShow;
    QToolBar *bar;
    // //QWidget *widget;
    // QLabel *show;
    // QPushButton *push;
    // // QHBoxLayout *ly;
    // // QVBoxLayout *layout2;
    QActionGroup *group;
    QLabel* testPointShow;
    QTextBrowser *tb;
    QListView *points_listview;
    //QStringListModel *points_listview_model;
    QStandardItemModel *points_listview_itemModel;
    
    // QPointer<QPushButton> push;

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