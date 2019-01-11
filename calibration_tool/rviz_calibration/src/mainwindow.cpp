#include "mainwindow.h"

namespace rviz_calibration
{
MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent)
{
        paintWidget = new PaintWidget(this);
        view = new View(this);

        bar = this->addToolBar("Tools");
        group = new QActionGroup(bar);

        testPointShow = new QLabel(this);
        QWidget *widget = new QWidget(this);
        QLabel *show = new QLabel(this);
        QPushButton *push = new QPushButton("test",this);
        QHBoxLayout *ly = new QHBoxLayout;
        QVBoxLayout *layout2 = new QVBoxLayout;
        tb = new QTextBrowser(this);
        points_listview = new QListView(this);
        points_listview_itemModel = new QStandardItemModel(this);

        ly->addWidget(show,1);
        ly->addWidget(tb,3);
        ly->addWidget(points_listview,3);
        ly->addStretch(9);
        ly->addWidget(testPointShow);
        ly->addWidget(push,1);
        layout2->addWidget(view,6);
        layout2->addLayout(ly,1);
        widget->setLayout(layout2);

        QAction *drawLineAction = new QAction("Line", bar);
        drawLineAction->setIcon(QIcon(":/line.png"));
        drawLineAction->setToolTip(tr("Draw a line."));
        drawLineAction->setStatusTip(tr("Draw a line."));
        drawLineAction->setCheckable(true);
        drawLineAction->setChecked(true);
        group->addAction(drawLineAction);
        bar->addAction(drawLineAction);

        QAction *drawRectAction = new QAction("Rectangle", bar);
        drawRectAction->setIcon(QIcon(":/rect.png"));
        drawRectAction->setToolTip(tr("Draw a rectangle."));
        drawRectAction->setStatusTip(tr("Draw a rectangle."));
        drawRectAction->setCheckable(true);
        group->addAction(drawRectAction);
        bar->addAction(drawRectAction);

        QLabel *statusMsg = new QLabel(this);
        statusBar()->addWidget(statusMsg);

        setCentralWidget(widget);

        connect(drawLineAction, SIGNAL(triggered()), this, SLOT(drawLineActionTriggered()));
        connect(drawRectAction, SIGNAL(triggered()), this, SLOT(drawRectActionTriggered()));
        connect(this, SIGNAL(changeCurrentShape(Shape::Code)),paintWidget, SLOT(setCurrentShape(Shape::Code)));
        connect(view,SIGNAL(sendP(QPointF)),this,SLOT(showPoints(QPointF)));
        connect(push,SIGNAL(clicked(bool)),this,SLOT(writeFile()));
        connect(view,SIGNAL(sendColorIndex(int)),this,SLOT(showColorName(int)));
        connect(this,SIGNAL(signal2ChangePaintWidget()),paintWidget,SLOT(resetItem()));
        connect(view,SIGNAL(deleteLastPoint()),paintWidget,SLOT(removeLastItem()));
        connect(view,SIGNAL(scaleChanged(double)),paintWidget,SLOT(getScaleFactor(double)));
        connect(view,SIGNAL(repaintPoints()),paintWidget,SLOT(repaint()));
        connect(view,SIGNAL(updateListView()),this,SLOT(updateText()));
        connect(this,SIGNAL(changeViewScale()),view,SLOT(changeView()));
}

MainWindow::~MainWindow()
{
    if(paintWidget != NULL)
    {
        delete paintWidget;
        paintWidget = NULL;
    }
    if(view != NULL)
    {
        delete view;
        view = NULL;
    }
    if(group != NULL)
    {
        delete group;
        group = NULL;
    }
    if(bar != NULL)
    {
        delete bar;
        bar = NULL;
    } 
    if(testPointShow != NULL)
    {
        delete testPointShow;
        testPointShow = NULL;
    }
    if(tb != NULL)
    {
        delete tb;
        tb = NULL;
    }
}

void MainWindow::updateText(){
    
}

void MainWindow::resetViewScale()
{
    Q_EMIT changeViewScale();
}

void MainWindow::showColorName(int count)
{
    //qDebug()<<"test";
    QColor line_color;
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
    tb->setTextColor(line_color);
}

void MainWindow::drawLineActionTriggered()
{
    Q_EMIT changeCurrentShape(Shape::Line);
}

void MainWindow::drawRectActionTriggered()
{
    Q_EMIT changeCurrentShape(Shape::Rect);
}

void MainWindow::setBackGroundPic(QPixmap image)
{
    backgroundPic = image;
        
    QGraphicsScene *sc = dynamic_cast<QGraphicsScene*>(paintWidget);//new QGraphicsScene;

    sc->setBackgroundBrush(backgroundPic);

    sc->setSceneRect(0, 0, image.width(), image.height());
    //qDebug()<<image.width()<<","<<image.height();
    view->setScene(sc);
}

void MainWindow::showPoints(QPointF p)
{
    testPointShow->setText(QString::number(p.x()) + "," + QString::number(p.y()));
    s += QString::number(p.x()) + "," + QString::number(p.y()) + "\n";
    tb->insertPlainText(QString::number(p.x()) + "," + QString::number(p.y()) + "\n");
    
    QStringList strList;
    strList.append(QString::number(p.x()) + "," + QString::number(p.y()));
    int nCount = strList.size();
       for(int i = 0; i < nCount; i++)
       {
           QString string = static_cast<QString>(strList.at(i));
           QStandardItem *item = new QStandardItem(string);
           points_listview_itemModel->appendRow(item);
       }
      points_listview->setModel(points_listview_itemModel);
}

void MainWindow::writeFile()
{
    QString write_path = QString::fromStdString(DEFAULT_SOURCE_PATH) + "/data/points.txt";
    QFile file(write_path);
    if(!file.open(QIODevice::ReadWrite | QIODevice::Append))
    {
        qDebug()<<file.errorString();
    }
    QTextStream str(&file);
    str<<s<<endl;
    file.write("this is a test for myself \n");
    file.close();
}

void MainWindow::changePaintWidget()
{
        Q_EMIT signal2ChangePaintWidget();
}

}
