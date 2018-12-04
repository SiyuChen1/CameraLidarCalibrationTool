#include <stdio.h>

#include <qpainter.h>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QMouseEvent>
#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <QDialog>
#include <QGraphicsItem>
#include <QSpinBox>
 
#include "calibration.h"

 
namespace rviz_calibration
{
    // 构造函数，初始化变量
    ImageCalibrator::ImageCalibrator( QWidget* parent ):rviz::Panel( parent )
    {

        //ui.pushButton->setObjectName("test");
        // Initialize Form
        ui.setupUi(this);

        ui.spinBox->setRange(1,10);
        ui.spinBox->setSingleStep(1);
        ui.spinBox->setValue(1);
        
        ui.pushButton->setEnabled(false);

        flag_image_video = true;

        frame_video = 2 ;
        factor = 1 ;

        //ui.graphicsView->hide();

        UpdateTopicList();

        w =  new MainWindow(this);
        //w = MainWindow(this);

        // If combobox is clicked, topic list will be update
        ui.image_topic_comboBox->installEventFilter(this);

        QObject::connect(this,SIGNAL(selectPoint(QPoint)),this,SLOT(showPoint(QPoint)));
        QObject::connect(ui.image_topic_comboBox,SIGNAL(activated(int)),this,SLOT(image_topic_comboBox_activated(int)));
        QObject::connect(this,SIGNAL(clearPonit()),this,SLOT(clearSelectPonit()));
        QObject::connect(ui.pushButton,SIGNAL(clicked()),this,SLOT(toImageCalibration()));
        QObject::connect(this,SIGNAL(setPixMap(QPixmap)),w,SLOT(setBackGroundPic(QPixmap)));
        QObject::connect(this,SIGNAL(resetPaintWidget()),w,SLOT(changePaintWidget()));
        QObject::connect(ui.pushButton_2,SIGNAL(clicked()),this,SLOT(saveCurrentPixmap()));
        QObject::connect(ui.spinBox,SIGNAL(valueChanged(int)),this,SLOT(setVideoFrame(int)));
        QObject::connect(this,SIGNAL(scale2normal()),w,SLOT(resetViewScale()));
    }


    void ImageCalibrator::setVideoFrame(int i)
    {
        factor = i;
    }
    void ImageCalibrator::saveCurrentPixmap()
    {
        ui.pushButton->setEnabled(true);
        Q_EMIT setPixMap(view_on_ui);
        Q_EMIT resetPaintWidget();
        Q_EMIT scale2normal();
    }
    void ImageCalibrator::UpdateTopicList(void)
    {
        QStringList image_topic_list;

        QString image_topic_current = ui.image_topic_comboBox->currentText();
        
        if (image_topic_current == "") {
            image_topic_current = kBlankTopic;
        }


        // // Insert blank topic name to the top of the lists
        image_topic_list << kBlankTopic;

        // Get all available topic
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        // Analyse topics
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
            const ros::master::TopicInfo &info = *it;
            const QString topic_name = QString::fromStdString(info.name);
            const QString topic_type = QString::fromStdString(info.datatype);

            // Check whether this topic is image
            if (topic_type.contains(kImageDataType) == true) {
                image_topic_list << topic_name;
                continue;
            }

        }

        //remove all list items from combo box
        ui.image_topic_comboBox->clear();

        // set new items to combo box
        ui.image_topic_comboBox->addItems(image_topic_list);
 

        ui.image_topic_comboBox->insertSeparator(1);


        // set last topic as current
        int image_topic_index = ui.image_topic_comboBox->findText(image_topic_current);
     
        if (image_topic_index != -1) {
            ui.image_topic_comboBox->setCurrentIndex(image_topic_index);
        }

    }

    //The event filter to catch clicking on combo box
    bool ImageCalibrator::eventFilter(QObject* object, QEvent* event)
    {
        if (event->type() == QEvent::MouseButtonPress) {
            // combo box will update its contents if this filter is applied
            UpdateTopicList();
        }

        return QObject::eventFilter(object, event);

    }

    //void ImageCalibrator::ImageCallback(const sensor_msgs::Image::ConstPtr& msg)
    void ImageCalibrator::ImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
    {
        image_count ++;

        default_count ++;

        //const auto &encoding = sensor_msgs::image_encodings::BGR8;
        viewed_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        if(image_count % (frame_video * factor ) == 1)
        {
            view_on_ui = convert_image::CvMatToQPixmap(viewed_image);
        }

        update();

    }

    void ImageCalibrator::image_topic_comboBox_activated(int index)
    {
        // Extract selected topic name from combo box
        std::string selected_topic = ui.image_topic_comboBox->itemText(index).toStdString();

        if (selected_topic == kBlankTopic.toStdString() || selected_topic == "")
        {
            sub_image.shutdown();

            return;
        }

        // if selected topic is not blank or empty, start callback function
        //default_image_shown_ = false;
        sub_image = nh.subscribe<sensor_msgs::CompressedImage>(selected_topic , 1 , &ImageCalibrator::ImageCallback , this);
    }

    void ImageCalibrator::showPoint(QPoint p)
    {
        point_count ++ ;
        //ui.textBrowser->insertPlainText("Number of Point " + QString::number(point_count) + '(' + QString::number(p.x()) + ',' + QString::number(p.y()) + ')' + "\n");
    }

    void ImageCalibrator::paintEvent(QPaintEvent *e)
    {
        if (default_count == 0)
        {
            QString s = "/home/nio/Pictures/xl.jpg";

            QPixmap ima_show(s);


            QSize q_size= ui.widget->size();
            //QSize q_size= ui.graphicsView->size();

            ima_show = ima_show.scaled(q_size,Qt::KeepAspectRatio);

            QPainter p(this);

            p.drawPixmap(ui.widget->pos(), ima_show);
            //p.drawPixmap(ui.graphicsView->pos(), ima_show);

            p.setPen(QPen(Qt::red, 2, Qt::DashDotLine)); //设置封闭图像的填充颜色,从BrushStyle文件中找，要学会查询函数的使用准则

            //Get data from transport buffer
            QPoint po;
            if(point_trans_buffer.try_pop(po))
            {
                //Push data into display buffer
                point_display_buffer.push(po);
            }


            //loop the display buffer
            point_display_buffer.mut.lock();
            for(auto point:point_display_buffer.data_queue)
            {
                p.drawPoint(point);
            }
            point_display_buffer.mut.unlock();
        }
        
        else
        {

            QPixmap ima_show(view_on_ui);


            QSize q_size= ui.widget->size();
            //QSize q_size= ui.graphicsView->size();

            ima_show = ima_show.scaled(q_size,Qt::KeepAspectRatio);

            QPainter p(this);

            p.drawPixmap(ui.widget->pos(), ima_show);
            //p.drawPixmap(ui.graphicsView->pos(), ima_show);

            p.setPen(QPen(Qt::red, 2, Qt::DashDotLine)); //设置封闭图像的填充颜色,从BrushStyle文件中找，要学会查询函数的使用准则

            //Get data from transport buffer
            QPoint po;
            if(point_trans_buffer.try_pop(po))
            {
            //Push data into display buffer
                point_display_buffer.push(po);
            }


        //loop the display buffer
            point_display_buffer.mut.lock();
            for(auto point:point_display_buffer.data_queue)
            {
                p.drawPoint(point);
            }
            point_display_buffer.mut.unlock();

        }
        

    }

    void ImageCalibrator::mousePressEvent(QMouseEvent *event)
    {
        QPoint p = ui.widget->pos();
        int p_x = p.x();
        int p_y = p.y();
        int p_w = ui.widget->width();
        int p_h = ui.widget->height();
        //QPoint p = ui.graphicsView->pos();
        //int p_x = p.x();
        //int p_y = p.y();
        //int p_w = ui.graphicsView->width();
        //int p_h = ui.graphicsView->height();

        //event->button();
        if(event->button() == Qt::LeftButton)
        {
            QPoint p = event->pos();
            if(event->x() >= p_x && event->x() <= (p_x + p_w) && event->y() >= p_y && event->y() <= (p_y+p_h))
            {
                point_trans_buffer.push(p);
            }
            //point_trans_buffer.push(p);
            Q_EMIT selectPoint(p);

            update();
        }
        if(event->button() == Qt::RightButton)
        {
            if(event->x() >= p_x && event->x() <= (p_x + p_w) && event->y() >= p_y && event->y() <= (p_y+p_h))
            {
                point_display_buffer.mut.lock();

            //clear the display buffer
                point_display_buffer.data_queue.clear();

            //loop the display buffer
                point_display_buffer.mut.unlock();

                point_count = 0;

                Q_EMIT clearPonit();

                update();
            }
            //loop the display buffer

        }
    }

    void ImageCalibrator::clearSelectPonit()
    {
        //ui.textBrowser->clear();
    }

    void ImageCalibrator::toImageCalibration()
    {
        // MainWindow w;
        // QObject::connect(this,SIGNAL(setPixMap(QPixmap)),&w,SLOT(setBackGroundPic(QPixmap)));
        // w.show();
        
        // w =  new MainWindow(this);

        // QObject::connect(this,SIGNAL(setPixMap(QPixmap)),w,SLOT(setBackGroundPic(QPixmap)));
        w->show();
    }

    GraphicsView::GraphicsView(QWidget *parent) :
    QGraphicsView(parent)
    {
        QObject::connect(this,SIGNAL(updateDraw()),this,SLOT(Draw()));
        scene = new QGraphicsScene;
        scene->setSceneRect(0, 0, 2400, 2300);
    }

    void GraphicsView::mousePressEvent(QMouseEvent *event)
    {
    // 分别获取鼠标点击处在视图、场景和图形项中的坐标，并输出
        QPoint viewPos = event->pos();
        qDebug() << "viewPos: " << viewPos;
        QPointF scenePos = mapToScene(viewPos);
        qDebug() << "scenePos: " << scenePos;

        drawItem->setPos(scenePos);
        Q_EMIT updateDraw();
        // QGraphicsItem *item = scene()->itemAt(scenePos, QTransform());
        // if (item) {
        //     QPointF itemPos = item->mapFromScene(scenePos);
        //     qDebug() << "itemPos: " << itemPos;
        // }
    }

    void GraphicsView::keyPressEvent(QKeyEvent *event)
    {
        switch (event->key()) {
        case Qt::Key_Plus:
            scale(1.2,1.2);
            break;
        case Qt::Key_Minus:
            scale(1/1.2,1/1.2);
            break;
        case Qt::Key_Right:
            rotate(30);
            break;
        }
        QGraphicsView::keyPressEvent(event);
    }

    void GraphicsView::Draw()
    {
        scene->addItem(drawItem);
        this->setScene(scene);
        this->setBackgroundBrush(QPixmap("/home/nio/Pictures/xl.jpg"));
        this->resize(400,300);
    }

} 
 
// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_calibration::ImageCalibrator, rviz::Panel )

// END_TUTORIAL

