#ifndef CALIBRATION_H
#define CALIBRATION_H

#ifndef Q_MOC_RUN
//所需要包含的头文件


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  // opencv 常用头文件

#include "convert_image.h" // opencv Mat格式转换成 QImage QPixmap

#include <ros/ros.h>
#include <ros/console.h> // 使用 ros message，nodehandle 等文件

#include <rviz/panel.h>   //plugin基类的头文件
#include <sensor_msgs/Image.h>

#include "ui_calibration.h"  // 使用.ui 文件时必须 #include "ui_*.h"

#include "ThreadSafeQueue.h" // 使用 线程安全队列

#include <QGraphicsView>

#include <QGraphicsItem>

#include <QGraphicsScene>

#include "mainwindow.h"

#endif

namespace rviz_calibration
{

// 所有的plugin都必须是rviz::Panel的子类
class ImageCalibrator: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT

public:

    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    ImageCalibrator( QWidget* parent = 0 );

    // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
    // 中，数据就是topic的名称

    //virtual void load( const rviz::Config& config );
    //virtual void save( rviz::Config config ) const;

    
public:
    // The function to update topic list that can be selected from the UI
    void UpdateTopicList(void);

    //The event filter to catch clicking on combo box
    bool eventFilter(QObject* object, QEvent* event);

    //The function callback
    void ImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    //void ImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    

    //rewrite the paint function
    void paintEvent(QPaintEvent *e);

    //rewrite the mousepressevent function
    void mousePressEvent(QMouseEvent *event);

    //The queue is used to dispaly points on screen
    ThreadSafeQueue<QPoint> point_display_buffer;

    //The Queue is used to transmit points
    ThreadSafeQueue<QPoint> point_trans_buffer;


private:

    //the subscriber 
    ros::Subscriber sub_image;

    //the publisher
    ros::Publisher pub_point;

    //the rosnode
    ros::NodeHandle nh;

    //The UI components
    Ui::calibration ui;

    // The opencv elements to show 
    cv::Mat viewed_image;

    //used to control the frame of image showed
    int image_count = 0;

    //used to write the sum of points
    int point_count = 0;

    //the pixmap show on widget
    QPixmap view_on_ui;

    QPixmap ima_show;

    //const QString kImageDataType, select the type of message
    const QString kImageDataType = "sensor_msgs";

    // The blank topic name
    const QString kBlankTopic = "-----";

    int default_count = 0 ;

    bool flag_image_video;

    MainWindow *w;
    // MainWindow w;

    int frame_video;

    int factor;

Q_SIGNALS:

    //when left button of mouse is clicked ,signal will be emited
    void selectPoint(QPoint p);

    //signal to clear all points on screen
    void clearPonit();

    void setPixMap(QPixmap image);

    void resetPaintWidget();

    void scale2normal();

private Q_SLOTS:
    //   We can skip "connect" process by defining naming
    //   of slot function like on_"widget_name"_"signal_name"

    void image_topic_comboBox_activated(int index);

    void showPoint(QPoint p);

    void clearSelectPonit();

    void toImageCalibration();

    void saveCurrentPixmap();

    void setVideoFrame(int i);

};

class GraphicsView: public QGraphicsView
{
    Q_OBJECT
public:
    explicit GraphicsView(QWidget *parent = 0);
protected:
    void mousePressEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
private:
    QGraphicsItem *drawItem;
    QGraphicsScene *scene;
Q_SIGNALS:
    void updateDraw();
public Q_SLOTS:
    void Draw();
};


} // end namespace rviz_plugin_tutorials
 
#endif // CALIBRATION_H
