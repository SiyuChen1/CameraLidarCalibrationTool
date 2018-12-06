#ifndef CALIBRATION_H
#define CALIBRATION_H

#ifndef Q_MOC_RUN
//所需要包含的头文件

#include <stdio.h> //c++

#include <ros/ros.h>
#include <ros/console.h> // 使用 ros message，nodehandle 等文件
#include <rviz/panel.h>   //plugin基类的头文件

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h> //callback 函数使用的sensor_msgs std_msgs 消息类型

#include "rviz_calibration/projection_matrix.h"
#include "rviz_calibration/PointsImage.h" //ros中自定义的msg

#include <QGraphicsView>
#include <QMap>
#include <QGraphicsItem>
#include <QStringListModel>
#include <QStandardItemModel>
#include <QGraphicsScene>
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
#include <QFileDialog>
#include <QUrl>
#include <QMessageBox> //qt头文件

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  // opencv 常用头文件

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> //pcl 常用头文件

#include "convert_image.h" // opencv Mat格式转换成 QImage QPixmap

#include "ui_calibration.h"  // 使用.ui 文件时必须 #include "ui_*.h"
#include "ThreadSafeQueue.h" // 使用 线程安全队列
#include "mainwindow.h" //图像标定窗口的头文件

#endif

namespace rviz_calibration
{

class DrawPoints{
  public:
    explicit DrawPoints(void);
    void Draw(const rviz_calibration::PointsImage::ConstPtr& points, cv::Mat& image, int drawn_size);

  private:
    cv::Mat color_map_;

};
// DrawPoints Draw()实现在image 绘制 pointcloud 投射到image上的点

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
    // The function to update image topic list that can be selected from the UI
    void UpdateTopicList(void);

    // The function to update PointCloud topic list that can be selected from the UI
    void UpdatePointCloudTopicList();

    //The event filter to catch clicking on combo box
    bool eventFilter(QObject* object, QEvent* event);

    //The function callback
    void CompressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    void ImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    
    void ImageHeaderCallback(const sensor_msgs::Image::ConstPtr& msg);

    void CompressedHeaderImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    void HeaderCallback(const std_msgs::Header::ConstPtr& header);

    void DistCorrectionCompressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    void DistCorrectionImageCallback(const sensor_msgs::Image::ConstPtr& msg);

    void PointCloud2ImageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    //rewrite the paint function
    void paintEvent(QPaintEvent *e);

    //rewrite the mousepressevent function
    void mousePressEvent(QMouseEvent *event);

    //The queue is used to dispaly points on screen
    ThreadSafeQueue<QPoint> point_display_buffer;

    //The Queue is used to transmit points
    ThreadSafeQueue<QPoint> point_trans_buffer;

    // show image use QLabel
    void showImage(QPixmap &picture);

    // 使用opencv的 yaml 读取 相机内参矩阵 和 投影矩阵
    void ReadCameraParameter(QString parameter_path);

    // 将opencv mat数据格式转换成 QString
    QString Mat2QString(cv::Mat &mat);

    // 管理指针，可以写成重载函数
    void pointerManagement(QStandardItem* p);

    // 初始化 投影矩阵
    void initMatrix(const cv::Mat& cameraExtrinsicMat);

    // 利用相机内外参矩阵将pointcloud 转换成 二维坐标下的平面点
    rviz_calibration::PointsImage pointcloud2_to_image(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2, const cv::Mat& cameraExtrinsicMat, const cv::Mat& cameraMat,
        const cv::Mat& distCoeff, const cv::Size& imageSize);

private:

    // 储存 topic 的name 和 type
    QMap<QString,QString> map_topicName_topicType;

    //the rosnode
    ros::NodeHandle nh;

    // 订阅 图像
    ros::Subscriber sub_image;

    // 订阅 pointcloud
    ros::Subscriber sub_points;

    // 订阅 畸变校正后的image
    ros::Subscriber sub_image_for_rect;

    // 订阅 camerainfo
    ros::Subscriber sub_camerainfo;

    // 订阅 image的header
    ros::Subscriber sub_header;

    // 订阅 去掉地面后的点云
    ros::Subscriber sub_points_no_groud;

    // publish 
    ros::Publisher pub_points;

    ros::Publisher pub_image_rected;

    ros::Publisher pub_time;

    ros::Publisher pub_point_image;

    ros::Publisher pub_camerainfo;

    ros::Publisher pub_point;

    ros::Publisher pub_projection;

    // 投影矩阵
    rviz_calibration::projection_matrix camera_to_wrold_projection;

    // pointcloud2Image
    rviz_calibration::PointsImage::Ptr points_image_msg;

    std::string camera_info_name = "camera_info";

    std::string projection_matrix_name = "projection_matrix";

    sensor_msgs::CameraInfo camera_info_msg_;

    // MainWindow w;
    MainWindow *w;

    //The UI components
    Ui::calibration ui;

    //used to control the frame of image showed
    int image_count = 0;

    //used to write the sum of points
    int point_count = 0;

    int default_count = 0 ;

    //the pixmap show on widget
    QPixmap view_on_ui;

    QPixmap ima_show;

    QString image_topic_name_current;

    std::string image_topic_name_std;

    //const QString kImageDataType, select the type of message
    const QString kImageDataType_1 = "sensor_msgs/Image";

    const QString kImageDataType_2 = "sensor_msgs/CompressedImage";

    const QString kPointCloudDataType = "sensor_msgs/PointCloud2";

    // The blank topic name
    const QString kBlankTopic = "-----";

    QString parameter_path = "";

    // The opencv elements to show 
    cv::Mat viewed_image;

    cv::Mat CameraExtrinsicMat;

    cv::Mat invRt, invTt;
    
    cv::Mat CameraMat;
    
    cv::Mat DistCoeff;
    
    cv::Size ImageSize;
    
    std::string DistModel;

    std::string CameraType;
    //"normal" means 普通相机
    //"fisheye" means 鱼眼相机

    // used to show camerainfo
    QStringListModel *Model;

    QStandardItemModel *ItemModel;

    // flags to judge if camrainfo is deliveried 
    bool camera_info_delivery = false;

    bool init_matrix = false;

    bool points_image_sended = false;

    bool points_show_with_image = false;

    bool camera_is_fisheye = false;

    bool flag_image_video = true;

    int frame_video = 2;

    int factor = 1;

    // 将pointcloud 转成 二维平面上的点
    DrawPoints points_drawer;

Q_SIGNALS:

    //when left button of mouse is clicked ,signal will be emited
    void selectPoint(QPoint p);

    //signal to clear all points on screen
    void clearPonit();

    void setPixMap(QPixmap image);

    void resetPaintWidget();

    void scale2normal();

private Q_SLOTS:

    void image_topic_comboBox_activated(int index);

    void comboBox_PointCloud_activated(int index);

    void showPoint(QPoint p);

    void clearSelectPonit();

    void toImageCalibration();

    void saveCurrentPixmap();

    void setVideoFrame(int i);

    void openFile();

    void ShowCameraInfo();

    void PublishCameraInfo();

    void StartRectifierNode(bool checked);

    void startConvertPoint2Image(bool checked);

    void update_points_show_with_image(bool checked);

    void updateCamearaType(bool checked);

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
// 目前class GraphicsView 并未使用


} // end namespace rviz_plugin_tutorials
 
#endif // CALIBRATION_H
