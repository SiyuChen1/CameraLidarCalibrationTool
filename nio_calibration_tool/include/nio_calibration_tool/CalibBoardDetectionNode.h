//
// Created by wlh on 18-11-14.
//

#ifndef PROJECT_CALIBRATION_BOARD_DETECTION_NODE_H
#define PROJECT_CALIBRATION_BOARD_DETECTION_NODE_H

#include <nio_calibration_tool/CalibBoardDetector.h>

class CalibBoardDetectionNode
{
public:
    CalibBoardDetectionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    ~CalibBoardDetectionNode();

    void run();

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

    void generate_marker_array(visualization_msgs::MarkerArray& marker_array,
                               const CalibBoardDetector::PointCloudArray& vertex_pt_array);

    pcl::PointCloud<pcl::PointXYZRGB> generate_colored_cloud(const CalibBoardDetector::PointCloudArray& cloud_array);

private:
    ros::NodeHandle& m_nh_;
    ros::NodeHandle& m_pnh_;
    ros::Subscriber m_sub_cloud_;
    ros::Publisher m_pub_marker_;
    ros::Publisher m_pub_cloud_;

    std::string m_save_path_;

    CalibBoardDetector m_detector_;
};

#endif //PROJECT_CALIBRATION_BOARD_DETECTION_NODE_H
