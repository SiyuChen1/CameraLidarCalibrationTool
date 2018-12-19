//
// Created by wlh on 18-6-1.
//


//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/intersections.h>


//custom

#include <pcl/segmentation/extract_clusters.h>
#include <nio_calibration_tool/CalibBoardDetector.h>
#include <nio_calibration_tool/CalibBoardDetectionNode.h>

CalibBoardDetectionNode::CalibBoardDetectionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : m_nh_(nh), m_pnh_(pnh)
{
    //get parameters
    m_pnh_.param<std::string>("save_path", m_save_path_, "");

    m_sub_cloud_ = m_nh_.subscribe("/velodyne64/PointCloud2", 2, &CalibBoardDetectionNode::pointCloudCallback, this);
    m_pub_marker_ = m_nh_.advertise<visualization_msgs::MarkerArray>("calib_markers", 2);
    m_pub_cloud_ = m_nh_.advertise<sensor_msgs::PointCloud2>("cloud_clustered", 2);
}

CalibBoardDetectionNode::~CalibBoardDetectionNode()
{

}

void CalibBoardDetectionNode::run() {
    ros::spin();
}

void CalibBoardDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_in) {

    CalibBoardDetector::PointCloud cloud;
    pcl::fromROSMsg(*cloud_in, cloud);
    m_detector_.detect(cloud);

    auto vertex_pt_array = m_detector_.get_vertex_array();
    visualization_msgs::MarkerArray marker_array;
    generate_marker_array(marker_array, vertex_pt_array);
    m_pub_marker_.publish(marker_array);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_colored = generate_colored_cloud(m_detector_.get_cloud_clustered() );
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_colored, cloud_msg);
    cloud_msg.header.frame_id = "velodyne64";
    cloud_msg.header.stamp = ros::Time::now();
    m_pub_cloud_.publish(cloud_msg);

    if(vertex_pt_array.size() == 4){
        std::fstream ofile;
        std::stringstream ss;
        ss<<ros::Time::now().toNSec();
        std::string filename = m_save_path_+ ss.str() + ".txt";
        ofile.open(filename, std::ios_base::app);
        if(ofile.is_open()){
            for(size_t i=0; i<vertex_pt_array.size(); ++i){
                for(size_t j=0; j<vertex_pt_array[i].size(); ++j){
                    ofile<<vertex_pt_array[i][j].x<<" "<<vertex_pt_array[i][j].y<<" "<<vertex_pt_array[i][j].z<<std::endl;
                }
                ofile<<std::endl;
            }
        }
        else{
            std::cout<<"open file error!"<<std::endl;
        }
    }
}

void CalibBoardDetectionNode::generate_marker_array(visualization_msgs::MarkerArray& marker_array,
                                                    const CalibBoardDetector::PointCloudArray& vertex_pt_array) {
    int pt_id = 0;
    for(size_t v_id=0; v_id<vertex_pt_array.size(); ++v_id){

        auto& vertex_pts = vertex_pt_array[v_id];
        visualization_msgs::Marker txt_marker;
        txt_marker.id = pt_id++;
        txt_marker.header.frame_id = "velodyne64";
        txt_marker.header.stamp = ros::Time::now();
        txt_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt_marker.action = visualization_msgs::Marker::ADD;
        txt_marker.ns = "order";

        txt_marker.pose.position.x = vertex_pts[0].x;
        txt_marker.pose.position.y = vertex_pts[0].y;
        txt_marker.pose.position.z = vertex_pts[0].z;
        txt_marker.pose.orientation.x = 0.0;
        txt_marker.pose.orientation.y = 0.0;
        txt_marker.pose.orientation.z = 0.0;
        txt_marker.pose.orientation.w = 1.0;

        txt_marker.scale.x = 2;
        txt_marker.scale.y = 2;
        txt_marker.scale.z = 2;
        txt_marker.color.r = 1.0;
        txt_marker.color.g = 1.0;
        txt_marker.color.b = 1.0;
        txt_marker.color.a = 1.0;
        std::stringstream ss;
        ss<<v_id;
        txt_marker.text = ss.str();
        marker_array.markers.push_back(txt_marker);

        for(auto pt: vertex_pts){
            visualization_msgs::Marker point_marker;
            point_marker.id = pt_id++;
            point_marker.header.frame_id = "velodyne64";
            point_marker.header.stamp = ros::Time::now();
            point_marker.type = visualization_msgs::Marker::SPHERE;
            point_marker.action = visualization_msgs::Marker::ADD;
            point_marker.ns = "vertex_points";
//            point_marker.lifetime = ros::Duration(0.1);

            point_marker.scale.x = 0.2;
            point_marker.scale.y = 0.2;
            point_marker.scale.z = 0.2;
            point_marker.color.g = 1.0;
            point_marker.color.a = 1.0;

            point_marker.pose.position.x = pt.x;
            point_marker.pose.position.y = pt.y;
            point_marker.pose.position.z = pt.z;
            marker_array.markers.push_back(point_marker);
        }

        for(int i=0; i<vertex_pts.size(); ++i){
            visualization_msgs::Marker txt_marker;
            txt_marker.id = pt_id++;
            txt_marker.header.frame_id = "velodyne64";
            txt_marker.header.stamp = ros::Time::now();
            txt_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt_marker.action = visualization_msgs::Marker::ADD;
            txt_marker.ns = "text";

            txt_marker.pose.position.x = vertex_pts[i].x;
            txt_marker.pose.position.y = vertex_pts[i].y;
            txt_marker.pose.position.z = vertex_pts[i].z;
            txt_marker.pose.orientation.x = 0.0;
            txt_marker.pose.orientation.y = 0.0;
            txt_marker.pose.orientation.z = 0.0;
            txt_marker.pose.orientation.w = 1.0;

            txt_marker.scale.x = 1;
            txt_marker.scale.y = 1;
            txt_marker.scale.z = 1;
            txt_marker.color.r = 1.0;
            txt_marker.color.g = 0.0;
            txt_marker.color.b = 0.0;
            txt_marker.color.a = 1.0;

            std::stringstream ss;
            ss<<i;
            txt_marker.text = ss.str();
            marker_array.markers.push_back(txt_marker);
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB> CalibBoardDetectionNode::generate_colored_cloud(const CalibBoardDetector::PointCloudArray& cloud_array){
    pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;
    int color_table[6] = {0xff0000, 0xff8800, 0xffff00, 0x00ff00, 0x0000ff, 0xff00ff};
    for(size_t i=0; i<cloud_array.size(); ++i){
        for(size_t j=0; j<cloud_array[i].points.size(); ++j){
            pcl::PointXYZRGB p_color;
            p_color.x = cloud_array[i].points[j].x;
            p_color.y = cloud_array[i].points[j].y;
            p_color.z = cloud_array[i].points[j].z;
            p_color.rgb = *reinterpret_cast<float*>(&color_table[i%6]);
            cloud_colored.push_back(p_color);
        }
    }
    return cloud_colored;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_board_detection_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    CalibBoardDetectionNode node(nh, pnh);
    node.run();

    return 0;
}
