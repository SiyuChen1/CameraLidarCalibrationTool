//
// Created by wlh on 18-11-14.
//

#include <nio_calibration_tool/CalibBoardDetector.h>

CalibBoardDetector::CalibBoardDetector()
{

}

CalibBoardDetector::~CalibBoardDetector()
{

}

void CalibBoardDetector::detect(const CalibBoardDetector::PointCloud &cloud) {
    m_cloud_src_ = cloud;
    _height_filter(m_cloud_src_);
    _down_sample(m_cloud_src_);
    m_cloud_cluster_array_ = _euclidean_cluster(m_cloud_src_);

    m_vertex_array_.clear();
    for(auto cluster: m_cloud_cluster_array_){
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        bool plane_flag = _ransac_plane_fitting(cluster, coefficients, inliers);
        if(!plane_flag)
            continue;

        PointCloud cloud_projected = _project_cloud(cluster, coefficients, inliers);

        PointCloud edge_points = _find_contour(cloud_projected);
    

        PointCloudArray line_vec;
        std::vector<pcl::ModelCoefficients> coeff_vec;

        if(edge_points.size()<10) 
            continue;

        _edge_lines_fitting(edge_points, line_vec, coeff_vec);

        if(coeff_vec.size()!=4)
            continue;

        PointCloud inter_points = _find_intersection_points(coeff_vec);
//        std::sort(inter_points.begin(), inter_points.end(), _point_height_cmp);
        if(inter_points.size()==4){
            _sort_vertex(inter_points);
            m_vertex_array_.push_back(inter_points);
        }

        _sort_vertex_array(m_vertex_array_);


//         if(inter_points.size()==4){
// //            _sort_vertex(inter_points);
//             std::sort(inter_points.begin(), inter_points.end(), _point_height_cmp);
//             for(auto pt: inter_points)
//                 std::cout<<pt<<std::endl;
//             std::cout<<std::endl;
//         }

    }
}

CalibBoardDetector::PointCloudArray& CalibBoardDetector::get_vertex_array() {
    return m_vertex_array_;
}

CalibBoardDetector::PointCloudArray& CalibBoardDetector::get_cloud_clustered() {
    return m_cloud_cluster_array_;
}

void CalibBoardDetector::_height_filter(CalibBoardDetector::PointCloud &cloud) {
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.3,1.5);
    pass.filter(cloud);
}

void CalibBoardDetector::_down_sample(CalibBoardDetector::PointCloud &cloud) {
    PointCloud cloud_downsample;
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.setInputCloud(cloud.makeShared());
    voxel.filter(cloud);
}

std::vector<CalibBoardDetector::PointCloud> CalibBoardDetector::_euclidean_cluster(const CalibBoardDetector::PointCloud &cloud) {
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud.makeShared());
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance (0.2);
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (300);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud.makeShared());
    ec.extract (cluster_indices);

    PointCloudArray cloud_clustered;
    for (int i=0; i< cluster_indices.size(); ++i)
    {
        PointCloud cloud_cluster;
        for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin (); pit != cluster_indices[i].indices.end (); ++pit){
            Point p;
            p.x = cloud.points[*pit].x;
            p.y = cloud.points[*pit].y;
            p.z = cloud.points[*pit].z;
            cloud_cluster.points.push_back (p);
        }
        cloud_clustered.push_back(cloud_cluster);
    }
    return cloud_clustered;
}

bool CalibBoardDetector::_ransac_plane_fitting(const CalibBoardDetector::PointCloud &cloud,
                                               pcl::ModelCoefficients::Ptr coeff,
                                               pcl::PointIndices::Ptr inliers) {
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (cloud.makeShared());
    seg.segment (*inliers, *coeff);

    double ratio = inliers->indices.size() * 1.0 / cloud.points.size();
    if(ratio>0.95) return true;
    else return false;
}

CalibBoardDetector::PointCloud CalibBoardDetector::_project_cloud(const CalibBoardDetector::PointCloud &cloud,
                                              const pcl::ModelCoefficients::Ptr coeff,
                                              const pcl::PointIndices::Ptr inliers) {
    // Project the model inliers
    PointCloud cloud_projected;
    pcl::ProjectInliers<Point> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud.makeShared());
    proj.setModelCoefficients (coeff);
    proj.filter (cloud_projected);

    return cloud_projected;
}

CalibBoardDetector::PointCloud CalibBoardDetector::_find_contour(const PointCloud &cloud) {
    // Create a Concave Hull representation of the projected inliers
    PointCloud cloud_hull;
    pcl::ConcaveHull<Point> chull;
    chull.setInputCloud (cloud.makeShared());
    chull.setAlpha (0.1);
    chull.reconstruct (cloud_hull);

    return cloud_hull;
}

void CalibBoardDetector::_edge_lines_fitting(CalibBoardDetector::PointCloud &cloud,
                                             PointCloudArray& line_vec,
                                             std::vector<pcl::ModelCoefficients>& coeff_vec) {
    int nr_points = (int) cloud.points.size ();
    // While 10% of the original cloud is still there

    pcl::ModelCoefficients::Ptr line_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices);
    pcl::SACSegmentation<Point> line_seg;
    line_seg.setOptimizeCoefficients (true);
    line_seg.setModelType (pcl::SACMODEL_LINE);
    line_seg.setMethodType (pcl::SAC_RANSAC);
    line_seg.setMaxIterations(100);
    line_seg.setDistanceThreshold (0.05);

    while (cloud.points.size () > 0.1 * nr_points){

        line_seg.setInputCloud (cloud.makeShared());
        line_seg.segment (*line_inliers, *line_coefficients);

        pcl::ExtractIndices<Point> extract;
//        PointCloud::Ptr cloud_f (new PointCloud);
        PointCloud cloud_line;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(line_inliers);

        extract.setNegative (false);
        extract.filter (cloud_line);

        extract.setNegative (true);
        extract.filter (cloud);

        line_vec.push_back(cloud_line);
        coeff_vec.push_back(*line_coefficients);
//        std::cout<<"line_coefficients: "<<line_coefficients->values[0]<<std::endl;
    }
}

CalibBoardDetector::PointCloud CalibBoardDetector::_find_intersection_points(const std::vector<pcl::ModelCoefficients> &coeff_array) {

    CalibBoardDetector::PointCloud intersect_point_vec;

//    std::cout<<"coeff_array size: "<<coeff_array.size()<<std::endl;

    for(size_t j=0; j<coeff_array.size(); ++j){
        for(size_t k=j+1; k<coeff_array.size(); ++k){
            const pcl::ModelCoefficients& line_coeff0 = coeff_array[j];
            const pcl::ModelCoefficients& line_coeff1 = coeff_array[k];

            Eigen::Vector3f v0, v1;
            v0 << line_coeff0.values[3], line_coeff0.values[4], line_coeff0.values[5];
            v1 << line_coeff1.values[3], line_coeff1.values[4], line_coeff1.values[5];
            double cos_val = v0.dot(v1)/(v0.norm()*v1.norm());

            Eigen::Vector3f v2;
            v2 << 0.0, 0.0, 1.0;
            double cos_val0 = v0.dot(v2)/(v0.norm()*v2.norm());
            double cos_val1 = v1.dot(v2)/(v1.norm()*v2.norm());

            double ang0 = RAD2DEG(acos(cos_val0));
            double ang1 = RAD2DEG(acos(cos_val1));

            if(fabs(ang0)<20 || fabs(ang0-90)<20 || fabs(ang1)<20 || fabs(ang1-90)<20)
                continue;

            if(fabs(cos_val)>0.3)
                continue;

//            std::cout<<"cos_val: "<<cos_val<<std::endl;
//            std::cout<<"val0: "<<cos_val0<<" val1: "<<cos_val1<<std::endl;
//            std::cout<<"ang0: "<<ang0<<" ang1: "<<ang1<<std::endl;

            Eigen::Vector4f intersect_point;
            bool is_intersect = pcl::lineWithLineIntersection(line_coeff0, line_coeff1, intersect_point);
            if(is_intersect){
                Point pt;
                pt.x = intersect_point[0];
                pt.y = intersect_point[1];
                pt.z = intersect_point[2];
                intersect_point_vec.push_back(pt);
//                _cal_bearing(pt);
//                std::cout<<"inter point: "<<intersect_point<<std::endl;
                    Point pt0;
                    pt0.x = 1.0;
                    pt0.y = 0;
                    pt0.z = 0;
                    // std::cout<<"angle: "<<RAD2DEG( atan2(pt.y, pt.x) )<<" "<<_point_bearing_cmp(pt, pt0)<<std::endl;
            }
        }
    }

    return intersect_point_vec;
}

double CalibBoardDetector::_cal_bearing(const CalibBoardDetector::Point &pt) {
    //todo transform bearing
    double bearing = RAD2DEG( atan2(pt.y, pt.x) ) ;
    if(bearing<0) bearing+= 360;
    std::cout<<"-bering: "<<(-bearing)<<std::endl;
    return -bearing;
}

void CalibBoardDetector::_sort_vertex(CalibBoardDetector::PointCloud &vertex_points) {
    auto points = vertex_points.points;
    CalibBoardDetector::PointCloud points_sorted;
    points_sorted.resize(points.size());
    std::sort(points.begin(), points.end(), _point_height_cmp);
    points_sorted[1] = points[0];
    points_sorted[3] = points[3];

    std::sort(points.begin(), points.end(), _point_bearing_cmp);
    points_sorted[0] = points[0];
    points_sorted[2] = points[3];

    vertex_points = points_sorted;
}

void CalibBoardDetector::_sort_vertex_array(CalibBoardDetector::PointCloudArray &vertex_array_) {
    std::sort(vertex_array_.begin(), vertex_array_.end(), _cloud_bearing_cmp);
}

double CalibBoardDetector::_point_cross(const CalibBoardDetector::Point &a, const CalibBoardDetector::Point &b) {
        return a.x*b.y - a.y*b.x;
}

bool CalibBoardDetector::_point_bearing_cmp(const Point &a, const Point &b) {
        return (a.x*b.y - a.y*b.x) <= 0;
}

bool CalibBoardDetector::_cloud_bearing_cmp(const PointCloud& cloud_a, const PointCloud& cloud_b){
    Point pt_a, pt_b;
    pcl::computeCentroid(cloud_a, pt_a);
    pcl::computeCentroid(cloud_b, pt_b);
    return _point_bearing_cmp(pt_a, pt_b);
}