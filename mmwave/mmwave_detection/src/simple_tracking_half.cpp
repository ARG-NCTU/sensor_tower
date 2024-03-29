#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream> 
#include <vector>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>
// ROS related
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL related
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// Custom ROS service

using namespace std;
using namespace pcl;
std::string frame;
//  _   _                _           
// | | | |              | |          
// | |_| | ___  __ _  __| | ___ _ __ 
// |  _  |/ _ \/ _` |/ _` |/ _ \ '__|
// | | | |  __/ (_| | (_| |  __/ |   
// \_| |_/\___|\__,_|\__,_|\___|_|   
//

class SimpleTrackingNode{
public:
    SimpleTrackingNode(ros::NodeHandle* nh);
private:
    bool srv_trigger_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void mmwave_data_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg);

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher pub_filtered_pc;
    //ros::Publisher pub_marker_array_;
    ros::Subscriber sub_mmwave_pc;

    // PCL
    PointCloud<PointXYZ>::Ptr scene_cloud_;
    uint32_t cnt_pc_; // pair of pc
    std::vector<PointCloud<PointXYZ>> source_clouds;
    
    // TF
    tf::TransformListener *tf_listener_;

    // Visulization
    visualization_msgs::MarkerArray marker_array_;
    const uint32_t fixed_shape_ = visualization_msgs::Marker::LINE_STRIP;
};


//  _____                          
// /  ___|                         
// \ `--.  ___  _   _ _ __ ___ ___ 
//  `--. \/ _ \| | | | '__/ __/ _ \
// /\__/ / (_) | |_| | | | (_|  __/
// \____/ \___/ \__,_|_|  \___\___|
//

// Constructor
SimpleTrackingNode::SimpleTrackingNode(ros::NodeHandle* nh):nh_(*nh){
    // PointCloud variable init
    scene_cloud_.reset(new PointCloud<PointXYZ>());
    // source_clouds.reset(new std::vector<PointCloud<PointXYZ>>);

    pub_filtered_pc = nh_.advertise<sensor_msgs::PointCloud2>("filtered_pc", 1);
    //pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("detection_markers", 1);

    sub_mmwave_pc = nh_.subscribe("mmwave_mapping", 1, &SimpleTrackingNode::mmwave_data_cb, this);

    // ROS service
    tf_listener_ = new tf::TransformListener();

    // vector<PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud<PointXYZ>::Ptr>> source_clouds;
}


void SimpleTrackingNode::mmwave_data_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg){
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    // copyPointCloud(*scene_cloud_, *cloud);
    pcl::fromROSMsg(*in_cloud_msg, *cloud);

    source_clouds.push_back(*cloud);
    cnt_pc_++;
    //cout << source_clouds.size() << endl;
    if(source_clouds.size() == 5){
        vector<PointCloud<PointXYZ>>::iterator it;
        for(it = source_clouds.begin(); it != source_clouds.end(); it++)
            *scene_cloud_ += *it;

        // Filter point cloud by specific dimension
        pcl::PointCloud<PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZ>);
        pcl::ConditionAnd<PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("x", pcl::ComparisonOps::GT, -5.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("x", pcl::ComparisonOps::LT, 5.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("y", pcl::ComparisonOps::GT, -5.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("y", pcl::ComparisonOps::LT, 5.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("z", pcl::ComparisonOps::LT, 3.0)));
        // Build the filter
        pcl::ConditionalRemoval<PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (scene_cloud_);
        condrem.setKeepOrganized(true);
        // Apply filter
        condrem.filter(*cloud_filtered);

        // Remove NaN point
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud_filtered);
        outrem.setRadiusSearch(1.2);
        outrem.setMinNeighborsInRadius(3);
        // apply filter
        outrem.filter(*cloud_filtered);

        // cloud_filtered->header.frame_id = in_cloud_msg->header.frame_id;
        
        cloud_filtered->header.frame_id = frame;
        pcl_conversions::toPCL(ros::Time::now(), cloud_filtered->header.stamp);
        
        pub_filtered_pc.publish(cloud_filtered);

        //cout <<  cloud_filtered->header.frame_id << endl;
        //cout << cloud_filtered->points.size() << endl;
         if(cloud_filtered->points.size() > 1){
             // ROS_WARN_STREAM("There is no point after filtering on mmwave raw data. Skip this callback.");
             // return;
    
             // Creating the KdTree object for the search method of the extraction
             pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
             tree->setInputCloud(cloud_filtered);

             std::vector<pcl::PointIndices> cluster_indices;
             pcl::EuclideanClusterExtraction<PointXYZ> ec;
             ec.setClusterTolerance (0.8);  // unit: m
             ec.setMinClusterSize (2);     
             ec.setMaxClusterSize (50);   
             ec.setSearchMethod (tree);      
             ec.setInputCloud (cloud_filtered);
             ec.extract (cluster_indices);

             int j = 0;
             marker_array_.markers.clear();
             for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){ 
                 pcl::PointCloud<PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<PointXYZ>);
                 for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                     cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
                 cloud_cluster->width = cloud_cluster->points.size();
                 cloud_cluster->height = 1;
                 cloud_cluster->is_dense = true;

                 Eigen::Vector4f centroid;
                 pcl::compute3DCentroid(*cloud_cluster, centroid);
             }       
        }
        scene_cloud_.reset(new PointCloud<PointXYZ>());
    
        source_clouds.erase(source_clouds.begin());
    }
    

}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_tracking_half");
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ros::NodeHandle nh_local("~"); // create a node handle; need to pass this to the class constructor
    nh_local.getParam("frame", frame);
    SimpleTrackingNode node(&nh);
    ros::spin();
    return 0;
}