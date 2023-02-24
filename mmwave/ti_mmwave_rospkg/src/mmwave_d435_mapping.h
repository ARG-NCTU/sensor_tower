#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <vector>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>
// Ros
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>
// Pcl load and ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
// Pcl icp
#include <pcl/registration/icp.h>
// Pcl downsampling
#include <pcl/filters/voxel_grid.h>

using namespace ros;
using namespace pcl;
using namespace std;

class MMmapping{
  public:
	MMmapping();
	//void pc0_callback(const sensor_msgs::PointCloud2 msg);    // sensor_msgs::PointCloud2
	//void pc1_callback(const sensor_msgs::PointCloud2 msg); 
	void pc2_callback(const sensor_msgs::PointCloud2 msg);   
	void pc3_callback(const sensor_msgs::PointCloud2 msg);  // sensor_msgs::PointCloud2
	void pc4_callback(const sensor_msgs::PointCloud2 msg);  // sensor_msgs::PointCloud2 for realsense
	void pose_callback(const geometry_msgs::Pose msg); 
	void timerCallback(const ros::TimerEvent& event);   // sensor_msgs::PointCloud2
	Eigen::Matrix4f get_transfrom(string link_name);


  private:
	Subscriber pose_sub;
	//Subscriber pc_sub0;
	//Subscriber pc_sub1;
	Subscriber pc_sub2;
	Subscriber pc_sub3;
	Subscriber pc_sub4;
	Publisher pc_map;
	Publisher pose_pub_client;
	geometry_msgs::PoseArray posearr_msg;

	//Eigen::Matrix4f trans0;
	//Eigen::Matrix4f trans1;
	Eigen::Matrix4f trans2;
	Eigen::Matrix4f trans3;
	Eigen::Matrix4f trans4;

	PointCloud<PointXYZ>::Ptr map;
	PointCloud<PointXYZ>::Ptr new_map;
	PointCloud<PointXYZ>::Ptr map_process;
	//PointCloud<PointXYZ>::Ptr pc_input0;
	PointCloud<PointXYZ>::Ptr pc_slam_input0;
	//PointCloud<PointXYZ>::Ptr pc_input1;
	PointCloud<PointXYZ>::Ptr pc_input2;
	PointCloud<PointXYZ>::Ptr pc_input3;
	PointCloud<PointXYZ>::Ptr pc_input4;
	PointCloud<PointXYZ>::Ptr pc_filter;
	PointCloud<PointXYZ>::Ptr result;
	PointCloud<PointXYZ>::Ptr scene_cloud_;
	std::vector<PointCloud<PointXYZ>> source_clouds;
	int count;
	VoxelGrid<PointXYZ> downsample;

	tf::TransformBroadcaster br;
	tf::TransformListener listener;

	sensor_msgs::PointCloud2 ros_cloud_msg;
	sensor_msgs::PointCloud2 origin_map;

	Publisher marker_pub;
	uint32_t shape;
	visualization_msgs::Marker marker;

	bool mm_wave1_receive = false;
	bool mm_wave2_receive = false;
	bool d435_receive = false;
};