#include "mmwave_d435_mapping.h"

MMmapping::MMmapping(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	//map_process.reset(new PointCloud<PointXYZ>());
	//pc_input0.reset(new PointCloud<PointXYZ>());
	//pc_input1.reset(new PointCloud<PointXYZ>());
	pc_input2.reset(new PointCloud<PointXYZ>());
	pc_input3.reset(new PointCloud<PointXYZ>());
	pc_input4.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	//result.reset(new PointCloud<PointXYZ>());
	scene_cloud_.reset(new PointCloud<PointXYZ>());
	std::vector<PointCloud<PointXYZ>> source_clouds;

    posearr_msg = geometry_msgs::PoseArray();
	posearr_msg.header = std_msgs::Header();

	//shape = visualization_msgs::Marker::LINE_STRIP;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;

	//trans0 = get_transfrom("mmwave_right_link");
	//trans1 = get_transfrom("mmwave_left_link");
	trans2 = get_transfrom("mmwave_right_back_link");
	trans3 = get_transfrom("mmwave_left_back_link");
	trans4 = get_transfrom("camera_color_optical_frame");

    pc_map = nh.advertise<sensor_msgs::PointCloud2> ("mmwave_mapping", 10);
	//pc_sub0 = nh.subscribe("radar_0/ti_mmwave/ti_mmwave/radar_scan_pcl", 1, &MMmapping::pc0_callback, this);
	//pc_sub1 = nh.subscribe("radar_1/ti_mmwave/ti_mmwave/radar_scan_pcl", 1, &MMmapping::pc1_callback, this);
    pc_sub2 = nh.subscribe("filtered_pc_mmwave2", 1, &MMmapping::pc2_callback, this);
    pc_sub3 = nh.subscribe("filtered_pc_mmwave3", 1, &MMmapping::pc3_callback, this);
	pc_sub4 = nh.subscribe("/d435_crop", 1, &MMmapping::pc4_callback, this);
}


Eigen::Matrix4f MMmapping::get_transfrom(string link_name){
	tf::StampedTransform transform;
	Eigen::Matrix4f trans;

	try{
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("base_link", link_name, ros::Time(0), five_seconds);
		listener.lookupTransform("base_link", link_name, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return trans;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ()); 
	Eigen::Matrix3f mat = q.toRotationMatrix();
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			0, 0, 0, 1;
	return trans;
}

void MMmapping::timerCallback(const ros::TimerEvent& event)
{
	if (d435_receive && mm_wave1_receive && mm_wave2_receive){
		//*map += *pc_input0;
		//*map += *pc_input1;
		*map += *pc_input2;
		*map += *pc_input3;
		cout <<"\nmmwave~~~"<<map->size();
		*map += *pc_input4;
		cout <<"\nd435~~~"<<map->size();
		cout<<"\n=====";
		//cout <<map->size();
		toROSMsg(*map, ros_cloud_msg);
		ros_cloud_msg.header.frame_id = "base_link";
		pc_map.publish(ros_cloud_msg);
		map->points.clear();
		//pc_input0->points.clear();
		//pc_input1->points.clear();
		pc_input2->points.clear();
		pc_input3->points.clear();
		pc_input4->points.clear();

		mm_wave1_receive = false;
		mm_wave2_receive = false; 
		d435_receive = false;
	}
}
/*
void MMmapping::pc0_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input0);

	
	pcl::transformPointCloud (*pc_input0, *pc_input0, trans0);
}

void MMmapping::pc1_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input1);

	pcl::transformPointCloud (*pc_input1, *pc_input1, trans1);
}
*/
void MMmapping::pc2_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input2);

	pcl::transformPointCloud (*pc_input2, *pc_input2, trans2);

	mm_wave1_receive = true;
}
void MMmapping::pc3_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input3);

	pcl::transformPointCloud (*pc_input3, *pc_input3, trans3);

	mm_wave2_receive = true;
}

void MMmapping::pc4_callback(const sensor_msgs::PointCloud2 msg){

	fromROSMsg (msg, *pc_input4);

	pcl::transformPointCloud (*pc_input4, *pc_input4, trans4);

	d435_receive = true;
}


int main(int argc, char** argv){
	init(argc, argv, "orb_process_base");
	MMmapping icp;
	NodeHandle nh;
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), &MMmapping::timerCallback, &icp);
	spin();
	return 0;
}
