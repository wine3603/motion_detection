/*******************************************************************
 * Copyright (c) 2016.09.12 The Univ. of Tokyo	
 *
 * @file urg_copy.cpp && urg_assembler_node.cpp
 * @brief point cloud assermbler for nodding URG
 * @author Tianwei Zhang
 *******************************************************************/
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <pcl_ros/point_cloud.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamixel_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "CameraParams.h"

//#include <rdpl.h>
#include <real_time_scanline.h>

#define EXTRINSIC_FILE_PARAM "~extrinsic_file"

const float bad_point = std::numeric_limits<float>::quiet_NaN();
static tf::TransformListener *slistener;
static pcl::PointCloud<pcl::PointXYZI> sPCLCloud;
static pcl::PointCloud<pcl::PointXYZRGB> sPCLCloudRGB;
static ros::Publisher s_cloud_pub;
//static ros::Publisher wide_pub;
static std::string s_frame_id;
static cv_bridge::CvImagePtr s_cv_ptr;	
//static sensor_msgs::CameraInfo s_camera_info;
//static CameraExtrinsicParams s_extparams;
static double dtol_x = 0.0715;
static double dtol_y = 0;
static double dtol_z = 0.0255;
static bool color_mode = false;

static pcl::PointCloud<pcl::PointXYZI> scanline;//save each line 1081
static pcl::PointCloud<pcl::PointXYZI>  Edge_Point;//save splited pt 
//static ros::Publisher s_edge_pub;

void dynamixel_cb(const dynamixel_msgs::JointStateConstPtr& jstate)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
    tf::Quaternion q;
/*	transform.setOrigin( tf::Vector3(0., 0., 0.) );
    q.setRPY(0, jstate->current_pos, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, jstate->header.stamp, "dynamixel_base", "dynamixel_link"));

*/	transform.setOrigin( tf::Vector3(dtol_x, dtol_y, dtol_z) );
    q.setRPY(0, jstate->current_pos, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, jstate->header.stamp, "dynamixel_base", "laser"));
}

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	geometry_msgs::PointStamped p, p2;
	p.header = scan->header;
    s_frame_id = scan->header.frame_id;
	tf::StampedTransform transform;
    try{
      slistener->lookupTransform("laser", "dynamixel_base",  
                               scan->header.stamp, transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
	  return;
    }

	std::vector<pcl::PointXYZRGB> rgbpoints;
	int scan_skip;
	ros::param::get("urg_scan_skip", scan_skip);
	for (int i=0; i< scan->ranges.size(); i+=scan_skip) {
		double d = scan->ranges[i];
        bool have_intensity = scan->intensities.size() > 0;
        double intensity = 0.;
		if (scan->range_min+0.1 < d && d < scan->range_max) {
			double ang = scan->angle_increment * i + scan->angle_min;
			p.point.x = d * cos(ang);
			p.point.y = d * sin(ang);
			p.point.z = 0;
            slistener->transformPoint("dynamixel_base", scan->header.stamp, p, "laser", p2);
           // slistener->transformPoint("dynamixel_base", scan->header.stamp-ros::Duration(0.015), p, "laser", p2);
            if (have_intensity) {
               // intensity = scan->intensities[i];//here I put range data into intensity for adaptive line split epscilon
                intensity = scan->ranges[i];
            }
            pcl::PointXYZI point(intensity);
            point.x = p2.point.x;
            point.y = p2.point.y;
            point.z = p2.point.z;
            scanline.points.push_back(point);

            pcl::PointXYZRGB pointrgb(0, 0, 0);
			pointrgb.x = p2.point.x;
			pointrgb.y = p2.point.y;
			pointrgb.z = p2.point.z;
			rgbpoints.push_back(pointrgb);
			//printf("%f %f %f\n", p2.point.x, p2.point.y, p2.point.z);
		}
		else {
		pcl::PointXYZI p (intensity);
		p.x = p.y = p.z = bad_point;
		scanline.points.push_back(p);
		}
	}

	while ( scanline.points.size() > 1080){
		// rdpl rdpl_im;
//std::cout<<"epsilon  :"<<epsilon<<std::endl;
	//	double epsilon =  ( scan->range_min + scan->range_max)*0.001;
	//	PointCloudT tempCloud;
//		rdpl_im.rdp_implimentation(scanline, tempCloud, epsilon);
//	for(int i = 0; i<tempCloud.points.size();i++){
//		Edge_Point.points.push_back(tempCloud.points[i]);
//		}
//	tempCloud.points.clear();
	for( int i = 0 ; i < scanline.points.size(); i++){
		sPCLCloud.points.push_back(scanline.points[i]);
//	if(i = 500)
//		Edge_Point.points.push_back(scanline.points[i]);
		}
	sPCLCloud.height++;

//	scanline.header.frame_id = "dynamixel_base";
//	s_cloud_pub.publish(scanline);
	scanline.points.clear();
	}
}

void command_cb(const std_msgs::String::ConstPtr &msg)
{
    std::vector<std::string> str_list;
    //sPCLCloud.header.frame_id = "dynam;
    //sPCLCloud.width = sPCLCloud.points.size();
    boost::algorithm::split(str_list, msg->data, boost::is_any_of(" "));
    if (str_list[0] == "publish") {
        sensor_msgs::PointCloud2 cloud_msg;
		if (color_mode) {
			pcl::toROSMsg(sPCLCloudRGB, cloud_msg);
		} else {
			pcl::toROSMsg(sPCLCloud, cloud_msg);
		}
//	sensor_msgs::PointCloud2 wide_msg;
//	pcl::toROSMsg(Edge_Point, wide_msg);
        cloud_msg.header.frame_id= "dynamixel_base";
	cloud_msg.header.stamp  = ros::Time::now();
        s_cloud_pub.publish(cloud_msg);
//	wide_msg.header.frame_id = "dynamixel_base";
//	wide_pub.publish(wide_msg);
    }
    if (str_list[0] == "save" && str_list.size() > 1) {
		if (color_mode) {
			pcl::io::savePCDFile(str_list[1], sPCLCloudRGB);
		} else {
			pcl::io::savePCDFile(str_list[1], sPCLCloud);
		}
    }
    if (str_list[0] == "clear") {
        sPCLCloud.points.clear();
        sPCLCloudRGB.points.clear();
	sPCLCloud.height = 0;
	Edge_Point.points.clear();
    }
    if (str_list[0] == "color") {
		color_mode = true;
	}
    if (str_list[0] == "intensity") {
		color_mode = false;
	}
//std::cout<<"PCD size :"<<sPCLCloud.points.size()<<"  height:  "<<sPCLCloud.height<< "  width  :" << sPCLCloud.width << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "urg_assembler");
    ros::NodeHandle node;
    slistener = new tf::TransformListener();
    sPCLCloud.points.reserve(1000000); 
    scanline.points.reserve(2000);
    sPCLCloud.height = 0;
    sPCLCloud.width = 1081;//init by zhang  2016.1.11

    s_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("time_cloud", 100);
//    wide_pub = node.advertise<sensor_msgs::PointCloud2>("wide_cloud", 100);
    ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("/most_intense", 100, laser_cb);
    ros::Subscriber sub2 = node.subscribe<std_msgs::String>("/urg_assembler_command", 100, command_cb);
    ros::Subscriber sub3 = node.subscribe<dynamixel_msgs::JointState>("/hokuyo_tilt_controller/state", 100, dynamixel_cb);

	std::string extfile("extparams.xml");
	if (ros::param::has(EXTRINSIC_FILE_PARAM)) {
		ros::param::get(EXTRINSIC_FILE_PARAM, extfile);
	}
	if (!ros::param::has("urg_scan_skip")) {
		ros::param::set("urg_scan_skip", 1);//irie set 2 here/16.01.11
	}

    ros::spin();
    return 0;
}
