/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % 2016 05 01 @ the Univ. of Tokyo
  % Tianwei Zhang
  % file: real_time_scanline.h
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#ifndef REAL_TIME_SCANLINE_H
#define    REAL_TIME_SCANLINE_H

#include<math.h>
#include<time.h>
#include<ros/ros.h>
#include<iostream>

#include<pcl/point_types.h>
#include<pcl/common/time.h>
#include<pcl/pcl_base.h>
#include<pcl/point_cloud.h>
#include<pcl/common/eigen.h>
#include<pcl/io/pcd_io.h>
#include<pcl/console/parse.h>
#include<pcl/filters/filter.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_ros/transforms.h>

#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>

#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_datatypes.h>
#include<dynamixel_msgs/JointState.h>
#include<nav_msgs/Odometry.h>
#include<visualization_msgs/Marker.h>
//#include<list>

using namespace Eigen;
namespace region_growing{
	typedef pcl::PointXYZI PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef struct {PointT point_s; 
		size_t index_s;
		PointT point_e;
		size_t index_e;
		size_t height;} split_point;
	typedef std::vector <split_point> split_line;
	typedef std::vector <split_line> split_image;

	typedef struct { float RMS;
			 size_t size; //number of line segs
			 //Eigen::Vector3f n;
			split_line lines;	
			Vector3f n;
			float dis;} Plane;//save planes para

	typedef std::vector <Plane> Planes;


	static split_image image_helper;
	/////save plane seg candidant
	static Plane patch;
	////save plane maps
//	static split_image patch_map;
	////plane parameters d and n(vector)
	static size_t min_size = 5;
	/// min seeding line distance and length
	///plane fitting threshod
	static float theta_grow = 0.03;
	static float theta_seed = 0.02;
//	static PointCloudT plane_cloud;
	static float RMS;
//	static Plane plane;
	static Planes planes;

static int patch_no, patchmap_no, comtru_no, extend_, seeds_;
	class region_grow { 
		public:
			bool  prepare (split_image &image) ;
			split_image grow(split_image &image_vec);
			PointCloudT get_normals();
			split_image merge_plane();
			inline float sq(float x) {return x * x;}
		private:

			////get data from split pt struct
			split_point get_data(split_line::iterator &itr);
			///comp if a b are coplanar
			bool compare(split_line::iterator & a, split_line::iterator &b);
			/////compute plane formular A*m = b /2016/02/16
			bool coplanar_or_not(split_line::iterator &c);
			bool coplanar_or_not(split_line::iterator &a,split_line::iterator &b,split_line::iterator &c);
			//////add a b to same plane candidante
			void add_into_patch(split_line::iterator &a);
			////comput plane formular, return vec_m, n is nor(m), d is reciprocal(m), input is the point cloud of query pt
			bool  plane_formular(PointCloudT &sigma);
			bool  recom_plane(PointCloudT &sigma);
			float det(PointT &a, PointT &b, PointT  &c);	
			void extend (split_line::iterator &itr);				
			void seeding ();				
			void make_patch(split_line::iterator &a,split_line::iterator &b, split_line::iterator &c, float &RMS, Vector3f &n, float &d);			
			bool a_is_in_patch(split_line::iterator &a);
			PointT minus ( PointT &a, PointT &b);
			///martrix product 3x3 return in R1
			float mar_mul_R1(PointT &n_3, PointT &b_3 );
		public: 

		//	split_image::iterator extend_line;
			split_image::iterator query_line;
			split_line::iterator query_pt_itr;
		//	split_line::iterator extend_pt_itr;




	};
}
#endif
