/*******************************************************************
  copyright (c) 2016 07 06 the Univ of Tokyo
 *
 * @file_pcd.cpp
 * @brief  code for rsj 2016, body detection and motion recovery
 * @author Tianwei Zhang
 *******************************************************************/

#include <real_time_scanline.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
int pcd_index = 0;
PointCloudT::Ptr ex_pcd(new PointCloudT());
ros::Publisher* new_ptr;
ros::Publisher* ex_ptr;
ros::Publisher* diff_ptr;
ros::Publisher* rebody_ptr;
typedef struct {
	int ind;
	int h;
} body_pt;
typedef std::vector <body_pt> body_line;
typedef std::vector <body_line> body_cloud;
//pcl::io::savePCDFile("NorFilted-cloud.pcd", *colored_voxel_cloud);
void  veloEsti(PointCloudT::Ptr &body)
{
	int size_in = body->points.size();
	//int length = body->points[size_in - 1].intensity - body->points[0].intensity;
	int min = body->points[0].intensity;
	int max = min;
	for(int i =0 ; i < size_in; i++)
	{
		int in = body->points[i].intensity;
		if (in > max)
		{ max = in;
			continue;}
		if (in < min)
		{	min = in;
			continue;}
	}
	int len = max - min + 1;
//	std::cout<<" min is " << min << " max is " << max<< " len is "<< len << " size " << body->size()<<std::endl;
	body_cloud Body(len);
//	std::cout<< "body size is "<< Body.size() <<std::endl;
	for(int i = 0; i< size_in; i++)
	{
		int in = body->points[i].intensity;
		int num = in - min;
		//	std::vector<std::vector<int> >::iterator itr_tem = itr + index;
		//	std::vector< int>::iterator int_itr = itr_tem
		//	 itr_tem->push_back(in); 
		body_pt temp_pt;
		temp_pt.h = num;
		temp_pt.ind = i;
		Body.at(num).push_back(temp_pt);
	}
	body_cloud::iterator itr = Body.begin();
	int bSize = Body.size();
	PointCloudT centrID;
//	centrID.resize(bSize);
	for (; itr != Body.end();itr++)
	{
		//std::cout<< "line : " << itr->begin()->h<<" pt size "<<itr->size()<<std::endl;
		body_line::iterator litr = itr->begin();
		PointT temp = (0,0,0);
		int line_size = itr->size();
		if (line_size !=0)// one line has 0 pt, but the further line has 1;. bug in invalid intensity;
		temp.intensity = body->points[litr->ind].intensity;
		for (; litr!= itr->end();litr++)
		{
			temp.x += body->points[litr->ind].x;
			temp.y += body->points[litr->ind].y;
		}
		temp.x /= line_size;
		temp.y /= line_size;
		temp.z = line_size;
	//	std::cout<<temp.x<<", ";

		centrID.push_back(temp);
	}
	PointCloudT Velo;
	for (int i = 1; i< centrID.size(); i++)
	{
		if(centrID.points[i].z > 1)
		{
			PointT velo;
			velo.x = centrID.points[i].x - centrID.points[i-1].x;
			velo.y  = centrID.points[i].y - centrID.points[i-1].y;
			velo.x /= 0.05;
			velo.y /= 0.05;
			velo.intensity = centrID.points[i].intensity;
			Velo.push_back(velo);
		//	std::cout<<velo.y<<", ";
		}
	}

//	std::cout<< "cent size " << centrID.size()<< " velo size " << Velo.size()<<std::endl; 
	float vx,vy;
	for(int  i = 0; i< Velo.size(); i++)
	{
		vx += Velo.points[i].x;
		vy += Velo.points[i].y;
	}
	vx /= Velo.size();
	vy /= Velo.size();	
      std::cout<< " velo is " << vx << ", " << vy<< std::endl;
	PointCloudT re_body;
//	re_body = *body;
	body_cloud::iterator re_itr = Body.begin();

	for(;re_itr != Body.end();re_itr++)
	{
		body_line::iterator re_line = re_itr->begin();
		for(;re_line != re_itr->end();re_line++)
		{
			PointT temp = body->points[re_line->ind];	 
			temp.x =  body->points[re_line->ind].x - re_line->h * vx * 0.05;
			temp.y =  body->points[re_line->ind].y - re_line->h * vy * 0.05;
//		std::cout<<temp<<std::endl;
			re_body.push_back(temp);
		}
	}	
//std::cout<<" re body "<< re_body.size()<< " body size " << body->size()<<std::endl;
re_body.header.frame_id = body->header.frame_id;
new_ptr->publish(body);
//pcl::io::savePCDFile("body.pcd", re_body);
rebody_ptr->publish(re_body);
}

PointCloudT::Ptr Diff_pcd(PointCloudT::Ptr &cloudA, PointCloudT::Ptr &cloudB)
{
	// Read two PCD files from disk.
	if (cloudA->points.size()==0||cloudB->points.size()==0)
	{
		std::cout<<"Cloud is empty"<<std::endl;
	}

	// Change detector object, with a resolution of 128
	// (resolution at lowest octree level).
	pcl::octree::OctreePointCloudChangeDetector<PointT> octree(0.4f);

	// Add cloudA to the octree.
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();
	// The change detector object is able to store two clouds at the same time;
	// with this we can reset the buffer but keep the previous tree saved.
	octree.switchBuffers();
	// Now, add cloudB to the octree like we did before.
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();

	std::vector<int> newPoints;
	// Get a vector with the indices of all points that are new in cloud B,
	// when compared with the ones that existed in cloud A.
	octree.getPointIndicesFromNewVoxels(newPoints);
	PointCloudT::Ptr diff_pcd (new PointCloudT());
	diff_pcd->header.frame_id = cloudB->header.frame_id;
	for (size_t i = 0; i < newPoints.size(); ++i)
	{	
		if(cloudB->points[newPoints[i]].x> 1.0&& cloudB->points[newPoints[i]].x <3.0&&cloudB->points[newPoints[i]].y<1.0)
			diff_pcd->push_back(cloudB->points[newPoints[i]]);
		//		 was not in cloud A but is in cloud B" << std::endl;
	}
	std::cout <<"diff pcd size "<< newPoints.size()<<" , " << diff_pcd->points.size() << std::endl;
	diff_ptr->publish(diff_pcd);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (diff_pcd);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.20); // 2cm
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (diff_pcd);
	ec.extract (cluster_indices);

	if(cluster_indices.size() != 0)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			PointCloudT::Ptr cloud_cluster (new PointCloudT);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				cloud_cluster->points.push_back (diff_pcd->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			cloud_cluster->header.frame_id = cloudB->header.frame_id;
		//	new_ptr->publish(cloud_cluster);
	//		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	veloEsti(cloud_cluster);
		}
	}
	return diff_pcd;
}

void pcd_cb(const sensor_msgs::PointCloud2ConstPtr &pcd)
{
	if(!pcd->is_dense)
		return;
	PointCloudT::Ptr new_pcd(new PointCloudT());
	pcl::fromROSMsg(*pcd, *new_pcd);
pcl::io::savePCDFile("walking.pcd", *pcd);
	float width = 1080;
	int h = 1;
	for(int i = 0; i< new_pcd->points.size();i++)
	{
		if(i <= width*h)
			new_pcd->points[i].intensity = h;
		else{
			++h;
			new_pcd->points[i].intensity = h;
		}
	}
	//std::cout<<"new pcd size  "<<ater<<std::endl;

	//  int new_pcd_index = pcd_index;
	//std::cout<<new_pcd_index<<std::endl;
	//  if (new_pcd_index == 0)
	//	return;

	//	PointCloudT::Ptr surfpcd(new PointCloudT());
	//	surfpcd = new_pcd;
	//	pcl::PointCloud<PointT>::Ptr pcd_DS(new PointCloudT());
	//	pcl::VoxelGrid<PointT> downSizeFilter;
	//	downSizeFilter.setInputCloud(surfpcd);
	//	downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
	//	downSizeFilter.filter(*pcd_DS);
	//new_ptr->publish(new_pcd);
	if (ex_pcd->points.size() != 0)
	{
		ex_ptr->publish(ex_pcd);
double start = pcl::getTime();
		Diff_pcd(ex_pcd, new_pcd);
double end = pcl::getTime();
std::cout<< "time is "<< end - start<<std::endl;
	}
	ex_pcd->clear();
	*ex_pcd = *new_pcd;
	new_pcd->clear();
	pcd_index++;

}  

int main (int argc, char** argv)
{
	ros::init (argc, argv, "save_pcd");
	ros::NodeHandle nh;
	ros::Subscriber sub_pc2 = nh.subscribe<sensor_msgs::PointCloud2>("/sensor/hokuyo_lidar",5, pcd_cb);

	ros::Publisher pub_new_pcd = nh.advertise<sensor_msgs::PointCloud2> ("/new_pcd", 10);
	ros::Publisher pub_ex_pcd = nh.advertise<sensor_msgs::PointCloud2> ("/ex_pcd", 10);
	ros::Publisher pub_diff_pcd = nh.advertise<sensor_msgs::PointCloud2> ("/diff_pcd", 10);
	ros::Publisher pub_rebody = nh.advertise<sensor_msgs::PointCloud2> ("/rebody_pcd", 10);
	new_ptr = &pub_new_pcd;
	ex_ptr = &pub_ex_pcd;
	diff_ptr = &pub_diff_pcd;
	rebody_ptr = &pub_rebody;

	ros::spin ();
	return 0;
}

