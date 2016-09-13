////// compute SHOT descriptor for PCD registration//////////
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>	
#include <pcl/filters/voxel_grid.h>
typedef  pcl::PointXYZI PointT;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;
typedef  PointCloudT::Ptr  CloudPtr;
	int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<PointT>::Ptr cloud(new PointCloudT);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) != 0)
	{
		return -1;
	}

// Estimate the normals.
/*
	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
	ne.setMaxDepthChangeFactor(0.1f);
	ne.setNormalSmoothingSize(20.0f);
	ne.setInputCloud(cloud);
	PCL_ERROR("error\n");
	ne.compute(*normals);
	PCL_ERROR("error\n");

	// visualize normals
	 pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	 viewer.setBackgroundColor (0.0, 0.0, 0.5);
	 viewer.addPointCloudNormals<PointT,pcl::Normal>(cloud, normals);
	 while (!viewer.wasStopped ()) {
	   viewer.spinOnce ();
	}
*/
	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*cloud,*cloud,index);
	float down_sam = 0.05;
	pcl::VoxelGrid<PointT> grid;
	grid.setLeafSize (down_sam,down_sam,down_sam);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
//	grid.setInputCloud(cloud_out);
//	grid.filter(*cloud_out);
	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.1);
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// SHOT estimation object.
	pcl::SHOTEstimation<PointT, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(cloud);
	shot.setInputNormals(normals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.5);

	shot.compute(*descriptors);

}
