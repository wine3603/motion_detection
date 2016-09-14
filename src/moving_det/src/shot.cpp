////// compute SHOT descriptor for PCD registration//////////
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/features/shot.h>
#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/cloud_viewer.h>	
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
typedef  pcl::PointXYZI PointT;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;
typedef  PointCloudT::Ptr  CloudPtr;
typedef pcl::SHOT352 DescriptorT;
typedef pcl::PointCloud<pcl::SHOT352>::Ptr ShotCloudT;
	 pcl::visualization::PCLVisualizer viewer("PCL Viewer");
CloudPtr DS_temp(new pcl::PointCloud<pcl::PointXYZI>);
ShotCloudT shot(CloudPtr &cloud)
{
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*cloud,*cloud,index);
PCL_ERROR("nan size %d \n",cloud->size());
  pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud);
      sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
	  sor.filter (*cloud);
PCL_ERROR("after filtered size %d \n",cloud->size());


	float down_sam = 0.1;
	CloudPtr DS_cloud(new PointCloudT);
	pcl::VoxelGrid<PointT> grid;
	grid.setLeafSize (down_sam,down_sam,down_sam);
	grid.setInputCloud(cloud);
	grid.filter(*DS_cloud);
PCL_ERROR("DS_SIZE %d \n",DS_cloud->size());
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setKSearch(20);
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	CloudPtr corner(new PointCloudT);
/*	float nx,ny,nz,curve;
	pcl::KdTreeFLANN<PointT> KDF_search;
	KDF_search.setInputCloud(cloud);
	std::vector<int> neigh_indices(10);
	std::vector<float> neigh_sqr_dists(10);
//	std::vector<int> corner_index;
	for(int i = 0; i<DS_cloud->size();i++){
		int found_neighs = KDF_search.nearestKSearch (DS_cloud->points[i], 50, neigh_indices, neigh_sqr_dists);
		normalEstimation.computePointNormal(*cloud, neigh_indices, nx,ny,nz,curve);
		pcl::Normal normal_pt;
		Eigen::Vector4f normal(nx,ny,nz,curve);
		flipNormalTowardsViewpoint (DS_cloud->points[i], 0.0, 0.0, 0.0, normal);
		normal_pt.normal_x = normal[0];
		normal_pt.normal_y = normal[1];
		normal_pt.normal_z = normal[2];
		normal_pt.curvature = curve;
		normals->push_back(normal_pt);
*/
	//		if (curve >= 0.02){
//			corner_index.push_back(i);
//		}
//		std::cout<<i<<" normal  "<<normal_pt<<std::endl;
//	}
	DS_temp->clear();
	DS_temp  = DS_cloud;
	// SHOT estimation object.
	pcl::SHOTEstimationOMP<PointT, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(DS_cloud);
	shot.setInputNormals(normals);
	shot.setSearchSurface (cloud);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.5);
	shot.compute(*descriptors);

	pcl::PointCloud<pcl::SHOT352>::Ptr corner_shot(new pcl::PointCloud<pcl::SHOT352>());
	for(int i = 0; i< DS_cloud->size();i++){
		if (normals->points[i].curvature > 0.1&&pcl_isfinite(descriptors->points[i].descriptor[0]))	
		{			corner_shot->push_back(descriptors->points[i]);
			corner->push_back(DS_cloud->points[i]);
		}
	}
	PCL_ERROR("corner_shot no %d of %d \n", corner_shot->size(), descriptors->size());
/*	pcl::KdTreeFLANN<PointT> corner_nei_search;
	corner_nei_search.setInputCloud(DS_cloud);
	std::vector<int> neigh_(30);
	std::vector<float> neigh_dists(30);
	for(int i=0; i<corner_index.size();i++){
		int index = corner_index.at(i);
		int found_neighs = KDF_search.nearestKSearch (DS_cloud->points[index],30, neigh_, neigh_dists);
		Eigen::VectorXf descriptor;
shot.computePointSHOT(index, neigh_, neigh_dists, descriptor);
//descriptors->push_back(descriptor);
	}
*/
// visualize normals
//	 viewer.addPointCloud<PointT>(cloud,"oricloud"); viewer.addPointCloudNormals<PointT,pcl::Normal>(DS_cloud, normals,1,0.1f,"normal");
pcl::visualization::PointCloudColorHandlerCustom<PointT> sou_color_handler (corner, 0, 0, 255);
viewer.addPointCloud (corner, sou_color_handler, "corner");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"corner");

	return corner_shot;//descriptors;
}

int main(int argc, char**argv){
	// Object for storing the point cloud.
	pcl::PointCloud<PointT>::Ptr target(new PointCloudT);
	pcl::PointCloud<PointT>::Ptr source(new PointCloudT);
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<PointT>(argv[1], *target) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<PointT>(argv[2], *source) != 0)
	{
		return -1;
	}
	 viewer.setBackgroundColor (0,0,0);
	ShotCloudT tar_shot = shot(target);
	CloudPtr tar_DS (new PointCloudT);
        *tar_DS	= *DS_temp;
	ShotCloudT sou_shot = shot(source);
	CloudPtr sou_DS (new PointCloudT);
        *sou_DS	= *DS_temp;

	//start corresponding
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
pcl::KdTreeFLANN<DescriptorT> match_search;
match_search.setInputCloud (tar_shot);

//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
for (size_t i = 0; i < sou_shot->size (); ++i)
{
	std::vector<int> tar_neigh_indices (1);
	std::vector<float> tar_neigh_sqr_dists (1);
	if (!pcl_isfinite (sou_shot->at (i).descriptor[0])) //skipping NaNs
	{
		continue;
	}
	int found_neighs = match_search.nearestKSearch (sou_shot->at (i), 1, tar_neigh_indices, tar_neigh_sqr_dists);
	if(found_neighs == 1 && tar_neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
	{
		pcl::Correspondence corr (tar_neigh_indices[0], static_cast<int> (i), tar_neigh_sqr_dists[0]);
		model_scene_corrs->push_back (corr);
	}
}
std::cout << "Correspondences found: " <<model_scene_corrs->size() << std::endl;
// visualize corrs
viewer.addPointCloud<PointT>(source, "sou");
pcl::visualization::PointCloudColorHandlerCustom<PointT> sou_color_handler (sou_DS, 0,255, 0);
viewer.addPointCloud (sou_DS, sou_color_handler, "sou_pt");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"sou_pt");
viewer.addPointCloud<PointT>(target,"tar");
PCL_ERROR("tar_ds size %d \n", tar_DS->size());

for (size_t j = 0; j < model_scene_corrs->size(); j++)
{
	std::stringstream ss_line;
	ss_line << "correspondence_line" <<"_" << j;
	PointT& model_point = tar_DS->at (model_scene_corrs->at(j).index_query);
	PointT& scene_point = sou_DS->at (model_scene_corrs->at(j).index_match);
//	std::cout<<tar_DS->at(model_scene_corrs->at(j).index_query);
//	std::cout<<sou_DS->at(model_scene_corrs->at(j).index_match)<<std::endl;

	//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
	viewer.addLine<PointT, PointT> (model_point, scene_point, 0, 255, 0, ss_line.str ());

}

while (!viewer.wasStopped ()) {
	viewer.spinOnce ();
}

}

