#include<pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include<pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/console/parse.h>
#include<fstream>
//#include<Eigen/dense>
typedef pcl::PointXYZI PointT;

int main(int argc, char** argv)

{
pcl::PointCloud<PointT>::Ptr combine(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	if (pcl::console::find_switch (argc, argv, "-cut"))
{
pcl::io::loadPCDFile<PointT>(argv[1], *combine);
pcl::PointCloud<PointT> cloud;
for(int i =0; i< combine->size();i++)
{
PointT p = combine->at(i);
if (p.x>0.5&&p.x<5)
cloud.points.push_back(p);
}
pcl::io::savePCDFile("save.pcd",cloud);
return -1;
}
	int i = argc;
	pcl::PointCloud<PointT>::Ptr combine(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	/// if "-tf", using the tf.txt to trans cloud2 to cloud1's frame and then
	//combine cloud2 to cloud1's frame Note that tf.txt should be the tf frome 2->1
	if (pcl::console::find_switch (argc, argv, "-tf")){
	pcl::PointCloud<PointT>::Ptr cloud_1 (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_2 (new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile<PointT>(argv[1], *cloud_1);
		pcl::io::loadPCDFile<PointT>(argv[2], *cloud_2);
		//	Eigen::Transform< Scalar, 3, Eigen::Affine >  	transform; 
	
		Eigen::Matrix4f matrix;
		std::ifstream in;
		char output[16];
		float Tar[16];
		in.open("tf.txt");
		if (in.is_open()) {
			int i =0;
			while (!in.eof()) {
				in >> output;
				float temp = atof(output);
				Tar[i] = temp;
				i++;
			}
		}
		in.close();
		matrix << Tar[0], Tar[1], Tar[2], Tar[3], 
		       Tar[4],Tar[5], Tar[6], Tar[7],
		       Tar[8], Tar[9], Tar[10], Tar[11],
		       Tar[12], Tar[13],  Tar[14], Tar[15];
		std::cout<<matrix<<std::endl;


		Eigen::Matrix4f rot;
	rot<< 	       Tar[0],Tar[1], Tar[2], 
		       Tar[4], Tar[5], Tar[6], 
		       Tar[12], Tar[9],  Tar[10]; 


	pcl::transformPointCloud (*cloud_2, *cloud_2, matrix);
	*cloud_1 += *cloud_2;

	pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05);
	approximate_voxel_filter.setInputCloud (cloud_1);

		PCL_ERROR("before filterd....%d \n", cloud_1->size());
	approximate_voxel_filter.filter (*cloud_1);
		PCL_ERROR("after filterd....%d \n", cloud_1->size());
	pcl::io::savePCDFile("com_using_tf.pcd", *cloud_1);
	}
	//pcl::console::parse (argc, argv, "-tf", iteration_times);


	for(int j = 1; j<i; j++){
		if(pcl::io::loadPCDFile<PointT>(argv[j], *cloud) == -1)	{
			PCL_ERROR("wrong.....\n");
			return (-1);
		} 
		*combine += *cloud;
		cloud->clear();
		PCL_ERROR("....%d.of %d \n", j,i);
	}

	pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05);
	approximate_voxel_filter.setInputCloud (combine);

		PCL_ERROR("before filterd....%d \n", combine->size());
	approximate_voxel_filter.filter (*combine);
		PCL_ERROR("after filterd....%d \n", combine->size());
	
/*     	pcl::StatisticalOutlierRemoval<PointT> sor;
	    sor.setInputCloud(combine);
	     sor.setMeanK(20);
	     sor.setStddevMulThresh(1.0);
	    sor.filter(*combine);

		PCL_ERROR("remove outlier....%d \n", combine->size());
*/
	pcl::io::savePCDFile("combined_pcd.pcd", *combine);
	PCL_INFO("combine pcd size %d from %d pcd \n", combine->size(), i-1);
}
