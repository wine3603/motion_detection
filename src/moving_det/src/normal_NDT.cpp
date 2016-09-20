///////////////////////////normal_distributions_transform.cpp from PCL
//#include <iostream>
#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

#include<pcl/console/parse.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/integral_image_normal.h>
#include<pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <iostream>
#include <fstream>

typedef pcl::PointXYZI PointT;
int
main (int argc, char** argv)
{
  // Loading first scan of room.
  if (argc < 2){
  PCL_ERROR("input pcd less than 2\n");
  return (-1);
  }
  pcl::PointCloud<PointT>::Ptr target_cloud (new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT> (argv[1], *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file target.pcd \n");
    return (-1);
  }

  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, index);
  std::cout << "Loaded " << target_cloud->size () << " data points from target.pcd" << std::endl;
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(target_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
//  sor.filter(*target_cloud);
//  pcl::io::savePCDFileASCII("target.pcd", *target_cloud);
//  filter target pcd
//  pcl::PointCloud<PointT>::Ptr filtered_tar (new pcl::PointCloud<PointT>);
bool Filter_Tar;
  if (pcl::console::find_switch (argc, argv, "-f")){
	  Filter_Tar = true;
  }
  else Filter_Tar = false;
if (Filter_Tar){
  pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.02, 0.02, 0.02);
  approximate_voxel_filter.setInputCloud (target_cloud);
  approximate_voxel_filter.filter (*target_cloud);
//  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
//            << " data points from source.pcd" << std::endl;
  std::cout << "Filtered target pcd" << target_cloud->size () << " points " << std::endl;
}
  // Loading second scan of room from new perspective.
  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT> (argv[2], *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file source.pcd \n");
    return (-1);
  }
  
  pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, index);
  std::cout << "Loaded " << input_cloud->size () << " data points from source.pcd" << std::endl;
  pcl::StatisticalOutlierRemoval<PointT> sor1;
  sor1.setInputCloud(input_cloud);
  sor1.setMeanK(50);
  sor1.setStddevMulThresh(1.0);
//  sor1.filter(*input_cloud);
//  pcl::io::savePCDFile("source.pcd", *input_cloud);
//  std::cout << "saved " << input_cloud->size () << " data points from target.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
  pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
//  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
//            << " data points from source.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<PointT, PointT> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.2);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  if (pcl::console::find_switch (argc, argv, "-i")){
	  int iteration_times;
	  pcl::console::parse (argc, argv, "-i", iteration_times);
  ndt.setMaximumIterations (iteration_times);
  PCL_ERROR("Max iterate times %d \n", iteration_times);
  }
  else
  ndt.setMaximumIterations (30);


  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
	  float Rot_Z,t_x,t_y;
	  std::string saveFile;
	  bool saveOrNot;
  if (pcl::console::find_switch (argc, argv, "-r")){
	  Rot_Z = Rot_Z*3.1415926/180;
	  pcl::console::parse (argc, argv, "-r", Rot_Z);
  PCL_ERROR("rot_z %f \n", Rot_Z);
  }
  else Rot_Z = 0.0f;  
  if (pcl::console::find_switch (argc, argv, "-x")){
	  pcl::console::parse (argc, argv, "-x", t_x);
  PCL_ERROR("t_x %f \n", t_x);
  }
  else t_x = 0.0f;  
  if (pcl::console::find_switch (argc, argv, "-y")){
	  pcl::console::parse (argc, argv, "-y", t_y);
  PCL_ERROR("t_y %f \n", t_y);
  }
  else t_y = 0.0f;  
  // if save combined pcd, -s is the output name;
  if (pcl::console::find_switch (argc, argv, "-s")){
	  pcl::console::parse (argc, argv, "-s", saveFile);
	  saveFile += ".pcd";
	  saveOrNot = true;
	  std::cout<<"combined output.pcd is: "<<saveFile<<std::endl;
  }

  else saveOrNot = false;
  Eigen::AngleAxisf init_rotation (Rot_Z, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (t_x, t_y, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<PointT>::Ptr output_cloud (new pcl::PointCloud<PointT>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
  std::cout<<ndt.getFinalTransformation()<<std::endl;
  
  // Saving transformed input cloud.
  std::string source_tran_;
  source_tran_ = argv[2];
  source_tran_.erase(source_tran_.length()-4, 4);
  source_tran_ += "to";
  source_tran_ += argv[1];
 // source_tran_<<argv[2]<<"trans_2_"<<argv[1]<<".pcd";
  pcl::io::savePCDFileASCII (source_tran_, *output_cloud);
  std::cout<<"sou_pcd size :" <<output_cloud->size()<<std::endl;
  //save the combined pcd to target frame
  std::ofstream file("tf.txt");
if (file.is_open())
{
	// MatrixXf m = MatrixXf::Random(30,3);
	file << ndt.getFinalTransformation() << '\n';
	//  file << "m" << '\n' <<  colm(m) << '\n'	
}
file.close();
if(saveOrNot)
	pcl::io::savePCDFile(saveFile, *target_cloud + *output_cloud);
  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<PointT>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<PointT> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<PointT>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<PointT> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
