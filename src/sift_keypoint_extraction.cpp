#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/impl/filter.hpp>

int
main(int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1) // load the file
  {
  PCL_ERROR ("Couldn't read file\n");
  return -1;
  }
  std::cout << "points: " << cloud->size () <<std::endl;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  std::cout << "points: " << cloud->size () <<std::endl;

  // Parameters for sift computation
//   const float min_scale = 0.1f;
  const float min_scale = 0.05f;
  const int n_octaves = 6;
  const int n_scales_per_octave = 10;
  const float min_contrast = 0.5f;
//   const float min_contrast = 0.0f;
  
  
  // Estimate the sift interest points using Intensity values from RGB values
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud);
  sift.compute(result);
  
  // Copying the pointwithscale to pointxyz so as visualize the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(result, *cloud_temp);

  // Saving the resultant cloud 
  std::cout << std::endl << "Resulting sift points are of size: " << cloud_temp->size () <<std::endl;
//   pcl::io::savePCDFileASCII("sift_points.pcd", *cloud_temp);

  

  // Visualization of keypoints along with the original cloud
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud, 255, 255, 0);
  viewer.setBackgroundColor( 1.0, 1.0, 1.0 );
  viewer.addPointCloud(cloud, "cloud");
  viewer.addCoordinateSystem();
//   viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
//   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  while(!viewer.wasStopped ())
  {
  viewer.spinOnce ();
  }
  

  
  return 0;
  
}