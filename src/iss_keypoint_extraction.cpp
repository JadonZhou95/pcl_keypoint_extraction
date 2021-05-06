#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/fpfh.h>
#include <iostream>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>
// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
 
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;
 
		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;
 
	return resolution;
}
 
int
main(int argc, char** argv)
{
	// Objects for storing the point cloud and the keypoints.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
 
	// Read a PCD file from disk.
	std::string filename = std::string(argv[1]);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) != 0)
	{
		return -1;
	}
 
	// ISS keypoint detector object.
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
	detector.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	detector.setSearchMethod(kdtree);
	double resolution = computeCloudResolution(cloud);
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector.setSalientRadius(6 * resolution);
	// Set the radius for the application of the non maxima supression algorithm.
	detector.setNonMaxRadius(3.5 * resolution);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	// detector.setMinNeighbors(5);
    detector.setMinNeighbors(5);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector.setThreshold21(0.975);
    // detector.setThreshold21(0.90);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector.setThreshold32(0.975);
    // detector.setThreshold32(0.90);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector.setNumberOfThreads(4);
 
	detector.compute(*keypoints);
    std::cout << "Found " << keypoints->size() << " points." << std::endl;
	
	
/*	
	// Compute SHOT Descriptors (local)
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
	
	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(keypoints);
    normalEstimation.setKSearch(10);
    //normalEstimation.setRadiusSearch(0.03);
	kdtree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	
	
//	// SHOT estimation object.
//	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
//    shot.setInputCloud(keypoints);
//	shot.setInputNormals(normals);
//	// The radius that defines which of the keypoint's neighbors are described.
//	// If too large, there may be clutter, and if too small, not enough points may be found.
//    shot.setSearchMethod (kdtree);
//    shot.setRadiusSearch(0.05);
 
//	shot.compute(*descriptors);

    // FPFH estimation object.
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(keypoints);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    fpfh.setKSearch(15);

    fpfh.compute(*descriptors);
	
	
	
	std::size_t pos = filename.find(".pcd");
	std::string base_filename = filename.substr(0,pos);
	std::string keypoints_filename = base_filename + "_keypoints.pcd";
	std::string descriptors_filename = base_filename + "_descriptors.pcd";
	pcl::io::savePCDFileASCII(keypoints_filename, *keypoints);
	pcl::io::savePCDFileASCII(descriptors_filename, *descriptors);
*/	
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 0, 0);
  viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
  viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
  viewer.addPointCloud(keypoints, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  while(!viewer.wasStopped ())
  {
  viewer.spinOnce ();
  }
	
	
	
}