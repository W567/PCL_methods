#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <time.h>
int
 main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile ("output.pcd", *cloud_in);

  pcl::io::loadPCDFile ("model_nobase.pcd", *cloud_out);


  pcl::console::TicToc tt;
  std::cerr << "Loading...\n", tt.tic ();
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  icp.setMaximumIterations(50);
  icp.setRANSACIterations (50);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::io::savePCDFileBinary ("icp1.pcd", Final);
  std::cerr << ">> Done: " << tt.toc () << " ms.\n";

  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*cloud_in, transformed_cloud, icp.getFinalTransformation());
  pcl::io::savePCDFileBinary ("icp.pcd", transformed_cloud);


 return (0);
}
