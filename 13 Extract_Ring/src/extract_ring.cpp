#include "cook_basis.h"
#include "cook_seg.h"
#include "cook_io.h"
#include "cook_geometry.h"
#include "extra_func.h"
#include "alignment.h"
#include <pcl/console/time.h>
#include <time.h>
#include <pcl/registration/icp.h>


typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointP;
typedef pcl::PointCloud<pcl::PointXYZ>  PointC;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalP;
typedef pcl::PointCloud<pcl::Normal> NormalC;

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  loadCloud(argc,argv,cloud);

  const float voxel_grid_size = 0.003;//0.0026  //0.003
  DownSampleCloud(cloud,cloud,voxel_grid_size);

  // Remove Outliers from the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_voxel (new pcl::PointCloud<pcl::PointXYZ>);
  RemoveOutlier(cloud,cloud_sor_voxel,50,2);

  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  EstimateNormal(cloud_sor_voxel,normal,50);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  SegmentPlane(cloud_sor_voxel,normal,coefficients_plane,inliers_plane,0.01);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobase (new pcl::PointCloud<pcl::PointXYZ>);
  ExtractCloud(cloud_sor_voxel,inliers_plane,cloud_nobase,true);
  RemoveOutlier(cloud_nobase,cloud,50,0.2);



  pcl::PointCloud<pcl::PointXYZ>::Ptr ring (new pcl::PointCloud<pcl::PointXYZ>);

  ExtractRing(cloud, ring, 0.0204618,-0.00585903,0.348537,0.095,0.12);
  ShowCloud(ring);
  WriteCloud(ring,"ring.pcd");
  std::cout << "result : " << GetDirection(ring,0.0204618,-0.00585903,0.348537,0.0852831,0.01) << std::endl;

/*
  int a = -3.14/4.2;
  float b = -3.14 /4.2 ;
  int c = -3.14/2.2;
  float d = -3.14 / 2.2;
  std::cout << "-3/4 d = " << a << std::endl;
  std::cout << "-3/4 f = " << b << std::endl;
  std::cout << "-3/2 d = " << c << std::endl;
  std::cout << "-3/2 f = " << d << std::endl;

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud);

/*
  const float voxel_grid_size = 0.003;//0.0026  //0.003
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  DownSampleCloud(cloud,tempCloud,voxel_grid_size);
*/


//  std::cout << "result : " << GetDirection(cloud,0.010639,-0.00924377,0.347666,0.0949598,0.01) << std::endl;

  return (0);
}
