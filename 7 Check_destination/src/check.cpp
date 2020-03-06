#include "cook_basis.h"
#include "cook_seg.h"
#include "cook_io.h"
#include "cook_geometry.h"
#include "alignment.h"
#include <pcl/console/time.h>
#include <time.h>
#include <pcl/registration/icp.h>


typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointP;
typedef pcl::PointCloud<pcl::PointXYZ>  PointC;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalP;
typedef pcl::PointCloud<pcl::Normal> NormalC;

// check destination
// remove the plane of desktop and see if any clusters still exist in the place where the plate is to be placed
// ignore clusters with small number of points  
int
main (int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  loadCloud(argc,argv,cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
  PrePassThrough(cloud,cloud_passthrough,0,0.5,"z");
  DownSampleCloud(cloud_passthrough,cloud,0.003f); //0.003
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed (new pcl::PointCloud<pcl::PointXYZ>);
  RemoveOutlier(cloud,cloud_removed,50,1);
  cloud = cloud_removed;

  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  EstimateNormal(cloud,normal,50);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  SegmentPlane(cloud,normal,coefficients_plane,inliers_plane,0.02);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>);
  ExtractCloud(cloud,inliers_plane,plane,false);
  RemoveOutlier(plane,plane,50,1);

  PrePassThrough(cloud,cloud,-1.3*0.1,1.3*0.1,"x");
  PrePassThrough(cloud,cloud,-1.3*0.1,1.3*0.1,"y");
  ShowCloud(cloud);

  int num_cluster = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  num_cluster = ExtractClusters(cloud,clusters,0.005,60000,200);

  if(num_cluster >1)
  {
    std::cout << "This place has been occupied!" << std::endl;
    return 0;
  }
  else if(num_cluster <1)
  {
    std::cout << "Error!" << std::endl;
    return 0;
  }
  else
  {
    float error;
    error = distanceC2C(cloud,plane);
    std::cout << "error = " << error << std::endl;
    if(error <0.000010)
    {
      std::cout<< meanZ(cloud) << std::endl;
      return meanZ(cloud);
    }
    else
    {
      return 0;
    }
  }
}
