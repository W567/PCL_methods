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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  LoadCloud(argc,argv,cloud);
  TransformCloud(cloud,cloud,0,0,-0.005,"x",0);
  ShowCloud(cloud);
  WriteCloud(cloud,"model.pcd");

}
