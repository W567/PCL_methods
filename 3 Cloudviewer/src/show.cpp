#include "cook_io.h"
#include "cook_basis.h"
#include "cook_seg.h"
#include "cook_geometry.h"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointP;
typedef pcl::PointCloud<pcl::PointXYZ>  PointC;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalP;
typedef pcl::PointCloud<pcl::Normal> NormalC;

int
main (int argc, char** argv)
{

  // Read in the cloud data
  PointP cloud (new PointC);
  PointP cloud1 (new PointC);
  loadCloud(argc,argv,cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  ShowCloud(cloud,0.05);


  return 0;
}
