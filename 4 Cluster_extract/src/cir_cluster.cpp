#include <pcl/point_types.h>
#include "cook_basis.h"
#include "cook_io.h"
#include "cook_seg.h"
#include "cook_geometry.h"

typedef pcl::PointXYZ PointT;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointP;
typedef pcl::PointCloud<pcl::PointXYZ>  PointC;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalP;
typedef pcl::PointCloud<pcl::Normal> NormalC;

int
main (int argc, char** argv)
{
    // Read in the cloud data
    PointP cloud (new PointC);
    loadCloud(argc,argv,cloud);
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    ShowCloud(cloud);
    int num_cluster = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    num_cluster = ExtractClusters(cloud,clusters,0.5,60000,10);

    for(int i=0; i< num_cluster;i++)
    {
        ShowCloud(clusters[i]);
    }
    return (0);
}
