#include "cook_io.h"
#include "cook_basis.h"
#include "cook_seg.h"
#include "cook_geometry.h"
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>

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

  // Build a passthrough filter to remove spurious NaNs
  PointP cloud_passthrough (new PointC);
  PrePassThrough(cloud,cloud_passthrough,0,0.4,"z");

  //PointP cloud_downsample (new PointC);
  DownSampleCloud(cloud_passthrough,cloud,0.0016f); //0.003

//  PointP cloud_passthrough2 (new PointC);
//  PrePassThrough(cloud_passthrough,cloud_passthrough2,-0.12,0.1,"x");
  PointP cloud_removed (new PointC);
  RemoveOutlier(cloud,cloud_removed,50,0.3);
  cloud = cloud_removed;

  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  EstimateNormal(cloud,normal,50);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  SegmentPlane(cloud,normal,coefficients_plane,inliers_plane,0.015);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobase (new pcl::PointCloud<pcl::PointXYZ>);
  ExtractCloud(cloud,inliers_plane,cloud_nobase,true);
  RemoveOutlier(cloud_nobase,cloud,50,0.2);

  int num_cluster = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  num_cluster = ExtractClusters(cloud,clusters,0.005,60000,3000);

  if(num_cluster < 1)
  {
    std::cout << "No suitable model given!" << std::endl;
    return 0;
  }
  else
  {
    int num_model = 0;
    for(int i=0;i<num_cluster;i++)
    {
      DownSampleCloud(clusters[i],clusters[i],0.003f);
      std::vector<float> min_max;
      findMinMax(*clusters[i],min_max);
      float x_diff,y_diff,z_diff;
      x_diff = min_max[1] - min_max[0];
      y_diff = min_max[3] - min_max[2];
      z_diff = min_max[5] - min_max[4];
      std::cout << "check the target" << std::endl;
      if(x_diff<0.3 && abs(x_diff - y_diff)<0.02 && z_diff < 0.06)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr projection (new pcl::PointCloud<pcl::PointXYZ>);
        PCACloud(clusters[i],projection);

        int index=0;
        int temp = 0;
        for(int k = 0;k< projection->points.size();k++)
        {
          index = temp > projection->points[k].x ? index : k;
        }
        if(projection->points[index].z < 0)
        {
          TransformCloud(projection,projection, 0, 0, 0, "x", 3.14);
        }

        ShowCloud(projection);
        std::stringstream ss;
        ss << "model_" << num_model << ".pcd";
        WriteCloud(projection,ss.str());
        num_model++;
        std::cout << "find a model" << std::endl;
      }
    }
    return num_model;
  }
}
