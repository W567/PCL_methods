#include "cook_basis.h"
#include "cook_seg.h"
#include "cook_io.h"
#include "cook_geometry.h"
#include "alignment.h"
#include <pcl/console/time.h>
#include <time.h>
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointP;
typedef pcl::PointCloud<pcl::PointXYZ>  PointC;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalP;
typedef pcl::PointCloud<pcl::Normal> NormalC;

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{
  if (argc < 3)
  {
    printf ("No target PCD file given!\n");
    return (-1);
  }
  clock_t startTime,endTime;
  startTime= clock();
  pcl::console::TicToc tt;
  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  InputStream(object_templates,argv[1]);

  // Load the target cloud PCD file
  std::cerr << "Loading...\n", tt.tic ();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[2], *cloud);
  std::cerr << ">> Done: " << tt.toc () << " ms.\n";
/*
  // Preprocess the cloud by...
  // ...removing distant points
  std::cerr << "PassThrough,DownSample,RemoveOutlier...\n", tt.tic ();
  const float depth_limit = 0.4;
  PrePassThrough(cloud,cloud,0,depth_limit);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.003;//0.0026  //0.003
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  DownSampleCloud(cloud,tempCloud,voxel_grid_size);

  // Remove Outliers from the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_voxel (new pcl::PointCloud<pcl::PointXYZ>);
  RemoveOutlier(tempCloud,cloud_sor_voxel,50,2);
  cloud = cloud_sor_voxel;
  std::cerr << ">> Done: " << tt.toc () << " ms.\n";

*/

  int alig_model_count=0;
  std::vector<PlateInformation> infor;
  alig_model_count=AlignAllModels(cloud,object_templates,0.000015,infor);

  std::cout<<"the count of models that have been aligned: " << alig_model_count << std::endl;
  std::cout<<"plate information:" << std::endl;
  for(int i=0;i<alig_model_count;i++)
  {
    std::cout << " " <<std::endl;
    std::cout << "plate NO." << i << std::endl;
    std::cout<<"x: "<< infor[i].x << " cm " << std::endl;
    std::cout<<"y: "<< infor[i].y << " cm " << std::endl;
    std::cout<<"z: "<< infor[i].z << " cm " << std::endl;
    std::cout<<"r: "<< infor[i].radius << " cm " << std::endl;
  }

  pcl::io::savePCDFileBinary ("sence_after_extract.pcd", *cloud);
  endTime = clock();
  std::cout << "Total time : " << (double)(endTime-startTime)/ CLOCKS_PER_SEC << "s" << std::endl;
  ShowCloud(cloud);

  return (0);
}
