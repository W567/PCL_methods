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
  std::cerr << "Loading...\n", tt.tic ();
  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  InputStream(object_templates,argv[1]);

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[2], *cloud);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";


  // Preprocess the cloud by...
  // ...removing distant points
  std::cerr << "Preparing...\n", tt.tic ();
  const float depth_limit = 0.4;
  PrePassThrough(cloud,cloud,0,depth_limit);
  //ShowCloud(cloud);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.003;//0.0026  //0.003
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  DownSampleCloud(cloud,tempCloud,voxel_grid_size);
  //ShowCloud(cloud);

  // Remove Outliers from the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_voxel (new pcl::PointCloud<pcl::PointXYZ>);
  RemoveOutlier(tempCloud,cloud_sor_voxel,50,2);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  std::cerr << "Extracting Plane...\n", tt.tic ();
  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  EstimateNormal(cloud_sor_voxel,normal,50);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  SegmentPlane(cloud_sor_voxel,normal,coefficients_plane,inliers_plane,0.01);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobase (new pcl::PointCloud<pcl::PointXYZ>);
  ExtractCloud(cloud_sor_voxel,inliers_plane,cloud_nobase,true);
  RemoveOutlier(cloud_nobase,cloud,50,0.2);
  //ShowCloud(cloud);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  //ShowCloud(cloud);

  int alig_model_count=0;
  std::vector<PlateInformation> infor;
  alig_model_count=AlignAllModels(cloud,object_templates,0.000030,infor);

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

  //ShowCloud(cloud);

  return (0);
}
