#include "cook_basis.h"
#include "cook_seg.h"
#include "cook_io.h"
#include "cook_geometry.h"
#include "alignment.h"
#include <pcl/console/time.h>
#include <time.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>


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

  pcl::console::TicToc tt;
  std::cerr << "Loading...\n", tt.tic ();

  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  InputStream(object_templates,argv[1]);

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile (argv[2], *cloud);
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
  RemoveOutlier(cloud_nobase,cloud,50,2);
  //ShowCloud(cloud);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  //ShowCloud(cloud);

  std::vector<my_results> this_results;
  this_results.clear();
  my_results average_result;
  std::vector<my_results> one_result;
  one_result.clear();

  //int test_parameters[] = {1,5,10,15,20,25,30,35,40,45,50,55,60,70,80};  // feature and normal k
  //float test_parameters[] = {0.001f,0.003f,0.006f,0.01f,0.015f,0.02f,0.025f,0.03f,0.04f};   //feature and normal radius
  int test_parameters[] = {200};
  //int test_parameters[] = {75,100,120,150,200,250,300};  // iterator
  //float test_parameters[] = {0.01f,0.02f,0.03f,0.04f,0.05f,0.06f,0.07f,0.08f,0.09f,0.1f};     // min_sample_distance_
  //float test_parameters[] = {0.00001f,0.00005,0.0001,0.0005,0.001,0.003,0.008,0.01,0.015};  // max_correspondence_distance_

  //int test_parameters[] = {3,4,5,6,10};

  size_t cnt = sizeof(test_parameters)/sizeof(int);
  std::vector<int> parameters(test_parameters,test_parameters+cnt);


  for(int i=0; i < parameters.size();i++)
  {
    std::cout << " " << std::endl;
    std::cout << " " << std::endl;

    std::cout << "----------------------change feature_k_-------------------------" << std::endl;

    object_templates[0].processInput();
    //------------------------------------------------------------


    average_result.sac_score = 0.0;
    average_result.icp_score = 0.0;
    average_result.cost_time = 0.0;
    average_result.sac_score_stddev = 0.0;
    average_result.icp_score_stddev = 0.0;
    std::vector<float> temp_sac;
    std::vector<float> temp_icp;
    temp_sac.clear();
    temp_icp.clear();
    for(int j=0; j < 20; j++)
    {
//      std::cout << "calculating result for the " << j+1 << "th time." << std::endl;
      std::cout << "calculating results for parameter--" << i << " at the " << j << "th time. " << "-----------------" <<std::endl;
//      startTime= clock();
      AlignAllModels(cloud,object_templates,0.000040,one_result,parameters[i]);
      //AlignAllModels(cloud,object_templates,0.000040,one_result,1);
//      endTime = clock();
//      std::cout << "Total time : " << (double)(endTime-startTime)/ CLOCKS_PER_SEC << "s" << std::endl;
      //average_result.sac_score += one_result[j].sac_score;
      //average_result.icp_score += one_result[j].icp_score;
      average_result.cost_time += one_result[j].cost_time;
      temp_sac.push_back(one_result[j].sac_score);
      temp_icp.push_back(one_result[j].icp_score);
    }
//    average_result.sac_score /= 20;
//    average_result.icp_score /= 20;
    average_result.cost_time /= 20;

    double aver1,aver2;
    double dev1,dev2;

    pcl::getMeanStdDev(temp_sac,aver1,dev1);
    pcl::getMeanStdDev(temp_icp,aver2,dev2);

    average_result.sac_score = aver1;
    average_result.sac_score_stddev = dev1;
    average_result.icp_score = aver2;
    average_result.icp_score_stddev = dev2;

    this_results.push_back(average_result);
    one_result.clear();
    temp_sac.clear();
    temp_icp.clear();
  }

  std::cout << " " << std::endl;
  std::cout << "sac_score:" <<std::endl;
  for(int i=0; i<this_results.size();i++)
  {
    std::cout<< this_results[i].sac_score << std::endl;
  }


  std::cout << " " << std::endl;
  std::cout << "sac_score_stddev:" <<std::endl;
  for(int i=0; i<this_results.size();i++)
  {
    std::cout<< this_results[i].sac_score_stddev << std::endl;
  }



  std::cout << " " << std::endl;
  std::cout << "icp_score:" <<std::endl;
  for(int i=0; i<this_results.size();i++)
  {
    std::cout<< this_results[i].icp_score << std::endl;
  }


  std::cout << " " << std::endl;
  std::cout << "icp_score_stddev:" <<std::endl;
  for(int i=0; i<this_results.size();i++)
  {
    std::cout<< this_results[i].icp_score_stddev << std::endl;
  }


  std::cout << " " << std::endl;
  std::cout << "cost_time:" <<std::endl;
  for(int i=0; i<this_results.size();i++)
  {
    std::cout<< this_results[i].cost_time << std::endl;
  }

  //ShowCloud(cloud);

  return (0);
}
