#ifndef __PRETREATMENT_H_
#define __PRETREATMENT_H_

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <string>
#include <iostream>
#include <iostream>
#include <vector>

void ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name);
void WriteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name);
void PrePassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float min,float max,std::string axis = "z");
void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float size = 0.003f);
void RemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, int meanK = 50, float thresh = 0.1);
void EstimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normal_out, int rangeK = 50);
void ShowCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,int coor_size=1.0,int R=0,int G=0,int B=0);
void SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_plane,pcl::PointIndices::Ptr& inliers_plane);
void SegmentCircle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_circle,pcl::PointIndices::Ptr& inliers_circle,float r_min,float r_max);
void ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointIndices::Ptr inliers_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,bool state = false);
void ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,std::vector<pcl::PointIndices> cluster_indices,float tolerance,int max_size = 25000,int min_size = 100);
void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float x,float y,float z,std::string axis,float angle);


#endif
