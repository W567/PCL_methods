#ifndef COOK_IO_H_
#define COOK_IO_H_

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <iostream>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <stdlib.h>

void ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name);
void WriteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name);
void ShowCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,int coor_size=1.0,int R=0,int G=0,int B=0);
int LoadCloud(int argc,char** argv,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name,int num, std::string ext=".pcd");

#endif //#ifndef COOK_IO_H_
