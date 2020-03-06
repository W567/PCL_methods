#ifndef __EXTRA_FUNC_H_
#define __EXTRA_FUNC_H_

#include "cook_seg.h"
#include "cook_basis.h"
#include "cook_io.h"


struct PlateInformation
{
  float x;           //0
  float y;           //1
  float radius;      //2
  float top2desk;    //3
  float bottom2deskC;//4
  //float topself;     //4
  float bottom2desk; //5
  float bottomself;  //6
  float angle;       //7
  int target;        //8
  float bottomselfC; //9

  float food;
  int order;
  int extracted;
};

void DeleteCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y, float radius, bool state = true);
void ExtractCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y,float radius,bool state = true);
void ExtractRing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y, float radius_min, float radius_max);
bool GetObstacleAngles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float x, float y, std::vector<angleRanges>& angles, float tolerance);
bool GetDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float x,float y,float z,float radius, float& angle,float tolerance);
float NormalizeAngle(float angle_in);
float HeightDetermination(float r);
void CheckOrderWithAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_angle, std::vector<PlateInformation>& infor_pre, std::vector<PlateInformation>& infor_af);
void checkFood(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,PlateInformation& infor);

bool GetSizeOrder(std::vector<PlateInformation>& infor);
void GetSequence(std::vector<PlateInformation>& infor_af);


#endif  //#define __EXTRA_FUNC_H_
