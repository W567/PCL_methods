/*
 * pointCloudLib.cpp
 *
 *  Created on: 2017/05/25
 *      Author: cook
 */

// ======================================================================
//
//	include
//
// =======================================================================
#include "pcProcessLib.h"

// ======================================================================
//
//	cvt point Cloud RTC -- PCL
//
// =======================================================================

void cvt_RTCpc_to_PCLpc(
		RTC::PointCloud& pc,
		pcl::PointCloud<pcl::PointXYZRGB>& pclPc) {

	pclPc.points.clear();
	const int size_mat = pc.points.length();
	for (int i = 0; i < size_mat; i++) {
		pcl::PointXYZRGB p;
		p.x = pc.points[i].point.x;
		p.y = pc.points[i].point.y;
		p.z = pc.points[i].point.z;
		p.b = pc.points[i].colour.b;
		p.g = pc.points[i].colour.g;
		p.r = pc.points[i].colour.r;
		pclPc.push_back(p);
	}
}
/////////////////////////////////////////
//For  pcl::PointCloud<pcl::PointXYZ>
int cvt_RTCpc_to_PCLpc(
		RTC::PointCloud& pc,
		pcl::PointCloud<pcl::PointXYZ>& pclPc) {

	int NumValidPoints = 0;
	pclPc.points.clear();
	const int size_mat = pc.points.length();
	for (int i = 0; i < size_mat; i++)
	{
		pcl::PointXYZ p;
		p.x = pc.points[i].point.x;
		p.y = pc.points[i].point.y;
		p.z = pc.points[i].point.z;

		NumValidPoints++;
		pclPc.push_back(p);
	}
	return NumValidPoints;
}

void cvt_PCLpc_to_RTCpc(
		pcl::PointCloud<pcl::PointXYZRGB>& pclPc,
		RTC::PointCloud& pc) {

	const int size_mat = pclPc.size();

	RTC::PointCloudPointList pclist = RTC::PointCloudPointList(size_mat);

	for (int i = 0; i < size_mat; i++) {

		pclist[i].point.x = pclPc.points[i].x;
		pclist[i].point.y = pclPc.points[i].y;
		pclist[i].point.z = pclPc.points[i].z;
		pclist[i].colour.r = pclPc.points[i].r;
		pclist[i].colour.g = pclPc.points[i].g;
		pclist[i].colour.b = pclPc.points[i].b;

	}
	pc.points = pclist;

}
/////////////////////////////////////////
//For  pcl::PointCloud<pcl::PointXYZ>
void cvt_PCLpc_to_RTCpc(
		pcl::PointCloud<pcl::PointXYZ>& pclPc,
		RTC::PointCloud& pc) {

	const int size_mat = pclPc.size();

	RTC::PointCloudPointList pclist = RTC::PointCloudPointList(size_mat);

	for (int i = 0; i < size_mat; i++) {

		pclist[i].point.x = pclPc.points[i].x;
		pclist[i].point.y = pclPc.points[i].y;
		pclist[i].point.z = pclPc.points[i].z;
		pclist[i].colour.r = 255;
		pclist[i].colour.g = 255;
		pclist[i].colour.b = 255;

	}
	pc.points = pclist;

}

// ======================================================================
//
//	Cut about z axis under __
//
// =======================================================================
void cut_pointCloud_z(
		pcl::PointCloud<pcl::PointXYZRGB>& cloud,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud_fi9lters,
		int cut_value) {

	pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud.makeShared());
	pass.setFilterFieldName("z");
	pass.setFilterLimits(cut_value, 1);
	//pass.setFilterLimitsNegative (true);
	pass.filter(cloud_filtered);

}
/////////////////////////////////////////
//For  pcl::PointCloud<pcl::PointXYZ>
void cut_pointCloud_z(
		pcl::PointCloud<pcl::PointXYZ>& cloud,
		pcl::PointCloud<pcl::PointXYZ>& cloud_fi9lters,
		int cut_value) {

	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud.makeShared());
	pass.setFilterFieldName("z");
	pass.setFilterLimits(cut_value, 1);
	//pass.setFilterLimitsNegative (true);
	pass.filter(cloud_filtered);

}
