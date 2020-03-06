/*
 * pointCloudLib.h
 *
 *  Created on: 2017/05/25
 *      Author: cook
 */

#ifndef INCLUDE_RTC_POINTCLOUDPROCESS_PCPROCESSLIB_H_
#define INCLUDE_RTC_POINTCLOUDPROCESS_PCPROCESSLIB_H_

// ======================================================================
//
//	include
//
// =======================================================================

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// ======================================================================
//
//	prototype declaration
//
// =======================================================================

void cvt_RTCpc_to_PCLpc(
		RTC::PointCloud& pc,
		pcl::PointCloud<pcl::PointXYZRGB>& pclPc);
int cvt_RTCpc_to_PCLpc(
		RTC::PointCloud& pc,
		pcl::PointCloud<pcl::PointXYZ>& pclPc);

void cvt_PCLpc_to_RTCpc(
		pcl::PointCloud<pcl::PointXYZRGB>& pclPc,
		RTC::PointCloud& pc);
void cvt_PCLpc_to_RTCpc(
		pcl::PointCloud<pcl::PointXYZ>& pclPc,
		RTC::PointCloud& pc);

void cut_pointCloud_z(
		pcl::PointCloud<pcl::PointXYZRGB>& cloud,
		pcl::PointCloud<pcl::PointXYZRGB>& cloud_fi9lters,
		int cut_value);
void cut_pointCloud_z(
		pcl::PointCloud<pcl::PointXYZ>& cloud,
		pcl::PointCloud<pcl::PointXYZ>& cloud_fi9lters,
		int cut_value);

#endif /* INCLUDE_RTC_POINTCLOUDPROCESS_PCPROCESSLIB_H_ */
