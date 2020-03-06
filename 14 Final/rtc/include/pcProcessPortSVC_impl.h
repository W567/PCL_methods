// -*-C++-*-
/*!
 * @file  pcProcessPortSVC_impl.h
 * @brief Service implementation header of pcProcessPort.idl
 *
 */

#include "InterfaceDataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "pcProcessPortSkel.h"

#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifndef PCPROCESSPORTSVC_IMPL_H
#define PCPROCESSPORTSVC_IMPL_H

/*!
 * @class ComPcProcessSVC_impl
 * Example class implementing IDL interface ComPcProcess
 */
class ComPcProcessSVC_impl: public virtual POA_ComPcProcess,
		public virtual PortableServer::RefCountServantBase {
private:
	// Make sure all instances are built on the heap by making the
	// destructor non-public
	//virtual ~ComPcProcessSVC_impl();

public:
	/*!
	 * @brief standard constructor
	 */
	ComPcProcessSVC_impl(
			RTC::PointCloud *m_p,
			RTC::InPort<RTC::PointCloud> *m_p_i,
			RTC::PointCloud *m_p2,
			RTC::InPort<RTC::PointCloud> *m_p_i2,
			RTC::PointCloud *m_p3,
			RTC::InPort<RTC::PointCloud> *m_p_i3,
			RTC::PointCloud *m_p4,
			RTC::InPort<RTC::PointCloud> *m_p_i4);

	ComPcProcessSVC_impl();
	/*!
	 * @brief destructor
	 */
	virtual ~ComPcProcessSVC_impl();

	// attributes and operations
	ComPcProcess::MultiPointCloud_slice* get_pointCloud(::CORBA::Short CameraBitMask, ::CORBA::Boolean& flag);
	::CORBA::Boolean save_pointCloud(::CORBA::Short CameraBitMask, const char* str);

	::CORBA::Long plate_modelling(const ::ComPcProcess::Matrix4X4F matrix_in);
  ::CORBA::Boolean single_plate_modelling(const ::ComPcProcess::Matrix4X4F matrix_in);
	::CORBA::Double height_determine(::CORBA::Double r);
	::CORBA::Double add_PointCloud(const ::ComPcProcess::Matrix4X4F matrix_in);
	::CORBA::Double check_destination(::CORBA::Double r, ::CORBA::Double x, ::CORBA::Double y);
	::CORBA::Long plate_recog(const ::ComPcProcess::Matrix4X4F matrix_in, ::ComPcProcess::plate_info information, ::CORBA::Double& r_max);
	::CORBA::Double get_FirstCloud(const ::ComPcProcess::Matrix4X4F matrix_in);


	// --------------------------------------------
	RTC::PointCloud				 *m_pointCloud_in;
	RTC::InPort<RTC::PointCloud> *m_pointCloud_inIn;
	RTC::PointCloud				 *m_pointCloud_in2;
	RTC::InPort<RTC::PointCloud> *m_pointCloud_in2In;
	RTC::PointCloud				 *m_pointCloud_in3;
	RTC::InPort<RTC::PointCloud> *m_pointCloud_in3In;
	RTC::PointCloud				 *m_pointCloud_in4;
	RTC::InPort<RTC::PointCloud> *m_pointCloud_in4In;

	// ---------------------------------------------
	static pcl::PointCloud<pcl::PointXYZ>::Ptr scene;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr scene_nobase_;
	static float height_ ;
};




#endif // PCPROCESSPORTSVC_IMPL_H
