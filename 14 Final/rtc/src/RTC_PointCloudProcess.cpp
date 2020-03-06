// -*- C++ -*-
/*!
 * @file  RTC_PointCloudProcess.cpp
 * @brief RTC for PointCloud process
 * @date $Date$
 *
 * $Id$
 */

#include "RTC_PointCloudProcess.h"

// Module specification
// <rtc-template block="module_spec">
static const char* rtc_pointcloudprocess_spec[] =
  {
    "implementation_id", "RTC_PointCloudProcess",
    "type_name",         "RTC_PointCloudProcess",
    "description",       "RTC for PointCloud process",
    "version",           "2.0.0",
    "vendor",            "Keisuke Nagano, Dong chenyu",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RTC_PointCloudProcess::RTC_PointCloudProcess(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_command_inIn("command_in", m_command_in),
    m_pointCloud_inIn("pointCloud_in", m_pointCloud_in),
    m_pointCloud_in2In("pointCloud_in2", m_pointCloud_in2),
    m_pointCloud_in3In("pointCloud_in3", m_pointCloud_in3),
    m_pointCloud_in4In("pointCloud_in4", m_pointCloud_in4),
    m_command_outOut("command_out", m_command_out),
    m_value_outOut("value_out", m_value_out),
    m_svPcProcessPort("svPcProcess"),
    m_sv_pp(&m_pointCloud_in		,
            &m_pointCloud_inIn	,
            &m_pointCloud_in2	,
            &m_pointCloud_in2In,
            &m_pointCloud_in3	,
            &m_pointCloud_in3In,
            &m_pointCloud_in4	,
            &m_pointCloud_in4In	)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RTC_PointCloudProcess::~RTC_PointCloudProcess()
{
}



RTC::ReturnCode_t RTC_PointCloudProcess::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("command_in", m_command_inIn);
  addInPort("pointCloud_in", m_pointCloud_inIn);
  addInPort("pointCloud_in2", m_pointCloud_in2In);
  addInPort("pointCloud_in3", m_pointCloud_in3In);
  addInPort("pointCloud_in4", m_pointCloud_in4In);
  
  // Set OutPort buffer
  addOutPort("command_out", m_command_outOut);
  addOutPort("value_out", m_value_outOut);
  
  // Set service provider to Ports
  m_svPcProcessPort.registerProvider("sv_pp", "ComPcProcess", m_sv_pp);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_svPcProcessPort);


  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  std::cout<<"RTC_PointCloudProcess initial over."<<std::endl;
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RTC_PointCloudProcess::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RTC_PointCloudProcess::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RTC_PointCloudProcess::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RTC_PointCloudProcess::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void RTC_PointCloudProcessInit(RTC::Manager* manager)
  {
    coil::Properties profile(rtc_pointcloudprocess_spec);
    manager->registerFactory(profile,
                             RTC::Create<RTC_PointCloudProcess>,
                             RTC::Delete<RTC_PointCloudProcess>);
  }
  
};


