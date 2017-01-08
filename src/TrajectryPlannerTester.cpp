// -*- C++ -*-
/*!
 * @file  TrajectryPlannerTester.cpp
 * @brief Trajectry Planner Test Component 
 * @date $Date$
 *
 * $Id$
 */

#include "TrajectryPlannerTester.h"

// Module specification
// <rtc-template block="module_spec">
static const char* trajectryplannertester_spec[] =
  {
    "implementation_id", "TrajectryPlannerTester",
    "type_name",         "TrajectryPlannerTester",
    "description",       "Trajectry Planner Test Component ",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "MotionPlanning",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "1",

    // Widget
    "conf.__widget__.debug", "text",
    // Constraints

    "conf.__type__.debug", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
TrajectryPlannerTester::TrajectryPlannerTester(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_collisionPort("collision")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
TrajectryPlannerTester::~TrajectryPlannerTester()
{
}



RTC::ReturnCode_t TrajectryPlannerTester::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_collisionPort.registerConsumer("Manipulation_CollisionDetectionService", "Manipulation::CollisionDetectionService", m_collisionDetectionService);
  
  // Set CORBA Service Ports
  addPort(m_collisionPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TrajectryPlannerTester::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t TrajectryPlannerTester::onExecute(RTC::UniqueId ec_id)
{
  std::string buffer;
  std::cout << "[TrajectryPlannerTester] Input Command [collision]:" << std::ends;
  std::cin >> buffer;
  if (buffer == "collision") {
    std::cout << "[TrajectryPlannerTester] Start Collision Test." << std::endl;
    
    Manipulation::RobotIdentifier manipInfo;
    manipInfo.name = CORBA::string_dup("orochi");
    Manipulation::RobotJointInfo jointSeq;
    jointSeq.jointInfoSeq.length(7);
    for(int i = 0;i < 7;i++) {      
      jointSeq.jointInfoSeq[i].name = "";
      jointSeq.jointInfoSeq[i].jointAngle = 0;
      jointSeq.jointInfoSeq[i].jointDistance = 0;
      jointSeq.jointInfoSeq[i].linkLength = 0;
      jointSeq.jointInfoSeq[i].linkTwist = 0;
      jointSeq.jointInfoSeq[i].maxAngle = 0;
      jointSeq.jointInfoSeq[i].minAngle = 0;
    }

    jointSeq.jointInfoSeq[1].jointAngle = 1.0;
    Manipulation::CollisionInfo_var collision;
    bool collide = m_collisionDetectionService->isCollide(manipInfo, jointSeq, collision);
    std::cout << "[TrajectryPlannerTester] collide = " << collide << std::endl;
    if (collide) {
      std::cout << " - collide with " << collision->name << std::endl;
    }
  }
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t TrajectryPlannerTester::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectryPlannerTester::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void TrajectryPlannerTesterInit(RTC::Manager* manager)
  {
    coil::Properties profile(trajectryplannertester_spec);
    manager->registerFactory(profile,
                             RTC::Create<TrajectryPlannerTester>,
                             RTC::Delete<TrajectryPlannerTester>);
  }
  
};


