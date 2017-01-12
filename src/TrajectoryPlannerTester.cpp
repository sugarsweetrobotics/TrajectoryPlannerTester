// -*- C++ -*-
/*!
 * @file  TrajectoryPlannerTester.cpp
 * @brief Trajectory Planner Test Component 
 * @date $Date$
 *
 * $Id$
 */

#include "TrajectoryPlannerTester.h"

// Module specification
// <rtc-template block="module_spec">
static const char* trajectoryplannertester_spec[] =
  {
    "implementation_id", "TrajectoryPlannerTester",
    "type_name",         "TrajectoryPlannerTester",
    "description",       "Trajectory Planner Test Component ",
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
TrajectoryPlannerTester::TrajectoryPlannerTester(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_collisionPort("collision"),
    m_kinematicsPort("kinematics")    

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
TrajectoryPlannerTester::~TrajectoryPlannerTester()
{
}



RTC::ReturnCode_t TrajectoryPlannerTester::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_collisionPort.registerConsumer("Manipulation_CollisionDetectionService", "Manipulation::CollisionDetectionService", m_collisionDetectionService);
  m_kinematicsPort.registerConsumer("Manipulation_KinematicSolverService", "Manipulation::KinematicSolverService", m_kinematicsService);  
  
  // Set CORBA Service Ports
  addPort(m_collisionPort);
  addPort(m_kinematicsPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t TrajectoryPlannerTester::onExecute(RTC::UniqueId ec_id)
{
  std::string buffer;
  std::cout << "[TrajectoryPlannerTester] Input Command [collision|kinematics]:" << std::ends;
  std::cin >> buffer;
  if (buffer == "collision") {
    std::cout << "[TrajectoryPlannerTester] Start Collision Test." << std::endl;
    
    Manipulation::RobotIdentifier robotID;
    robotID.name = CORBA::string_dup("orochi");
    
    Manipulation::JointAngleSeq jointSeq;
    jointSeq.length(7);
    for(int i = 0;i < 7;i++) {
      jointSeq[i].data = 0;
    }
    jointSeq[1].data = 1.0;
    Manipulation::CollisionPairSeq_var collision;
    Manipulation::ReturnValue_var retval = m_collisionDetectionService->isCollide(robotID, jointSeq, collision);
    if (collision->length() > 0) {
      std::cout << "[TrajectoryPlannerTester] collide" << std::endl;
    } else {
      std::cout << "[TrajectoryPlannerTester] no collision" << std::endl;
    }
  }else if (buffer == "kinematics") {
    std::cout << "[TrajectoryPlannerTester] Start Kinematics Test." << std::endl;

    Manipulation::EndEffectorPose pose;
    pose.pose.position.x = 0.08867;
    pose.pose.position.y = -0.00525;
    pose.pose.position.z = 0.720;
    pose.pose.orientation.r = 0.1402;
    pose.pose.orientation.p  =0.4442;
    pose.pose.orientation.y = 0.2053;

    Manipulation::JointAngleSeq startJoints;
    startJoints.length(6);
    startJoints[0].data = -0.261;
    startJoints[1].data = -0.209;
    startJoints[2].data = 1.047;
    startJoints[3].data = 0.698;
    startJoints[4].data = 0.698;
    startJoints[5].data = -0.523;
      
    Manipulation::JointAngleSeq_var targetJoints;
    Manipulation::ReturnValue_var retval = m_kinematicsService->solveKinematics(pose, startJoints, targetJoints);
    if (retval->id == Manipulation::OK) {
      std::cout << "[TrajectoryPlannerTester] Solved." << std::endl;
      for(int i = 0;i < targetJoints->length();i++) {
	std::cout << " - " << targetJoints[i].data << std::endl;
      }
    } else {
      std::cout << "[TrajectoryPlannerTester] Error: " << (const char*)retval->message  << std::endl;
    }
    
  }
  
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t TrajectoryPlannerTester::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TrajectoryPlannerTester::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void TrajectoryPlannerTesterInit(RTC::Manager* manager)
  {
    coil::Properties profile(trajectoryplannertester_spec);
    manager->registerFactory(profile,
                             RTC::Create<TrajectoryPlannerTester>,
                             RTC::Delete<TrajectoryPlannerTester>);
  }
  
};


