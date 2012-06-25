/*****************************************************************************
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*****************************************************************************/
/*!
  \file   servoInf.hh
  \brief  Provides the class that will read and write from the ROS system.

  More information on USARSim may be found at www.usarsim.sourceforge.net.
  USARSim provides a physics based 3D simulation of various robots, sensors,
  humans, and automation equipment. This interface allows full access to
  the simulation. This current factoring of the code was performed by
  Stephen Balakirsky and is based on code from Fred Proctor.

  \code CVS Status:
  $Author: dr_steveb $
  $Revision: $
  $Date: $
  \endcode

  \author Stephen Balakirsky
  \date   October 19, 2011
*/
#ifndef __servoInf__
#define __servoInf__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "genericInf.hh"
#include "simware.hh"
#include "usarsimInf.hh"


////////////////////////////////////////////////////////////////
// structures
////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// class
////////////////////////////////////////////////////////////////
class ServoInf:public GenericInf
{
public:
  ROBOT_TYPE botType;
  enum servoMutex
  {
    SERVO_SET_KEY = 101,
    SERVO_STAT_KEY
  };

    ServoInf ();
   ~ServoInf ();
  const UsarsimActuator *getActuator(unsigned int num);
  unsigned int getNumActuators();
  const std::string getPlatformName();
  const geometry_msgs::Vector3 getPlatformSize();
  int init (GenericInf * siblingIn);
  int msgOut ();
  int msgIn ();
  int peerMsg (sw_struct * sw);
  
private:
  std::string odomName;
  static void *servoSetMutex;
  //  ros::Rate *loopRate;
  ros::NodeHandle n;
  UsarsimPlatform *basePlatform;
  UsarsimGrdVeh grdVehSettings;
  UsarsimSensor sensorSettings;
 
  
  //! We will always need a transform
  tf::TransformBroadcaster rosTfBroadcaster;
  //! Actuators
  std::vector < UsarsimActuator > actuators;
  //! Odometry sensors 
  std::vector < UsarsimOdomSensor > odometers;
  //! Range scanner sensors
  std::vector < UsarsimRngScnSensor > rangeScanners;
  //! Object sensors
  std::vector < UsarsimObjectSensor > objectSensors;

  int actuatorIndex (std::vector < UsarsimActuator > &actuatorsIn,
		     std::string name);
  int odomSensorIndex (std::vector < UsarsimOdomSensor > &sensors,
		       std::string name);
  int rangeSensorIndex (std::vector < UsarsimRngScnSensor > &sensors,
			std::string name);
  int objectSensorIndex (std::vector < UsarsimObjectSensor > &sensors, 
  	std::string name);
  int copyActuator (UsarsimActuator * sen, const sw_struct * sw);
  int copyObjectSensor(UsarsimObjectSensor * sen, const sw_struct *sw);
  int copyIns (UsarsimOdomSensor * sen, const sw_struct * sw);
  int copyGrdVehSettings (UsarsimGrdVeh * settings, const sw_struct * sw);
  int copyRangeScanner (UsarsimRngScnSensor * sen, const sw_struct * sw);
  int copyStaticVehSettings (UsarsimPlatform * settings, const sw_struct * sw);
  void VelCmdCallback (const geometry_msgs::TwistConstPtr & msg);
  int updateActuatorTF(UsarsimActuator *act, const sw_struct *sw);
  void updateActuatorCycle(UsarsimActuator *act);
  bool updateTrajectory(UsarsimActuator *act, const sw_struct *sw);
  bool checkTrajectoryGoal(UsarsimActuator *act, const sw_struct *sw);
  
};

#endif
