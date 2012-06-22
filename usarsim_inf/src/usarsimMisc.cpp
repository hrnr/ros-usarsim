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
  \file   usarsimMisc.cpp
  \brief  provides misc classes that are needed by usarsim. This includes:
  
  UsarsimList: Class for maintaining lists of a particular type of sensor.
  UsarsimGrdVeh: Class for reprenting ground vehicle parameters
  UsarsimFlippers: Class for representing flipper parameters
  UsarsimPlatform: General platform class

  \code CVS Status:
  $Author: dr_steveb $
  $Revision: $
  $Date: $
  \endcode

  \author Stephen Balakirsky
  \date   October 19, 2011
*/
#include "usarsimMisc.hh"

////////////////////////////////////////////////////////////////////////
// TrajectoryPoint
////////////////////////////////////////////////////////////////////////
TrajectoryPoint::TrajectoryPoint()
{
}


////////////////////////////////////////////////////////////////////////
// UsarsimList
////////////////////////////////////////////////////////////////////////
UsarsimList::UsarsimList (int typeIn)
{
  sw.time = 0;
  sw.op = SW_NONE;
  sw.type = typeIn;
  sw.name = "";
  didConfMsg = 0;
  didGeoMsg = 0;
}

void
UsarsimList::setName (const char *name)
{
  sw.name = name;
}

UsarsimList *
UsarsimList::classFind (std::string name)
{
  UsarsimList *ptr;

  ptr = this;
  while (ptr->sw.name != "")
    {
      if (ptr->sw.name == name)
	{
	  /* found it */
	  return ptr;
	}
      ptr = ptr->next;
    }

  /* a new one-- fill in the terminal empty structure... */
  ptr->sw.name = name;
  ptr->didConfMsg = 0;
  ptr->didGeoMsg = 0;

  /* ...and get a new terminal empty structure */
  ptr->next = new UsarsimList (ptr->sw.type);
  return ptr;
}

////////////////////////////////////////////////////////////////////////
// UsarsimPlatform
////////////////////////////////////////////////////////////////////////
UsarsimPlatform::UsarsimPlatform ()
{
  cycleTime = 0;
  platformName = "";
  platformSize.x = 0;
  platformSize.y = 0;
  platformSize.z = 0;
  mass = 0;
  cg.x = 0;
  cg.y = 0;
  cg.z = 0;
  steerType = SW_STEER_UNKNOWN;
}

////////////////////////////////////////////////////////////////////////
// UsarsimGrdVeh
////////////////////////////////////////////////////////////////////////
UsarsimGrdVeh::UsarsimGrdVeh ():UsarsimPlatform ()
{
  maxWheelRot = 0;
  maxTorque = 0;
  wheelSeparation = 0;
  wheelRadius = 0;
  wheelBase = 0;
  maxSteerAngle = 0;
  maxCrabAngle = 0;
  minTurningRadius = 0;
  flippers = NULL;
  numFlippers = 0;
}

////////////////////////////////////////////////////////////////////////
// UsarsimFlippers: Provides FlipperSettings
////////////////////////////////////////////////////////////////////////
UsarsimFlippers::UsarsimFlippers ()
{
  fType = FLIPPER_NONE_TYPE;
  minAngle = 0;;
  maxAngle = 0;
  length = 0;
  width = 0;
  flipperTrans.translation.x = 0;
  flipperTrans.translation.y = 0;
  flipperTrans.translation.z = 0;
  flipperTrans.rotation = tf::createQuaternionMsgFromYaw (0.);
}

////////////////////////////////////////////////////////////////////////
// UsarsimSensor
////////////////////////////////////////////////////////////////////////
UsarsimSensor::UsarsimSensor ()
{
  time = 0;
}

////////////////////////////////////////////////////////////////////////
// OdomSensor
////////////////////////////////////////////////////////////////////////
UsarsimOdomSensor::UsarsimOdomSensor ():UsarsimSensor ()
{
  lastPosition.linear.x = 0;
  lastPosition.linear.y = 0;
  lastPosition.linear.z = 0;
  lastPosition.angular.x = 0;
  lastPosition.angular.y = 0;
  lastPosition.angular.z = 0;
}

////////////////////////////////////////////////////////////////////////
// Range Scanner
////////////////////////////////////////////////////////////////////////
UsarsimRngScnSensor::UsarsimRngScnSensor ():UsarsimSensor ()
{
}

////////////////////////////////////////////////////////////////////////
// UsarsimConverter
////////////////////////////////////////////////////////////////////////
geometry_msgs::Vector3
UsarsimConverter::PointToVector(geometry_msgs::Point pointIn)
{
  geometry_msgs::Vector3 retValue;

  retValue.x = pointIn.x;
  retValue.y = pointIn.y;
  retValue.z = pointIn.z;
  return retValue;
}

geometry_msgs::Point
UsarsimConverter::VectorToPoint(geometry_msgs::Vector3 pointIn)
{
  geometry_msgs::Point retValue;

  retValue.x = pointIn.x;
  retValue.y = pointIn.y;
  retValue.z = pointIn.z;
  return retValue;
}

////////////////////////////////////////////////////////////////////////
// Actuator
////////////////////////////////////////////////////////////////////////
UsarsimActuator::UsarsimActuator (GenericInf *parentInf):UsarsimSensor ()
{
  infHandle = parentInf;
  
  trajectoryServer = NULL;
  // initialize cycle timer to contain 5 values
  cycleTimer.cycleDeque.clear();
  for( int count=0; count<5; count++)
    cycleTimer.cycleDeque.push_back(0.);
  cycleTimer.lastTime = ros::Time::now();
  cycleTimer.cycleTime = 0;
}

UsarsimActuator::~UsarsimActuator()
{
	if(trajectoryServer)
	{
		delete trajectoryServer;
		trajectoryServer = NULL;
	}
}

void UsarsimActuator::trajectoryCallback()
{
  control_msgs::FollowJointTrajectoryGoal newGoal = *(trajectoryServer->acceptNewGoal());
  ros::Time currentTime = ros::Time::now();
  TrajectoryPoint goal;
    for(unsigned int pointCount=0; pointCount<newGoal.trajectory.points.size(); pointCount++)
    {
		goal.numJoints = newGoal.trajectory.joint_names.size();
		for(unsigned int jointCount=0; jointCount<goal.numJoints; jointCount++)
		{
			goal.jointGoals[jointCount] = newGoal.trajectory.points[pointCount].positions[jointCount];
			if(newGoal.goal_tolerance.size() > jointCount)
			goal.tolerances[jointCount] = newGoal.goal_tolerance[jointCount].position;
			else
			goal.tolerances[jointCount] = 0.1; //default should be set through parameter
		}
		goal.time = currentTime + newGoal.trajectory.points[pointCount].time_from_start;
		currentTrajectory.goals.push_back(goal);
    }
      
    for(unsigned int jointCount=0; jointCount<goal.numJoints; jointCount++)
      {
	currentTrajectory.finalGoal.jointGoals[jointCount] = goal.jointGoals[jointCount]; 
	currentTrajectory.finalGoal.tolerances[jointCount] = goal.tolerances[jointCount];
      }
	
    goal = currentTrajectory.goals.front();
    currentTrajectory.goals.pop_front();

    sw_struct sw;
    sw.type = SW_ROS_CMD_TRAJ;
    sw.name = name;
    sw.data.roscmdtraj.number = goal.numJoints;
    for(unsigned int i = 0; i<goal.numJoints; i++)
    {
		// send first goal point to the robotic arm
		sw.data.roscmdtraj.goal[i] = goal.jointGoals[i];
    }
    infHandle->sibling->peerMsg(&sw);
}

void UsarsimActuator::setUpTrajectory()
{
	trajectoryServer = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(name + "_controller/follow_joint_trajectory/", false);
	if(trajectoryServer)
	{
		trajectoryServer->registerGoalCallback(boost::bind(&UsarsimActuator::trajectoryCallback, this));
		trajectoryServer->start();
	}
}
bool UsarsimActuator::isTrajectoryActive()
{
	if(trajectoryServer)
		return trajectoryServer->isActive();
	return false;
}
void UsarsimActuator::setTrajectoryResult(control_msgs::FollowJointTrajectoryResult result)
{
	if(trajectoryServer)
	{
		if(result.error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
			trajectoryServer->setSucceeded(result);
		else
			trajectoryServer->setAborted(result);
	}
}