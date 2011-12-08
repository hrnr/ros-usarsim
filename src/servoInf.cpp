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
  \file   servoInf.cpp
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
#include "servoInf.hh"

void
ServoInf::VelCmdCallback (const geometry_msgs::TwistConstPtr & msg)
{
  sw_struct sw;

  /*
  ROS_INFO ("servoInf received: <%f %f %f> <%f %f %f>",
	    msg->linear.x,
	    msg->linear.y,
	    msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
  */
  sw.type = SW_ROS_CMD_VEL;
  sw.data.roscmdvel.linearx = msg->linear.x;
  sw.data.roscmdvel.lineary = msg->linear.y;
  sw.data.roscmdvel.linearz = msg->linear.z;
  sw.data.roscmdvel.angularx = msg->angular.x;
  sw.data.roscmdvel.angulary = msg->angular.y;
  sw.data.roscmdvel.angularz = msg->angular.z;
  sibling->peerMsg (&sw);
  return;
}

ServoInf::ServoInf ():GenericInf ()
{
  botType = SW_ROBOT_UNKNOWN;
  // set platform pointer to something to avoid core dumps
  basePlatform = &grdVehSettings;
}

const UsarsimActuator*
ServoInf::getActuator( unsigned int num)
{
  if( num > actuators.size() )
    return NULL;
  return &actuators[num];
}

unsigned int
ServoInf::getNumActuators()
{
  return actuators.size();
}

const std::string
ServoInf::getPlatformName()
{
  return basePlatform->platformName;
}

const geometry_msgs::Vector3 
ServoInf::getPlatformSize()
{
  return basePlatform->platformSize;
}

/* The interface must be initialized prior to use.
   Still more to do in this routine!
*/
int
ServoInf::init (GenericInf * usarsimIn)
{
  if (!nh->getParam ("/usarsim/odomSensor", odomName))
    {
      odomName = std::string("");
      ROS_DEBUG ("Parameter /usarsim/odomSensor not set");
    }
  else
    ROS_DEBUG ("Parameter /usarsim/odomSensor: %s", odomName.c_str ());

  sibling = usarsimIn;
  servoSetMutex = ulapi_mutex_new (SERVO_SET_KEY);
  if (servoSetMutex == NULL)
    {
      ROS_ERROR ("Unable to create servoSetMutex");
      return -1;
    }
  ROS_INFO ("servoInf initialized");
  return 1;
}

int
ServoInf::peerMsg (sw_struct * sw)
{
  int num;

  switch (sw->type)
    {
    case SW_ACT:
      switch (sw->op)
	{
	case SW_ACT_STAT:
	  num = actuatorIndex (actuators, sw->name);
	  if( copyActuator( &actuators[num], sw ) )
	    actuators[num].pub.publish (actuators[num].jstate);
	  break;

	case SW_ACT_SET:
	  num = actuatorIndex (actuators, sw->name);
	  copyActuator( &actuators[num], sw );
	  //	  rosTfBroadcaster.sendTransform (actuators[num].tf);
	  //	  ROS_INFO ( "Act setting %d joints", actuators[num].jointTf.size() );
	  //	  for( unsigned int count=0; count<actuators[num].jointTf.size(); 
	  //	       count++ )
	  //	    rosTfBroadcaster.sendTransform (actuators[num].jointTf[count]);
	  break;

	default:
	  break;
	}
      break;

    case SW_SEN_INS:
      switch (sw->op)
	{
	case SW_SEN_INS_STAT:
	  ROS_DEBUG
	    ("Ins status for %s at time %f: %f,%f,%f %f,%f,%f",
	     sw->name.c_str (), sw->time, sw->data.ins.position.x,
	     sw->data.ins.position.y, sw->data.ins.position.z,
	     sw->data.ins.position.roll,
	     sw->data.ins.position.pitch,
	     sw->data.ins.position.yaw);

	  num = odomSensorIndex (odometers, sw->name);
	  if (copyIns (&odometers[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (odometers[num].tf);
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       odometers[num].tf.header.frame_id.c_str(),
		       odometers[num].tf.child_frame_id.c_str());
	      */
	    }
	  /*
	  ROS_INFO("Sending odometer message for %s <%f %f>", 
		   odometers[num].odom.header.frame_id.c_str (),
		   odometers[num].odom.pose.pose.position.x,
		   odometers[num].odom.pose.pose.position.y);
	  */
	  odometers[num].pub.publish (odometers[num].odom);
	  break;
	case SW_SEN_INS_SET:
	  ROS_DEBUG ("Ins settings for %s: %f %f,%f,%f %f,%f,%f",
		     sw->name.c_str (),
		     sw->data.ins.period,
		     sw->data.ins.mount.x,
		     sw->data.ins.mount.y,
		     sw->data.ins.mount.z,
		     sw->data.ins.mount.roll,
		     sw->data.ins.mount.pitch,
		     sw->data.ins.mount.yaw);
	  num = odomSensorIndex (odometers, sw->name);
	  if (copyIns (&odometers[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (odometers[num].tf);
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       odometers[num].tf.header.frame_id.c_str(),
		       odometers[num].tf.child_frame_id.c_str());
	      */
	    }
	  break;
	default:
	  break;
	}
      break;

    case SW_ROBOT_FIXED:
      switch (sw->op)
	{
	case SW_DEVICE_STAT:
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      botType = SW_ROBOT_STATIC_VEH;
	      // first time we know about the robot type
	      if (copyStaticVehSettings (basePlatform, sw) == 1)
		{
		  rosTfBroadcaster.sendTransform (basePlatform->tf);
		  /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   basePlatform->tf.header.frame_id.c_str(),
			   basePlatform->tf.child_frame_id.c_str(),
			   basePlatform->tf.transform.translation.x,
			   basePlatform->tf.transform.translation.y);
		  */
		}
	    }
	  else if (botType != SW_ROBOT_STATIC_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("Fixed Robot.stat overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	    rosTfBroadcaster.sendTransform (basePlatform->tf);
	    /*
	    ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
		     basePlatform->tf.header.frame_id.c_str(),
		     basePlatform->tf.child_frame_id.c_str(),
		     basePlatform->tf.transform.translation.x,
		     basePlatform->tf.transform.translation.y);
	    */
	    }
	  break;

	case SW_DEVICE_SET:
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      // first time we know about the robot type
	      botType = SW_ROBOT_STATIC_VEH;
	      if (copyStaticVehSettings (basePlatform, sw) == 1)
		{
		  rosTfBroadcaster.sendTransform (basePlatform->tf);
		  /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   basePlatform->tf.header.frame_id.c_str(),
			   basePlatform->tf.child_frame_id.c_str(),
			   basePlatform->tf.transform.translation.x,
			   basePlatform->tf.transform.translation.y);
		  */
		}
	      else
		{
		  ROS_ERROR ("Error copying robot status");
		}
	    }
	  else if (botType != SW_ROBOT_STATIC_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("StaticVehicle.set overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	      rosTfBroadcaster.sendTransform (basePlatform->tf);
	      /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   basePlatform->tf.header.frame_id.c_str(),
			   basePlatform->tf.child_frame_id.c_str(),
			   basePlatform->tf.transform.translation.x,
			   basePlatform->tf.transform.translation.y);
	      */
	    }
	  break;
	default:
	  ROS_WARN ("unknown sw operand for class %s with operand %d",
		    swTypeToString (sw->type), sw->op);
	  break;
	}
      break;

    case SW_ROBOT_GROUNDVEHICLE:
      switch (sw->op)
	{
	case SW_ROBOT_STAT:
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      botType = SW_ROBOT_GRD_VEH;
	      // first time we know about the robot type
	      if (copyGrdVehSettings (&grdVehSettings, sw) == 1)
		{
		  rosTfBroadcaster.sendTransform (grdVehSettings.tf);
		  /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   grdVehSettings.tf.header.frame_id.c_str(),
			   grdVehSettings.tf.child_frame_id.c_str(),
			   grdVehSettings.tf.transform.translation.x,
			   grdVehSettings.tf.transform.translation.y);
		  */
		}
	    }
	  else if (botType != SW_ROBOT_GRD_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("GroundVehicle.stat overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	    rosTfBroadcaster.sendTransform (grdVehSettings.tf);
	    /*
	    ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
		     grdVehSettings.tf.header.frame_id.c_str(),
		     grdVehSettings.tf.child_frame_id.c_str(),
		     grdVehSettings.tf.transform.translation.x,
		     grdVehSettings.tf.transform.translation.y);
	    */
	    }
	  break;

	case SW_ROBOT_SET:
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      // first time we know about the robot type
	      botType = SW_ROBOT_GRD_VEH;
	      if (copyGrdVehSettings (&grdVehSettings, sw) == 1)
		{
		  rosTfBroadcaster.sendTransform (grdVehSettings.tf);
		  /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   grdVehSettings.tf.header.frame_id.c_str(),
			   grdVehSettings.tf.child_frame_id.c_str(),
			   grdVehSettings.tf.transform.translation.x,
			   grdVehSettings.tf.transform.translation.y);
		  */
		}
	      else
		{
		  ROS_ERROR ("Error copying robot status");
		}
	    }
	  else if (botType != SW_ROBOT_GRD_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("GroundVehicle.set overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	      rosTfBroadcaster.sendTransform (grdVehSettings.tf);
	      /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   grdVehSettings.tf.header.frame_id.c_str(),
			   grdVehSettings.tf.child_frame_id.c_str(),
			   grdVehSettings.tf.transform.translation.x,
			   grdVehSettings.tf.transform.translation.y);
	      */
	    }
	  break;
	}
      break;

    case SW_SEN_RANGESCANNER:
      switch (sw->op)
	{
	case SW_SEN_RANGESCANNER_STAT:
	  ROS_DEBUG ("RangeScanner status for %s at time %f: ",
		     sw->name.c_str (), sw->time);
	  num = rangeSensorIndex (rangeScanners, sw->name);
	  if (copyRangeScanner (&rangeScanners[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (rangeScanners[num].tf);
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       rangeScanners[num].tf.header.frame_id.c_str(),
		       rangeScanners[num].tf.child_frame_id.c_str());
	      ROS_INFO("Sending rangescanner message for %s", sw->name.c_str ());
	      */
	      rangeScanners[num].pub.publish (rangeScanners[num].scan);
	    }
	  else
	    {
	      ROS_ERROR
		("RangeScanner error for %s: can't copy it.",
		 sw->name.c_str ());
	      return -1;
	    }
	  break;

	case SW_SEN_RANGESCANNER_SET:
	  ROS_DEBUG ("RangeScanner settings for %s mount: %s ",
		     sw->name.c_str (),
		     sw->data.rangescanner.mount.offsetFrom);
	  /*
	     ROS_DEBUG ("%f %f %f %f %f,%f,%f %f,%f,%f",
	     sw->data.rangescanner.minrange,
	     sw->data.rangescanner.maxrange,
	     sw->data.rangescanner.resolution,
	     sw->data.rangescanner.fov,
	     sw->data.rangescanner.mount.x,
	     sw->data.rangescanner.mount.y,
	     sw->data.rangescanner.mount.z,
	     sw->data.rangescanner.mount.roll,
	     sw->data.rangescanner.mount.pitch,
	     sw->data.rangescanner.mount.yaw);
	   */
	  num = rangeSensorIndex (rangeScanners, sw->name);
	  if (copyRangeScanner (&rangeScanners[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (rangeScanners[num].tf);
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       rangeScanners[num].tf.header.frame_id.c_str(),
		       rangeScanners[num].tf.child_frame_id.c_str());
	      */
	    }
	  else
	    {
	      ROS_ERROR
		("RangeScanner error for %s: can't copy it.",
		 sw->name.c_str ());
	      return -1;
	    }
	  break;
	default:
	  ROS_ERROR ("invalid operation: %d\n", sw->op);
	  return -1;
	  break;
	}
      break;





    default:
      ROS_WARN ("unknown sw class %s with operand %d",
		swTypeToString (sw->type), sw->op);
      break;
    }

  return 1;
}

int
ServoInf::msgIn ()
{
  ROS_INFO ("In servoInf msgIn");
  static ros::NodeHandle n;	// need new nodehandle for receivingmessages

  // manage subscriptions
  static ros::Subscriber sub =
    n.subscribe ("cmd_vel", 10, &ServoInf::VelCmdCallback, this);
  ROS_INFO ("servoInf going to spin");
  ros::spin ();
  return 1;
}

int
ServoInf::msgOut (void)
{
  ROS_INFO ("In servoInf msgOut");
  return 1;
}

ServoInf::~ServoInf ()
{
  if (servoSetMutex != NULL)
    {
      ulapi_mutex_delete (servoSetMutex);
      servoSetMutex = NULL;
    }
}

int
ServoInf::copyActuator (UsarsimActuator * act, const sw_struct * sw)
{
  ros::Time currentTime;
  tf::Quaternion quat;
  geometry_msgs::Quaternion quatMsg;
  geometry_msgs::TransformStamped currentJointTf;
  std::stringstream tempSS;
  tf::TransformListener tfListener(ros::Duration(10));
  geometry_msgs::PoseStamped linkPose, basePose;
  int invertZ = 0;
  double mountRoll, mountZ;

  currentTime = ros::Time::now ();
  act->jointTf.clear();
  currentJointTf.header.stamp = currentTime;
  if (!ulapi_strcasecmp (sw->data.actuator.mount.offsetFrom, "HARD") ||
      !ulapi_strcasecmp (sw->data.actuator.mount.offsetFrom,
			 basePlatform->platformName.c_str ()))
    {
      act->tf.header.frame_id = "base_link";
      act->jstate.header.frame_id = "base_link";
      invertZ = 1;
    }
  else
    {
      ROS_INFO( "actuator base being set to %s since platform is %s",
		sw->data.actuator.mount.offsetFrom,
		basePlatform->platformName.c_str());
      act->tf.header.frame_id = sw->data.actuator.mount.offsetFrom;
      act->jstate.header.frame_id = sw->data.actuator.mount.offsetFrom;
    }

  // compute transform for entire package
  mountRoll = sw->data.actuator.mount.roll;
  mountZ = sw->data.actuator.mount.z;
  if( invertZ )
    {
      mountRoll += M_PI;
      mountZ *= -1;
    }
  quat = tf::createQuaternionFromRPY ( mountRoll,
				      sw->data.actuator.mount.pitch,
				      sw->data.actuator.mount.yaw);
  tf::quaternionTFToMsg (quat, quatMsg);
  act->tf.transform.translation.x = sw->data.actuator.mount.x;
  act->tf.transform.translation.y = sw->data.actuator.mount.y;
  act->tf.transform.translation.z = mountZ;
  act->tf.transform.rotation = quatMsg;
  act->tf.header.stamp = currentTime;
  act->tf.child_frame_id = act->name;

  rosTfBroadcaster.sendTransform (act->tf);
  ROS_INFO( "sent transform from \"%s\" to \"%s\"", act->tf.header.frame_id.c_str(), act->tf.child_frame_id.c_str() );

  act->jstate.header.stamp = currentTime;
  act->jstate.position.clear();
  act->jstate.name.clear();
  for( int i=0; i<sw->data.actuator.number; i++ )
    {
      tempSS.str("");
      tempSS << i+1; // link is array index + 1;
      currentJointTf.child_frame_id = std::string("Link_") + tempSS.str ();
      if( sw->data.actuator.link[i].parent == 0 )
	currentJointTf.header.frame_id = act->tf.child_frame_id;
      else
	{
	  tempSS.str("");
	  tempSS << sw->data.actuator.link[i].parent;
	  currentJointTf.header.frame_id = std::string("Link_") + tempSS.str ();
	}
      quat = tf::createQuaternionFromRPY (sw->data.actuator.link[i].mount.roll,
					  sw->data.actuator.link[i].mount.pitch,
					  sw->data.actuator.link[i].mount.yaw);
      tf::quaternionTFToMsg (quat, quatMsg);

      linkPose.header.frame_id = currentJointTf.header.frame_id;
      linkPose.header.stamp = currentTime;
      linkPose.pose.position.x = sw->data.actuator.link[i].mount.x;
      linkPose.pose.position.y = sw->data.actuator.link[i].mount.y;
      linkPose.pose.position.z = sw->data.actuator.link[i].mount.z;
      linkPose.pose.orientation = quatMsg;
      basePose = linkPose;
      /*
      ROS_ERROR( "Frame: %s position: %f %f %f orientation: %f %f %f %f",
		 basePose.header.frame_id.c_str(), 
		 basePose.pose.position.x,
		 basePose.pose.position.y,
		 basePose.pose.position.z,
		 basePose.pose.orientation.x,
		 basePose.pose.orientation.y,
		 basePose.pose.orientation.z,
		 basePose.pose.orientation.w );
      */
		 
      currentJointTf.transform.translation = UsarsimConverter::PointToVector(basePose.pose.position);
      currentJointTf.transform.rotation = basePose.pose.orientation;
      rosTfBroadcaster.sendTransform (currentJointTf);
      //      return 0;

      act->jointTf.push_back(currentJointTf);

      // now create actuator message
      tempSS.str("");
      tempSS << i+1;
      act->jstate.name.push_back((std::string("Link_") + tempSS.str ()));
      act->jstate.position.push_back(sw->data.actuator.link[i].position);
    }	    
  //  ROS_ERROR( "CopyAct success!!" );
  return 1;
}

int
ServoInf::copyGrdVehSettings (UsarsimGrdVeh * settings, const sw_struct * sw)
{
  settings->steerType = sw->data.groundvehicle.steertype;
  // the platform name needs to be set to "base_link"
  // and we will need to change any sensor mounted to the 
  // platform to point to the "base_link"
  settings->platformName = std::string (sw->name);
  //  settings->platformName = std::string ("base_link");
  settings->platformSize.x = sw->data.groundvehicle.length;
  settings->platformSize.y = sw->data.groundvehicle.width;
  settings->platformSize.z = sw->data.groundvehicle.height;
  settings->mass = sw->data.groundvehicle.mass;
  settings->cg.x = sw->data.groundvehicle.cg.x;
  settings->cg.y = sw->data.groundvehicle.cg.y;
  settings->cg.z = sw->data.groundvehicle.cg.z;
  settings->maxWheelRot = sw->data.groundvehicle.max_speed;
  settings->maxTorque = sw->data.groundvehicle.max_torque;
  settings->wheelSeparation = sw->data.groundvehicle.wheel_separation;
  settings->wheelRadius = sw->data.groundvehicle.wheel_radius;
  settings->wheelBase = sw->data.groundvehicle.wheel_base;
  settings->maxSteerAngle = sw->data.groundvehicle.max_steer_angle;
  settings->minTurningRadius = sw->data.groundvehicle.min_turning_radius;
  return 1;
}

int
ServoInf::copyStaticVehSettings (UsarsimPlatform * settings, const sw_struct * sw)
{
  // the platform name needs to be set to "base_link"
  // and we will need to change any sensor mounted to the 
  // platform to point to the "base_link"
  settings->platformName = std::string (sw->name);
  return 1;
}

/* returns 1 if we need to send out a sensor tf message
   returns 0 if no sensor tf message needed
*/
int
ServoInf::copyIns (UsarsimOdomSensor * sen, const sw_struct * sw)
{
  int retValue;
  ros::Time currentTime;
  tf::Quaternion quat;
  geometry_msgs::Quaternion quatMsg;
  std::string sen_frame_id, sen_child_id;
  currentTime = ros::Time::now ();

  quat = tf::createQuaternionFromRPY (sw->data.ins.mount.roll,
				      sw->data.ins.mount.pitch,
				      sw->data.ins.mount.yaw);
  tf::quaternionTFToMsg (quat, quatMsg);
  // set how sensor is mounted to vehicle
  retValue = 1;
  if( sen->name == odomName )
    {
      basePlatform->tf.transform.translation.x = sw->data.ins.mount.x;
      basePlatform->tf.transform.translation.y = sw->data.ins.mount.y;
      basePlatform->tf.transform.translation.z = sw->data.ins.mount.z;
      basePlatform->tf.transform.rotation = quatMsg;
      //      basePlatform->tf.header.frame_id = sen->name.c_str ();
      basePlatform->tf.header.frame_id = "base_footprint";
      basePlatform->tf.header.stamp = currentTime;
      basePlatform->tf.child_frame_id = "base_link";
      //  basePlatform->tf.child_frame_id = basePlatform->platformName.c_str ();
      sen_child_id = std::string("base_footprint");
      sen_frame_id = std::string("odom");
    }
  else
    {
      sen_child_id = std::string("base_") + sen->name;
      sen_frame_id = sen->name;
    }
  // now set up the sensor
  sen->tf.transform.translation.x = sw->data.ins.position.x;
  sen->tf.transform.translation.y = sw->data.ins.position.y;
  sen->tf.transform.translation.z = sw->data.ins.position.y;
  quat = tf::createQuaternionFromRPY (sw->data.ins.position.roll,
				      sw->data.ins.position.pitch,
				      sw->data.ins.position.yaw);
  tf::quaternionTFToMsg (quat, quatMsg);
  sen->tf.transform.rotation = quatMsg;
  sen->tf.child_frame_id = sen_child_id;
  sen->tf.header.frame_id = sen_frame_id;
  sen->tf.header.stamp = currentTime;

  // odom message
  sen->odom.header.stamp = currentTime;
  sen->odom.header.frame_id = sen->tf.header.frame_id;
  sen->odom.child_frame_id = sen->tf.child_frame_id;

  // set the position
  sen->odom.pose.pose.position.x = sw->data.ins.position.x;
  sen->odom.pose.pose.position.y = sw->data.ins.position.y;
  sen->odom.pose.pose.position.z = sw->data.ins.position.z;
  sen->odom.pose.pose.orientation = quatMsg;

  // set the velocity
  double dt = sw->time - sen->time;
  geometry_msgs::Vector3 currentAngular;
  currentAngular.x = sw->data.ins.mount.roll;
  currentAngular.y = sw->data.ins.mount.pitch;
  currentAngular.z = sw->data.ins.mount.yaw;
  sen->odom.twist.twist.linear.x =
    (sw->data.ins.position.x - sen->lastPosition.linear.x) / dt;
  sen->odom.twist.twist.linear.y =
    (sw->data.ins.position.y - sen->lastPosition.linear.y) / dt;
  sen->odom.twist.twist.linear.z =
    (sw->data.ins.position.z - sen->lastPosition.linear.z) / dt;
  sen->odom.twist.twist.angular.x =
    (currentAngular.x - sen->lastPosition.angular.x) / dt;
  sen->odom.twist.twist.angular.y =
    (currentAngular.y - sen->lastPosition.angular.y) / dt;
  sen->odom.twist.twist.angular.z =
    (currentAngular.z - sen->lastPosition.angular.z) / dt;

  // set last position and time

  sen->lastPosition.linear.x = sen->odom.pose.pose.position.x;
  sen->lastPosition.linear.y = sen->odom.pose.pose.position.y;
  sen->lastPosition.linear.z = sen->odom.pose.pose.position.z;
  sen->lastPosition.angular = currentAngular;
  sen->time = sw->time;
  return retValue;
}

int
ServoInf::copyRangeScanner (UsarsimRngScnSensor * sen, const sw_struct * sw)
{
  int flipScanner = 1; // if set to 1, flip direction of range scanner pan
  ros::Time currentTime;
  tf::Quaternion quat;
  geometry_msgs::Quaternion quatMsg;
  quat = tf::createQuaternionFromRPY (sw->data.rangescanner.mount.roll,
				      sw->data.rangescanner.mount.pitch,
				      sw->data.rangescanner.mount.yaw);
  tf::quaternionTFToMsg (quat, quatMsg);

  currentTime = ros::Time::now ();

  sen->tf.transform.translation.x = sw->data.rangescanner.mount.x;
  sen->tf.transform.translation.y = sw->data.rangescanner.mount.y;
  sen->tf.transform.translation.z = sw->data.rangescanner.mount.z;
  sen->tf.transform.rotation = quatMsg;
  sen->tf.header.stamp = currentTime;
  //  sen->tf.header.frame_id = "Rangescanner";
  sen->tf.child_frame_id = sen->name;
  if (!ulapi_strcasecmp (sw->data.rangescanner.mount.offsetFrom, "HARD"))
    {
      sen->tf.header.frame_id = "base_link";
    }
  else if( !ulapi_strcasecmp (sw->data.rangescanner.mount.offsetFrom,
			      basePlatform->platformName.c_str ()))
    {
      sen->tf.header.frame_id = "base_link";
    }
  else
    {
      ROS_INFO( "laser base being set to %s since platform is %s",
		sw->data.rangescanner.mount.offsetFrom,
		basePlatform->platformName.c_str());
      sen->tf.header.frame_id = sw->data.rangescanner.mount.offsetFrom;
    }

  sen->scan.header.stamp = currentTime;
  //  sen->scan.header.frame_id = sen->tf.header.frame_id;
  sen->scan.header.frame_id = sen->name;
  sen->scan.angle_min =  -sw->data.rangescanner.fov / 2.;
  sen->scan.angle_max =  sw->data.rangescanner.fov / 2.;
  sen->scan.angle_increment = sw->data.rangescanner.resolution;
  sen->scan.time_increment = 0;	// (1 / laser_frequency) / (num_readings);
  sen->scan.range_min = sw->data.rangescanner.minrange;
  sen->scan.range_max = sw->data.rangescanner.maxrange;

  //  sen->scan.set_ranges_size((unsigned int)sw->data.rangescanner.number);
  //  sen->scan.set_intensities_size((unsigned int)0);
  sen->scan.ranges.clear();
  sen->scan.intensities.clear();
  if( flipScanner )
    {
      for (int i = sw->data.rangescanner.number-1; i>= 0; i--)
	{
	  sen->scan.ranges.push_back (sw->data.rangescanner.range[i]);
	}
    }
  else
    {
      for (int i = 0; i < sw->data.rangescanner.number; i++)
	{
	  sen->scan.ranges.push_back (sw->data.rangescanner.range[i]);
	}
    }
  return 1;
}


/*
  Each general data array sensor (tachometer, odometer, etc.) uses one
  of the several SensorData structures.  These data and functions
  determine which index in the array of SensorData structures is
  assigned to which name.
*/
int
ServoInf::actuatorIndex (std::vector < UsarsimActuator > &actuatorsIn,
			   std::string name)
{
  unsigned int t;
  UsarsimActuator newActuator;
  std::string pubName;

  for (t = 0; t < actuatorsIn.size (); t++)
    {
      if (name == actuatorsIn[t].name)
	return t;		// found it
    }

  ROS_INFO ("Adding actuator: %s", name.c_str ());

  //unable to find the sensor, so must create it.
  newActuator.name = name;
  newActuator.time = 0;
  pubName = name + "_status";
  newActuator.pub = nh->advertise < sensor_msgs::JointState > (pubName.c_str (), 2);
  newActuator.tf.header.frame_id = "base_link";
  newActuator.tf.child_frame_id = name.c_str ();
  newActuator.jstate.header.frame_id = "base_link";
  actuatorsIn.push_back (newActuator);
  return actuatorsIn.size () - 1;
}

int
ServoInf::odomSensorIndex (std::vector < UsarsimOdomSensor > &sensors,
			   std::string name)
{
  unsigned int t;
  UsarsimOdomSensor newSensor;

  for (t = 0; t < sensors.size (); t++)
    {
      if (name == sensors[t].name)
	return t;		// found it
    }

  ROS_INFO ("Adding sensor: %s", name.c_str ());
  if( odomName == std::string(""))
    odomName = name;

  //unable to find the sensor, so must create it.
  newSensor.name = name;
  newSensor.time = 0;
  newSensor.pub = nh->advertise < nav_msgs::Odometry > (name.c_str (), 2);
  newSensor.tf.header.frame_id = "base_link";
  newSensor.tf.child_frame_id = name.c_str ();


  sensors.push_back (newSensor);
  return sensors.size () - 1;
}

int
ServoInf::rangeSensorIndex (std::vector < UsarsimRngScnSensor > &sensors,
			    std::string name)
{
  unsigned int t;
  UsarsimRngScnSensor newSensor;

  for (t = 0; t < sensors.size (); t++)
    {
      if (name == sensors[t].name)
	return t;		// found it
    }

  ROS_INFO ("Adding sensor: %s", name.c_str ());

  //unable to find the sensor, so must create it.
  newSensor.name = name;
  newSensor.time = 0;
  newSensor.pub = nh->advertise < sensor_msgs::LaserScan > (name.c_str (), 2);
  newSensor.tf.header.frame_id = "base_link";
  newSensor.tf.child_frame_id = name.c_str ();

  sensors.push_back (newSensor);
  return sensors.size () - 1;
}

void *
  ServoInf::servoSetMutex = NULL;
