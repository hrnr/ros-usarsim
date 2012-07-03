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
  \file   usarsim_urdf_gen.cpp
  \brief  Creates a urdf representation of the current robot from usarsim

  More information on USARSim may be found at www.usarsim.sourceforge.net.
  USARSim provides a physics based 3D simulation of various robots, sensors,
  humans, and automation equipment. This interface allows full access to
  the simulation. This current factoring of the code was performed by
  Stephen Balakirsky and is based on code from Fred Proctor.

  \code CVS Status:
  $Author: dr_steveb $
  $Revision: $
  $Date:  $
  \endcode

  \author Stephen Balakirsky
  \date   October 16, 2011
*/
#include <stdio.h>
#include "ros/ros.h"
#include "ulapi.hh"
#include "servoInf.hh"
#include "usarsimInf.hh"
//#include "geometry_msgs/QuaternionStamped.h"
//#include "LinearMath/btTransform.h"


void
rosThread (void *arg)
{
  ServoInf *servo = reinterpret_cast < ServoInf * >(arg);

  //  servo->this = ((RosThreadArgs*)arg)->thisPtr;
  servo->msgIn ();
  ROS_WARN ("Servo thread exited");
}
//adds an empty link to represent a robot component (i.e. effector or sensor)
void addComponentLink(const UsarsimSensor *sen, FILE *fp)
{
	fprintf(fp, "\t<link name = \"%s\">\n",sen->name.c_str());
	fprintf(fp, "\t</link>\n");
}
//adds a joint that links an effector or sensor to its parent
void addComponentParentJoint(const UsarsimSensor *sen, FILE *fp)
{
	fprintf(fp, "\t<joint name = \"%s_mount\" type = \"fixed\">\n", sen->name.c_str());
	fprintf(fp, "\t\t<parent link = \"%s\" />\n", sen->tf.header.frame_id.c_str());
	fprintf(fp, "\t\t<child link = \"%s\" />\n", sen->tf.child_frame_id.c_str());
	fprintf(fp, "\t\t<origin xyz = \"%f %f %f\" rpy = \"%f %f %f\" />\n",
	sen->tf.transform.translation.x, sen->tf.transform.translation.y, sen->tf.transform.translation.z,
	0.0,0.0,0.0);
	fprintf(fp, "\t</joint>\n");
}
int
main (int argc, char **argv)
{
  ServoInf *servo;		// servo level interface
  UsarsimInf *usarsim;		// usarsim interface
  ros::Time currentTime, startTime;
  double roll, pitch, yaw;
  tf::Quaternion bt_q;
  FILE *fp;
  std::string fileName;
  
  // init ros
  ros::init (argc, argv, "usarsim");
  //  ros::Rate r(60);

  servo = new ServoInf ();
  usarsim = new UsarsimInf ();

  // this code uses the ULAPI library to provide portability
  // between different operating systems and architectures
  if (ULAPI_OK != ulapi_init (UL_USE_DEFAULT))
    {
      ROS_FATAL ("can't initialize ulapi");
      return 1;
    }

  // initialize the ROS interface wrapper
  servo->init (usarsim);
  //make sure ROS publishes the TF tree for robot models so the urdf can be generated.
  servo->setBuildingTFTree();
  
  // initialize the USARSim interface wrapper
  usarsim->init (servo);

  // main loop
  startTime = ros::Time::now ();

  //ROS_INFO ("Waiting 5 sec. for system to stabilize\n");
  while ((usarsim->getNH ())->ok ())
    {
      if (usarsim->msgIn () != 1)
	{
	  ROS_ERROR ("Error from usarsimInf");
	}
      currentTime = ros::Time::now ();
      if( (currentTime.sec - startTime.sec ) > 5. )
	break;
      //else
	//ROS_INFO ("Waiting %f more seconds (cur: %d start: %d", (float)(5.-(currentTime.sec - startTime.sec )),
	//	  currentTime.sec, startTime.sec);
    }
  ROS_ERROR ("Generating urdf file...\n");
  fileName = servo->getPlatformName().c_str () + std::string(".xml");
  fp = fopen( fileName.c_str (), "w" );
  if( fp == NULL )
    {
      ROS_ERROR( "Unable to open urdf file" );
      return (-1);
    }

  const UsarsimActuator *actPt;
  double length = 0;
  geometry_msgs::Vector3 platformSize;
  int numJoints;
  fprintf( fp, "<\?xml version=\"1.0\"\?>\n" );
  fprintf( fp, "<robot name=\"%s\">\n", servo->getPlatformName().c_str () );
  //base robot link (empty)
  fprintf( fp, "\t<link name=\"base_link\" />\n");
  
  //loop through all actuators, adding link elements
  for( unsigned int i=0; i<servo->getNumActuators(); i++ )
  {
    actPt = servo->getActuator(i);
    platformSize = servo->getPlatformSize();
    numJoints = actPt->jointTf.size();
    for( unsigned int j=0; j<actPt->jointTf.size(); j++ )
    {
	 	fprintf( fp, "\t<link name =\"%s\">\n", actPt->jointTf[j].header.frame_id.c_str () );
	  	fprintf( fp, "\t\t<visual>\n");
      	fprintf( fp, "\t\t\t<geometry>\n");  
	    length = sqrt( (actPt->jointTf[j].transform.translation.x -
	      actPt->tf.transform.translation.x) *
	     (actPt->jointTf[j].transform.translation.x -
	      actPt->tf.transform.translation.x) +
	     (actPt->jointTf[j].transform.translation.y -
	      actPt->tf.transform.translation.y) *
	     (actPt->jointTf[j].transform.translation.y -
	      actPt->tf.transform.translation.y) +
	     (actPt->jointTf[j].transform.translation.z -
	      actPt->tf.transform.translation.z) *
	     (actPt->jointTf[j].transform.translation.z -
	      actPt->tf.transform.translation.z) );
	    pitch = atan2((actPt->jointTf[j].transform.translation.z - actPt->tf.transform.translation.z),
			  (actPt->jointTf[j].transform.translation.x - actPt->tf.transform.translation.x));
	    yaw = atan2((actPt->jointTf[j].transform.translation.y - actPt->tf.transform.translation.y),
			(actPt->jointTf[j].transform.translation.x - actPt->tf.transform.translation.x));
	    fprintf( fp, "\t\t\t\t<box size = \"%f 0.05 0.05\"/>\n",//"\t\t\t\t<cylinder length=\"%f\" radius =\".05\"/>\n",
		   length );
	    fprintf( fp, "\t\t\t</geometry>\n" );
	    fprintf( fp, "\t\t\t<origin xyz=\"%f %f %f\" rpy=\"%.2f %.2f %.2f\" />\n",
		   (actPt->jointTf[j].transform.translation.x -actPt->tf.transform.translation.x)/2.,
		   (actPt->jointTf[j].transform.translation.y -actPt->tf.transform.translation.y)/2.,
		   (actPt->jointTf[j].transform.translation.z -actPt->tf.transform.translation.z)/2.,
	           0., pitch, yaw );
	    fprintf( fp, "\t\t</visual>\n" );
	  	fprintf( fp, "\t</link>\n");
      }
      //last arm link: assume half the length, same rpy as previous link
      length = length/2;
      fprintf( fp, "\t<link name=\"%s_link%d\">\n", actPt->name.c_str(), numJoints);
      fprintf( fp, "\t\t<visual>\n");
      fprintf( fp, "\t\t\t<geometry>\n" );
      fprintf( fp, "\t\t\t\t<box size = \"%f 0.05 0.05\"/>\n",//"\t\t\t\t<cylinder length=\"%f\" radius =\".05\"/>\n",
		   length );
      fprintf( fp, "\t\t\t</geometry>\n" );
	  fprintf( fp, "\t\t\t<origin xyz=\"%f 0 0\" rpy=\"%.2f %.2f %.2f\" />\n",
		   length/2., 0.0,0.0,0.0);//roll, pitch, yaw );
	  fprintf( fp, "\t\t</visual>\n" );
	  fprintf( fp, "\t</link>\n");
	  
	  //tip link (used for actuator control)
	  fprintf( fp, "\t<link name = \"%s_tip\" />\n",actPt->name.c_str());
    }
    ROS_ERROR("Done with actuator links.");
    for(unsigned int i = 0;i<servo->getNumExtras();i++)
    {
    	addComponentLink(servo->getComponent(i), fp);
    }
    ROS_ERROR("Done with component links.");
    for(unsigned int i=0;i<servo->getNumActuators();i++)
    {
    	actPt = servo->getActuator(i);
      	platformSize = servo->getPlatformSize();
      	numJoints = actPt->jointTf.size();
      	//base joint
		fprintf( fp, "\t<joint name=\"%s_mount\" type=\"fixed\">\n", actPt->name.c_str());
		fprintf( fp, "\t\t<parent link=\"base_link\" />\n");
		fprintf( fp, "\t\t<child link=\"%s\" />\n", actPt->jointTf[0].header.frame_id.c_str());
		//add a line here for the offset from robot base to actuator base
		fprintf( fp, "\t</joint>\n");
    	for( unsigned int j=0; j<actPt->jointTf.size(); j++ )
		{
		  fprintf( fp, "\t<joint name=\"%s_joint_%d\" type=\"revolute\">\n",actPt->name.c_str(), j+1);
		  fprintf( fp, "\t\t<parent link=\"%s\"/>\n", actPt->jointTf[j].header.frame_id.c_str () );
		  fprintf( fp, "\t\t<child link=\"%s\"/>\n", actPt->jointTf[j].child_frame_id.c_str () );
		  tf::quaternionMsgToTF(actPt->jointTf[j].transform.rotation, bt_q);
		  ROS_ERROR("quaternion in from link %d: %f, %f, %f, %f", j, bt_q.x(), bt_q.y(), bt_q.z(), bt_q.w());
		  btMatrix3x3(bt_q).getRPY( roll, pitch, yaw );
		  fprintf( fp, "\t\t<origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\" />\n",
			   actPt->jointTf[j].transform.translation.x - actPt->tf.transform.translation.x,
			   actPt->jointTf[j].transform.translation.y - actPt->tf.transform.translation.y,
			   actPt->jointTf[j].transform.translation.z - actPt->tf.transform.translation.z,
			   roll, pitch, yaw );
		  fprintf( fp, "\t\t<axis xyz=\"0.0 0.0 1\" />\n" );
		  fprintf( fp, "\t\t<limit effort=\"%f\" lower=\"%f\" upper=\"%f\" velocity=\"0.5\" />\n", actPt->maxTorques[j], actPt->minValues[j], actPt->maxValues[j]);
		  fprintf( fp, "\t</joint>\n\n");
		}
		//joint for tip link
		fprintf( fp, "\t<joint name = \"%s_tip_joint\" type = \"fixed\">\n", actPt->name.c_str());
		fprintf( fp, "\t\t<parent link=\"%s\" />\n", actPt->jointTf[numJoints - 1].child_frame_id.c_str());
		fprintf( fp, "\t\t<child link=\"%s_tip\" />\n", actPt->name.c_str());
		fprintf( fp, "\t</joint>\n");
	}
	for(unsigned int i = 0;i<servo->getNumExtras();i++)
    {
    	addComponentParentJoint(servo->getComponent(i), fp);
    }
  fprintf( fp, "</robot>\n" );
  ulapi_exit ();
  return (1);
}
