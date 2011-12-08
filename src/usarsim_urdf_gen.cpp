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

  // initialize the USARSim interface wrapper
  usarsim->init (servo);

  // main loop
  startTime = ros::Time::now ();

  ROS_INFO ("Waiting 5 sec. for system to stabilize\n");
  while ((usarsim->getNH ())->ok ())
    {
      if (usarsim->msgIn () != 1)
	{
	  ROS_ERROR ("Error from usarsimInf");
	}
      currentTime = ros::Time::now ();
      if( (currentTime.sec - startTime.sec ) > 5. )
	break;
      else
	ROS_INFO ("Waiting %f more seconds (cur: %d start: %d", (float)(5.-(currentTime.sec - startTime.sec )),
		  currentTime.sec, startTime.sec);
    }
  ROS_INFO ("Generating urdf file...\n");
  fileName = servo->getPlatformName().c_str () + std::string(".urdf");
  fp = fopen( fileName.c_str (), "w" );
  if( fp == NULL )
    {
      ROS_ERROR( "Unable to open urdf file" );
      return (-1);
    }

  const UsarsimActuator *actPt;
  double length;
  geometry_msgs::Vector3 platformSize;

  fprintf( fp, "<\?xml version=\"1.0\"\?>\n" );
  for( unsigned int i=0; i<servo->getNumActuators(); i++ )
    {
      actPt = servo->getActuator(i);
      platformSize = servo->getPlatformSize();
      fprintf( fp, "<robot name=\"%s\">\n", servo->getPlatformName().c_str () );
      fprintf( fp, "\t<link name=\"%s\" >\n", actPt->name.c_str () );
      fprintf( fp, "\t\t<visual>\n");
      fprintf( fp, "\t\t\t<geometry>\n" );
      fprintf( fp, "\t\t\t\t<box size=\"%f %f %f\"/>\n",
	       platformSize.x, platformSize.y,
	       platformSize.z );
      fprintf( fp, "\t\t\t</geometry>\n" );
      fprintf( fp, "\t\t\t<material name=\"blue\">\n");
      fprintf( fp, "\t\t\t\t<color rgba=\"0 0 .8 .6\"/>\n");
      fprintf( fp, "\t\t\t</material>\n");
      fprintf( fp, "\t\t</visual>\n");
      fprintf( fp, "\t</link>\n");
      for( unsigned int j=0; j<actPt->jstate.name.size(); j++ )
	{
	  // compute values
	  if( j == 0 )
	    {
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
	    }
	  else
	    {
	      length = sqrt( (actPt->jointTf[j].transform.translation.x - 
			      actPt->jointTf[j-1].transform.translation.x) *
			     (actPt->jointTf[j].transform.translation.x - 
			      actPt->jointTf[j-1].transform.translation.x) +
			     (actPt->jointTf[j].transform.translation.y - 
			      actPt->jointTf[j-1].transform.translation.y) *
			     (actPt->jointTf[j].transform.translation.y - 
			      actPt->jointTf[j-1].transform.translation.y) +
			     (actPt->jointTf[j].transform.translation.z - 
			      actPt->jointTf[j-1].transform.translation.z) *
			     (actPt->jointTf[j].transform.translation.z - 
			      actPt->jointTf[j-1].transform.translation.z));

	    }
	  fprintf( fp, "\t<link name=\"%s\" >\n", actPt->jstate.name[j].c_str () );
	  fprintf( fp, "\t\t<visual>\n");
	  fprintf( fp, "\t\t\t<geometry>\n" );
	  fprintf( fp, "\t\t\t\t<cylinder length=\"%f\" radius =\".05\"/>\n",
		   length );

	  fprintf( fp, "\t\t\t</geometry>\n" );
	  fprintf( fp, "\t\t\t<material name=\"blue\"/>\n");
	  tf::quaternionMsgToTF(actPt->jointTf[j].transform.rotation, bt_q);
	  btMatrix3x3(bt_q).getRPY( roll, pitch, yaw );
	  fprintf( fp, "\t\t<origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\" />\n",
		   actPt->jointTf[j].transform.translation.x,
		   actPt->jointTf[j].transform.translation.y,
		   actPt->jointTf[j].transform.translation.z,
		   roll, pitch, yaw );
	  fprintf( fp, "\t\t</visual>\n");
	  fprintf( fp, "\t</link>\n");
	}
      fprintf( fp, "\n\n" );

      for( unsigned int j=0; j<actPt->jointTf.size(); j++ )
	{
	  fprintf( fp, "\t<joint name=\"joint%d\" type=\"continuous\">\n", j+1);
	  fprintf( fp, "\t\t<parent link=\"%s\"/>\n", actPt->jointTf[j].header.frame_id.c_str () );
	  fprintf( fp, "\t\t<child link=\"%s\"/>\n", actPt->jointTf[j].child_frame_id.c_str () );
	  tf::quaternionMsgToTF(actPt->jointTf[j].transform.rotation, bt_q);
	  btMatrix3x3(bt_q).getRPY( roll, pitch, yaw );
	  fprintf( fp, "\t\t<origin xyz=\"%.2f %.2f %.2f\" rpy=\"%.2f %.2f %.2f\" />\n",
		   actPt->jointTf[j].transform.translation.x - actPt->tf.transform.translation.x,
		   actPt->jointTf[j].transform.translation.y - actPt->tf.transform.translation.y,
		   actPt->jointTf[j].transform.translation.z - actPt->tf.transform.translation.z,
		   roll, pitch, yaw );
	  fprintf( fp, "\t\t<axis xyz=\"0.0 0.0 1\" />\n" );
	  fprintf( fp, "\t</joint>\n\n");
	}
      fprintf( fp, "</robot>\n" );
    }
  ulapi_exit ();
  return (1);
}
