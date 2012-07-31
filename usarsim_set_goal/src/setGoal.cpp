#include <string>
#include <actionlib/client/simple_action_client.h>


#include "navigationGoal.hh"
int main(int argc, char** argv)
{
	ros::init(argc, argv, "set_goal", ros::init_options::AnonymousName);
	ros::NodeHandle nh;	
	tf::TransformListener tfListener;
	std::string actName;
	std::string orientationFrame = "local";
	std::string positionFrame = "global";
	bool controlOffset = true;
	bool setOrientation = true;

	NavigationGoal goal;
	goal.setupActuator();
	goal.setTransformListener(&tfListener);
	nh.getParam("/goalset/orientationFrame", orientationFrame);
	nh.getParam("/goalset/positionFrame",positionFrame);
	nh.getParam("/goalset/controlOffset",controlOffset);
	
	goal.setPositionFrameType(positionFrame);
	goal.setOrientationFrameType(orientationFrame);
	goal.setPositionTolerance(0.01);
	goal.setOrientationTolerance(0.04);
	goal.resetOrientation();
	
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_"+goal.getActName(), true);
	move_arm.waitForServer();
	ROS_INFO("Connected to navigation server");
		
	bool finishOnTime = false;
	
	float xGoal, yGoal, zGoal;
	float roll, pitch, yaw;
	std::cout<<"x goal position: ";
	std::cin>>xGoal;
	std::cout<<"y goal position: ";
	std::cin>>yGoal;
	std::cout<<"z goal position: ";
	std::cin>>zGoal;
	
	if(setOrientation)
	{
		std::cout<<"roll: ";
		std::cin>>roll;
		std::cout<<"pitch: ";
		std::cin>>pitch;
		std::cout<<"yaw: ";
		std::cin>>yaw;
		goal.moveOrientation(roll, pitch, yaw);
	}
	/*
	float x,y,z,w;
	std::cout<<"x: ";
	std::cin>>x;
	std::cout<<"y: ";
	std::cin>>y;
	std::cout<<"z: ";
	std::cin>>z;
	std::cout<<"w: ";
	std::cin>>w;
	goal.moveOrientation(x,y,z,w);*/
	goal.setTargetPointFrame("KR60Cup");
	
	if(controlOffset)
		goal.moveOffset(xGoal, yGoal, zGoal);
	else
		goal.movePosition(xGoal, yGoal, zGoal);
	
	move_arm.sendGoal(goal.getGoal());
	//ros::Duration(0.5).sleep();
	//move_arm.cancelGoal();
	finishOnTime = move_arm.waitForResult(ros::Duration(200.0));
	if(!finishOnTime)
	{
		move_arm.cancelGoal();
		ROS_INFO("Move arm timed out.");
	}
	else
	{
		//response from server
		actionlib::SimpleClientGoalState state = move_arm.getState();
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Action finished: %s",state.toString().c_str());
		else
			ROS_INFO("Action failed: %s",state.toString().c_str());			
	}
	ros::shutdown();
}
