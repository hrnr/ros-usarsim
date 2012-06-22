#ifndef __setGoal__
#define __setGoal__

#include <ros/ros.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <tf/transform_listener.h>

enum frame_type
{
	GLOBAL=0,
	LOCAL
};

class NavigationGoal
{
public:
	NavigationGoal(std::string actuatorName, int linkCount);
	void movePosition(float xGoal, float yGoal, float zGoal);
	void moveOffset(float xGoal, float yGoal, float zGoal);
	void setPositionFrameType(const std::string& frame);
	void setOrientationFrameType(const std::string& frame);
	void resetOrientation();
	void setPositionTolerance(double tolerance);
	void setOrientationTolerance(double tolerance);
	arm_navigation_msgs::MoveArmGoal getGoal();
private:
	tf::TransformListener listener;
	arm_navigation_msgs::MoveArmGoal goal;
	bool useGlobalPositionFrame, useGlobalOrientationFrame;
	std::string actName;
	std::string effectorFrame;
	int linkNum;
	void setGlobalPositionGoal(float xGoal, float yGoal, float zGoal);
};

#endif
