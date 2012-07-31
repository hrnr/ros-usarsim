#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_transform");
	ros::NodeHandle nh;
	ros::Rate r(2.0);
	tf::TransformBroadcaster broadcaster;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion quat;
	quat.setEuler(3.14159, 0, 0);
	transform.setRotation(quat);
	ROS_ERROR("ROS Version is %d %d %d",ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH);
	while(ros::ok())
	{
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rviz_base", "odom"));
		r.sleep();
	}
	return 0;
}
