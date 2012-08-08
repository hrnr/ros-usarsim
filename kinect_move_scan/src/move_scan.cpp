#include "ros/ros.h"
#include "usarsim_inf/RangeImageScan.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_scan");
  ros::NodeHandle nh;
  ros::Publisher scanner = nh.advertise<usarsim_inf::RangeImageScan>("KinectDepth/command", 1);
  ros::Publisher mover = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  usarsim_inf::RangeImageScan scan;
  geometry_msgs::Twist move;
    move.linear.x = 1;
  geometry_msgs::Twist reverse;
    reverse.linear.x = -1;
  geometry_msgs::Twist stop;
  geometry_msgs::Twist right;
    right.angular.z = -1;
  geometry_msgs::Twist left;
    left.angular.z = 1;
  sleep(1);
  double start_time = ros::Time::now().toSec();
  double end_time = 0;

  //First Movement
  while((start_time + 66) > end_time) //48
  {
    scanner.publish(scan);
    sleep(6);
    mover.publish(left);
    usleep(100000);
    mover.publish(stop);
    sleep(2);
    end_time = ros::Time::now().toSec();
  }


  //Second Movement
  start_time = ros::Time::now().toSec();
  while((start_time + 120) > end_time) //72
  {
    scanner.publish(scan);
    sleep(6);
    mover.publish(move);
    usleep(100000);
    mover.publish(stop);
    sleep(2);
    end_time = ros::Time::now().toSec();
  }

  //Third Movement
  start_time = ros::Time::now().toSec();
  while((start_time + 70) > end_time) //48
  {
    scanner.publish(scan);
    sleep(6);
    mover.publish(left);
    usleep(100000);
    mover.publish(stop);
    sleep(2);
    end_time = ros::Time::now().toSec();
  }

  //Fourth Movement
  start_time = ros::Time::now().toSec();
  while((start_time + 200) > end_time) //100
  {
    scanner.publish(scan);
    sleep(6);
    mover.publish(move);
    usleep(100000);
    mover.publish(stop);
    sleep(2);
    end_time = ros::Time::now().toSec();
  }

  return 0;
}
