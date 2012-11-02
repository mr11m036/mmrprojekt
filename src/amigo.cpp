/*
 * amigo_tut1.cpp
 *
 *  Created on: 26.10.2012
 *      Author: mario
 */

#include "std_msgs/String.h"
#include <stdio.h>
#include <math.h>
#include <Aria.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>     //for sonar data
#include "nav_msgs/Odometry.h"
#include "ROSARIA/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"	//for tf::getPrefixParam
#include "tf/transform_datatypes.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double xPos=msg->pose.pose.position.x;
    double yPos=msg->pose.pose.position.y;
    //get Quaternion anglular information
    double x=msg->pose.pose.orientation.x;
    double y=msg->pose.pose.orientation.y;
    double z=msg->pose.pose.orientation.z;
    double w=msg->pose.pose.orientation.w;
    //convert to pitch
    double angle=atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z);
    ROS_INFO("%f %f %f",xPos,yPos,angle);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle nh_("~");
  ros::Publisher cmdPub = nh_.advertise  < geometry_msgs::Twist >( "/RosAria/cmd_vel" , 1 ) ;

  ros::Subscriber sub_image_ = nh_.subscribe<nav_msgs::Odometry>("/RosAria/pose", 1, &chatterCallback);

  geometry_msgs::Twist cmd;

  ros::Rate pub_rate(20);
  cmd.linear.x = 1;
  cmd.linear.y = 1;
  cmd.angular.z = 1;


  while (true)
  {
  ros::spinOnce();
  cmdPub.publish(cmd);
  pub_rate.sleep();
  }

  return 0;
}


