/*
 * amigo.cpp
 *
 *  Created on: 26.10.2012
 *  Modified on: 2.11.2012
 *  			Modified teleop_pr2_keyboard.cpp to fit Project purposes.
 *      Author: Mario
 *
 *      This program uses parts of the teleop_pr2_keyboard.cpp in modified form.
 *      Its disclaimer follows.
 *
 * ------------------------------------------------------------------------------
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "libTELEOP.h"
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



void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "amigo_control_keyboard");

  TeleopAmigoKeyboard tak;
  tak.init();

  signal(SIGINT,quit);
 tak.keyboardLoop();


  geometry_msgs::Twist cmd;

  return 0;
}


