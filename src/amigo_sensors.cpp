/*
 * amigo_senors.cpp
 *
 *  Created on: 03.11.2012
 *      Author: mario
 */



#include "libTELEOP.h"
#include "libSENSOR.h"
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



void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "amigo_sensors_sonar");

  SensorSonar senson;
  senson.init();
  ros::Rate pub_rate(20);
  signal(SIGINT,quit);

  while (true)
    {
    ros::spinOnce();
    pub_rate.sleep();
    }

  return 0;
}




