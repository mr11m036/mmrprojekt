/**********************************************************************\
* Dateiname: libSENSOR.cpp
* Autor : Mario Grotschar
* Projekt : MMR 3 Projekt
* Copyright (C) <<COPYRIGHT>>
*
* Kurzbeschreibung:*
* Datum: Autor: Grund der Aenderung:
* 3.11.2012 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*

\**********************************************************************/
/*--- #includes der Form <...> ---------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <Aria.h>
#include <sensor_msgs/PointCloud.h>     //for sonar data

/*--- #includes der Form "..." ---------------------------------------*/

#include "libSENSOR.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "ROSARIA/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"	//for tf::getPrefixParam
#include "tf/transform_datatypes.h"

using namespace std;

void SensorSonar::callback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
	geometry_msgs::Point32 p;
	int csize;
	csize= msg->get_channels_size();
	//p = msg->points;
	printf ("Cloud: x = %d\n", csize);
	//printf ("Cloud: x = %f, y = %f, z = %f\n", p.x, p.y, p.z);


}

