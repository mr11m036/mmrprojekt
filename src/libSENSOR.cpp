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
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <float.h>
#include <stdexcept>

/*--- #includes der Form "..." ---------------------------------------*/

#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"
#include "ros/macros.h"
#include "ros/assert.h"
#include "angles/angles.h"
#include <bits/stl_iterator_base_funcs.h>
#include <bits/functexcept.h>
#include <bits/concept_check.h>
#include <initializer_list>
#include "std_msgs/Header.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"
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

	// Dieser Link war hilfreich.
	//http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Sensors
	//sx = msg->points[1].x;
	//sy = msg->points[1].y;
	//sz = msg->points[1].z;
	//printf ("Cloud: x = %f, y = %f, z = %f\n", sx, sy, sz);

	for (int i=0; i < 8; i++) {
		SensorSonar::distCurrent[i] = SensorSonar::calcDistance(msg->points[i].x, msg->points[i].y);
		// sz will always remain 0.
	}

}


float SensorSonar::getDistance(int sensornr) {
	float result;
	try {
		if (sensornr >= 8)
			throw "distanceCurrent out of bounds. Return value set to 0.";
	} catch (char * str) {
		cout << "Exception raised: " << str << '\n';
		result = 0;
		return result;
	}

	result = SensorSonar::distCurrent[sensornr];
	return result;

}

float SensorSonar::calcDistance(float x, float y) {
	double result;
	try {
	result = sqrt(pow(x, 2) + pow(y, 2));
	if (result > FLT_MAX)
		throw "Float overflow in calcDistance. Return value set to 0.";
	}
	catch (char * str) {
		result = 0;
		cout << "Exception raised: " << str << '\n';
	}

	return result;

}

void SensorSonar::callbackOdometry(nav_msgs::Odometry msg)
{
    //This is the call back function to process odometry messages coming from Stage.
    px = initialX + msg.pose.pose.position.x;
    py = initialY + msg.pose.pose.position.y;
    ptheta = angles::normalize_angle_positive(asin(msg.pose.pose.orientation.z) * 2);
    ROS_INFO("x odom %f y odom %f theta %f", px, py, ptheta);

}

void SensorSonar::callbackBumper(ROSARIA::BumperState msg)
{
    //This is the call back function to process odometry messages coming from Stage.
	frontBumper = msg.front_bumpers[0];
	rearBumper = msg.rear_bumpers[0];

}

bool SensorSonar::getFrontBumper(void)
{
	return frontBumper;
}

bool SensorSonar::getRearBumper(void)
{
	return rearBumper;
}

double SensorSonar::getX(void)
{
	return px;
}

double SensorSonar::getY(void)
{
	return py;
}

double SensorSonar::getTheta(void)
{
	return ptheta;
}

//http://pastebin.com/fiQRVayf
