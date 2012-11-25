/**********************************************************************\
* Dateiname: libSENSOR.h
* Autor : Mario Grotschar
* Projekt : MMR 3 Projekt
* Copyright (C) <<COPYRIGHT>>
*
* Kurzbeschreibung: Headerfile für libSENSOR.cpp
*
* Datum: Autor: Grund der Aenderung:
* 3.11.2012 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*

\**********************************************************************/


#ifndef _INCL_SENSOR
#define _INCL_SENSOR
/*--- #includes der Form <...> ---------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <Aria.h>
#include <sensor_msgs/PointCloud.h>     //for sonar data

/*--- #includes der Form "..." ---------------------------------------*/

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



class SensorSonar
{
  private:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  uint32_t queue_size;

 /* Der AmigoBot hat 8 Ultraschallsensoren.
	Nr. 0: +90
	Nr. 1: +41
	Nr. 2: +15
	Nr. 3: -15
	Nr. 4: -41
	Nr. 5: -90
	Nr. 6: -145
	Nr. 7: +145

		  +90
	+145   |    +15
	-------|-------> Vorne
	-145   |    -15
          -90
*/

  float calcDistance(float x, float y);

  public:

  float  distCurrent[8];

  float getDistance (int sensornr);

 // SensorSonar() : SIZESENSOR (8) {queue_size=1;}

  void init()
  {
	queue_size=1;
    sub = nh_.subscribe <sensor_msgs::PointCloud>  ("/RosAria/sonar",1, &SensorSonar::callback, this);
	ros::NodeHandle n_private("~");

  }


  ~SensorSonar()   { }
  void callback(const sensor_msgs::PointCloud::ConstPtr &msg);
//http://stanford-ros-pkg.googlecode.com/svn-history/r146/trunk/recyclerbot/src/object_detector_node.cpp
};

#endif //_INCL_SENSOR
