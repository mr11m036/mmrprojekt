/*
 * amigo_senors.cpp
 *
 *  Created on: 03.11.2012
 *      Author: mario
 */


#include "libSENSOR.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <math.h>
#include <Aria.h>
#include <ncurses.h>

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
//#include "libSENSOR.h"


void quit(int sig)
{
  //tcsetattr(kfd, TCSANOW, &cooked);
  endwin();
  exit(0);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "amigo_sensors_sonar");
//float test;
  SensorSonar senson;
  senson.init();
  ros::Rate pub_rate(20);

  signal(SIGINT,quit);

  initscr();			/* Start curses mode 		*/


  while (true)
    {
	  ros::spinOnce();
	  pub_rate.sleep();
	  clear();

	  printw ("Pose\n");
	  printw ("===================\n");
	  printw ("X: %f \n", senson.getX());
	  printw ("Y: %f \n", senson.getY());
	  printw ("Pheta: %f \n", senson.getTheta());
	  printw (" \n");
	  printw ("Bumpers\n");
	  printw ("===================\n");
	  if (senson.getFrontBumper())
			  printw ("Kontakt vorne.\n");
	  else
		  	  printw ("Kein Kontakt vorne.\n");

	  if ( senson.getRearBumper())
			  printw ("Kontakt am Heck.\n");
	  else
		  	  printw ("Kein Kontakt am Heck.\n");
	  printw (" \n");
	  printw ("Sonar\n");
	  printw ("===================\n");
	  printw ("Sensor 0: %f \n", senson.getDistance(0));
	  printw ("Sensor 1: %f \n", senson.getDistance(1));
	  printw ("Sensor 2: %f \n", senson.getDistance(2));
	  printw ("Sensor 3: %f \n", senson.getDistance(3));
	  printw ("Sensor 4: %f \n", senson.getDistance(4));
	  printw ("Sensor 5: %f \n", senson.getDistance(5));
	  printw ("Sensor 6: %f \n", senson.getDistance(6));
	  printw ("Sensor 7: %f \n", senson.getDistance(7));

	  refresh();
    }
endwin();
  //endwin();
  return 0;
}




