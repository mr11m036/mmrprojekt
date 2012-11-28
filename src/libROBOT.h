/*
 * libROBOT.h
 *
 *  Created on: 28.11.2012
 *      Author: amigo
 */
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
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "stdlib.h"
#include "math.h"
#include <sstream>

#ifndef LIBROBOT_H_
#define LIBROBOT_H_
#define diaPath "../log/Diagnostics.txt"
#define errPath "../log/Errors.txt"

using namespace std;


class ActionPause : public ArAction
{
public:
  // constructor, sets myMaxSpeed and myStopDistance
  ActionPause(double time);
  // destructor. does not need to do anything
  virtual ~ActionPause(void) {};
  // called by the action resolver to obtain this action's requested behavior
  virtual ArActionDesired *fire(ArActionDesired currentDesired);
protected:
  double pauseTime;
  ArActionDesired myDesired;
};

ActionPause::ActionPause(double time) :
  ArAction("Pause")
{
  pauseTime = time;
  myDesired.reset();
}

ArActionDesired *ActionPause::fire(ArActionDesired currentDesired)
{
  ros::Duration(pauseTime).sleep();
  return &myDesired;
}


class RosAriaNode
{
  public:
	template<typename T>
		std::string toString(T t)
		{
		std::ostringstream s;
		s << t;
		return s.str();
		}

	//GLOBAL VARIABLES AND OBJECTS
	double dVelLeft, dVelRight;
	//ArRobot robot_sensor;
	int iBeQueueLength;
	int iBeCount;

	int beLib(int id);
	double readParam(int iBeID, int iPaID);
	int readBehav(int iBeID, int iPaID);
	int getBeQueueLenght();
	int logError(string errorDesc, bool bReset);
	int logDia(string diaDesc, bool bReset);


    RosAriaNode(ros::NodeHandle n);
    virtual ~RosAriaNode();

  public:
    int Setup();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
    void spin();
    void publish();
    void sonarConnectCb();

	int bSimpleAvoid();
	int bGotoXY();
	ArRobot *robot;
  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher bumpers_pub;
    ros::Publisher sonar_pub;
    ros::Subscriber cmdvel_sub;

    ros::Time veltime;

    std::string serial_port;

    ArSimpleConnector *conn;

    ActionPause *pause;
    nav_msgs::Odometry position;
    ROSARIA::BumperState bumpers;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;

		//for odom->base_link transform
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::TransformStamped odom_trans;
    //for resolving tf names.
    std::string tf_prefix;
    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    //Sonar support
    bool use_sonar;		// enable and publish sonars
};



#endif /* LIBROBOT_H_ */
