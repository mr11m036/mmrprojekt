/*
 * libROBOT.cpp
 *
 *  Created on: 28.11.2012
 *      Author: amigo
 */

#include "libROBOT.h"



void RosAriaNode::sonarConnectCb()
{
  if (sonar_pub.getNumSubscribers() == 0)
  {
    robot->disableSonar();
    use_sonar = false;
  }
  else
  {
    robot->enableSonar();
    use_sonar = true;
  }
}

RosAriaNode::RosAriaNode(ros::NodeHandle nh) :
  myPublishCB(this, &RosAriaNode::publish), use_sonar(false)
{
  // read in config options
  n = nh;

  // !!! port !!!
  n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
  ROS_INFO( "using port: [%s]", serial_port.c_str() );

  /*
   * Figure out what frame_id's to use. if a tf_prefix param is specified,
   * it will be added to the beginning of the frame_ids.
   *
   * e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
   * roslaunch files)
   * will result in the frame_ids being set to /MyRobot/odom etc,
   * rather than /odom. This is useful for Multi Robot Systems.
   * See ROS Wiki for further details.
   */
  tf_prefix = tf::getPrefixParam(n);
  frame_id_odom = tf::resolve(tf_prefix, "odom");
  frame_id_base_link = tf::resolve(tf_prefix, "base_link");
  frame_id_bumper = tf::resolve(tf_prefix, "bumpers_frame");
  frame_id_sonar = tf::resolve(tf_prefix, "sonar_frame");

  // advertise services
  pose_pub = n.advertise<nav_msgs::Odometry>("pose",1000);
  bumpers_pub = n.advertise<ROSARIA::BumperState>("bumper_state",1000);
  sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar", 50, boost::bind(&RosAriaNode::sonarConnectCb, this),
    boost::bind(&RosAriaNode::sonarConnectCb, this));

  // subscribe to services
  cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function < void(const geometry_msgs::TwistConstPtr&)>) boost::bind( &RosAriaNode::cmdvel_cb, this, _1 ));

  veltime = ros::Time::now();

  FlagArPoseList = false;
}

RosAriaNode::~RosAriaNode()
{
  //disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();
  //robot->enableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}

int RosAriaNode::Setup()
{
  ArArgumentBuilder *args;
  args = new ArArgumentBuilder();

  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-rh"); //pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-rrtp"); //pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-rp"); //pass robot's serial port to Aria
    args->add(serial_port.c_str());
  }

  args->add("-rlpr"); //log received packets
  args->add("-rlps"); //log sent packets
  args->add("-rlvr"); //log received velocities
  conn = new ArSimpleConnector(args);

  robot = new ArRobot();
  pause = new ActionPause(4e-3);

  ArLog::init(ArLog::File, ArLog::Verbose, "aria.log", true);

  // Connect to the robot
  if (!conn->connectRobot(robot)) {
    ArLog::log(ArLog::Terse, "rotate: Could not connect to robot! Exiting.");
    return 1;
  }

  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->enableSonar();

  robot->addSensorInterpTask("PublishingTask", 100, &myPublishCB);
//  robot->addAction(pause, 10);
  robot->runAsync(true);

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  FlagArPoseList = false;

  return 0;
}

void RosAriaNode::spin()
{
  ros::spin();
}

void RosAriaNode::publish()
{
//  robot->lock();
  pos = robot->getPose();
//  robot->unlock();
  tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000, pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000; //Aria returns velocity in mm/s.
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;

  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = ros::Time::now();
  pose_pub.publish(position);
  ROS_INFO("rcv: %f %f %f", position.header.stamp.toSec(), (double) position.twist.twist.linear.x, (double) position.twist.twist.angular.z);

	// publishing transform odom->base_link
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = frame_id_odom;
	odom_trans.child_frame_id = frame_id_base_link;

	odom_trans.transform.translation.x = pos.getX()/1000;
	odom_trans.transform.translation.y = pos.getY()/1000;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTh()*M_PI/180);

	odom_broadcaster.sendTransform(odom_trans);

  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = ros::Time::now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  //ROS_INFO("Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  //ROS_INFO("Rear bumpers:%s", bumper_info.str().c_str());

  bumpers_pub.publish(bumpers);

  /*
   * Publish sonar information, if necessary.
   */
  if (use_sonar) {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    //sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;

    std::stringstream sonar_debug_info;
    sonar_debug_info << "Sonar readings: ";
    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
	      ROS_WARN("Did not receive a sonar reading.");
	      continue;
      }

      //getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      /*
       * local (x,y). Appears to be from the centre of the robot, since values may
       * exceed 5000. This is good, since it means we only need 1 transform.
       * x & y seem to be swapped though, i.e. if the robot is driving north
       * x is north/south and y is east/west.
       */
      //ArPose sensor = reading->getSensorPosition();	//position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX()
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", "
      //                  << sensor.getY() << ") ;; " ;

      //add to cloud
      geometry_msgs::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    ROS_DEBUG_STREAM(sonar_debug_info.str());

    sonar_pub.publish(cloud);
  }

  ros::Duration(1e-3).sleep();
}

void
RosAriaNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
  veltime = ros::Time::now();
  ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );

//  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
//  robot->unlock();
  ROS_INFO("snd: %f %f %f", veltime.toSec(), (double) msg->linear.x, (double) msg->angular.z);
}



//BEHAVIOR LIBRARY

int RosAriaNode::beLib(int id)//int id is the Behavior ID of the Behavior that is to be called
{
	switch(id)
	{
	case 0:
		//This signifies an empty field in the behavior queue so the cycle is terminated
		//Can be used for diagnostic purposes
		return 1;
		break;

	case 1:
		//Demo Behavior with ID 1: Simple avoidance behavior
		  ROS_INFO("bSimpleAvoid");
		return bSimpleAvoid();
		break;

	case 2:
		//Demo Behavior with ID 2: Simple goto behavior
		return bGotoXY();
		break;

	case 3:
		//Demo Behavior with ID 2: Simple goto behavior
		 ROS_INFO("bGotoSquare");
		 return bGotoCircle();
		break;

	case 4:
		//Demo Behavior with ID 2: Simple goto behavior
		 ROS_INFO("bGotoXYPath");
		 return bGotoXYPath();
		break;

	}
	return 0;
}
//END OF BEHAVIOR LIBRARY


//MISC FUNCTIONS

/*Function reads from the textfile corresponding to the Behavior ID and returns the value of line iPaId without
the description in the File*/
double RosAriaNode::readParam(int iBeID, int iPaID)
{
	string sParam;
	string sPath = "../param/";//Path must be correct
	sPath += toString(iBeID);
	sPath += ".txt";//Behavior ID is added to the Path, Config file can now be found automatically

	ifstream inFile(sPath.c_str());//File is Opened

	iPaID++;

	if(inFile.is_open())
	{
		for(int i=0;i < iPaID;i++)//Lines are read until the requested line is reached
		{
			getline(inFile,sParam);
		}

		for(unsigned int i=0;i < sParam.length();i++)//Useless data is cut from the requested line
		{
			if(sParam[i]==':')
			sParam.erase(0,++i);
		}
	}

	inFile.close();//File is closed
	return atof(sParam.c_str());//Remaining String is converted typecasted to double and returned




}

int RosAriaNode::readBehav(int iBeID, int iPaID)//Function is similar to readParam with some small exceptions
{
	string sParam;
	string sPath = "../param/BehaviorQueue.txt";//Path must be correct

	ifstream inFile(sPath.c_str());

	for(int i=0;i <= iBeID;i++)
	{
		getline(inFile,sParam);
	}

	if(iPaID==0)
	{
		for(unsigned int i=0;i < sParam.length();i++)
		{
			if(sParam[i]==':')
				sParam.erase(i);
		}
	}
	else if(iPaID==1)
	{
		for(unsigned int i=0;i < sParam.length();i++)
		{
			if(sParam[i]==':')
				sParam.erase(0,++i);
		}
	}

	inFile.close();
	return atoi(sParam.c_str());
}

int RosAriaNode::getBeQueueLenght()//Determines the lenght of the behavior queue
{
	int iCounter = 0;//Counts the Number of Lines in the file
	string sVoid;
	string sPath = "../param/BehaviorQueue.txt";//Path must be correct

	ifstream inFile(sPath.c_str());

	while(!inFile.eof())//Loop is terminated once the end of the Behavior Queue is reached
	{
		getline(inFile,sVoid);
		iCounter++;
	}

	inFile.close();
	return iCounter;//Number of lines is returned
}

int RosAriaNode::logError(string errorDesc, bool bReset)//Writes error description to Errors.txt, errpath is the path to said file
{
	fstream errFile;
	if(!bReset)
		errFile.open(errPath, ios::out|ios::app);//If bReset is not set, the data is appended to the end of the file
	else
		errFile.open(errPath, ios::out);//If bReset is set, the file is cleared before new data is written to it
	if(errFile.is_open())
	{
		errFile << errorDesc;//Error description is written to the file
	}
	errFile.close();//File is closed
	return 0;
}

int RosAriaNode::logDia(string diaDesc, bool bReset)//Similar to logError, but writes to Diagnostics.txt
{
	fstream diaFile;
	if(!bReset)
		diaFile.open(diaPath, ios::out|ios::app);
	else
		diaFile.open(diaPath, ios::out);
	if(diaFile.is_open())
	{
		diaFile << diaDesc;
	}
	diaFile.close();
	return 0;
}

//END OF MISC FUNCTIONS


//BEHAVIOR MODULES


int RosAriaNode::bSimpleAvoid()
{
	double dMaxVel = readParam(1,0);//Reads Parameters from textfile
	double dFrontTrigDist = readParam(1,1);
	double dFrontVelMod = readParam(1,2);
	double dSideTrigDist = readParam(1,3);
	double dSideVelMod = readParam(1,4);
	double dActiveDist = readParam(1,5);
	  ROS_INFO("dFrontTrigDist:%f", dFrontTrigDist);
	int iLMod = 1;//Modifiers used to implement rotation
	int iRMod = 1;

	double dRealVel = dMaxVel;//Default speed is set to the maximum allowed velocity taken form the config file

	if((robot->getSonarRange(2) < dActiveDist)||(robot->getSonarRange(3) < dActiveDist)||(robot->getSonarRange(0) < dActiveDist) || (robot->getSonarRange(1) < dActiveDist) || (robot->getSonarRange(4) < dActiveDist) || (robot->getSonarRange(5) < dActiveDist))
	{//Checks if there are any objects within a given range, behavior is not activated if this is not the case

		if((robot->getSonarRange(2) < dFrontTrigDist)||(robot->getSonarRange(3) < dFrontTrigDist))//Checks if the object is to the front
		{
			dRealVel *= dFrontVelMod;//Decreases velocity
			  //ROS_INFO("Decreasing velocity to :%f", dRealVel);
		}

		if((robot->getSonarRange(0) < dSideTrigDist) || (robot->getSonarRange(1) < dSideTrigDist))//Checks if the object is to the left
		{
			iLMod = -1;//Sets the modifier for the left wheel to -1 so that the robot will steer to the right
			dRealVel *= dSideVelMod;//Velocity is modified
			 // ROS_INFO("Decreasing to :%f", dRealVel);
		}

		if((robot->getSonarRange(4) < dSideTrigDist) || (robot->getSonarRange(5) < dSideTrigDist))
		{
			iRMod = -1;
			dRealVel *= dSideVelMod;

		}
		dVelLeft += iLMod * dRealVel;//Calculated motor data is passed on to the global velocity control variables
		dVelRight += iRMod * dRealVel;
		  ROS_INFO("dVelLeft:%f", dVelLeft);
		  ROS_INFO("dVelRight:%f", dVelRight);
		return 1;
	}
	else
		return 0;
}

int RosAriaNode::bGotoCircle()
{
	double dMaxVel = readParam(3,2);//Reads Parameters from textifle
	double dRadius	= readParam(3,0);
	double dDirection	= readParam(3,1); //-1 clockwise +1 counter clockwise

	ArPose current;//Contains the current pose of the robot

	if (dDirection == -1)
	{
		dVelLeft = (dRadius + (double)RADSTAND/2)*dMaxVel;
		dVelRight = (dRadius - (double)RADSTAND/2)*dMaxVel;
		  ROS_INFO("dVelLeft Rot:%f", dVelLeft);
		  ROS_INFO("dVelRight Rot:%f", dVelRight);
	}
	else if (dDirection == 1)
	{
		dVelLeft = (dRadius - (double)RADSTAND/2)*dMaxVel;
		dVelRight = (dRadius + (double)RADSTAND/2)*dMaxVel;
	}
	else
	{
		return 1;
	}



	return 0;
}

int RosAriaNode::bGotoXYPath()
{
	double dMaxVel = readParam(4,1);//Reads Parameters from textifle
	double dProximity = readParam(4,2);
	double dRotVel = readParam(4,3);
	double dAccuracy = readParam(4,4);
	int    iPoints =readParam(4,0);

	ArPose target;//Instance of ArPose Object, contains the target coordinates
	ArPose current;//Contains the current pose of the robot

		if (FlagArPoseList == false)
		{
			// Get poses
			FlagArPoseList = true;

			for (int loop = 0; loop < iPoints; loop++)
			{
				target.setX(readParam(4,2*loop+5));
				target.setY(readParam(4,2*loop+6));
				targetList.push_back(target);
			}
			targetListIT = targetList.begin();
			/*
			ROS_INFO("Setting Path");
			targetListIT = targetList.begin();
			target.setX(300);
			target.setY(0);
			targetList.push_back(target);
			target.setX(300);
			target.setY(300);
			targetList.push_back(target);
			target.setX(0);
			target.setY(300);
			targetList.push_back(target);
			target.setX(0);
			target.setY(0);
			targetList.push_back(target);
			targetListIT = targetList.begin();
			*/
			return 1;
		}
		else
		{

			if ( targetListIT != targetList.end() && !targetList.empty())
			{

				target.setX(targetListIT->getX());
				target.setY(targetListIT->getY());
				ROS_INFO("X: %f",targetListIT->getX() );
				ROS_INFO("Y: %f",targetListIT->getY() );


			current = robot->getPose();

			if(current.findDistanceTo(target) > dProximity)//Behavior is activated if the robot is not already within a certian distance to the target
			{
				//Angle to target is calculated and compared to the Pose of the robot
				//the robot is then set to rotate if the angles are not similar
				if(current.findAngleTo(target) > current.getTh() + dAccuracy)
				{
					dVelRight -= dRotVel;
					dVelLeft += dRotVel;
					//  ROS_INFO("bGotoXY dVelLeftOverloaded %f",dVelLeft);
				}
				else if(current.findAngleTo(target) < current.getTh() - dAccuracy)
				{
					dVelRight += dRotVel;
					dVelLeft -= dRotVel;
				}
				//if the angles are similar, the robot drives forward in order to approach his destionation
				else
				{
					dVelRight += dMaxVel;
					dVelLeft += dMaxVel;
				}
				return 1;
			}

			else
				{
					++targetListIT;
					return 0;
				}
			}
			else
			{
				return 0;
			}
		}
}

int RosAriaNode::bGotoXY()
{
	double dMaxVel = readParam(2,2);//Reads Parameters from textifle
	double dProximity = readParam(2,3);
	double dRotVel = readParam(2,4);
	double dAccuracy = readParam(2,5);

	ArPose target;//Instance of ArPose Object, contains the target coordinates
	ArPose current;//Contains the current pose of the robot
	target.setX(readParam(2,0));
	target.setY(readParam(2,1));
	current = robot->getPose();

	if(current.findDistanceTo(target) > dProximity)//Behavior is activated if the robot is not already within a certian distance to the target
	{
		//Angle to target is calculated and compared to the Pose of the robot
		//the robot is then set to rotate if the angles are not similar
		if(current.findAngleTo(target) > current.getTh() + dAccuracy)
		{
			dVelRight -= dRotVel;
			dVelLeft += dRotVel;
		}
		else if(current.findAngleTo(target) < current.getTh() - dAccuracy)
		{
			dVelRight += dRotVel;
			dVelLeft -= dRotVel;
		}
		//if the angles are similar, the robot drives forward in order to approach his destionation
		else
		{
			dVelRight += dMaxVel;
			dVelLeft += dMaxVel;
		}
		return 1;
	}
	else return 0;
}
