/**********************************************************************\
 * Dateiname: libTELEOP.cpp
 * Autor : Mario Grotschar
 * Projekt : MMR 3 Projekt
 * Copyright (C) <<COPYRIGHT>>
 *
 * Kurzbeschreibung: Bibliotehk für die Keyboardeingabe.Die Eingabe wird
 * über das cmd_vel Topic gesendet. In CMakeLists.txt muss die Bibliothek
 * angegeben werden.
 * rosbuild_add_library(libTELEOP src/libTELEOP.cpp)
 * target_link_libraries(amigo libTELEOP)
 *
 * Datum: Autor: Grund der Aenderung:
 * 3.11.2012 Mario Grotschar Neuerstellung
 * <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
 *
 *
 *
 * This program uses parts of the teleop_pr2_keyboard.cpp in modified form.
 * Its disclaimer follows.
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
\**********************************************************************/

/*--- #includes der Form <...> ---------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <Aria.h>
#include <sensor_msgs/PointCloud.h>     //for sonar data

/*--- #includes der Form "..." ---------------------------------------*/

#include "libTELEOP.h"
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


/**********************************************************************\
* Funktionsname: keyboardLoop
*
* Kurzbeschreibung: Diese Funktion liest Keyboardeingaben ein, und
* sendet diese über das Topic cmd_vel.
*
\**********************************************************************/


void TeleopAmigoKeyboard::keyboardLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

//  puts("Reading from keyboard");
//  puts("---------------------------");
//  puts("Use 'WASD' to translate");
//  puts("Use 'QE' to yaw");
//  puts("Press 'Shift' to run");
  	puts("Amigo Control\n");

    puts("Warte auf Keyboardeingabe!");
    puts("---------------------------");
    puts("'WASD' verwenden für translatorische Bewegung");
    puts("'QE' um zu rotieren");
    puts("'Shift' drücken um zu sprinten");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    switch(c)
    {
      // Walking
    case KEYCODE_W:
      cmd.linear.x = walk_vel;
      dirty = true;
      break;
    case KEYCODE_S:
      cmd.linear.x = - walk_vel;
      dirty = true;
      break;
    case KEYCODE_A:
      cmd.linear.y = walk_vel;
      dirty = true;
      break;
    case KEYCODE_D:
      cmd.linear.y = - walk_vel;
      dirty = true;
      break;
    case KEYCODE_Q:
      cmd.angular.z = yaw_rate;
      dirty = true;
      break;
    case KEYCODE_E:
      cmd.angular.z = - yaw_rate;
      dirty = true;
      break;

      // Running
    case KEYCODE_W_CAP:
      cmd.linear.x = run_vel;
      dirty = true;
      break;
    case KEYCODE_S_CAP:
      cmd.linear.x = - run_vel;
      dirty = true;
      break;
    case KEYCODE_A_CAP:
      cmd.linear.y = run_vel;
      dirty = true;
      break;
    case KEYCODE_D_CAP:
      cmd.linear.y = - run_vel;
      dirty = true;
      break;
    case KEYCODE_Q_CAP:
      cmd.angular.z = yaw_rate_run;
      dirty = true;
      break;
    case KEYCODE_E_CAP:
      cmd.angular.z = - yaw_rate_run;
      dirty = true;
      break;
    }


    if (dirty == true)
    {
      vel_pub_.publish(cmd);
    }


  }
}
