/*
 * SubDiveAlarm.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: bholdaway
 */
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"

float DEPTH = 0.0;


void pressureDataCallback(const std_msgs::Float32::ConstPtr& msg);
void subDepthCallback(const std_msgs::Float32::ConstPtr& msg);

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "SubDiveAlarm");
  ros::NodeHandle nh;

  ros::Subscriber targetSub = nh.subscribe("Target_Depth", 1000, pressureDataCallback);
  ros::Subscriber depthSub = nh.subscribe("Sub_Depth", 1000, subDepthCallback);

  ros::spin();
  return 0;
}

void pressureDataCallback(const std_msgs::Float32::ConstPtr& msg)
{
  if (msg->data < DEPTH)
  {
    system("mplayer /opt/robosub/sounds/klaxonAlarm.ogg");
  }
}

void subDepthCallback(const std_msgs::Float32::ConstPtr& msg)
{
  DEPTH = msg->data;
}
