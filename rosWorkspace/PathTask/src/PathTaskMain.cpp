#include "ros/ros.h"
#include "PathTask.hpp"
#include <stdio.h>

bool isEnabled = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PathTask");
  PathTask task;
  ros::spin();

  return 0;
}
