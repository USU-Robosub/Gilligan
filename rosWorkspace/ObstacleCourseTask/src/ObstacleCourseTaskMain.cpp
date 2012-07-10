#include "ros/ros.h"
#include "ObstacleCourseTask.hpp"

int main(int argc, char **argv)
{
  ObstacleCourseTask task();
  ros::spin();
  return 0;
}
