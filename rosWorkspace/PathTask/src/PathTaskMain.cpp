#include "ros/ros.h"
#include "PathTask.hpp"

bool isEnabled = false;

int main(int argc, char **argv)
{
	PathTask task();
	ros::spin();

	return 0;
}
