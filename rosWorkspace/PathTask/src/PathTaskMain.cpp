#include "ros/ros.h"
#include "PathTask.hpp"
#include <stdio.h>

bool isEnabled = false;

int main(int argc, char **argv)
{
	printf("Before init\n");  
ros::init(argc, argv, "PathTask");
	printf("After init\n");
	PathTask task;
	printf("After constructor\n");
	ros::spin();

	return 0;
}
