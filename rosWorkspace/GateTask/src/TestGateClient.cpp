#include<iostream>
#include<vector>
#include "ros/ros.h"
#include "GateTask/Toggle.h"
#include <cstdlib>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "GateClient");
	
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<GateTask::Toggle>("toggleGateServ");
	GateTask::Toggle srv;
	srv.request.enabled = atoi(argv[1]);
	if(client.call(srv))
	{
		ROS_INFO("Result: %d", (int)srv.response.result);
	}
	else
	{
		ROS_ERROR("something bad happened.");
		return 1;
	}
	
	
	return 0;
}

