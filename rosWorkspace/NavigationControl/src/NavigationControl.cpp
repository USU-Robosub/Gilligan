#include "ros/ros.h"
#include "NavigationControl/Navigate.h"

using namespace std;

bool navigateCallback(NavigationControl::Navigate::Request &req, NavigationControl::Navigate::Response &res);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "NavigationControl");

	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("navControlServ", navigateCallback);
	ros::spin();
	
	return 0;
}

bool navigateCallback(NavigationControl::Navigate::Request &req, NavigationControl::Navigate::Response &res)
{
	// Get request information
	int cameraDirection = req.camera_direction; // 0 = forward, 1 = downward
	int x = req.desired_x;
	int y = req.desired_y;
	float rotation = req.desired_rotation; // Required for downward direction only
	
	// TODO: Decide how to align with desired point/rotation
	
	// TODO: Tell motor controllers what to do
	
	// Set result value. 0 = success, others = error
	res.result = 0;
	
	return true;
}

