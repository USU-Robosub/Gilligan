#include "ros/ros.h"
#include "BuoysTask/Toggle.h"
#include "SubImageRecognition/ImgRecObject.h"

using namespace std;

bool toggleCallback(BuoysTask::Toggle::Request &req, BuoysTask::Toggle::Response &res);
void imgRecCallback(const SubImageRecognition::ImgRecObject &msg);

bool isEnabled = false;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "BuoysTask");

	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("toggleBuoysServ", toggleCallback);
	ros::Subscriber subscriber = n.subscribe("image_recognition/forward/buoys", 10, imgRecCallback);
	ros::spin();
	
	return 0;
}

bool toggleCallback(BuoysTask::Toggle::Request &req, BuoysTask::Toggle::Response &res)
{
	isEnabled = (req.enabled != 0);
	res.result = (int) isEnabled;
	return true;
}

void imgRecCallback(const SubImageRecognition::ImgRecObject &msg)
{
	if (isEnabled) {
		// TODO: Get/validate info from image recognition
		
		// TODO: Optionally send request to NavigationControl
	}
}

