#include "ros/ros.h"
#include "ObstacleCourseTask/Toggle.h"
#include "SubImageRecognition/ImgRecObject.h"

using namespace std;

bool toggleCallback(ObstacleCourseTask::Toggle::Request &req, ObstacleCourseTask::Toggle::Response &res);
void imgRecCallback(const SubImageRecognition::ImgRecObject &msg);

bool isEnabled = false;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ObstacleCourseTask");

	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("toggleObstacleCourseServ", toggleCallback);
	ros::Subscriber subscriber = n.subscribe("image_recognition/forward/obstacle_course", 10, imgRecCallback);
	ros::spin();
	
	return 0;
}

bool toggleCallback(ObstacleCourseTask::Toggle::Request &req, ObstacleCourseTask::Toggle::Response &res)
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

