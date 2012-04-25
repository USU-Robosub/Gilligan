#include "ros/ros.h"
#include "GateTask/Toggle.h"
#include "SubImageRecognition/ImgRecObject.h"
#include <ctime>
#include <string>
#include <list>

using namespace std;

bool toggleCallback(GateTask::Toggle::Request &req, GateTask::Toggle::Response &res);
void imgRecCallback(const SubImageRecognition::ImgRecObject &msg);

const int MAX_NUM_MSG = 10;
const float MIN_CONFIDENCE = 0.4;
//This is the min number of pixels we look for on the top crossbar before diving to allow a center pass through the gate 
const int HEIGHT_DIVE = 100; 

bool isEnabled = false;

//vectors to store multiple callbacks
list<SubImageRecognition::ImgRecObject> listOfMsgs;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "GateTask");

	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("toggleGateServ", toggleCallback);
	ros::Subscriber subscriber = n.subscribe("image_recognition/forward/gate", 10, imgRecCallback);
	ros::spin();
	
	return 0;
}

bool toggleCallback(GateTask::Toggle::Request &req, GateTask::Toggle::Response &res)
{
	isEnabled = (req.enabled != 0);
	res.result = (int) isEnabled;
	return true;
}

/*
The info from the callback will return a horizontal rectangle that should represent the top crossbar
This task will keep the last 10 msg's in memory, and will only look at messages greater than the specified 
MIN_CONFIDENCE (currently 0.4)
*/
void imgRecCallback(const SubImageRecognition::ImgRecObject &msg)
{
	if (isEnabled) 
	{
		//Only want to add the info to the list if the confidence is above the min requirement
		if(msg.confidence > MIN_CONFIDENCE)
		{
			listOfMsgs.push_front(msg);
			//Once max size is surpassed, pop the last element in the list
			if(listOfMsgs.size() > MAX_NUM_MSG)
				listOfMsgs.pop_back();
			
			int newX = 0;
			int newY = 220;
			
			//Keep center_x == 0 to keep the submarine in the middle area of the gate
			
			
			//Keep center_y == 220 to keep the submarine below the top crossbar
			//center_y == 220 will put the center of the crossbar 100px from the top 
			
			//If height of rectangle is above a certain threshold (HEIGHT_DIVE), we want to dive about 1.5 - 2 feet and keep true (BBT FTW)
			//we do this because we know that we are close enough to the gate that we can lose sight of the gate and still pass through.
			
			// TODO: Get/validate info from image recognition
			
			// TODO: Optionally send request to NavigationControl
		}
	}
}

