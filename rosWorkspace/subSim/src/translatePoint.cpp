#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "SubImageRecognition/ImgRecObject.h"
#include "Robosub/Point.h"

ros::Publisher Output;
ros::Subscriber Input;
ros::NodeHandle* nh;

int OffsetX, OffsetY;

void PointCallback(const SubImageRecognition::ImgRecObject::ConstPtr& msg) {
	Robosub::Point ret;
	ret.x = msg->center_x + OffsetX;
	ret.y = msg->center_y = OffsetY;
	Output.publish(ret);
}

void OffsetXCallback(const std_msgs::Int32::ConstPtr& msg) {
	OffsetX = msg->data;
}

void OffsetYCallback(const std_msgs::Int32::ConstPtr& msg) {
	OffsetY = msg->data;
}

void SetInput(const std_msgs::String::ConstPtr& msg) {
	Input = nh.subscribe(msg->data, 1, PointCallback);
}

void SetOutput(const std_msgs::String::ConstPtr& msg) {
	Output = nh->advertise<Robosub::Point>(msg->data, 1);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleHeadingController");
	nh = new ros::NodeHandle();
	ros::Subscriber ChangeInput = nh->subscribe("ChangeInput", 1, SetInput);
	ros::Subscriber ChangeOutput = nh->subscribe("ChangeOutput", 1, SetOutput);
	ros::Subscriber OffsetXSub = nh->subscribe("OffsetX", 1, OffsetXCallback);
	ros::Subscriber OffsetYSub = nh->subscribe("OffsetY", 1, OffsetYCallback);
	ros::spin();
}
