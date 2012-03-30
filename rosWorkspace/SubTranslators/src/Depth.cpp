#include "ros/ros.h"
#include "std_msgs/Float32.h"

float offset = 83;
float scalingFactor = .2; //Ticks per foot

ros::Publisher depthPub;

void tickMeasureCallback(const std_msgs::Float32 msg) {
	float ticks = msg.data;
	float currentDepth = (ticks-offset) * scalingFactor;
	std_msgs::Float32 outMsg;
	outMsg.data = currentDepth;
	depthPub.publish(outMsg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "DepthTranslator");
	ros::NodeHandle nh;
	ros::Subscriber curTicks = nh.subscribe("/Pressure_Data", 1, tickMeasureCallback);
	depthPub = nh.advertise<std_msgs::Float32>("/Sub_Depth", 1);
	ros::spin();
}
