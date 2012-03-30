#include "ros/ros.h"
#include "std_msgs/Float32.h"

float offset = 83;
float scalingFactor = 0.2; //feet per tick
float currentDepthRaw = 16.6;
float lastTicks = 83;

ros::Publisher depthPub;

void tickMeasureCallback(const std_msgs::Float32 msg) {
	lastTicks = msg.data;
	currentDepthRaw = (lastTicks-offset) * scalingFactor;
	std_msgs::Float32 outMsg;
	outMsg.data = currentDepthRaw;
	depthPub.publish(outMsg);
}

void calibrationCallback(const std_msgs::Float32 msg) {
	float depth = msg.data;
	if(depth == 0) {
		offset = lastTicks;
	} else {
		scalingFactor = depth / (lastTicks - offset);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "DepthTranslator");
	ros::NodeHandle nh;
	ros::Subscriber curTicks = nh.subscribe("/Pressure_Data", 1, tickMeasureCallback);
	ros::Subscriber calibrationSub = nh.subscribe("/Calibrate_Depth", 1, calibrationCallback);
	depthPub = nh.advertise<std_msgs::Float32>("/Sub_Depth", 1);
	ros::spin();
}
