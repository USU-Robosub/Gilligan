#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "subSim/mixer.h"
#include "std_msgs/Int16.h"
#include <stdio.h>

ros::Publisher outputPublisher;

float confidence = .5;

void inputCallback(const std_msgs::Float32::ConstPtr& inMsg) {
		subSim::mixer msg;
		msg.value = inMsg->data;
		msg.confidence = confidence;
		outputPublisher.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "FloatRepeater");
	ros::NodeHandle nh;
	outputPublisher = nh.advertise<subSim::mixer>("output", 1);
	ros::Subscriber inputSubscriber = nh.subscribe("input", 20, inputCallback);
	ros::spin();
}
