#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "subSim/mixer.h"
#include "std_msgs/Int16.h"
#include <stdio.h>

ros::Publisher outputPublisher;
ros::Subscriber feedbackSubscriber;

float currentValue;
float currentConfindence;

void inputCallback(const subSim::mixer::ConstPtr& msg) {
	float value = msg->value;
	float a = msg->confidence;
	if(a > 1)
		a = 1;
	if(a <= 0)
		return;
	float b = currentConfindence;
	bool useInvert = false;
	float smallest = a;
	if( b < a ) 
		smallest = b;
	if(1 - a < smallest || 1 - b < smallest)
		useInvert = true;
	float aPrime, bPrime, y;
	if(useInvert) {
		y = 2 - (a + b);
		if(y == 0)
			y = 1;
		aPrime = 1 - (1 - a) / y;
		bPrime = 1 - (1 - b) / y;
	} else {
		y = a + b;
		if(y == 0)
			y = 1;
		aPrime = a / y;
		bPrime = b / y;
	}
	printf("a' = %f \tb' = %f\n", aPrime, bPrime);
	currentValue = value * aPrime + currentValue * bPrime;
	currentConfindence = a * aPrime + currentConfindence * bPrime;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleMixer");
	currentValue = 0;
	currentConfindence = 0;
	ros::NodeHandle nh;
	outputPublisher = nh.advertise<subSim::mixer>("output", 1);
	ros::Subscriber inputSubscriber = nh.subscribe("input", 20, inputCallback);
	ros::Rate hertz(30);
	while(ros::ok()) {
		subSim::mixer msg;
		msg.value = currentValue;
		msg.confidence = currentConfindence;;
		currentConfindence *= .999;
		outputPublisher.publish(msg);
		ros::spinOnce();
		hertz.sleep();
	}
}
