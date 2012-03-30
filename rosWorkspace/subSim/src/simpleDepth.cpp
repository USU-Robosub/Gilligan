#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "motor.h"

float targetDepth = 0;

const int LEFT_MOTOR = 0x400;
const int RIGHT_MOTOR = 0x200;
const int REVERSED = 0x100;

ros::Publisher depthMotor;

void mTargetDepthCallback(const std_msgs::Float32::ConstPtr& msg) {
	targetDepth = msg->data;
	printf("setting target depth to %f\n", targetDepth);
}

void setDepthSpeed(float speed) {
	speed *= -255;
	int speedInt = speed; //truncate to an integer value
	if(speedInt > 255)
		speedInt = 255;
	if(speedInt < -255)
		speedInt = -255;
//	setMotors(REAR_DEPTH_MOTOR_BIT | FRONT_DEPTH_MOTOR_BIT,  //<- bit mask
//			0,        0,          //<- Drive motors (ignored because of mask)
//			speedInt, speedInt,   //<- Depth motors 
//			0,        0           //<- Turn motors  (also ignored)
//		);
	setMotors(REAR_DEPTH_MOTOR_BIT | FRONT_DEPTH_MOTOR_BIT,  
			0,        0,          
			speedInt, speedInt, 
			0,        0          
		);
}

void mCurrentDepthCallback(const std_msgs::Float32::ConstPtr& msg) {
	float depth = msg->data;
	float speed = 0;
	if(depth - targetDepth > 1.5)
		speed = 1;
	else if(depth - targetDepth < -1.5)
		speed = -1;
	else
		speed = depth - targetDepth;
	setDepthSpeed(speed);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleDepthController");
	ros::NodeHandle nh;
	ros::Subscriber curDepth = nh.subscribe("Sub_Depth", 1, mCurrentDepthCallback);
	ros::Subscriber targetDepth = nh.subscribe("/Target_Depth", 1, mTargetDepthCallback);
	ros::spin();
}
