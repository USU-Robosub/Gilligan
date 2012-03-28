#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

float targetHeading = 0;

const int LEFT_MOTOR = 0x400;
const int RIGHT_MOTOR = 0x200;
const int REVERSED = 0x100;

ros::Publisher turnMotor;

std_msgs::Int16 GenMotorMessage(float speed) {
	int message = LEFT_MOTOR + RIGHT_MOTOR;
	int SpeedInt = speed * 255;
	if(SpeedInt < 0) {
		message |= REVERSED;
		SpeedInt = -SpeedInt;
	}
	if(SpeedInt > 255)
		SpeedInt = 255;
	std_msgs::Int16 msg;
	msg.data = message + SpeedInt;
	return msg;
}

void mTargetHeadingCallback(const std_msgs::Float32::ConstPtr& msg) {
	targetHeading = msg->data;
	while(targetHeading > 2 * M_PI)
		targetHeading -= 2*M_PI;
	while (targetHeading < 0)
		targetHeading += 2*M_PI;
	printf("setting target heading to %f\n", targetHeading);
}

void mCurrentHeadingCallback(const std_msgs::Float32::ConstPtr& msg) {
	float heading = msg->data;
	float speed = 0;
	if(heading - targetHeading > 1)
		speed = 1;
	else if(heading - targetHeading < -1)
		speed = -1;
	else
		speed = heading - targetHeading;
	turnMotor.publish(GenMotorMessage(speed));
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleHeadingController");
	ros::NodeHandle nh;
	turnMotor = nh.advertise<std_msgs::Int16>("/Motor_Driver_Turn", 1);
	ros::Subscriber curHeading = nh.subscribe("/sim/rotation/yaw", 1, mCurrentHeadingCallback);
	ros::Subscriber targetHeading = nh.subscribe("/target/yaw", 1, mTargetHeadingCallback);
	ros::spin();
}
