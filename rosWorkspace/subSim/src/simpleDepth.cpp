#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "motor.h"
#include <Robosub/ModuleEnableMsg.h>

#define OFF 0
#define ON 1

float targetDepth = 0;

const int LEFT_MOTOR = 0x400;
const int RIGHT_MOTOR = 0x200;
const int REVERSED = 0x100;

bool MODE = ON;

//TODO Subscribe this module to the IMU data
//TODO Add the PI Controller
//TODO Add the stabilizer: A PI controller?
//TODO Make the stabilizer independent and add the output to the motor signal
//TODO Make the stabilizer output differential (so it can add or substract from the motor signal)

ros::Publisher depthMotor;

void mTargetDepthCallback(const std_msgs::Float32::ConstPtr& msg) {
	targetDepth = msg->data;
	printf("setting target depth to %f\n", targetDepth);
}

void setDepthSpeed(float speed) {
	speed *= 255;
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
			-speedInt, -speedInt,
			0,        0
		);
}

void mEnabledCallback(const Robosub::ModuleEnableMsg::ConstPtr& msg) {
	if(msg->Module == "Simple_Depth")
		MODE = msg->State;
}

void mCurrentDepthCallback(const std_msgs::Float32::ConstPtr& msg) {
	if(MODE == OFF)
		return;
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
	ros::Subscriber imuAttitude = nh.subscribe("")
	ros::Subscriber enabled = nh.subscribe("/Module_Control", 1, mEnabledCallback);
	ros::spin();
}
