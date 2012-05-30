#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "motor.h"
#include <Robosub/ModuleEnableMsg.h>
#include <Robosub/Point.h>

#define OFF 0
#define ON 1

const int OFFSET_X = 0;
const int OFFSET_Y = 0;
const int LEFT_MOTOR = 0x400;
const int RIGHT_MOTOR = 0x200;
const int REVERSED = 0x100;

bool MODE = OFF;

int getSpeed(int value) {
	float factor = 1;
	value /= factor;
	if(value > 255)
		value = 255;
	if(value < -255)
		value = -255;
	return value;
}

void mEnabledCallback(const Robosub::ModuleEnableMsg::ConstPtr& msg) {
	if(msg->Module == "Center_On_Point")
		MODE = msg->State;
}

void mTargetCallback(const Robosub::Point::ConstPtr& msg) {
	if(MODE == OFF)
		return;

	int depthSpeedInt = getSpeed(msg->y - OFFSET_Y);
	int strafSpeedInt = getSpeed(msg->x - OFFSET_X);

	setMotors(REAR_TURN_MOTOR_BIT | FRONT_TURN_MOTOR_BIT |
			  REAR_DEPTH_MOTOR_BIT | FRONT_DEPTH_MOTOR_BIT,
			0,             0,          
			depthSpeedInt, depthSpeedInt, 
			strafSpeedInt, -strafSpeedInt          
		);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "SimpleDepthController");
	ros::NodeHandle nh;
	ros::Subscriber targetDepth = nh.subscribe("/Target_Point", 1, mTargetCallback);
	ros::Subscriber enabled = nh.subscribe("/Module_Control", 1, mEnabledCallback);
	ros::spin();
}
