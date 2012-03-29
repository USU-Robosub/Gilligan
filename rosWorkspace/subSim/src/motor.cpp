#include "motor.h"
#include <ros/ros.h>
#include <SubMotorController/MotorMessage.h>

void setMotors(unsigned int mask, 
		int DriveL, int DriveR, 
		int DiveF,  int DiveR, 
		int TurnF,  int TurnR) {
	static bool initialized = false;
	static ros::Publisher motorController;
	static ros::NodeHandle nh;
	if(!initialized) {
		motorController = nh.advertise<SubMotorController::MotorMessage>("/Motor_Control", 1);
		initialized = true;
	}
	SubMotorController::MotorMessage msg;
	msg.mask = mask;
	msg.Left = DriveL;
	msg.Right = DriveR;
	msg.FrontDepth = DiveF;
	msg.RearDepth = DiveR;
	msg.FrontTurn = TurnF;
	msg.RearTurn = TurnR;
	motorController.publish(msg);
}


