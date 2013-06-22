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

int filter(int speed) {
	return (double)speed /100.0 * 255;
}

void setDrive(int speed) {
	speed = filter(speed);
	setMotors(RIGHT_DRIVE_MOTOR_BIT | LEFT_DRIVE_MOTOR_BIT,
			speed, speed, 0, 0, 0, 0);
}

void setStraf(int speed) {
	speed = filter(speed);
	setMotors(FRONT_TURN_MOTOR_BIT | REAR_TURN_MOTOR_BIT,
			0, 0, speed, -speed, 0, 0);
}

void setTurn(int speed) {
	speed = filter(speed);
	setMotors(FRONT_TURN_MOTOR_BIT | REAR_TURN_MOTOR_BIT,
			0, 0, speed, speed, 0, 0);
}

void setDive(int speed) {
	speed = filter(speed);
	setMotors(FRONT_DEPTH_MOTOR_BIT | REAR_DEPTH_MOTOR_BIT,
			0, 0, 0, 0, speed, speed);
}
