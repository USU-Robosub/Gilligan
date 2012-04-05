#include "motorController.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <SubMotorController/MotorMessage.h>

#define LEFT_DRIVE_BIT  0x01
#define RIGHT_DRIVE_BIT 0x02
#define FRONT_DEPTH_BIT 0x04
#define REAR_DEPTH_BIT  0x08
#define FRONT_TURN_BIT  0x10
#define REAR_TURN_BIT   0x20

//using namespace LibSerial;

MotorControllerHandler* motorControllerDrive;
MotorControllerHandler* motorControllerDepth;
MotorControllerHandler* motorControllerTurn;

void setMotorSpeed(MotorControllerHandler* controller, int rightSpeed, int leftSpeed) {
	if(rightSpeed >= 256)
		rightSpeed = 255;
	if(rightSpeed <= -256)
		rightSpeed = -255;	
	if(leftSpeed >= 256)
		leftSpeed = 255;
	if(leftSpeed <= -256)
		leftSpeed = -255;	

	Message msg;
	msg.type = MOTOR_TYPE;

	if(leftSpeed > 0) {
		msg.DataC[0] = 0;
		msg.DataC[1] = leftSpeed;
	} else {
		int rev = -leftSpeed;
		msg.DataC[0] = rev;
		msg.DataC[1] = 0;
	}
	if(rightSpeed > 0) {
		msg.DataC[2] = 0;
		msg.DataC[3] = rightSpeed;
	} else {
		int rev = -rightSpeed;
		msg.DataC[2] = rev;
		msg.DataC[3] = 0;
	}

	controller->sendMessage(msg);
}


void motorMessage(const SubMotorController::MotorMessage::ConstPtr& msg) {
	static int curLDriveSpeed = 0,
			   curRDriveSpeed = 0,
			   curFDepthSpeed = 0,
			   curRDepthSpeed = 0,
			   curFTurnSpeed = 0,
			   curRTurnSpeed = 0;


	curLDriveSpeed = msg->mask & LEFT_DRIVE_BIT  ? msg->Left       : curLDriveSpeed;
	curRDriveSpeed = msg->mask & RIGHT_DRIVE_BIT ? msg->Right      : curRDriveSpeed;
	curFDepthSpeed = msg->mask & FRONT_DEPTH_BIT ? msg->FrontDepth : curFDepthSpeed;
	curRDepthSpeed = msg->mask & REAR_DEPTH_BIT  ? msg->RearDepth  : curRDepthSpeed;
	curFTurnSpeed  = msg->mask & FRONT_TURN_BIT  ? msg->FrontTurn  : curFTurnSpeed;
	curRTurnSpeed  = msg->mask & REAR_TURN_BIT   ? msg->RearTurn   : curRTurnSpeed;

	if(msg->mask & (LEFT_DRIVE_BIT | RIGHT_DRIVE_BIT))
		motorControllerDrive->setMotorSpeed(curRDriveSpeed, curLDriveSpeed);
	if(msg->mask & (FRONT_DEPTH_BIT | REAR_DEPTH_BIT))
		motorControllerDepth->setMotorSpeed(curRDepthSpeed, curFDepthSpeed);
	if(msg->mask & (FRONT_TURN_BIT | REAR_TURN_BIT))
		motorControllerTurn->setMotorSpeed(curRTurnSpeed,  curFTurnSpeed);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "SubMotorController");
	ros::NodeHandle nh;
	printf("waiting for the controllers to reset...\n");
	motorControllerDrive = new MotorControllerHandler(&nh, "/dev/controller_drive");
	motorControllerDepth = new MotorControllerHandler(&nh, "/dev/controller_dive");
	motorControllerTurn = new MotorControllerHandler(&nh, "/dev/controller_turn");
	sleep(2);
	ros::Subscriber MotorControl = nh.subscribe("/Motor_Control", 100, motorMessage);
	//ros::Publisher MotorCurrent = nh.advertise("/motorStatus/Current/DriveR", 1, std_msgs::Float32);
	while (ros::ok()) {
		motorControllerDrive->spinOnce();
		motorControllerDepth->spinOnce();
		motorControllerTurn->spinOnce();
		ros::spinOnce();
		usleep(10000);
	}
}
