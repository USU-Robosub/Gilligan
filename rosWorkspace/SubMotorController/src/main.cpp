#include <SerialStream.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "SubMotorController/MotorMessage.h"

#define LEFT_DRIVE_BIT  0x01
#define RIGHT_DRIVE_BIT 0x02
#define FRONT_DEPTH_BIT 0x04
#define REAR_DEPTH_BIT  0x08
#define FRONT_TURN_BIT  0x10
#define REAR_TURN_BIT   0x20

#define DRIVE_EXISTS 1
//#define DEPTH_EXISTS 1
//#define TURN_EXISTS  1

using namespace LibSerial;

enum MotorID {
	DRIVE,
	DEPTH,
	TURN
};

const int NUM_MOTOR_CONTROLLERS = 1;


SerialStream motorController [3];

void Error(char* message, char* responce) {
			printf("Error: Responce did not match message\n");
			printf("Sent \"%C %C %X %X %X %X %C\"\n",
						 message[0],
						 message[1],
						 message[2],
						 message[3],
						 message[4],
						 message[5],
						 message[6]);
			printf("Recieved \"%C %C %X %X %X %X %C\"\n",
						 responce[0],
						 responce[1],
						 responce[2],
						 responce[3],
						 responce[4],
						 responce[5],
						 responce[6]);
}

void setMotorSpeed(MotorID controller, int rightSpeed, int leftSpeed) {
	unsigned char message[] = "SD\0\0\0\0E";
	unsigned char responce[] = "Sd\0\0\0\0E";
	
	printf("setMotorSpeed(%d, %d, %d)\n", controller, rightSpeed, leftSpeed);

	if(rightSpeed < 0) {
		message[2] = leftSpeed;
		message[3] = 0;
	} else {
		message[2] = 0;
		message[3] = leftSpeed;
	}
	if(leftSpeed < 0) {
		message[4] = rightSpeed;
		message[5] = 0;
	} else {
		message[4] = 0;
		message[5] = rightSpeed;
	}

	printf("Array: ");
	for (int i = 0; i < 7; i++)
		printf("%d, ", message[i]);
	printf("\n");

	for(int i = 0;  i < 7; i++)
		motorController[controller] << message[i];
	for(int i = 0; i < 7; i++)
		motorController[controller] >> responce[i];
	for(int i = 0; i < 7; i++) {
		if((i != 1 && message[i] != responce[i] ) || message[i]-'A'+'a' != responce[i]) {
			Error((char*)message, (char*)responce);
		}
	}
}

float getCurrent(MotorID controller, int channel) {
	char message[] = "SCR\0\0\0E";
	char responce[] = "Sc\0\0\0\0E";
	if(channel == 1) 
		message[2] = 'L';
	for(int i = 0;  i < 7; i++)
		motorController[controller] << message[i];
	for(int i = 0; i < 7; i++)
		motorController[controller] >> responce[i];
	if(responce[0] != 'S' || responce[1] != 'c' || responce[6] != 'E') {
			Error(message, responce);
	}
	return *((float*)&responce[2]);
}

float getVoltage(MotorID controller) {
	char message[] = "SV\0\0\0\0E";
	char responce[] = "Sv\0\0\0\0E";
	for(int i = 0;  i < 7; i++)
		motorController[controller] << message[i];
	for(int i = 0; i < 7; i++)
		motorController[controller] >> responce[i];
	if(responce[0] != 'S' || responce[1] != 'c' || responce[6] != 'E') {
		Error(message, responce);
	}
	return *((float*)&responce[2]);
}

void motorMessage(const SubMotorController::MotorMessage::ConstPtr& msg) {
	static int curLDriveSpeed = 0,
			   curRDriveSpeed = 0,
			   curFDepthSpeed = 0,
			   curRDepthSpeed = 0,
			   curFTurnSpeed = 0,
			   curRTurnSpeed = 0;

	printf("msg: %d, %d, %d, %d, %d, %d, %d\n", msg->mask, msg->Left, msg->Right, msg->FrontDepth, msg->RearDepth, msg->FrontTurn, msg->RearTurn);

	curLDriveSpeed = msg->mask & LEFT_DRIVE_BIT  ? msg->Left       : curLDriveSpeed;
	curRDriveSpeed = msg->mask & RIGHT_DRIVE_BIT ? msg->Right      : curRDriveSpeed;
	curFDepthSpeed = msg->mask & FRONT_DEPTH_BIT ? msg->FrontDepth : curFDepthSpeed;
	curRDepthSpeed = msg->mask & REAR_DEPTH_BIT  ? msg->RearDepth  : curRDepthSpeed;
	curFTurnSpeed  = msg->mask & FRONT_TURN_BIT  ? msg->FrontTurn  : curFTurnSpeed;
	curRTurnSpeed  = msg->mask & REAR_TURN_BIT   ? msg->RearTurn   : curRTurnSpeed;

	setMotorSpeed(DRIVE, curLDriveSpeed, curRDriveSpeed);
	//setMotorSpeed(DEPTH, curFDepthSpeed, curFDepthSpeed);
	//setMotorSpeed(TURN,  curRTurnSpeed,  curRTurnSpeed);
}

int main(int argc, char** argv) {
	motorController[DRIVE].Open( "/dev/ttyUSB0" );
	//motorController[DEPTH].Open( "/dev/ttyUSB1" );
	//motorController[TURN] .Open( "/dev/ttyUSB2" );

	for (int i = 0; i < NUM_MOTOR_CONTROLLERS; i++) {
		if (!motorController[i].IsOpen()) {
			printf("Controller %d failed to open\n", i);
			return -1;
		}
	}

	for(int i = 0; i < NUM_MOTOR_CONTROLLERS; i++) {
		motorController[i].SetBaudRate( SerialStreamBuf::BAUD_115200 );
		motorController[i].SetCharSize( SerialStreamBuf::CHAR_SIZE_8 );
		motorController[i].SetNumOfStopBits( 1 );
		motorController[i].SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE );
		motorController[i].SetParity( SerialStreamBuf::PARITY_NONE );
	}

	

	ros::init(argc, argv, "SubImuController");

	ros::NodeHandle nh;

	ros::Subscriber MotorControl = nh.subscribe("/motorControl", 100, motorMessage);

	while (ros::ok()) {
		ros::spinOnce();
	}
}
