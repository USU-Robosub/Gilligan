#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "Robosub/HighLevelControl.h"
#include "Robosub/Point.h"
#include "SubMotorController/MotorMessage.h"

#include <string>

using namespace std;

#define LEFT_DRIVE_BIT  0x01
#define RIGHT_DRIVE_BIT 0x02
#define FRONT_DEPTH_BIT 0x04
#define REAR_DEPTH_BIT  0x08
#define FRONT_TURN_BIT  0x10
#define REAR_TURN_BIT   0x20

const double FORWARD_SPEED_CONST = .0000006;
const double FORWARD_DRAG_CONST = .99;
const double STRAF_SPEED_CONST = .0000004;
const double STRAF_DRAG_CONST = .98;

enum Mode {
	MANUAL,
	AUTOMATIC
};

const double Epsilon = .4;

Mode ForwardMode = MANUAL;
Mode StrafMode = MANUAL;
Mode TurnMode = MANUAL;
Mode DepthMode = MANUAL;
Mode YawMode = MANUAL;
Mode PitchMode = MANUAL;

double ForwardSpeed = 0;
double StrafSpeed = 0;
double TurnSpeed = 0;
double DepthSpeed = 0;
double YawSpeed = 0;
double PitchSpeed = 0;

double ForwardOffset = 0;
double StrafOffset = 0;
double TurnOffset = 0;
double DepthOffset = 0;
double YawOffset = 0;
double PitchOffset = 0;

double CurrentDepth = 0;
double CurrentYaw = 0;
double CurrentTurn = 0;
double CurrentPitch = 0;
double ForwardVelocity = 0;
double StrafVelocity = 0;

bool DepthInMotion = false;
ros::Publisher* motorPub;

void CenterOnPointCallback(Robosub::Point::ConstPtr msg) {
	TurnMode = AUTOMATIC;
	TurnOffset = msg->x * 60 / 940;
	DepthMode = AUTOMATIC;
	DepthOffset = (40 - msg->y) / 150.0;
}

void commandCallback(Robosub::HighLevelControl::ConstPtr msg) {
	if(msg->Direction == "Forward" && msg->MotionType == "Offset") {
		ForwardMode = AUTOMATIC;
		ForwardOffset = msg->Value;
	} else if (msg->Direction == "Forward" && msg->MotionType == "Manual") {
		ForwardMode = MANUAL;0
		ForwardSpeed = msg->Value;

	} else if(msg->Direction == "Turn" && msg->MotionType == "Offset") {
		TurnMode = AUTOMATIC;
		TurnOffset = msg->Value;
	} else if (msg->Direction == "Turn" && msg->MotionType == "Manual") {
		TurnMode = MANUAL;
		TurnSpeed = msg->Value;

	} else if(msg->Direction == "Straf" && msg->MotionType == "Offset") {
		StrafMode = AUTOMATIC;
		StrafOffset = msg->Value;
	} else if (msg->Direction == "Straf" && msg->MotionType == "Manual") {
		StrafMode = MANUAL;
		StrafSpeed = msg->Value;

	} else if(msg->Direction == "Depth" && msg->MotionType == "Offset") {
		DepthMode = AUTOMATIC;
		DepthOffset = msg->Value;
	} else if (msg->Direction == "Depth" && msg->MotionType == "Manual") {
		DepthMode = MANUAL;
		DepthSpeed = msg->Value;

	} else if(msg->Direction == "Yaw" && msg->MotionType == "Offset") {
		YawMode = AUTOMATIC;
		YawOffset = msg->Value;
	} else if (msg->Direction == "Yaw" && msg->MotionType == "Manual") {
		YawMode = MANUAL;
		YawSpeed = msg->Value;


	} else if(msg->Direction == "Pitch" && msg->MotionType == "Offset") {
		PitchMode = AUTOMATIC;
		PitchOffset = msg->Value;
	} else if (msg->Direction == "Pitch" && msg->MotionType == "Manual") {
		PitchMode = MANUAL;
		PitchSpeed = msg->Value;

	} else {
		printf("Unknown Direction: %s and Mode: %s\n", msg->Direction.c_str(), msg->MotionType.c_str());
	}
}

void sendMotorMessage(unsigned int mask, int FR, int FL, int TR, int TF, int DR, int DF) {
	SubMotorController::MotorMessage msg;
	msg.mask = mask;
	msg.Left = FL;
	msg.Right = FR;
	msg.FrontDepth = DF;
	msg.RearDepth = DR;
	msg.FrontTurn = TF;
	msg.RearTurn = TR;
	motorPub->publish(msg);
}

void updateDepth(std_msgs::Float32::ConstPtr msg) {
	DepthOffset = DepthOffset + CurrentDepth - msg->data;
	CurrentDepth = msg->data;
//	printf("DepthOffset = %f\n", DepthOffset);
}

void updateAttitude(const std_msgs::Float32MultiArray::ConstPtr msg) {
    //This function is where controllers should go.
    //It is called at 100MHz

    //YAW
    //Updates the yaw offset with the old value and the new value
	//              INPUT     + (           ERROR          )
	YawOffset = YawOffset + CurrentYaw - msg->data[2]; //Yaw is now in [2]
	TurnOffset = TurnOffset + CurrentYaw - msg->data[2];
	CurrentYaw = msg->data[2];

    //PITCH
    //Updates the yaw offset with the old value and the new value
    //              INPUT     + (           ERROR          )
	PitchOffset = PitchOffset + CurrentPitch - msg->data[1];
	CurrentPitch = msg->data[1];
//	printf("Turn Message = %lf\n", msg->data[0]);
}


void UpdateForwardVelocity() {
    //Seems to update a velocity memory by increasing based ont the speed constant
    //and limiting using the drag constant
	ForwardVelocity += ForwardSpeed * FORWARD_SPEED_CONST;
	ForwardVelocity *= FORWARD_DRAG_CONST;
}

void UpdateStrafVelocity() {
    //Seems to update a velocity memory by increasing based ont the speed constant
    //and limiting using the drag constant
	StrafVelocity += StrafSpeed * STRAF_SPEED_CONST;
	StrafVelocity *= STRAF_DRAG_CONST;
}

int sanitize(int speed) {

	if(speed > 0 && speed < 60)
		speed = 40;
	else if (speed < 0 && speed > -60)
		speed = -40;
	if(speed > 255)
		return 255;
	if(speed < -255)
		return -255;
	return speed;
}



int CalcDepth() {
	if(DepthMode == AUTOMATIC) {
		double speed = DepthOffset / 1.5;
		return sanitize(speed * 255.0);
	} else {
		return DepthSpeed;
	}
}

int CalcTurn() {
	if(TurnMode == AUTOMATIC) {
        //Update the current turning speed using only 12.5% of the given value
		TurnSpeed = sanitize(TurnOffset / 8.0 * 255)/2;
		printf("Offset = %f Speed = %f\n", TurnOffset, TurnSpeed);
	}
	return TurnSpeed;
}

int CalcStraf() {
	if(StrafMode == AUTOMATIC) {
		StrafOffset -= StrafVelocity;
		StrafSpeed = sanitize(StrafOffset / .5 * 255.0);
	}
	return StrafSpeed;
}

int CalcPitch() {
	if(PitchMode == AUTOMATIC) {
		double speed = PitchOffset / 10;
		return sanitize(speed * 255.0);
	} else {
		return PitchSpeed;
	}
}



void ManageForwardThrusters() {
	static int currentValueRight;
	static int currentValueLeft;

	UpdateForwardVelocity();
	if(ForwardMode == AUTOMATIC) {
		ForwardOffset -= ForwardVelocity;
//		printf("forward offset = %f\n", ForwardOffset);
//		printf("forward velocity = %f\n", ForwardVelocity);
		double speed = ForwardOffset / 1.5;
		ForwardSpeed = sanitize(speed * 255.0);
	}
	if(currentValueLeft != ForwardSpeed ||
			currentValueRight != ForwardSpeed) {
		sendMotorMessage(LEFT_DRIVE_BIT | RIGHT_DRIVE_BIT,
				ForwardSpeed, ForwardSpeed, 0, 0, 0, 0);
		currentValueRight = ForwardSpeed;
		currentValueLeft = ForwardSpeed;
	}
}

void ManageTurnThrusters() {
	static int currentValueRear;
	static int currentValueFront;

	int TurnThrust = CalcTurn();
	int StrafThrust = CalcStraf();

	int RearThrust = TurnThrust + StrafThrust;
	int FrontThrust = TurnThrust - StrafThrust;

	UpdateStrafVelocity();

	RearThrust = sanitize(RearThrust);
	FrontThrust = sanitize(FrontThrust);
	if(RearThrust != currentValueRear ||
	   FrontThrust != currentValueFront) {

		sendMotorMessage(REAR_TURN_BIT | FRONT_TURN_BIT,
				0, 0, RearThrust, FrontThrust, 0, 0);
		currentValueRear = RearThrust;
		currentValueFront = FrontThrust;
	}
}

void ManageDepthThrusters() {
	static int currentValueRear;
	static int currentValueFront;

	int DepthThrust = CalcDepth();
	int PitchThrust = CalcPitch();

	int FrontThrust = DepthThrust + PitchThrust;
	int RearThrust = DepthThrust - PitchThrust;

	if(true || RearThrust != currentValueRear ||
			FrontThrust != currentValueFront) {
		//printf("Depth %d %d\n", RearThrust, FrontThrust);
		sendMotorMessage(REAR_DEPTH_BIT | FRONT_DEPTH_BIT,
				0, 0, 0, 0, RearThrust, FrontThrust);
		currentValueRear = RearThrust;
		currentValueFront = FrontThrust;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "HighLevelControl");
	ros::NodeHandle nh;
	ros::Publisher motorPublisher = nh.advertise<SubMotorController::MotorMessage>("Motor_Control", 1);
	motorPub = &motorPublisher;
	ros::Subscriber DepthSub = nh.subscribe("Sub_Depth", 1, updateDepth);
	ros::Subscriber AttitudeSub = nh.subscribe("IMU_Attitude", 1, updateAttitude);
	ros::Subscriber CommandSub = nh.subscribe("High_Level_Motion", 10, commandCallback);
	ros::Subscriber PointSub = nh.subscribe("Center_On_Point", 10, CenterOnPointCallback);

	while(ros::ok()) {
		ManageForwardThrusters();
		ManageTurnThrusters();
		ManageDepthThrusters();
		ros::spinOnce();
		usleep(10000);
	}
}
