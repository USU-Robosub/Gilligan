#include "ros/ros.h"
#include "Robosub/HighLevelControl.h"
#include "SubMotorController/MotorMessage.h"

enum Mode {
	MANUAL,
	AUTOMATIC
};

Mode ForwardMode = MANUAL;
Mode StrafMode = MANUAL;
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

double CurrentDepth;
double CurrentYaw;
double CurrentTurn;
double CurrentPitch;
double CurrentForwardSpeed;
double CurrentStrafSpeed;

ros::Publisher* motorPub;

void commandCallback(Robosub::HighLevelControl::ConstPtr msg) {
	if(msg->Direction == "Forward" && msg->MotionType == "Offset") {
		ForwardMode = AUTOMATIC;
		ForwardOffset = msg->Value;
	} else if (msg->Direction == "Forward" && msg->MotionType == "Manual") {
		ForwardMode = MANUAL;
		ForwardSpeed = msg->Value;
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
}

void sendMotorMessage(unsigned int mask, int FR, int FL, int TR, int TF, int DR, int DF) {
	SubMotorController::MotorMessage msg;
	msg->mask = mask;
	msg->Left = FL;
	msg->Right = FR;
	msg->FrontDepth = DF;
	msg->RearDepth = DR;
	msg->FrontTurn = TF;
	msg->RearTurn = TR;
	motorPub->publish(msg);
}

void updateDepth(const std_msgs::Float32::ConstPtr& msg) {
	DepthOffset = DepthOffset + msg->data - CurrentDepth;
	CurrentDepth = msg->data;
	if(DepthMode == AUTOMATIC && DepthOffset < Epsilon && DepthInMotion) {
		DepthInMotion = false;
		SignalCompletion("Depth");
	}
}

void updateAttitude(const std_msgs::Float32Multiarray::ConstPtr msg) {
	YawOffset = YawOffset + msg->data[0] - CurrentYaw;
	CurrentYaw = msg[0];

	PitchOffset = PitchOffset + msg->data[1] - CurrentPitch;
	CurrentPitch = msg[1];
}

int CalcDepth() {
	if(DepthMode == AUTOMATIC) {
		int speed = DepthOffset / 1.5;
		return sanatize(speed);
	}
}

int CalcTurn() {
	if(TurnMode == AUTOMATIC) {
		int speed = TurnOffset / 20;
		return sanatize(speed);
	} else {
		return TurnSpeed;
	}
}

int CalcStraf() {
	if(StrafMode == AUTOMATIC) {
		int speed = StrafOffset / .5;
		return sanatize(speed);
	} else {
		return StrafSpeed;
	}
}

int CalcPitch() {
	if(PitchMode == AUTOMATIC) {
		int speed = PitchOffset / 10;
		return sanatize(speed);
	} else {
		return PitchSpeed;
	}
}

void ManageForwardThrusters() {
	static int currentValueRight;
	static int currentValueLeft;

	if(ForwardMode == MANUAL) {
		if(currentValueLeft  != ForwardSpeed ||
		   currentValueRight != ForwardSpeed) {
			sendMotorMessage(LEFT_DRIVE_BIT | RIGHT_DRIVE_BIT,
					ForwardSpeed, ForwardSpeed, 0, 0, 0, 0);
		}
	} else {
		//Write this later
	}
}

void ManageTurnThrusters() {
	static int currentValueRear;
	static int currentValueFront;

	int TurnThrust = CalcTurn();
	int StrafThrust = CalcStraf();

	int RearThrust = TurnThrust + StrafThrust;
	int FrontThrust = TurnThrust - StrafThrust;
	if(RearThrust != currentValueRear ||
	   FrontThrust != currentValueFront) {
		sendMotorMessage(REAR_TURN_BIT | FRONT_TURN_BIT,
				0, 0, RearThrust, FrontThrust, 0, 0);
	}
}

void ManageDepthThrusters() {
	static int currentValueRear;
	static int currentValueFront;

	int DepthThrust = CalcDepth();
	int PitchThrust = CalcPitch();

	int FrontThrust = DepthThrust + PitchThrust;
	int RearThrust = DepthThrust - PitchThrust;

	if(RearThrust != currentValueRear ||
	   FrontThrust != currentValueFront) {
		sendMotorMessage(REAR_DEPTH_BIT | FRONT_DEPTH_BIT,
				0, 0, 0, 0, RearThrust, FrontThrust);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv);
	ros::NodeHandle nh;
	motorPub = new ros::Publisher(
	ros::Subscriber DepthSub = nh.subscribe("/Sub_Depth", 1, updateDepth);
	ros::Subscriber AttitudeSub = nh.subscribe("/Sub_Attitude", 1, updateAttitude);
	ros::Subscriber CommandSub = nh.subscribe("/High_Level_Motion", 100, commandCallback);

	while(ros::ok()) {
		ManageForwardThrusters();
		ManageTurnThrusters();
		ManageDepthThrusters();
		ros::spinOnce();
		usleep(10000);
	}
}
