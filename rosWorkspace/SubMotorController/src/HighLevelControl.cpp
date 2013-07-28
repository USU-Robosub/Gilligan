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

#define TARGET_ZERO -500

const double FORWARD_SPEED_CONST = .0000006;
const double FORWARD_DRAG_CONST = .99;
const double STRAF_SPEED_CONST = .0000004;
const double STRAF_DRAG_CONST = .98;

const double LEFT_FWD_MULT = 0.78;
const double REAR_TURN_MULT = .8928; //right
const double FRONT_TURN_MULT = .83; //left
double RIGHT_PIVOT_MULT = 1.0; //Multiplier to change the pivot point to be under the camera
double LEFT_PIVOT_MULT = 1.0;

enum Mode {
	MANUAL,
	AUTOMATIC
};

const double Epsilon = .4;

Mode ForwardMode = MANUAL;
Mode StrafeMode = MANUAL;
Mode TurnMode = MANUAL;
Mode DepthMode = MANUAL;
Mode YawMode = MANUAL;
Mode PitchMode = MANUAL;
Mode PivotMode = MANUAL;

//Speeds come from the controllers
int ForwardSpeed = 0;
int StrafeSpeed = 0;
int TurnSpeed = 0;
int DepthSpeed = 0;
int YawSpeed = 0;
int PitchSpeed = 0;
int PivotSpeed = 0;

//Commands come from the tasks
double ForwardCommand = 0;
double StrafeCommand = 0;
double TurnCommand = 0;
double DepthCommand = 0;
double YawCommand = 0;
double PitchCommand = 0;

double CurrentDepth = 0;
double CurrentYaw = 0;
double CurrentTurn = 0;
double CurrentPitch = 0;
double ForwardVelocity = 0;
double StrafeVelocity = 0;

//These store the last speeds sent to the motors
int currentFwdRight = 0;
int currentFwdLeft = 0;
int currentTurnRear = 0;
int currentTurnFront = 0;
int currentDepthRear = 0;
int currentDepthFront = 0;
int currentPivotRear = 0;
int currentPivotFront = 0;

bool DepthInMotion = false;

ros::Publisher motorPublisher;
ros::Publisher depthPublisher;
ros::Publisher headingPublisher;

void CenterOnPointCallback(Robosub::Point::ConstPtr msg) {
	TurnMode = AUTOMATIC;
	TurnCommand = msg->x * 60 / 940;
	DepthMode = AUTOMATIC;
	DepthCommand = (40 - msg->y) / 150.0;
}

int makeSpeed(float percent)
{
    //The smallest output is 60
    //The largets output is 255

    //map from 100 to 255 and from 60 to 1 percent
    if (fabs(percent)<0.01)
        return 0;
    int p = (percent) * (255-60);
    return p>0?p+60:p-60;
    //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


}


//Changed the previous word of Offset to Command
//Changed the name Straf to Strafe EVERYWHERE
//All the MANUAL inputs are [0,1]

/*
AUTOMATIC ("Command") inputs differ:
Forward: percentage [0,1];
Turn: angle [-180,180];
Strafe: percentage [0,1];
Depth: distance [0,14]ft;
Pitch: angle [-10,10];
*/
void commandCallback(Robosub::HighLevelControl::ConstPtr msg) {
	if(msg->Direction == "Forward" && msg->MotionType == "Command") {
		ForwardMode = AUTOMATIC;
		ForwardCommand = msg->Value;
	} else if (msg->Direction == "Forward" && msg->MotionType == "Manual") {
		ForwardMode = MANUAL;
		ForwardSpeed = makeSpeed(msg->Value);

	} else if(msg->Direction == "Turn" && msg->MotionType == "Command") {
		TurnMode = AUTOMATIC;
		TurnCommand = msg->Value;
	} else if (msg->Direction == "Turn" && msg->MotionType == "Manual") {
		TurnMode = MANUAL;
		TurnSpeed = makeSpeed(msg->Value);

	} else if(msg->Direction == "Strafe" && msg->MotionType == "Command") {
		StrafeMode = AUTOMATIC;
		StrafeCommand = msg->Value;
	} else if (msg->Direction == "Strafe" && msg->MotionType == "Manual") {
		StrafeMode = MANUAL;
		StrafeSpeed = makeSpeed(msg->Value);

	} else if(msg->Direction == "Depth" && msg->MotionType == "Command") {
		DepthMode = AUTOMATIC;
		DepthCommand = msg->Value;
	} else if (msg->Direction == "Depth" && msg->MotionType == "Manual") {
		DepthMode = MANUAL;
		DepthSpeed = makeSpeed(msg->Value);

	} else if(msg->Direction == "Yaw" && msg->MotionType == "Command") {
		YawMode = AUTOMATIC;
		YawCommand = msg->Value;
	} else if (msg->Direction == "Yaw" && msg->MotionType == "Manual") {
		YawMode = MANUAL;
		YawSpeed = makeSpeed(msg->Value);


	} else if(msg->Direction == "Pitch" && msg->MotionType == "Command") {
		PitchMode = AUTOMATIC;
		PitchCommand = msg->Value;
	} else if (msg->Direction == "Pitch" && msg->MotionType == "Manual") {
		PitchMode = MANUAL;
		PitchSpeed = makeSpeed(msg->Value);

    /*} else if(msg->Direction == "Pivot" && msg->MotionType == "Command") {
		PitchMode = AUTOMATIC;
		PitchCommand = msg->Value;*/
	} else if (msg->Direction == "Pivot" && msg->MotionType == "Manual") {
		PivotMode = MANUAL;
		PivotSpeed = makeSpeed(msg->Value);
		
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
	motorPublisher.publish(msg);
}
/*
void updateDepth(std_msgs::Float32::ConstPtr msg) {
	DepthCommand = DepthCommand + CurrentDepth - msg->data;
	CurrentDepth = msg->data;
//	printf("DepthCommand = %f\n", DepthCommand);
}

void updateAttitude(const std_msgs::Float32MultiArray::ConstPtr msg) {
    //This function is where controllers should go.
    //It is called at 100MHz

    //YAW
    //Updates the yaw offset with the old value and the new value
	//              INPUT     + (           ERROR          )
	YawCommand = YawCommand + CurrentYaw - msg->data[2]; //Yaw is now in [2]
	TurnCommand = msg->data[2];
	CurrentYaw = msg->data[2];

    //PITCH
    //Updates the yaw offset with the old value and the new value
    //              INPUT     + (           ERROR          )
	PitchCommand = PitchCommand + CurrentPitch - msg->data[1];
	CurrentPitch = msg->data[1];
//	printf("Turn Message = %lf\n", msg->data[0]);
}


void UpdateForwardVelocity() {
    //Seems to update a velocity memory by increasing based ont the speed constant
    //and limiting using the drag constant
	ForwardVelocity += ForwardSpeed * FORWARD_SPEED_CONST;
	ForwardVelocity *= FORWARD_DRAG_CONST;
}

void UpdateStrafeVelocity() {
    //Seems to update a velocity memory by increasing based ont the speed constant
    //and limiting using the drag constant
	StrafeVelocity += StrafeSpeed * STRAF_SPEED_CONST;
	StrafeVelocity *= STRAF_DRAG_CONST;
}

*/
int sanitize(int speed) {

	if(speed >=0 && speed < 60)
		speed = 0;
	else if (speed <0 && speed > -60)
		speed = 0;
	if(speed > 255)
		return 255;
	if(speed < -255)
		return -255;
	return speed;
}


void setDepth(float input){
    std_msgs::Float32 msg;
    msg.data = input;
    depthPublisher.publish(msg);
}

void setHeading(float input){
    std_msgs::Float32 msg;
    msg.data = input;
    headingPublisher.publish(msg);
}


void CalcTurn() {
    //The input on AUTOMATIC is a heading in degrees
    //The heading needs be converted to yaw degrees.

    //Zero yaw starts when the IMU is powered, and
    //with drift, it means heading of zero may change

    //If AUTOMATIC, it sends the command to the Heading Controller
	if(TurnMode == AUTOMATIC && TurnCommand != TARGET_ZERO) {
        //Prepare and set Target_Heading and let the
		//controller take care of it
        setHeading(TurnCommand); //This sets TurnSpeed
		TurnCommand = TARGET_ZERO;
		//printf("Command = %f Speed = %i\n", TurnCommand, TurnSpeed);
	}
}

void CalcStrafe() {
    //Convert the percentage on AUTOMATIC to a speed
    if(StrafeMode == AUTOMATIC) {
        //Convert from percentage to speed
        StrafeSpeed = makeSpeed(StrafeCommand);
    }
}

void CalcDepth() {
	if(DepthMode == AUTOMATIC && DepthCommand != TARGET_ZERO) {
		//Prepare and set Target_Depth and let the
		//controller take care of it
        setDepth(DepthCommand);
		DepthCommand = TARGET_ZERO;
	}
}

void CalcPitch() {
	if(PitchMode == AUTOMATIC) {
		PitchSpeed = 0;
	}
}


//Changed the AUTOMATIC behavior to accept a percentage
//Included a bias of 75% for the left thruster to minimize drift
void ManageForwardThrusters() {
    //If AUTOMATIC, it converts from a percentage to a speed
    //otherwise it uses the value sent from Manual

	if(ForwardMode == AUTOMATIC) {

		ForwardSpeed = makeSpeed(ForwardCommand);
	}

    ForwardSpeed = sanitize(ForwardSpeed);

	if(currentFwdLeft != ForwardSpeed ||
			currentFwdRight != ForwardSpeed) {

		currentFwdLeft = LEFT_FWD_MULT*ForwardSpeed;
		currentFwdRight = ForwardSpeed;
		sendMotorMessage( RIGHT_DRIVE_BIT | LEFT_DRIVE_BIT,
				currentFwdRight, currentFwdLeft, 0, 0, 0, 0);

	}
}

void ManageTurnThrusters() {
    //This two take the commmands (AUTOMATIC mode) and convert them to speed values
	//Turning command
    CalcTurn();

	//Strafing command
	CalcStrafe();

	int RearThrust = TurnSpeed - StrafeSpeed;
	int FrontThrust = TurnSpeed + StrafeSpeed;

	RearThrust = sanitize(RearThrust);
	FrontThrust = sanitize(FrontThrust);

	if(RearThrust != currentTurnRear ||
	   FrontThrust != currentTurnFront) {

        //Include multipliers to compensate for differences in the motor
        //Strafe left: rear is faster than front
        //When rear is <0 multiply front
        //when front is <0 multiply rear
        if (RearThrust<0){ //LEFT
            currentTurnRear = RearThrust;
            currentTurnFront = FrontThrust*FRONT_TURN_MULT;
        }else if (FrontThrust<0){ //RIGHT
            currentTurnRear = RearThrust*REAR_TURN_MULT;
            currentTurnFront = FrontThrust;
	   }else{
            currentTurnRear = RearThrust;
            currentTurnFront = FrontThrust;
	   }

		sendMotorMessage(REAR_TURN_BIT | FRONT_TURN_BIT,
				0, 0, currentTurnRear, currentTurnFront, 0, 0);
//left rear <0
//right front <0
	}
}

void ManageDepthThrusters() {

	CalcDepth();
	CalcPitch(); //Should be zero until the controller is written

	int FrontThrust = DepthSpeed + PitchSpeed;
	int RearThrust = DepthSpeed - PitchSpeed;

	if(RearThrust != currentDepthRear ||
			FrontThrust != currentDepthFront) {

        //Include multipliers to compensate for differences in the motors
		currentDepthRear = RearThrust;
		currentDepthFront = FrontThrust;

		sendMotorMessage(REAR_DEPTH_BIT | FRONT_DEPTH_BIT,
				0, 0, 0, 0, RearThrust, FrontThrust);

	}
}

void ManagePivotThrusters(){
    int FrontThrust = PivotSpeed; //Choose which stays.
    int RearThrust = PivotSpeed;

    if(RearThrust != currentPivotRear ||
			FrontThrust != currentPivotFront) {

        //Include multipliers to compensate for differences in the motors
		//currentPivotRear = RearThrust;
		//currentPivotFront = FrontThrust;
		if (RearThrust<0){ //LEFT
            currentPivotRear = RearThrust;
            currentPivotFront = FrontThrust*RIGHT_PIVOT_MULT;
        }else if (FrontThrust<0){ //RIGHT
            currentPivotRear = RearThrust;
            currentPivotFront = FrontThrust*LEFT_PIVOT_MULT;
        }

		sendMotorMessage(REAR_TURN_BIT | FRONT_TURN_BIT,
				0, 0, currentPivotRear, currentPivotFront, 0, 0);

	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "HighLevelControl");
	ros::NodeHandle nh;

    if (argc>1){
        RIGHT_PIVOT_MULT = strtod(argv[1], NULL);
    }
    if (argc>2){
        LEFT_PIVOT_MULT = strtod(argv[2], NULL);
    }
    
	motorPublisher = nh.advertise<SubMotorController::MotorMessage>("Motor_Control", 100);//Should this be 10?
	depthPublisher = nh.advertise<std_msgs::Float32>("Target_Depth", 10);
	headingPublisher = nh.advertise<std_msgs::Float32>("Target_Heading", 10);

	//ros::Subscriber DepthSub = nh.subscribe("Sub_Depth", 1, updateDepth);
	//ros::Subscriber AttitudeSub = nh.subscribe("IMU_Attitude", 1, updateAttitude);
	ros::Subscriber CommandSub = nh.subscribe("High_Level_Motion", 100, commandCallback);
	ros::Subscriber PointSub = nh.subscribe("Center_On_Point", 10, CenterOnPointCallback);

	while(ros::ok()) {
		ManageForwardThrusters();
		ManageTurnThrusters();
		ManageDepthThrusters();
//		ManagePivotThrusters();

		//Could just prepare the currents on the Manage* and
        //send one motor message.

		ros::spinOnce();
		usleep(10000);
	}
}
