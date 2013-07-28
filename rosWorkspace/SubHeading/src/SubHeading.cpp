#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <Robosub/ModuleEnableMsg.h>
#include "Robosub/HighLevelControl.h"
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>

#define OFF 0
#define ON 1

/*******************************************
Heading controller for the sub

This module will try to maintain the sub's
heading after it is done moving.

It deactivates when it detects a motor
message sent to the strafe thrusters.
As soon as the strafe thrusters are idle,
the controller will save the current heading
and will try to maintain it.

It should work with both the IMU and a
camera point, preferably the point.

Because of the drift in the IMU, using it
will require the heading point to change
every so often. The IMU drifts even in rest
so we'll have to find at which rate so the
reference can be updated.

We can use an adaptive-gain proportional controller. Update the gain K as a function of the error
because the command needs to change rapidly in order to avoid having too much momentum.

So
 e = command-y
 u(t) = k*e
 k_dot = m*e or something like that



*******************************************/

//TODO Integrate the yaw drift into the controller

//How much thurst needs to be applied per degree of rotation
//This number should yield a command [0,1] though the
//output must be biased to go from 60 to 255
float KP = .035;

float MAX = 0.25; //This might saturate at a lot less than this
bool MODE = ON;

float targetHeading = 0; //This holds what was passed (a heading relative to the current yaw)
float targetYaw = 0;
float currYaw = 0.0;
float error = 0;
bool isMoving = false;



ros::Publisher motorControl;

void mSetTurnSpeed(float speed) {
    Robosub::HighLevelControl msg;
    msg.Direction = "Turn";
    msg.MotionType = "Manual";
    msg.Value = speed;

    motorControl.publish(msg);
}

void mSetTargetHeading(float target){
    Robosub::HighLevelControl msg;

      msg.Direction = "Turn";
      msg.MotionType = "Command";
      msg.Value = target;

      motorControl.publish(msg);
}

void mEnabledCallback(const Robosub::ModuleEnableMsg::ConstPtr& msg) {
    //This could reset the heading to yaw translator by setting zero to the current yaw.
	if(msg->Module == "SubHeading")
		MODE = msg->State;
}


void mTargetHeadingCallback(const std_msgs::Float32::ConstPtr& msg) {
    //The input is an offset from the current direction
    //if (msg->data != targetHeading){
        targetYaw = fmod((currYaw + msg->data), 180);
        //printf("setting target heading to %f\n", targetYaw);
        targetHeading = msg->data;
    //}
}



void mReadForwardStatus(const Robosub::HighLevelControl::ConstPtr& msg){
	if(msg->Direction == "Forward" && msg->MotionType == "Command" ){
		printf("Got msg: ");
        if(fabs(msg->Value) > 0.05){
            isMoving = true;

		    printf("Enabling\n");
		}
        else{
            isMoving = false;
		    printf("Disabling\n");
			//send a zero
		mSetTurnSpeed(0);	
		}
    }
}

void mHeadingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if(MODE == OFF)
        return;
    currYaw = msg->data[2];
	if(isMoving){
    //This needs to compensate for "natural" drifting
	error = currYaw - targetYaw;

    if (fabs(error)<0.02){ //Could change
        error = 0;
    }

    //Proportional
    float speed = KP*error;

    if (speed > MAX)
        speed = MAX;
    else if(speed < -MAX)
        speed = -MAX;


	mSetTurnSpeed(-speed);

	//Just for keeping track, publish the negative of the error as the new target

    mSetTargetHeading(-error);


	//printf("New turn speed: %f, for curr: %f, target: %f, error %f\n", speed, currYaw, targetYaw, error);

	}
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "SubHeading");
	ros::NodeHandle nh;

	motorControl = nh.advertise<Robosub::HighLevelControl>("High_Level_Motion", 100);

	ros::Subscriber target = nh.subscribe("/Target_Heading", 10, mTargetHeadingCallback);
	ros::Subscriber imuAttitude = nh.subscribe("/IMU_Attitude", 100, mHeadingCallback);
	ros::Subscriber enabled = nh.subscribe("/Module_Control", 1, mEnabledCallback);
	ros::Subscriber movement =nh.subscribe("/High_Level_Motion", 100, mReadForwardStatus);
	ros::spin();
/*
    //Debug
    while(ros::ok()){
        ros::spinOnce();
        usleep(100000);
    }*/
}
