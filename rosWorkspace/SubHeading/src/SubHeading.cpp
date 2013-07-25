#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <Robosub/ModuleEnableMsg.h>
#include "Robosub/HighLevelControl.h"
#include <sys/time.h>
#include <stdlib.h>

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
float KP = .1;

float MAX = 0.4; //This might saturate at a lot less than this
bool MODE = ON;

float targetYaw = 0;
float currYaw = 0.0;
float error = 0;



ros::Publisher motorControl;

void mTargetHeadingCallback(const std_msgs::Float32::ConstPtr& msg) {
    //The input is an offset from the current direction

	targetYaw = currYaw + msg->data;
	printf("setting target heading to %f\n", targetYaw);
}

void setTurnSpeed(float speed) {
  Robosub::HighLevelControl msg;
  msg.Direction = "Turn";
  msg.MotionType = "Command";
  msg.Value = speed;

  motorControl.publish(msg);
}

void mEnabledCallback(const Robosub::ModuleEnableMsg::ConstPtr& msg) {
    //This could reset the heading to yaw translator by setting zero to the current yaw.
	if(msg->Module == "SubHeading")
		MODE = msg->State;
}

int makeSpeed(float percent)
{
    //The smallest output is 60
    //The largets output is 255

    //map from 100 to 255 and from 60 to 1 percent
    if (fabs(percent)<0.01)
        return 0;
    return (percent - .01) * (255-60) / (1-.05) + 60;
    //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


}

void mHeadingCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if(MODE == OFF)
        return;
    currYaw = msg->data[2];
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


	setTurnSpeed(makeSpeed(speed));
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "SubHeading");
	ros::NodeHandle nh;

	motorControl = nh.advertise<Robosub::HighLevelControl>("High_Level_Motion", 10);

	ros::Subscriber targetDepth = nh.subscribe("/Target_Heading", 10, mTargetHeadingCallback);
	ros::Subscriber imuAttitude = nh.subscribe("/IMU_Attitude", 100, mHeadingCallback);
	ros::Subscriber enabled = nh.subscribe("/Module_Control", 1, mEnabledCallback);
	ros::spin();
}
