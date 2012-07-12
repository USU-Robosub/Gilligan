#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "subSim/motor.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/Point.h"
#include "Robosub/Line.h"

#define OFF 0
#define ON 1

const float MAX_THRESHOLD = 0.05; //5%
const float MIN_THRESHOLD = 0.01; //1%
//bool OMODE = OFF;
bool POINTMODE = OFF;
bool LINEMODE = OFF;
//bool Centered = false;
bool Begun = false;
bool OnLine = false;
bool Rotating = false;

float start_x = 0;
float start_y = 0;
float start_rot = 0;


void mEnabledCallback(const Robosub::ModuleEnableMsg::ConstPtr& msg) {
	if(msg->Module == "Center_on_Point"){
		POINTMODE = msg->State;
	} else if (msg->Module == "Center_on_Line"){
	    LINEMODE = msg->State;
	}
}


void mPointCallback(const Robosub::Point::ConstPtr& msg) {
	if(POINTMODE == OFF)
		return;
		// The speed is created based on the difference between
        // the target point and the center

    // If the it is the beginning of the process
    // save the target as the objective

    // Target points
    int target_x = msg->x;
    int target_y = msg->y;

    if (!target_x && !target_y ) {
//        Centered = true;
        Begun = false;
        start_x = 0;
        start_y = 0;

        setStraf(0);
        setDive(0);

        return;
    }


    if (!Begun){ // First assignment: create starting points
        start_x = msg->x;
        start_y = msg->y;
        Begun = true;
//        Centered = false;
    }


    // Calculate the directions what direction do we have to move?

    int dir_x = target_x>0 ? 1 : -1; //Left -1, Right +1
    int dir_y = target_y>0 ? -1 : 1; //Up -1, Down +1

    // Calculate the percentage (OR SIMPLY USE ERROR??)

    float per_x = (target_x/start_x);
    float per_y = (target_y/start_y);

    //  Control Process (calculate the right thrust)
    //  Use less than 100% based on how big the number is.

    //  Convert the distance percentage to thrust percentage

    // These are set assuming there is no bias on the voltage
    // sent to the thrusters
    int minT = 50; // to avoid sending values under the minimum.
    //maxT = 90; //To avoid moving too fast and sucking too much power
    int range = 40;


    int thrust_x = 0;
    int thrust_y = 0;

    if((per_x) <MAX_THRESHOLD){
        if((per_x)>MIN_THRESHOLD){ //Send a negative thrust to decrease the speed
            thrust_x = -60*dir_x;
        } else {
            thrust_x = 0;
        }
    } else {
        thrust_x = (range*per_x+minT)*dir_x;
    }

    if((per_y) <MAX_THRESHOLD){
        if((per_y)>MIN_THRESHOLD){ //Send a negative thrust to decrease the speed
            thrust_y = -60*dir_y;
        } else {
            thrust_y = 0; // We zeroed out in this direction
        }
    } else {
        if(dir_y>0) // && FORWARDCAMERA
            range = 30; //So we don't go up too fast
            thrust_y = (range*per_y+minT)*dir_y;
    }

    //Send the calculated speed to the motor
    setStraf(thrust_x);
    setDive(thrust_y);

}

bool moveToLine(int x, int y){
    //Returns 1 if on top of the line, 0 otherwise

    int target_x = x;
    int target_y = y;

    if (!target_x && !target_y ) {
//        Centered = true;
        Begun = false;

        start_x = 0;
        start_y = 0;

        setStraf(0);
        setDrive(0);

        return 1; //Centered
    }


    if (!Begun){ // First assignment: create starting points
        start_x = msg->x;
        start_y = msg->y;
        Begun = true;
//        Centered = false;
    }


    // Calculate the directions what direction do we have to move?

    int dir_x = target_x>0 ? 1 : -1; //Left -1, Right +1
    int dir_y = target_y>0 ? 1 : -1; //Reverse -1, Forward +1

    // Calculate the percentage (OR SIMPLY USE ERROR??)

    float per_x = (target_x/start_x);
    float per_y = (target_y/start_y);

    //  Control Process (calculate the right thrust)
    //  Use less than 100% based on how big the number is.

    //  Convert the distance percentage to thrust percentage

    // These are set assuming there is no bias on the voltage
    // sent to the thrusters
    int minT = 50; // to avoid sending values under the minimum.
    //maxT = 90; //To avoid moving too fast and sucking too much power
    int range = 40;


    int thrust_x = 0;
    int thrust_y = 0;

    if((per_x) <MAX_THRESHOLD ){
        if((per_x)>MIN_THRESHOLD){ //Send a negative thrust to decrease the speed
            thrust_x = -60*dir_x;
        } else {
            thrust_x = 0; // this line may be unnecessary
        }
    } else {
        thrust_x = (range*per_x+minT)*dir_x;
    }

    if((per_y) <MAX_THRESHOLD ){
        if((per_y)>MIN_THRESHOLD){ //Send a negative thrust to decrease the speed
            thrust_y = -60*dir_y;
        } else {
            thrust_y = 0; // We zeroed out in this direction
        }
    } else {
        if(dir_y>0) // && FORWARDCAMERA
            range = 30; //So we don't go up too fast
            thrust_y = (range*per_y+minT)*dir_y;
    }

    if (!thrust_x && !thrust_y)
        return 1; //No speed means we're <1% away


    //Send the calculated speed to the motor
    setStraf(thrust_x);
    setDrive(thrust_y);

    return 0;

}

float sanitize(float rot){
    //converts the given rotation to a number between 0-180
    if (rot>=180)
	return rot-360;
    return rot;
}

void mLineCallback(const Robosub::Line::ConstPtr& msg) {
	if(LINEMODE == OFF)
		return;

    OnLine = moveToLine(msg->x, msg->y);
    //Once it's on top of the line, rotate
    if (!OnLine){
        return; //keep rotating
    }

    int target_rot = sanitize(msg->rotation);
    if(!target_rot){
	 start_rot = 0;
         Rotating = 0;
	 setTurn(0);
	 return; //Stoppped by Higher Level
    }
    if(!Rotating){
        start_rot = target_rot;
    }

    float per_rot = (target_rot/start_rot);


    //This needs to be implemented better (any ideas?)
    //int direction = 1; //always rotate right?
    int direction = target_rot>0 ? 1 : -1; //-1 Rotate left


    int rate = 0;
    int range=40;
    int minR = 50;

    if(abs(per_rot)<MAX_THRESHOLD ){
        if(abs(per_rot)>.01)
            rate = -60*direction;
        else
            rate = 0;
    } else {
        rate = (range*per_rot+minR)*direction;
    }

    setTurn(rate);

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "NavCenterOnPoint");
	ros::NodeHandle nh;
	ros::Subscriber targetPoint = nh.subscribe("/Center_on_Point", 1, mPointCallback);
	ros::Subscriber targetLine = nh.subscribe("/Center_on_Line", 1, mLineCallback);
	ros::Subscriber enabled = nh.subscribe("/Module_Control", 1, mEnabledCallback);
	ros::spin();
}

