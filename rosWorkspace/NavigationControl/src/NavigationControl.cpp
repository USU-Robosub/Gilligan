#include "ros/ros.h"
#include "NavigationControl.hpp"
#include "Robosub/HighLevelControl.h"


NavigationControl::NavigationControl()
 : m_nodeHandle(),
   m_highLevelMotorPublisher()
{
  //bool OMODE = OFF;
  POINTMODE = OFF;
  LINEMODE = OFF;
  //bool Centered = false;
  Begun = false;
  OnLine = false;
  Rotating = false;

  start_x = 0;
  start_y = 0;
  start_rot = 0;

  m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motion", 10);
  ros::Subscriber targetPoint = m_nodeHandle.subscribe("Center_On_Point", 1, &NavigationControl::PointCallback, this);
  ros::Subscriber targetLine = m_nodeHandle.subscribe("Center_On_Line", 1, &NavigationControl::LineCallback, this);
  ros::Subscriber enabled = m_nodeHandle.subscribe("Module_Enable", 1, &NavigationControl::EnabledCallback, this);
}

NavigationControl::~NavigationControl()
{

}

/**
* @brief Translates a percentage to a thrust unit
*
* @param percent Percent of the distance
*/
float NavigationControl::makeVoltage(float percent)
{
    //The smallest output is 60
    //The largets output is 255
    
    //map from 100 to 255 and from 60 to 5 percent
      
    return (percent - 5) * (255-60) / (100-5) + 60;
    //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    
}
// Create a thrust for Diving
// and send it to the motor
void NavigationControl::setDive(float val)
{   
    
    float thrust = makeVoltage(val);
    publishMotor("Depth", "Manual",thrust);
    
}

void NavigationControl::setStrafe(float val)
{
    float thrust = makeVoltage(val);
    publishMotor("Straf", "Manual",thrust);
}

void NavigationControl::setDrive(float val)
{
    float thrust = makeVoltage(val);
    publishMotor("Forward", "Manual",thrust);

}

void NavigationControl::setTurn(float val)
{
    float thrust = makeVoltage(val);
    publishMotor("Turn", "Manual",thrust);
}

void NavigationControl::run()
{
    //if want to do something between spins
    //change to a while loop with ros::spinOnce() and sleep    
    ros::spin(); 
}

void NavigationControl::EnabledCallback(const Robosub::ModuleEnableMsg& msg) {
	if(msg.Module == "NavigationControl")
	{
		//POINTMODE = msg.State;

	}
}


void NavigationControl::PointCallback(const Robosub::Point& msg)
{
		// The speed is created based on the difference between
        // the target point and the center

    // If the it is the beginning of the process
    // save the target as the objective

    // Target points
    int target_x = msg.x;
    int target_y = msg.y;

    if (!target_x && !target_y ) {
//        Centered = true;
        Begun = false;
        start_x = 0;
        start_y = 0;

        setStrafe(0);
        setDive(0);

        return;
    }


    if (!Begun){ // First assignment: create starting points
        start_x = msg.x;
        start_y = msg.y;
        Begun = true;
//        Centered = false;
    }


    // Calculate the directions what direction do we have to move?

    int dir_x = target_x>0 ? 1 : -1; //Left -1, Right +1
    int dir_y = target_y>0 ? -1 : 1; //Up -1, Down +1

    // Calculate the percentage (OR SIMPLY USE ERROR??)

    float per_x = (target_x/240); //start_x);
    float per_y = (target_y/320); //start_y);

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
        if(dir_y>0){ // && FORWARDCAMERA
            range = 30; //So we don't go up too fast
            thrust_y = (range*per_y+minT)*dir_y;
		}
    }

    //Send the calculated speed to the motor
    setStrafe(thrust_x);
    setDive(thrust_y);

}

bool NavigationControl::moveToLine(int x, int y){
    //Returns 1 if on top of the line, 0 otherwise

    int target_x = x;
    int target_y = y;

    if (!target_x && !target_y ) {
//        Centered = true;
        Begun = false;

        start_x = 0;
        start_y = 0;

        setStrafe(0);
        setDrive(0);

        return 1; //Centered
    }


    if (!Begun){ // First assignment: create starting points
        start_x = x;
        start_y = y;
        Begun = true;
//        Centered = false;
    }


    // Calculate the directions what direction do we have to move?

    int dir_x = target_x>0 ? 1 : -1; //Left -1, Right +1
    int dir_y = target_y>0 ? 1 : -1; //Reverse -1, Forward +1

    // Calculate the percentage (OR SIMPLY USE ERROR??)

    float per_x = (target_x/240);//start_x);
    float per_y = (target_y/320);//start_y);

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
        if(dir_y>0){ //
            range = 30; //So we don't go up too fast
            thrust_y = (range*per_y+minT)*dir_y;
		}
    }

    if (!thrust_x && !thrust_y)
        return 1; //No speed means we're <1% away


    //Send the calculated speed to the motor
    setStrafe(thrust_x);
    setDrive(thrust_y);

    return 0;

}

float NavigationControl::sanitize(float rot){
    //converts the given rotation to a number between 0-180
    if (rot>=180)
	return rot-360;
    return rot;
}

void NavigationControl::LineCallback(const Robosub::Line msg) {
	if(LINEMODE == OFF)
		return;

    OnLine = moveToLine(msg.x, msg.y);
    //Once it's on top of the line, rotate
    if (!OnLine){
        return; //keep rotating
    }

    int target_rot = sanitize(msg.rotation);
    if(!target_rot){
	 start_rot = 0;
         Rotating = 0;
	 setTurn(0);
	 return; //Stoppped by Higher Level
    }
    if(!Rotating){
        start_rot = target_rot;
    }

    float per_rot = (target_rot/180);//start_rot);


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

/**
 * @brief plubish to the high level motor controller topic
 *
 * @param direction The direction: Forward, Straf, Turn
 * @param motion The motion type: Manual, Offset
 * @param value The value
 */
void NavigationControl::publishMotor(std::string direction, std::string motion, float value)
{
  Robosub::HighLevelControl msg;
  msg.Direction = direction;
  msg.MotionType = motion;
  msg.Value = value;

  m_highLevelMotorPublisher.publish(msg);
}
