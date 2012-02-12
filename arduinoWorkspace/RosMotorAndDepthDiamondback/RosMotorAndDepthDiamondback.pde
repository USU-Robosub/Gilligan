/*
 */

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32.h>
#include <SoftwareSerial.h>
#include <math.h>

#define FRONT_TURN_MOTOR     0
#define BACK_TURN_MOTOR      2
#define FRONT_DEPTH_MOTOR    0
#define BACK_DEPTH_MOTOR     2
#define LEFT_DRIVE_MOTOR     0
#define RIGHT_DRIVE_MOTOR    2


#define TURN_CONTROLLER_ADDRESS   128
#define DEPTH_CONTROLLER_ADDRESS  129
#define DRIVE_CONTROLLER_ADDRESS  130

//digital pins
#define rxPin                2
#define txPin                3
#define drainPin             5
#define motorKilledPin       6

//analog pins
#define depthPin             0

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
ros::NodeHandle  nh;
std_msgs::Float32 depthMsg;
std_msgs::UInt8 motorKillMsg;
ros::Publisher chatterDepth("Sub_Depth", &depthMsg);
ros::Publisher chatterMotorKilled("Motor_State", &motorKillMsg);

int publishTimer = 100;
float depth = 0;
int motorKilled = 0;


void sendCommand(byte address, byte command, byte data)
{
  byte check = (address + command + data) & 0b01111111;
  mySerial.print(address, BYTE);
  mySerial.print(command, BYTE);
  mySerial.print(data, BYTE);
  mySerial.print(check, BYTE);
}


//void messageCb( const std_msgs::UInt8MultiArray& msg)
ROS_CALLBACK(messageCb, std_msgs::UInt8MultiArray, msg)
  byte dir = msg.data[1] >> 7;
  if (msg.data[0] & 0x80 > 0) //front turn
  {
    sendCommand(TURN_CONTROLLER_ADDRESS, FRONT_TURN_MOTOR + dir, msg.data[1] & 0x7f);
  }
  if (msg.data[0] & 0x40 > 0) //back turn
  {
    sendCommand(TURN_CONTROLLER_ADDRESS, BACK_TURN_MOTOR + dir, msg.data[1] & 0x7f); 
  }
  if (msg.data[0] & 0x20 > 0) //front depth
  {
    sendCommand(DEPTH_CONTROLLER_ADDRESS, FRONT_TURN_MOTOR + dir, msg.data[1] & 0x7f); 
  }
  if (msg.data[0] & 0x10 > 0) //back depth
  {
    sendCommand(DEPTH_CONTROLLER_ADDRESS, BACK_TURN_MOTOR + dir, msg.data[1] & 0x7f); 
  }
  if (msg.data[0] & 0x8 > 0) //left
  {
    sendCommand(DRIVE_CONTROLLER_ADDRESS, LEFT_DRIVE_MOTOR + dir, msg.data[1] & 0x7f); 
  }
  if (msg.data[0] & 0x4 > 0) //right
  {
    sendCommand(DRIVE_CONTROLLER_ADDRESS, RIGHT_DRIVE_MOTOR + dir, msg.data[1] & 0x7f); 
  }
}

//ros::Subscriber<std_msgs::UInt8MultiArray> sub("sub_motor_driver", messageCb );
ros::Subscriber sub("Motor_Driver", &msg, messageCb);
void checkMotorKilled()
{
  uint8_t val = 0;
  
  val = digitalRead(motorKilledPin);
  
  if (val != motorKilled)
  {
    //send update
    motorKillMsg.data = val;
    motorKilled = val;
    chatterMotorKilled.publish(&motorKillMsg);
  }
}


void checkDepthPsi()
{
  float t;
   
  for (int i = 0; i < 100; i++)
  {
    t += (float)analogRead(depthPin);
  }
  
  t /= 100.0;
  
  //convert to psi
  t *= 0.0049; //mV
  t *= 21.375;
  
  //round
  //t = floor(t * 10 + 0.5)/10;
  t = floor(t);
  
  if (t != depth)
  {
    //send update
    depthMsg.data = t;
    chatterDepth.publish(&depthMsg);
  }
  
  depth = t;
}

void setup()
{
  //pinMode(rxPin, INPUT);
  //pinMode(txPin, OUTPUT);
  pinMode(drainPin, OUTPUT);
  mySerial.begin(9600);
  nh.initNode();
  nh.advertise(chatterDepth);
  nh.advertise(chatterMotorKilled);
  nh.subscribe(sub);
}

void loop()
{
  checkDepthPsi();
  checkMotorKilled();
  
  nh.spinOnce();
  delay(10);
}
