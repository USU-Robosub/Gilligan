/*
 */

#include <ros.h>
#include <std_msgs/Int16.h>
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
#define rxPin                3
#define txPin                2
#define drainPin             5
#define motorKilledPin       6

//analog pins
#define depthPin             0
#define controllerTempPin    1
#define caseTempPin          2

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
ros::NodeHandle  nh;
std_msgs::Float32 depthMsg;
std_msgs::UInt8 motorKillMsg;
std_msgs::Float32 caseTempMsg;
std_msgs::Float32 controllerTempMsg;

ros::Publisher chatterDepth("Sub_Depth", &depthMsg);
ros::Publisher chatterMotorKilled("Motor_State", &motorKillMsg);
ros::Publisher chatterCaseTemp("Motor_Case_Temp", &caseTempMsg);
ros::Publisher chatterControllerTemp("Motor_Controller_Temp", &controllerTempMsg);

int publishTimer = 100;
float depth = 0;
int motorKilled = 0;
float controllerTemp = 0.0;
float caseTemp = 0.0;


void sendCommand(byte address, byte command, byte data)
{
  byte check = (address + command + data) & 0b01111111;
  mySerial.print(address, BYTE);
  mySerial.print(command, BYTE);
  mySerial.print(data, BYTE);
  mySerial.print(check, BYTE);
}


void messageCb( const std_msgs::Int16& msg) {
  byte motorMask = msg.data >> 8;
  byte motorSpeed = msg.data & 0xff;
  byte dir = motorSpeed >> 7;
  
  //mySerial.print(motorMask);
  //mySerial.print(motorSpeed);
  if ((motorMask & 0x80) > 0) //front turn
  {
    sendCommand(TURN_CONTROLLER_ADDRESS, FRONT_TURN_MOTOR + dir, motorSpeed & 0x7f);
  }
  if ((motorMask & 0x40) > 0) //back turn
  {
    sendCommand(TURN_CONTROLLER_ADDRESS, BACK_TURN_MOTOR + dir, motorSpeed & 0x7f); 
  }
  if ((motorMask & 0x20) > 0) //front depth
  {
    sendCommand(DEPTH_CONTROLLER_ADDRESS, FRONT_TURN_MOTOR + dir, motorSpeed & 0x7f); 
  }
  if ((motorMask & 0x10) > 0) //back depth
  {
    sendCommand(DEPTH_CONTROLLER_ADDRESS, BACK_TURN_MOTOR + dir, motorSpeed & 0x7f); 
  }
  if ((motorMask & 0x8) > 0) //left
  {
    sendCommand(DRIVE_CONTROLLER_ADDRESS, LEFT_DRIVE_MOTOR + dir, motorSpeed & 0x7f); 
  }
  if ((motorMask & 0x4) > 0) //right
  {
    sendCommand(DRIVE_CONTROLLER_ADDRESS, RIGHT_DRIVE_MOTOR + dir, motorSpeed & 0x7f); 
  }
}

ros::Subscriber<std_msgs::Int16> sub("Motor_Data", messageCb );
//ros::Subscriber sub("Motor_Driver", &msg, messageCb);
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

void checkCaseTemperature()
{
  float temp = getTemperature(caseTempPin);
  
  if (temp != caseTemp)
  {
    caseTempMsg.data = temp;
    chatterCaseTemp.publish(&caseTempMsg);
    caseTemp = temp;
  }
}

void checkControllerTemperature()
{
  float temp = getTemperature(controllerTempPin);
  
  if (temp != controllerTemp)
  {
    controllerTempMsg.data = temp;
    controllerTemp = temp;
    chatterControllerTemp.publish(&controllerTempMsg);
  }
}

float getTemperature(int pin)
{
  float temp = 0.0;
  
  for (int i = 0; i < 100; i++)
  {
    temp += analogRead(caseTempPin);
  }
  temp /= 100;
  
  temp *= 4.9; //convert to mV
  temp /= 10; //convert to C
  
  return temp;
}

void setup()
{
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(drainPin, OUTPUT);
  mySerial.begin(9600);
  nh.initNode();
  nh.advertise(chatterDepth);
  nh.advertise(chatterMotorKilled);
  nh.advertise(chatterCaseTemp);
  nh.advertise(chatterControllerTemp);
  nh.subscribe(sub);
}

void loop()
{
  checkDepthPsi();
  checkMotorKilled();
  checkCaseTemperature();
  checkControllerTemperature();
  
  nh.spinOnce();
  delay(10);
}
