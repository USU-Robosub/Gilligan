/*
 */
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <SoftwareSerial.h>
#include <math.h>

//digital pins
#define motorKilledPin        12

//analog pins
#define depthPin              3
#define controller1TempPin    7
#define controller2TempPin    6
#define caseTempPin           2
#define computerCurrentPin    1
#define computerVoltagePin    0

#define SLAVE_ADDR            0x26
#define CMD_BYTE              0xA
#define TIMEOUT               1000
#define VOLTAGE_OFFSET        676

ros::NodeHandle  nh;
std_msgs::Float32 depthMsg;
std_msgs::UInt8 motorKillMsg;
std_msgs::Float32MultiArray controllerTempMsg;
std_msgs::Float32MultiArray computerCurVolt;

ros::Publisher chatterDepth("Pressure_Data", &depthMsg);
ros::Publisher chatterMotorKilled("Motor_State", &motorKillMsg);
ros::Publisher chatterControllerTemp("Controller_Box_Temp", &controllerTempMsg);
ros::Publisher chatterComputer("Computer_Cur_Volt", &computerCurVolt);

int publishTimer = 100;
float depth = 0;
int motorKilled = 0;
float controllerTemp = 0.0;
float controllerTemp2 = 0.0;
float caseTemp = 0.0;
float computerCurrent = 0.0;
float computerVoltage = 0.0;
float timer = 0.0;

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
  /*
  t *= 0.0049; //mV
  t *= 21.375;
  
  //round
  //t = floor(t * 10 + 0.5)/10;
  t = floor(t);
  */
    //send update
    depthMsg.data = t;
    chatterDepth.publish(&depthMsg);
  
  depth = t;
}

void checkControllerTemperature()
{
  controllerTemp = getTemperature(controller1TempPin);
  controllerTemp2 = getTemperature(controller2TempPin);
  caseTemp = getTemperature(caseTempPin);
  
  controllerTempMsg.data[0] = controllerTemp;
  controllerTempMsg.data[1] = controllerTemp2;
  controllerTempMsg.data[2] = caseTemp;
  chatterControllerTemp.publish(&controllerTempMsg);
}

float getTemperature(int pin)
{
  float temp = 0.0;
  
  for (int i = 0; i < 100; i++)
  {
    temp += analogRead(pin);
  }
  temp /= 100;
  
  temp *= 4.9; //convert to mV
  temp /= 10; //convert to C
  
  return temp;
}

void checkCurrentVoltage()
{
  int cur = analogRead(computerCurrentPin);
  int vol = analogRead(computerVoltagePin);
  
  computerCurrent = (cur / 14.99);
  computerVoltage = (vol / 49.83);
  
  computerCurVolt.data[0] = computerCurrent;
  computerCurVolt.data[1] = computerVoltage;
  chatterComputer.publish(&computerCurVolt);
}

void setup()
{
  controllerTempMsg.data_length = 3;
  controllerTempMsg.data = (float*)malloc(12);
  computerCurVolt.data_length = 2;
  computerCurVolt.data = (float*)malloc(8);
  
  Wire.begin();
  pinMode(motorKilledPin, INPUT);
  pinMode(2, OUTPUT);
  pinMode(2, HIGH);
  
  nh.initNode();
  nh.advertise(chatterDepth);
  nh.advertise(chatterMotorKilled);
  nh.advertise(chatterControllerTemp);
  nh.advertise(chatterComputer);
  timer = millis();
}

void loop()
{
  checkDepthPsi();
  checkMotorKilled();
  
  if (millis() - timer > 1000)
  {
    checkCurrentVoltage();
    checkControllerTemperature();
    timer = millis();
  }
  
  nh.spinOnce();
  delay(10);
}
