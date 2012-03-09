#include <ros.h>
#include <std_msgs/Int16.h>

#define LmotorA             3  // Left  motor H bridge, input A
#define LmotorB            11  // Left  motor H bridge, input B
#define RmotorA             5  // Right motor H bridge, input A
#define RmotorB             6  // Right motor H bridge, input B

#define Battery             0  // Analog input 00
#define RmotorC             6  // Analog input 06
#define LmotorC             7  // Analog input 07
#define Charger            13  // Low=ON High=OFF

#define HEARTBEAT          10000

ros::NodeHandle  nh;
long int counter = 0;
byte lMotorA = 0;
byte lMotorB = 0;
byte rMotorA = 0;
byte rMotorB = 0;

void executeMotors()
{
  analogWrite(lMotorA, lMotorA);
  analogWrite(LmotorB, lMotorB);
  analogWrite(RmotorA, rMotorA);
  analogWrite(RmotorB, rMotorB);
}

void setMotors(int val)
{
  byte motorSpeed = val & 0xff;
  
  if(val & 0x200) 
  {
    if(val & 0x100)
    {
      lMotorA = 0;
      lMotorB = motorSpeed;
    }
    else
    {
      lMotorA = motorSpeed;
      lMotorB = 0;
    }
  }
  if(val & 0x400) 
  {
    if(val & 0x100)
    {
      rMotorA = 0;
      rMotorB = motorSpeed;
    }
    else
    {
      rMotorA = motorSpeed;
      rMotorB = 0;
    }
  }
  
  counter = 0;
  executeMotors();
}

void messageCb( const std_msgs::Int16& msg)
{
  setMotors(msg.data);
}

ros::Subscriber<std_msgs::Int16> sub("Motor_Channel", messageCb );

void setup()
{
  pinMode(Charger, OUTPUT);
  digitalWrite(Charger, HIGH);
  
  analogWrite(LmotorA, 255);
  analogWrite(LmotorB, 0);
  analogWrite(RmotorA, 255);
  analogWrite(RmotorB, 0);
  
  delay(1000);
  
  analogWrite(LmotorA, 0);
  analogWrite(LmotorB, 0);
  analogWrite(RmotorA, 0);
  analogWrite(RmotorB, 0);
  
  nh.initNode();
  nh.subscribe(sub);
}


void loop()
{
  nh.spinOnce();
  counter++;
  
  if (counter > HEARTBEAT)
  {
    lMotorA = 0;
    lMotorB = 0;
    rMotorA = 0;
    rMotorB = 0;
    
    executeMotors;
  }
  
  delay(10);
}
