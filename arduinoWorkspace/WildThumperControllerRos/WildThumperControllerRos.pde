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

ros::NodeHandle  nh;

void setMotors(int val)
{
  byte motorSpeed = val & 0xff;
  
  if(val & 0x200) 
  {
    if(val & 0x100)
    {
      analogWrite(LmotorA, 0);
      analogWrite(LmotorB, motorSpeed);
    }
    else
    {
      analogWrite(LmotorA, motorSpeed);
      analogWrite(LmotorB, 0);
    }
  }
  if(val & 0x400) 
  {
    if(val & 0x100)
    {
      analogWrite(RmotorA, 0);
      analogWrite(RmotorB, motorSpeed);
    }
    else
    {
      analogWrite(RmotorA, motorSpeed);
      analogWrite(RmotorB, 0);
    }
  }
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
  digitalWrite(RmotorB, 0);
  delay(1000);
  
  analogWrite(LmotorA, 0);
  analogWrite(LmotorB, 0);
  analogWrite(RmotorA, 0);
  digitalWrite(RmotorB, 0);
  
  nh.initNode();
  nh.subscribe(sub);
}


void loop()
{
  nh.spinOnce();
  delay(10);
}
