/*
 */
#include <SoftwareSerial.h>
#include <math.h>

//digital pins
#define motorKilledPin        12
#define dropperLeftPin        2
#define dropperRightPin       3
#define torpedoLeftPin        9
#define torpedoRightPin       10
#define numberOfDroppers      2
#define numberOfTorpedos      2

#define PACKET_SIZE           5

//analog pins
#define depthPin              3
#define controller1TempPin    7
#define controller2TempPin    6
#define caseTempPin           2
#define computerCurrentPin    1
#define computerVoltagePin    0
#define floodPin1             4
#define floodPin2             5

#define FLOOD_DIF             100

#define SLAVE_ADDR            0x26
#define CMD_BYTE              0xA
#define TIMEOUT               1000
#define VOLTAGE_OFFSET        676

int publishTimer = 100;
float depth = 0;
int motorKilled = 0;
float controllerTemp = 0.0;
float controllerTemp2 = 0.0;
float caseTemp = 0.0;
float computerCurrent = 0.0;
float computerVoltage = 0.0;
float timer = 0.0;

byte packetBuf[PACKET_SIZE];

unsigned long floodCalib1 = 0;
unsigned long floodCalib2 = 0;


void setup()
{
  calibrateFloodSensor();
  pinMode(motorKilledPin, INPUT);
  digitalWrite(motorKilledPin, HIGH);
  pinMode(2, OUTPUT);
  pinMode(2, HIGH);
  
  timer = millis();
  Serial.begin(115200);
}

void loop()
{
  //read from serial
  if (Serial.available())
  {
    while (Serial.available() && Serial.peek() != 'S')
      Serial.read();
      
    if (Serial.available() >= PACKET_SIZE)
    {
      for (int i = 0; i < PACKET_SIZE; i++)
        packetBuf[i] = Serial.read();
      
      if (packetBuf[0] == 'S' && packetBuf[PACKET_SIZE-1] == 'E')
      {
        if (packetBuf[1] == 'T')
        {
          launchTorpedo(packetBuf[2] - '0');
        }
        else if (packetBuf[1] == 'D')
        {
          setDropperState(packetBuf[2] - '0', packetBuf[3] - '0');
        }
      }
    }
  }
  
  Serial.print("S");
  Serial.print(getTemperature(controller1TempPin));
  Serial.print(",");
  Serial.print(getTemperature(controller2TempPin));
  Serial.print(",");
  Serial.print(getTemperature(caseTemp));
  Serial.print(",");
  Serial.print(checkMotorKilled());
  Serial.print(",");
  Serial.print(checkVoltage());
  Serial.print(",");
  Serial.print(checkCurrent());
  Serial.print(",");
  Serial.print(checkDepthPsi());
  Serial.print(",");
  Serial.print(checkFlooded());
  Serial.print(",");
  Serial.print(digitalRead(dropperLeftPin));
  Serial.print(",");
  Serial.print(digitalRead(dropperRightPin));
  Serial.println("E");
  
  /*
  if (millis() - timer > 2000)
  {
    
    timer = millis();
  }
  */
  
  delay(10);
}

byte checkMotorKilled()
{
  byte val = 0;
  
  val = digitalRead(motorKilledPin);
  
  return val;
}


float checkDepthPsi()
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
  
  return t;
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

float checkVoltage()
{
  int vol = analogRead(computerVoltagePin);
  
  float tcomputerVoltage = (vol / 49.83);
  return tcomputerVoltage;
}

float checkCurrent()
{
  int cur = analogRead(computerCurrentPin);
  
  float tcomputerCurrent = (cur / 14.99);
  
  return tcomputerCurrent;
}

byte checkFlooded()
{
  byte ret = 0;
  int val = analogRead(floodPin1);
  int val2 = analogRead(floodPin2);
  
  if (val <= floodCalib1 - FLOOD_DIF || val2 <= floodCalib2 - FLOOD_DIF)
  {
    ret = 1;
  }
  
  return ret;
}

void calibrateFloodSensor()
{
  unsigned long t = 0, t2 = 0;
  
  for (int i = 0; i < 100; i++)
  {
    t += analogRead(floodPin1);
    t2 += analogRead(floodPin2);
  }
  
  t /= 100;
  t2 /= 100;
  
  floodCalib1 = t;
  floodCalib2 = t2;
}

void setDropperState(byte dropper, byte state)
{
  if (dropper < numberOfDroppers)
    digitalWrite(dropperLeftPin + dropper, state);
}

void launchTorpedo(byte torpedo)
{
  if (torpedo < numberOfTorpedos)
  {
    digitalWrite(torpedoLeftPin + torpedo, HIGH);
    delay(10);
    digitalWrite(torpedoLeftPin + torpedo, LOW);
  }
}
