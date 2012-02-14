#include <ADXL345.h>
#include <HMC58X3.h>
#include <ITG3200.h>

//#define DEBUG
#include "DebugUtils.h"

#include "FreeIMU.h"
#include "CommunicationUtils.h"
#include <Wire.h>

int raw_values[9];
//char str[512];
float ypr[3]; // yaw pitch roll
float val[9];

// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();

void setup() { 
  Serial.begin(115200);
  Wire.begin();
  
  delay(5);
  my3IMU.init(); // the parameter enable or disable fast mode
  delay(5);
}

void loop()
{ 
  ypr[0] = 0.0;
  ypr[1] = 0.0;
  ypr[2] = 0.0;
  
  for (int i = 0; i < 20; i++)
  {
    float yprTmp[3];
    my3IMU.getYawPitchRoll(yprTmp);
    ypr[0] += yprTmp[0];
    ypr[1] += yprTmp[1];
    ypr[2] += yprTmp[2];
  }
  ypr[0] /= 20.0;
  ypr[1] /= 20.0;
  ypr[2] /= 20.0;
  
  //Serial.print("Yaw: ");
  Serial.print(ypr[0]);
  Serial.print(" ");
  Serial.print(ypr[1]);
  Serial.print(" ");
  Serial.println(ypr[2]);
  
  delay(100);
}



