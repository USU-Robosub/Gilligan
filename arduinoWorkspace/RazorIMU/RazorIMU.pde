#include <ADXL345.h>
#include <HMC58X3.h>
#include <ITG3200.h>

//#define DEBUG
#include "DebugUtils.h"

#include "FreeIMU.h"
#include "CommunicationUtils.h"
#include <Wire.h>

#define AVERAGE  1

//int raw_values[9];
//char str[512];
float ypr[3]; // yaw pitch roll
//float val[9];
float myVals[9];

// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();

void setup() { 
  Serial.begin(115200);
  Wire.begin();
  
  delay(5);
  my3IMU.init(); // the parameter enable or disable fast mode
  delay(5);
  ypr[0] = 0.0;
  ypr[1] = 0.0;
  ypr[2] = 0.0;
  for(int i = 0; i < 9; i++)
    myVals[i] = 0.0;
  
  getVals();
}

void loop()
{
  for (int i = 0; i < AVERAGE; i++)
  {
    getVals();
  }
  
  ypr[0] /= (AVERAGE + 1);
  ypr[1] /= (AVERAGE + 1);
  ypr[2] /= (AVERAGE + 1);
  
  for (int i = 0; i < 9; i++)
      myVals[i] /= (AVERAGE + 1);
  
  //Serial.print("Yaw: ");
  Serial.print("S");
  Serial.print(ypr[0]);
  Serial.print(",");
  Serial.print(ypr[2]);
  Serial.print(",");
  Serial.print(ypr[1]);
  for (int i = 0; i < 9; i++)
  {
    Serial.print(",");
    Serial.print(myVals[i]);
  }
  Serial.println("E");
}


void getVals()
{
  float yprTmp[3];
  float accel[9];
  my3IMU.getYawPitchRoll(yprTmp, accel);
  ypr[0] += yprTmp[0];
  ypr[1] += yprTmp[2];
  ypr[2] += yprTmp[1];
    
  for (int i = 0; i < 9; i++)
    myVals[i] += accel[i];
}
