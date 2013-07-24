/**************************
* Water Sensor Controller *
* ************************/
<<<<<<< HEAD
//#include <Serial.h>
=======
>>>>>>> f134dfc40b15a7927fe1316235e8ba839c477293

#define BAUD 115200
#define MIN_PACKET_LENGTH 2

int gAnalogDevices = 8;
int gDelay = 2000;

void setup()
{
  
  Serial.begin(BAUD);
}

void loop()
{
  //build value to output
  uint8_t bytesPerReading = sizeof(uint16_t);
  uint8_t packetSize = gAnalogDevices * bytesPerReading;
  uint8_t value[packetSize];
  for (int i = 0; i < gAnalogDevices; i++)
  {
    uint16_t tmp = analogRead(i);
    int pos = i * sizeof(uint16_t);
    
    value[pos] = (uint8_t)(tmp >> 8);
    value[pos+1] = (uint8_t)(tmp & 0x00ff);
  }
  
  Serial.print('S');
  Serial.write(packetSize);
  Serial.write(bytesPerReading);
  for (uint8_t i = 0; i < packetSize; i++)
  {
    Serial.write(value[i]);
  }
  
  //TODO add ability to command this device via serial
  
  delay(gDelay);
  
}
