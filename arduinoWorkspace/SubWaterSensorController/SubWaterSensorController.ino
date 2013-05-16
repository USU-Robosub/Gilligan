/**************************
* Water Sensor Controller *
* ************************/
#include <Serial.h>
#define S1 6
#define S2 7
#define BAUD 115200
#define DELAY 200

void setup(){
  Serial.begin(BAUD);
}

void loop(){
  int s1 = analogRead(S1)>0.2?1:0;
  int s2 = analogRead(S2)>0.2?1:0;
  
  Serial.print("S");
  Serial.print(s1);
  Serial.print(s2);
  Serial.println("E");
  
  delay(DELAY);
  
}
