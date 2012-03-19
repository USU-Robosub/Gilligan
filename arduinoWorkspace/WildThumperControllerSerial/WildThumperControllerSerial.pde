#define LmotorA             3  // Left  motor H bridge, input A
#define LmotorB            11  // Left  motor H bridge, input B
#define RmotorA             5  // Right motor H bridge, input A
#define RmotorB             6  // Right motor H bridge, input B

#define Battery             0  // Analog input 00
#define RmotorC             6  // Analog input 06
#define LmotorC             7  // Analog input 07
#define Charger            13  // Low=ON High=OFF
#define MAX_MOTOR_CURRENT  550
#define FUSE_BLOW          700
#define HEART_BEAT_MS      1000


#define PACKET_SIZE         7
#define DATA_SIZE           4
#define START_SYNC_BYTE     0
#define CMD_BYTE            1
#define DATA_BYTE           2
#define END_SYNC_BYTE       6
#define START_SYNC_VAL      'S'
#define END_SYNC_VAL        'E'

#define DRIVE_CMD           'D'
#define DRIVE_RESP          'd'
#define VOLTAGE_CMD         'V'
#define VOLTAGE_RESP        'v'
#define CURRENT_CMD         'C'
#define CURRENT_RESP        'c'

byte buf[PACKET_SIZE];
float voltage = 0.0;
float rMotorCurrent = 0.0;
float lMotorCurrent = 0.0;
unsigned long startTime;

void setup()
{
  pinMode (Charger,OUTPUT);                                   // change Charger pin to output
  digitalWrite (Charger,1);                                   // disable current regulator to charge battery

  Serial.begin(115200);
  startTime = millis();
}

void loop()
{
  if (Serial.available() >= PACKET_SIZE)
  {
    for (int i = 0; i < PACKET_SIZE; i++)
    {
      buf[i] = Serial.read();
    }
    
    if (buf[START_SYNC_BYTE] == START_SYNC_VAL && buf[END_SYNC_BYTE] == END_SYNC_VAL)
    {
      //good packet
      if (buf[CMD_BYTE] == DRIVE_CMD)
      {
        setMotors(&buf[DATA_BYTE]);
        startTime = millis();
        sendMsg(DRIVE_RESP, &buf[DATA_BYTE], DATA_SIZE);
      }
      else if (buf[CMD_BYTE] == VOLTAGE_CMD)
      {
        sendMsg(CURRENT_RESP, voltage);
      }
      else if (buf[CMD_BYTE] == CURRENT_CMD)
      {
        if (buf[DATA_BYTE] == 'L')
        {
          sendMsg(CURRENT_RESP, lMotorCurrent);
        }
        else if (buf[DATA_BYTE] == 'R')
        {
          sendMsg(CURRENT_RESP, rMotorCurrent);
        }
      }
    }
  }
  
  if (millis() - startTime > HEART_BEAT_MS) //millis will roll over every 49.71 days, thus the motors will shutoff unexpectedly
  {
    analogWrite(LmotorA, 0);
    analogWrite(LmotorB, 0);
    analogWrite(RmotorA, 0);
    analogWrite(RmotorB, 0);
    startTime = millis();
  }
}

void sendMsg(byte cmd, byte* val, int size)
{
  byte pkt[PACKET_SIZE];
  
  pkt[START_SYNC_BYTE] = START_SYNC_VAL;
  pkt[END_SYNC_BYTE] = END_SYNC_VAL;
  pkt[CMD_BYTE] = cmd;
  for (int i = 0; i < size; i++)
  {
    pkt[DATA_BYTE+i] = val[i];
  }
  
  Serial.write(pkt, PACKET_SIZE);
}

void sendMsg(byte cmd, float val)
{
  byte pkt[PACKET_SIZE];
  
  pkt[START_SYNC_BYTE] = START_SYNC_VAL;
  pkt[END_SYNC_BYTE] = END_SYNC_VAL;
  pkt[CMD_BYTE] = cmd;
  (*(float*)&pkt[DATA_BYTE]) = val;
  
  Serial.write(pkt, PACKET_SIZE);
}

void setMotors(byte* pkt)
{
  analogWrite(LmotorA, pkt[0]);
  analogWrite(LmotorB, pkt[1]);
  analogWrite(RmotorA, pkt[2]);
  analogWrite(RmotorB, pkt[3]);
}

void readVoltage()
{
  int val = analogRead(Battery);
  voltage = val * 4.883; //convert to mV
}

void readCurrents()
{
  int lVal = analogRead(LmotorC);
  int rVal = analogRead(RmotorC);
  
  lMotorCurrent = lVal * 4.883; //convert to mV
  rMotorCurrent = rVal * 4.883;
  
  //TODO check motor over volatage
}
