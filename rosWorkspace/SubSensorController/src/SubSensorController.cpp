#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <cstring>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "timer.h"
#include <pthread.h>

#include <USBLibrary/SerialInterface.hpp>

#define STATE_WAITING_ON_FD 0
#define STATE_WORKING       1
#define VERBOSE 0



//int setupTTY(int fd);
//std::string getTTYLine(int fd);
bool timeLeft(struct timeval* start, struct timeval* timeout);
bool goodLine(std::string val, int varCount);
void torpedoCallback(const std_msgs::UInt8::ConstPtr& msg);
void dropperCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
bool checkLock(bool loop);
bool checkLock();
bool getLock();
bool releaseLock();

int controllerState = STATE_WAITING_ON_FD;
char* name;
int fd;
pthread_mutex_t myMutex;

void error(char * msg)
{
    perror(msg);
    exit(-1);
}

int main(int argc, char **argv)
{
  std::string file = "/dev/controller_sensor";
  UInt32 baudrate = 115200;
  SerialInterface usb("/dev/controller_sensor", baudrate);


  if (argc > 1)
  {
    file = argv[1];
    printf("%s info: Opening %s\n", argv[0], file.c_str());
  }

  if (pthread_mutex_init(&myMutex, NULL) != 0)
  {
    printf("Failed to get mutex: %s\n", strerror(errno));
    exit(-1);
  }

  name = argv[0];

  float temp[3] = {0.0};
  float pressure = 0.0;
  int motorKilled = 0, waterDetected, dropperLeft= 0, dropperRight= 0;
  float curVolt[2] = {0.0};
  int numVariables = 10;

  float ttemp[3] = {0.0};
  float tpressure = 0.0;
  int tmotorKilled = 0, twaterDetected= 0, tdropperLeft= 0, tdropperRight= 0;
  float tcurVolt[2] = {0.0};

  timer tempTimer, pressureTimer, motorTimer, currentTimer, waterTimer, dropperTimer;

  tempTimer.start(1, 0);
  pressureTimer.start(1, 0);
  motorTimer.start(1, 0);
  currentTimer.start(1, 0);
  waterTimer.start(1, 0);
  dropperTimer.start(1, 0);

  ros::init(argc, argv, "SubSensorController");
  ros::NodeHandle nh;

  ros::Publisher temperatuerPub = nh.advertise<std_msgs::Float32MultiArray>("Controller_Box_Temp", 1000);
  ros::Publisher pressurePub = nh.advertise<std_msgs::Float32>("Pressure_Data", 1000);
  ros::Publisher MotorPub = nh.advertise<std_msgs::UInt8>("Motor_State", 1000);
  ros::Publisher computerPub = nh.advertise<std_msgs::Float32MultiArray>("Computer_Cur_Volt", 1000);
  ros::Publisher waterPub = nh.advertise<std_msgs::UInt8>("Water_Detected", 1000);
  ros::Publisher dropperPub = nh.advertise<std_msgs::UInt8MultiArray>("Dropper_States", 1000);

  ros::Subscriber torpedoSub = nh.subscribe("Torpedo_Launch", 1000, torpedoCallback);
  ros::Subscriber dropperSub = nh.subscribe("Dropper_Activate", 1000, dropperCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    if (controllerState == STATE_WAITING_ON_FD)
    {
      if (checkLock() && getLock())
      {
        controllerState = STATE_WORKING;
        sleep(5);
        
        //fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        // if (fd == -1)
        // {
        //   printf("%s Error: System failed to open %s: %s(%d)\n", argv[0], file.c_str(), strerror(errno), errno);
        //   sleep(1);
        // }
        // else
        // {
        //   if (setupTTY(fd) == 0) 
        //   {
        //     controllerState = STATE_WORKING;

        //     sleep(5); //wait for sensor to boot and stabilize
        //   }
        //   else
        //     close(fd);
        // }
        releaseLock();
      }
    }
    else if (controllerState == STATE_WORKING)
    {
      std::string line = "";
      if (checkLock() && getLock())
      {
        UInt8 aBuf[baudrate];
        usb.recv(aBuf,baudrate);
        std::string temp ((const char*)aBuf,baudrate);
        line = temp;
        //line = getTTYLine(fd);
        //printf("got line %s\n", line.c_str());
        releaseLock();
      }

      if (line != "")
      {
        //printf("line: %s\n", line.c_str());
        if (line.length() > 0 && goodLine(line, numVariables))
        {
          int scanfVal;

          if ((scanfVal = sscanf(line.c_str(), "S%f,%f,%f,%d,%f,%f,%f,%d,%d,%dE", &ttemp[0], &ttemp[1], &ttemp[2], &tmotorKilled,
              &tcurVolt[0], &tcurVolt[1], &tpressure, &twaterDetected, &tdropperLeft, &tdropperRight)) == numVariables)
          {
            if ((temp[0] != ttemp[0]) || (temp[1] != ttemp[1]) || (temp[2] != ttemp[2]) || tempTimer.isTimeout())
            {
              //temperature
              std_msgs::Float32MultiArray tempMsg;
              temp[0] = ttemp[0];
              temp[1] = ttemp[1];
              temp[2] = ttemp[2];
              tempMsg.data.push_back(temp[0]);
              tempMsg.data.push_back(temp[1]);
              tempMsg.data.push_back(temp[2]);
              temperatuerPub.publish(tempMsg);
              tempTimer.start(1, 0);
              ros::spinOnce();
            }

            if (motorKilled != tmotorKilled || motorTimer.isTimeout())
            {
              //motorkilled
              std_msgs::UInt8 motorMsg;
              motorKilled = tmotorKilled;
              motorMsg.data = motorKilled;
              MotorPub.publish(motorMsg);
              motorTimer.start(1, 0);
              ros::spinOnce();
            }

            if ((curVolt[0] != tcurVolt[0]) || (curVolt[1] != tcurVolt[1] || currentTimer.isTimeout()))
            {
              //curvolt
              std_msgs::Float32MultiArray curMsg;
              curVolt[0] = tcurVolt[0];
              curVolt[1] = tcurVolt[1];
              curMsg.data.push_back(curVolt[0]);
              curMsg.data.push_back(curVolt[1]);
              computerPub.publish(curMsg);
              currentTimer.start(1, 0);
              ros::spinOnce();
            }

            if (pressure != tpressure || pressureTimer.isTimeout())
            {
              //depth
              std_msgs::Float32 pressureMsg;
              pressure = tpressure;
              pressureMsg.data = pressure;
              pressurePub.publish(pressureMsg);
              pressureTimer.start(1, 0);
              ros::spinOnce();
            }

            if (waterDetected != twaterDetected || waterTimer.isTimeout())
            {
              //water detected
              std_msgs::UInt8 waterMsg;
              waterDetected = twaterDetected;
              waterMsg.data = waterDetected;
              waterPub.publish(waterMsg);
              waterTimer.start(1, 0);
              ros::spinOnce();
            }

            if (tdropperLeft != dropperLeft || tdropperRight != dropperRight || dropperTimer.isTimeout())
            {
              //water detected
              std_msgs::UInt8MultiArray dropperMsg;
              dropperLeft = tdropperLeft;
              dropperRight = tdropperRight;
              dropperMsg.data.push_back(dropperLeft);
              dropperMsg.data.push_back(dropperRight);
              dropperPub.publish(dropperMsg);
              dropperTimer.start(1, 0);
              ros::spinOnce();
            }

            if (VERBOSE)
              ROS_INFO("published");
          }
          else
          {
            printf("%s info: Throwing away %d:\"%s\"\n", argv[0], scanfVal, line.c_str());
            fflush(stdout);
          }
        }
      }
    }
  }

  pthread_mutex_destroy(&myMutex);
  close(fd);
  return 0;
}


void torpedoCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  //launch a torpedo
  uint8_t data[] = {'S', 'T', '0', '0', 'E'};
  data[2] = msg->data + '0';

  //mutex lock
  if (checkLock(true) && getLock())
  {
    if (write(fd, data, 5) < 5)
    {
      printf("Failed to write LaunchTorpedo message to sensorBoard\n");
    }
    releaseLock();
  }
}

void dropperCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  //set dropper
  if (msg->data.size() >= 2)
  {
    uint8_t data[] = {'S', 'D', '0', '0', 'E'};
    data[2] = msg->data[0] + '0';
    data[3] = msg->data[1] + '0';

    //mutex lock
    if (checkLock(true) && getLock())
    {
      if (write(fd, data, 5) < 5)
      {
        printf("Failed to write LaunchTorpedo message to sensorBoard\n");
      }
      releaseLock();
    }
  }
}

bool goodLine(std::string val, int varCount)
{
  bool ret = false;
  int commaCount = 0;
  for (int i = 0; i < val.size(); i++)
  {
    if (val[i] == ',')
      commaCount++;
  }

  if (commaCount == varCount - 1)
  {
    ret = true;
  }

  return ret;
}

std::string getTTYLine(int fd)
{
  std::string ret = "";
  char lastChar;
  bool startFound = false;
  fd_set rdfs;
  struct timeval timeout, start;
  struct timezone tz;
  int val;

  FD_ZERO(&rdfs);
  FD_SET(fd, &rdfs);

  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  gettimeofday(&start, &tz);
  while (lastChar != '\n' && ros::ok() && timeLeft(&start, &timeout))
  {
    if (select(fd+1, &rdfs, NULL, NULL, &timeout) > 0)
    {
      if ((val = read(fd, &lastChar, 1)) == 1)
      {
        if (lastChar == 'S')
        {
          startFound = true;
        }
        else if (lastChar == 'E')
        {
          break;
        }
        else if (lastChar >= '+' && lastChar <= '9')
        {
          ret += lastChar;
        }
      }
      else
      {
        printf("%s Error: fd failed!\n", name);
        controllerState = STATE_WAITING_ON_FD;
        close(fd);
      }
    }
    else
    {
      printf("%s timeout %s\n", name, ret.c_str());
      ret = "";
      break;
    }
  }

  return ret;
}

bool timeLeft(struct timeval* start, struct timeval* timeout)
{
  bool ret = true;
  struct timeval now;
  struct timezone tz;
  gettimeofday(&now, &tz);

  long int usec = (start->tv_sec + timeout->tv_sec) * 1000000 + (start->tv_usec + timeout->tv_usec);
  long int nowUsec = (now.tv_sec * 1000000) + now.tv_usec;
  long int left = usec - nowUsec;

  if (usec <= nowUsec)
  {
    ret = false;
  }
  else
  {
    timeout->tv_sec = left / 1000000;
    timeout->tv_usec = left % 1000000;
  }

  *start = now;

  return ret;
}

int setupTTY(int fd)
{
  int ret = 0;

  fcntl(fd, F_SETFL, 0);
  struct termios port_settings;      // structure to store the port settings in

  cfsetispeed(&port_settings, B115200);    // set baud rates
  cfsetospeed(&port_settings, B115200);

  port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  port_settings.c_cflag &= ~CRTSCTS;

  port_settings.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  port_settings.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  port_settings.c_cc[VMIN]  = 0;
  port_settings.c_cc[VTIME] = 20;

  if(tcsetattr(fd, TCSANOW, &port_settings) < 0)    // apply the settings to the port
  {
    printf("%s Error: Failed to set serial settings\n", name);
    ret = -1;
  }

  return ret;
}

bool checkLock(bool loop)
{
  while (!checkLock());
  return true;
}

bool checkLock()
{
  return (pthread_mutex_trylock(&myMutex) ? false : true);
}

bool getLock()
{
  return (pthread_mutex_unlock(&myMutex) ? false : true);
}

bool releaseLock()
{
  return (pthread_mutex_unlock(&myMutex) ? false : true);
}
