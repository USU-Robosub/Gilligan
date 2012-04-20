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
#include "timer.h"

#define STATE_WAITING_ON_FD 0
#define STATE_WORKING       1
#define VERBOSE 0



int setupTTY(int fd);
std::string getTTYLine(int fd);
bool timeLeft(struct timeval* start, struct timeval* timeout);
bool goodLine(std::string val, int varCount);

int controllerState = STATE_WAITING_ON_FD;
char* name;

void error(char * msg)
{
    perror(msg);
    exit(-1);
}

int main(int argc, char **argv)
{
  int fd;
  std::string file = "/dev/controller_sensor";

  if (argc > 1)
  {
    file = argv[1];
    printf("%s info: Opening %s\n", argv[0], file.c_str());
  }

  name = argv[0];

  float temp[3] = {0.0};
  float pressure = 0.0;
  int motorKilled = 0, waterDetected;
  float curVolt[2] = {0.0};
  int numVariables = 8;

  float ttemp[3] = {0.0};
  float tpressure = 0.0;
  int tmotorKilled = 0, twaterDetected;
  float tcurVolt[2] = {0.0};

  timer tempTimer, pressureTimer, motorTimer, currentTimer, waterTimer;

  tempTimer.start(1, 0);
  pressureTimer.start(1, 0);
  motorTimer.start(1, 0);
  currentTimer.start(1, 0);
  waterTimer.start(1, 0);

  ros::init(argc, argv, "SubSensorController");
  ros::NodeHandle nh;

  ros::Publisher temperatuerPub = nh.advertise<std_msgs::Float32MultiArray>("Controller_Box_Temp", 1000);
  ros::Publisher pressurePub = nh.advertise<std_msgs::Float32>("Pressure_Data", 1000);
  ros::Publisher MotorPub = nh.advertise<std_msgs::UInt8>("Motor_State", 1000);
  ros::Publisher computerPub = nh.advertise<std_msgs::Float32MultiArray>("Computer_Cur_Volt", 1000);
  ros::Publisher waterPub = nh.advertise<std_msgs::UInt8>("Water_Detected", 1000);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    if (controllerState == STATE_WAITING_ON_FD)
    {
      fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

      if (fd == -1)
      {
        printf("%s Error: System failed to open %s: %s(%d)\n", argv[0], file.c_str(), strerror(errno), errno);
        sleep(1);
      }
      else
      {
        if (setupTTY(fd) == 0)
        {
          controllerState = STATE_WORKING;

          sleep(5); //wait for sensor to boot and stabilize
        }
        else
          close(fd);
      }
    }
    else if (controllerState == STATE_WORKING)
    {
      std::string line = "";
      line = getTTYLine(fd);
      //printf("line: %s\n", line.c_str());
      if (line.length() > 0 && goodLine(line, numVariables))
      {
        int scanfVal;

        if ((scanfVal = sscanf(line.c_str(), "%f,%f,%f,%d,%f,%f,%f,%d", &ttemp[0], &ttemp[1], &ttemp[2], &tmotorKilled,
            &tcurVolt[0], &tcurVolt[1], &tpressure, &twaterDetected)) == numVariables)
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

  close(fd);
  return 0;
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

