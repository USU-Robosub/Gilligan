/*
 * NewSubImuControllerMain.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: bholdaway
 */

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

#define STATE_WAITING_ON_FD 0
#define STATE_WORKING       1
#define VERBOSE             0

#define YAW   0
#define PITCH 1
#define ROLL  2
#define ACCEL 3
#define GYRO  6
#define MAGN  9

#define VARIABLE_COUNT 12

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
  std::string file = "/dev/controller_Imu";

  if (argc > 1)
  {
    file = argv[1];
    printf("%s info: Opening %s\n", argv[0], file.c_str());
  }

  name = argv[0];

  float data[VARIABLE_COUNT];
  float tdata[VARIABLE_COUNT];

  ros::init(argc, argv, "SubImuController");
  ros::NodeHandle nh;

  ros::Publisher headingPub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Attitude", 1000);
  ros::Publisher rawPub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Raw", 1000);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    if (controllerState == STATE_WAITING_ON_FD)
    {
      fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

      if (fd == -1)
      {
        printf("%s error: System failed to open %s: %s(%d)\n", argv[0], file.c_str(), strerror(errno), errno);
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
      if (line.length() > 0 && goodLine(line, VARIABLE_COUNT))
      {
        int scanfVal;

        if ((scanfVal = sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &tdata[0], &tdata[1], &tdata[2], &tdata[3], &tdata[4],
            &tdata[5], &tdata[6], &tdata[7], &tdata[8], &tdata[9], &tdata[10], &tdata[11])) == VARIABLE_COUNT)
        {
          //publish attitude
          std_msgs::Float32MultiArray attitudeMsg;
          for (int i = YAW; i < ACCEL; i++)
          {
            attitudeMsg.data.push_back(tdata[i]);
          }
          headingPub.publish(attitudeMsg);

          //publish raw
          std_msgs::Float32MultiArray rawMsg;
          for (int i = ACCEL; i < VARIABLE_COUNT; i++)
          {
            rawMsg.data.push_back(tdata[i]);
          }
          rawPub.publish(rawMsg);

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
  else
    printf("%s error: not enough commas\n", name);

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
      printf("%s Error: timeout %s\n", name, ret.c_str());
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

