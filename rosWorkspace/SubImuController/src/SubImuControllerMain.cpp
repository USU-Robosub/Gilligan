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


#define YAW   0
#define PITCH 1
#define ROLL  2

void setupTTY(int fd);
std::string getTTYLine(int fd);
void tokenize(std::string line, float* buf);


void error(char * msg)
{
    perror(msg);
    exit(-1);
}

int main(int argc, char **argv)
{
  int fd = 0;
  float buf[3];
  std::string file = "/dev/ttyUSB0";

  if (argc > 1)
  {
    file = argv[1];
  }

  fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1)
  {
    printf("System failed to open /dev/ttyUSB0: %s(%d)\n", strerror(errno), errno);
    return -1;
  }

  ros::init(argc, argv, "SubImuController");
  printf("ROS init complete\n");

  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Data", 1000);
  printf("ros publisher\n");

  ros::Rate loop_rate(1);
  printf("ros looprate\n");

  while (ros::ok())
  {
      std::string line = getTTYLine(fd);
      if (line.length() > 0)
      {
        std_msgs::Float32MultiArray msg;
        tokenize(line, buf);
        printf("Raw line: %s\n", line.c_str());
        printf("Data is: %f, %f, %f\n", buf[0], buf[1], buf[2]);
        msg.data.push_back(buf[0]);
        msg.data.push_back(buf[1]);
        msg.data.push_back(buf[2]);
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */

        //ROS_INFO("publishing value of %u", msg.data);

        chatter_pub.publish(msg);
        ROS_INFO("published");

        ros::spinOnce();
      }
  }

  close(fd);
  return 0;
}

std::string getTTYLine(int fd)
{
  std::string ret = "";
  char lastChar;
  bool startFound = false;
  fd_set rdfs;
  struct timeval timeout;

  timeout.tv_sec = 1;

  while (lastChar != '\n' && lastChar != 'E' && ros::ok())
  {
    if (select(fd, &rdfs, NULL, NULL, &timeout) > 0)
    {
      if (read(fd, &lastChar, 1) == 1)
      {
        if (lastChar == 'S')
        {
          startFound = true;
        }
        else if (startFound && lastChar != 'E')
        {
          ret += lastChar;
        }
      }
    }
    else
    {
      break; //select timed out
    }
  }

  return ret;
}

void setupTTY(int fd)
{
  fcntl(fd, F_SETFL, 0);
  struct termios port_settings;      // structure to store the port settings in

  cfsetispeed(&port_settings, B115200);    // set baud rates
  cfsetospeed(&port_settings, B115200);

  port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port

}

void tokenize(std::string line, float* buf)
{
  std::string yaw, pitch, roll;
  int mode = 0;
  buf[0] = 0.0;
  buf[1] = 0.0;
  buf[2] = 0.0;

  for (int i = 0; i < line.length(); i++)
  {
    if (line[i] != ' ')
    {
      if (mode == 0)
      {
        yaw += line[i];
      }
      else if (mode == 1)
      {
        pitch += line[i];
      }
      else
      {
        roll += line[i];
      }
    }
    else
    {
      mode++;
    }
  }

  if (yaw.length() >= 1)
  {
    buf[0] = atof(yaw.c_str());
  }

  if (pitch.length() >= 1)
  {
    buf[1] = atof(pitch.c_str());
  }

  if (roll.length() >= 1)
  {
    buf[2] = atof(roll.c_str());
  }
}
