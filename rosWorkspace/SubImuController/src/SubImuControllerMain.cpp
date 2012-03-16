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
bool timeLeft(struct timeval* start, struct timeval* timeout);


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
    printf("Opening %s\n", file.c_str());
  }

  fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1)
  {
    printf("System failed to open %s: %s(%d)\n", file.c_str(), strerror(errno), errno);
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
    printf("loop\n");
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
  struct timeval timeout, start;
  struct timezone tz;

  FD_ZERO(&rdfs);
  FD_SET(fd, &rdfs);

  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  gettimeofday(&start, &tz);
  while (lastChar != '\n' && ros::ok() && timeLeft(&start, &timeout))
  {
    if (select(fd+1, &rdfs, NULL, NULL, &timeout) > 0)
    {
      if (read(fd, &lastChar, 1) == 1)
      {
        if (lastChar == 'S')
        {
          startFound = true;
        }
        else if (lastChar == 'E')
        {
          startFound = false;
        }
        else
        {
          ret += lastChar;
        }
      }
    }
    else
    {
      printf("failure\n");
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
    printf("Failed to set serial settings\n");
    exit(-1);
  }

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
