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
#define ACCEL 0
#define GYRO  3
#define MAGN  6

#define VARIABLE_COUNT 12

void setupTTY(int fd);
std::string getTTYLine(int fd);
void tokenize(std::string line, float* buf);
bool timeLeft(struct timeval* start, struct timeval* timeout);
bool goodLine(std::string val);

void error(char * msg)
{
    perror(msg);
    exit(-1);
}

int main(int argc, char **argv)
{
  int fd = 0;
  float buf[VARIABLE_COUNT];
  std::string file = "/dev/controller_Imu";

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
  
  setupTTY(fd);

  sleep(5); //wait for IMU to boot and stabalize

  ros::init(argc, argv, "SubImuController");
  ros::NodeHandle nh;

  ros::Publisher headingPub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Attitude", 1000);
  ros::Publisher rawPub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Raw", 1000);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
      std::string line = getTTYLine(fd);
      if (line.length() > 0 && goodLine(line))
      {
        tokenize(line, buf);

        //send headings
        std_msgs::Float32MultiArray msg;
        msg.data.push_back(buf[YAW]);
        msg.data.push_back(buf[PITCH]);
        msg.data.push_back(buf[ROLL]);
        headingPub.publish(msg);

        //send accel
        std_msgs::Float32MultiArray rawMsg;
        for (int i = ACCEL; i < VARIABLE_COUNT; i++)
        {
          rawMsg.data.push_back(buf[i]);
        }
        rawPub.publish(rawMsg);

        ROS_INFO("published");

        ros::spinOnce();
      }
  }

  close(fd);
  return 0;
}

bool goodLine(std::string val)
{
  bool ret = false;
  int counter = 0;

  for (int i = 0; i < val.size(); i++)
  {
    if (val[i] == ',')
      counter++;
  }

  if (counter == VARIABLE_COUNT)

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
          break;
        }
        else if (lastChar >= '+' && lastChar <= '9')
        {
          ret += lastChar;
        }
      }
    }
    else
    {
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
  std::string tmp = "";
  int mode = 0;

  for (int i = 0; i < VARIABLE_COUNT; i++)
    buf[i] = 0.0;

  for (int i = 0; i < line.length(); i++)
  {
    if (line[i] != ',')
    {
      tmp += line[i];
    }
    else
    {
      buf[mode] = atof(tmp.c_str());
      mode++;

    }
  }
}
