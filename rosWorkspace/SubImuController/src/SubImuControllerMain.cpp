
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
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

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  int fd = 0;
  float buf[3];

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1)
  {
    printf("System failed to open /dev/ttyUSB0: %s(%d)\n", strerror(errno), errno);
    return -1;
  }

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "SubImuController");
  printf("ROS init complete\n");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Data", 1000);
  printf("ros publisher\n");

  ros::Rate loop_rate(1);
  printf("ros looprate\n");

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

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

        //loop_rate.sleep();
        ++count;
      }
  }

  close(fd);
  return 0;
}

std::string getTTYLine(int fd)
{
  std::string ret = "";
  char lastChar;
  fd_set rdfs;
  struct timeval timeout;

  timeout.tv_sec = 1;

  //n = select(fd, &rdfs, NULL, NULL, &timeout);
  while (lastChar != '\n' && ros::ok())
  {
    if (read(fd, &lastChar, 1) == 1)
    {
      ret += lastChar;
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
