#include <cstring>
#include <string>
#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8MultiArray.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "SubImuController");
  printf("ROS init complete\n");

  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16>("Motor_Driver_Drive", 1000);
  printf("ros publisher\n");
  ros::Rate loop_rate(1);
  printf("ros looprate\n");

  std_msgs::Int16 msg;
  //msg.data.push_back(0);
  //msg.data.push_back(0);
  uint8_t motorMask = 0x4;
  uint8_t speed;
  uint16_t val;

  while (ros::ok())
  {
    for (int i = 0; i <= 511 && ros::ok(); i++)
    {
      msg.data = motorMask;
      msg.data = msg.data << 8;
      msg.data += i;
      chatter_pub.publish(msg);
      printf("published: %d, %d\n", ((msg.data >> 8 ) & 0xff), msg.data & 0xff);
      ROS_INFO("published");
      usleep(1000000);
      ros::spinOnce();
    }

    usleep(1000000);

    motorMask = 0x40;
    for (int i = 0; i <= 511 && ros::ok(); i++)
    {
      msg.data = motorMask;
      msg.data = msg.data << 8;
      msg.data += i;
      chatter_pub.publish(msg);
      printf("published: %d, %d\n", ((msg.data >> 8 ) & 0xff), msg.data & 0xff);
      ROS_INFO("published");
      usleep(1000000);
      ros::spinOnce();
    }
  }

  return 0;
}
