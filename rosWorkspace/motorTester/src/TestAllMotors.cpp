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
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16>("Motor_Data", 1000);
  printf("ros publisher\n");
  ros::Rate loop_rate(1);
  printf("ros looprate\n");

  std_msgs::Int16 msg;
  //msg.data.push_back(0);
  //msg.data.push_back(0);
  uint8_t motorMask;
  uint8_t speed;
  uint16_t val;

  for (uint8_t i = 0; i < 8 && ros::ok(); i++)
  {
    val = 1;
    val = val << (15 - i);
    printf("val is %x\n", val);
    for (uint8_t j = 0; j <= 252 && ros::ok(); j+= 2)
    {
      msg.data = val;
      msg.data += j;
      chatter_pub.publish(msg);
      printf("published: %d, %d\n", ((msg.data >> 8 ) & 0xff), msg.data & 0xff);
      printf("value is %x\n", msg.data);
      ROS_INFO("published");

      usleep(1000000);

      ros::spinOnce();
    }

    msg.data &= 0xff00;
    chatter_pub.publish(msg);
    printf("published: %d, %d\n", ((msg.data >> 8 ) & 0xff), msg.data & 0xff);
    ROS_INFO("published");

    usleep(1000000);

    ros::spinOnce();
  }

  return 0;
}
