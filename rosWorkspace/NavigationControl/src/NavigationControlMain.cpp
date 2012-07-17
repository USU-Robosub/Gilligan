#include "NavigationControl.hpp"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "NavigationControl");
  NavigationControl task;
  task.run();
  return 0;
}
