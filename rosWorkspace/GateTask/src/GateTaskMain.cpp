#include "GateTask.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "GateTask");
  GateTask task;
  task.run();
  return 0;
}
