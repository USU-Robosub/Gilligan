#include "ros/ros.h"
#include "ObstacleCourseTask.hpp"
#include "SubImageRecognition/ImgRecObject.h"
#include "std_msgs/StringMultiArray.h"


ObstacleCourseTask::ObstacleCourseTask()
 : m_nodeHandle(),
   m_obstacleCourseSubscriber(),
   m_taskStateSubscriber(),
   m_highLevelMotorPublisher(),
   m_taskCompletePublisher(),
   m_isEnabled(false),
   m_foundFirst(false),
   m_distanceToObject(0.0)
{
  m_obstacleCourseSubscriber = m_nodeHandle.subscribe("img_rec/obstacle_course", 10, &PathTask::obstacleCourseCallback, this);
  m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &PathTask::moduleEnableCallback, this);
  m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motor_Control", 10);
  m_taskCompletePublisher = m_nodeHandle.advertise<std_msgs::StringMultiArray>("Task_Completion", 10);
}

ObstacleCourseTask::~ObstacleCourseTask()
{

}

void ObstacleCourseTask::moduleEnableCallback(const Robosub::ModuleEnableMsg& msg)
{
  if (msg.Module == "ObstacleCourseTask")
  {
    if (msg.State == true)
    {
      printf("ObstacleCourseTask: enabled\n");
      m_isEnabled = true;
    }
    else
    {
      printf("ObstacleCourseTask: Disabled\n");
      m_isEnabled = false;
    }
  }
}

void ObstacleCourseTask::obstacleCourseCallback(const SubImageRecognition::ImgRecObject& msg)
{
  if (m_isEnabled)
  {
    m_distanceToObject = computeDistanceToObject(msg.height, msg.width);
    float moveX = msg.center_x - 320;
    float moveY = msg.center_y - 240;

    if (moveX <= 330 && moveX >= 310 && moveY <= 250 && moveY >= 230)
    {
      //we have centered, drive forward
      publishMotor("Forward", "Offset", m_distanceToObject);
      reportSuccess(true);
    }
    else if (moveX > 330 || moveX < 310)
    {
      printf("Correcting straf by: %f\n", moveX);
      publishMotor("Straf", "Manual", moveX);
    }
    else if (moveY > 250 || moveY < 230)
    {
      printf("Correcting depth by: %f\n", moveY);
      publishMotor("Depth", "Manual", moveY);
    }
  }
}

float ObstacleCourseTask::computeDistanceToObject(float height, float width)
{

}

void ObstacleCourseTask::reportSuccess(bool success)
{
  std_msgs::StringMultiArray msg;

  msg.data.push_back("PathTask");
  if (success)
  {
    msg.data.push_back("Success");
  }
  else
  {
    msg.data.push_back("Failure");
  }
  m_taskCompletePublisher.publish(msg);
}
