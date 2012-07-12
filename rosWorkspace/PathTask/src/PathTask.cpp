#include "PathTask.hpp"
#include "Robosub/HighLevelControl.h"
#include "std_msgs/StringMultiArray.h"

/**
 * @brief The constructor. Subscribes to all needed topics and sets up callbacks
 */
PathTask::PathTask()
 : m_nodeHandle(),
   m_pathSubscriber(),
   m_taskStateSubscriber(),
   m_highLevelMotorPublisher(),
   m_taskCompletePublisher(),
   m_isEnabled(false),
   m_mode(0)
{
   m_pathSubscriber = m_nodeHandle.subscribe("img_rec/paths", 10, &PathTask::pathCallback, this);
   m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &PathTask::moduleEnableCallback, this);
   m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motor_Control", 10);
   m_taskCompletePublisher = m_nodeHandle.advertise<std_msgs::String>("Task_Completion", 10);
}

/**
 * @brief The destructor
 */
PathTask::~PathTask()
{

}

/**
 * @brief The module enable message callback
 *
 * @param msg The module enabled message
 */
void PathTask::moduleEnableCallback(const Robosub::ModuleEnableMsg& msg)
{
	if (msg.Module == "PathTask")
	{
		if (msg.State == true)
		{
			printf("PathTask: enabled\n");
			m_isEnabled = true;
			m_mode = 0;
		}
		else
		{
			printf("PathTask: Disabled\n");
			m_isEnabled = false;
		}
	}
}

/**
 * @brief The path callback
 *
 * @param msg The image recognition object message
 */
void PathTask::pathCallback(const SubImageRecognition::ImgRecObject& msg)
{
	if (m_isEnabled)
	{
	  //first get centered, then rotate, then recenter if needed
	  float moveX = msg.center_x - 320;
	  float moveY = msg.center_y - 240;
    float turn = msg.rotation;

    if (m_mode == 0)
    {
      if (moveX <= 330 && moveX >= 310 && moveY <= 250 && moveY >= 230)
      {
        //we have arrived, lets spin
        if (turn <= 1 && turn >= -1)
        {
          //signal complete
          reportSuccess(true);
        }
        else
        {
          m_mode = 1;
        }
      }
      else if (moveX > 330 || moveX < 310)
      {
        printf("Correcting straf by: %f\n", moveX);
        publishMotor("Straf", "Manual", moveX);
      }
      else if (moveY > 250 || moveY < 230)
      {
        printf("Correcting forward by: %f\n", moveY);
        publishMotor("Forward", "Manual", moveY);
      }
    }

    if (m_mode == 1)
    {
      if (turn <= 1 && turn >= -1)
      {
        m_mode = 0;
      }
      else
      {
        printf("Correcting turn by: %f\n", turn);
        publishMotor("Turn", "Manual", turn);
      }
    }
	}
}

void PathTask::reportSuccess(bool success)
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

void PathTask::publishMotor(std::string direction, std::string motion, float value)
{
  Robosub::HighLevelControl msg;
  msg.Direction = direction;
  msg.MotionType = motion;
  msg.Value = value;

  m_highLevelMotorPublisher.publish(msg);
}
