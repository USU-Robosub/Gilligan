#include "PathTask.hpp"
#include "Robosub/HighLevelControl.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <math.h>
#include <float.h>

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
   m_frames(0)
{
   printf("setup\n");
	fflush(NULL);
	m_pathSubscriber = m_nodeHandle.subscribe("img_rec/paths", 10, &PathTask::pathCallback, this);
   m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &PathTask::moduleEnableCallback, this);
   m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motion", 10);
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
			m_frames = 0;
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
  //TODO update to allow for multiple paths per frame
  //always choose the one in frame or the one closest to 0 degrees
	if (m_isEnabled)
	{
	  int hit = 0;

	  //first get centered, then rotate, then recenter if needed
	  float moveX = calculateDistanceFromCenter(msg.center_x, msg.width);
	  float moveY = calculateDistanceFromCenter(msg.center_y, msg.width);
    float turn = msg.rotation;

    if (moveX > 10 || moveX < -10)
    {
      printf("Correcting straf by: %f\n", moveX);
      publishMotor("Straf", "Offset", moveX/12); //inches to feet
      hit++;
    }

    if (moveY > 10 || moveY < -10)
    {
      printf("Correcting forward by: %f\n", moveY);
      publishMotor("Forward", "Offset", moveY/12);
      hit++;
    }

    if (turn > 1 || turn < -1)
    {
      printf("Correcting turn by: %f\n", turn);
      publishMotor("Turn", "Offset", turn);
      hit++;
    }

    if (hit == 0)
    {
      m_frames++;
    }
    else
    {

    }

    if (m_frames >= 10)
    {
      reportSuccess(true);
      m_frames = 0;
    }
	}
}

/**
 * @brief report the success or failure of the module
 *
 * @param success The success or failure
 */
void PathTask::reportSuccess(bool success)
{
	std_msgs::String msg;
	msg.data = "PathTask";
	if (success)
	{
		msg.data += " Success";
	}
	else
	{
	  msg.data = " Failure";
	}
	m_taskCompletePublisher.publish(msg);
}

/**
 * @brief plubish to the high level motor controller topic
 *
 * @param direction The direction: Forward, Straf, Turn
 * @param motion The motion type: Manual, Offset
 * @param value The value
 */
void PathTask::publishMotor(std::string direction, std::string motion, float value)
{
  Robosub::HighLevelControl msg;
  msg.Direction = direction;
  msg.MotionType = motion;
  msg.Value = value;

  m_highLevelMotorPublisher.publish(msg);
}

/**
 * @brief Calcualte the distance the center of the object is off from the center of the camera in inches
 *
 * @param centerDir The current center direction
 * @param width The width of the object
 *
 * @return The distance in inches
 */
float PathTask::calculateDistanceFromCenter(float centerDir, float width)
{
  float pxlPerInch = getPixelsPerInch(width, 6.0f);
  return centerDir * pxlPerInch;
}

/**
 * @brief Get the number of pixels per inch
 *
 * @param curWidthPixels The current width of a known object in pixels
 * @param expectedWidthInches The expected with of an object in inches
 *
 * @return The number of pixels per inch if everything goes well, or float_max if inputs were unacceptable
 */
float PathTask::getPixelsPerInch(float curWidthPixels, float expectedWidthInches)
{
  if (curWidthPixels > 0)
  {
    float cSqr = expectedWidthInches * expectedWidthInches;
    float bSqr = (expectedWidthInches/2) * (expectedWidthInches/2);
    float f = expectedWidthInches * sqrt(cSqr - bSqr);
    return expectedWidthInches * (f/curWidthPixels);
  }
  return FLT_MAX; //infinit
}
