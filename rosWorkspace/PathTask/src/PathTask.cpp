#include "PathTask.hpp"
#include "Robosub/HighLevelControl.h"
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
   m_pathDirSubscriber(),
   m_highLevelMotorPublisher(),
   m_taskCompletePublisher(),
   m_isEnabled(false),
   m_frames(0),
   m_direction("left"),
   m_msgList(),
   m_lastId(-1)
{
   printf("setup\n");
   fflush(NULL);
   m_pathSubscriber = m_nodeHandle.subscribe("img_rec/paths", 10, &PathTask::pathCallback, this);
   m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &PathTask::moduleEnableCallback, this);
   m_pathDirSubscriber = m_nodeHandle.subscribe("Path_Direction", 10, &PathTask::pathDirectionCallback, this);
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
      m_msgList.clear();
    }
    else
    {
      printf("PathTask: Disabled\n");
      m_isEnabled = false;
    }
  }
}

void PathTask::pathDirectionCallback(const std_msgs::String& msg)
{
  m_direction = msg.data;
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
	  if (msg.id == 0 && m_lastId != -1)
	  {
	    //find the correct target
	    SubImageRecognition::ImgRecObject tmp = getMessage();

	    manualMode(msg);
	    //offsetMode(msg);
	    m_msgList.clear();
	    m_msgList.push_back(msg);
	  }
	  else
	  {
	    m_msgList.push_back(msg);
	  }

    m_lastId = msg.id;
	}
}

void PathTask::manualMode(const SubImageRecognition::ImgRecObject& msg)
{
  int hit = 0;
  float moveX = msg.center_x;
  float moveY = msg.center_y;
  float turn = msg.rotation;

  if (moveX > 10 || moveX < -10)
  {
    printf("Correcting straf by: %f\n", moveX);
    publishMotor("Straf", "Manual", moveX);
    hit++;
  }
  else
  {
    publishMotor("Straf", "Manual", 0.0);
  }

  if (moveY > 10 || moveY < -10)
  {
    printf("Correcting forward by: %f\n", moveY);
    publishMotor("Forward", "Manual", moveY*2.0);
    hit++;
  }
  else
  {
    publishMotor("Forward", "Manual", 0.0);
  }

  if (turn > 1 || turn < -1)
  {
    printf("Correcting turn by: %f\n", turn);
    publishMotor("Turn", "Manual", turn);
    hit++;
  }
  else
  {
    publishMotor("Turn", "Manual", 0.0);
  }

  if (hit == 0)
  {
    m_frames++;
  }
  else
  {
    m_frames = 0;
  }

  if (m_frames >= 10)
  {
    reportSuccess(true);
    m_frames = 0;
    m_isEnabled = false;
  }
}

void PathTask::offsetMode(const SubImageRecognition::ImgRecObject& msg)
{
  int hit = 0;
  float moveX = calculateDistanceFromCenter(msg.center_x, msg.width);
  float moveY = calculateDistanceFromCenter(msg.center_y, msg.width);
  float turn = msg.rotation;

  if (moveX > 10 || moveX < -10)
  {
    printf("Correcting straf by: %f\n", moveX);
    publishMotor("Straf", "Offset", moveX > 0 ? 1 : -1); //inches to feet
    hit++;
  }
  else
  {
    publishMotor("Straf", "Offset", 0.0);
  }

  if (moveY > 10 || moveY < -10)
  {
    printf("Correcting forward by: %f\n", moveY);
    publishMotor("Forward", "Offset", moveY > 0 ? 1 : -1);
    hit++;
  }
  else
  {
    publishMotor("Forward", "Offset", 0.0);
  }

  if (turn > 1 || turn < -1)
  {
    printf("Correcting turn by: %f\n", turn);
    publishMotor("Turn", "Offset", turn);
    hit++;
  }
  else
  {
    publishMotor("Turn", "Offset", 0.0);
  }

  if (hit == 0)
  {
    m_frames++;
  }
  else
  {
    m_frames = 0;
  }

  if (m_frames >= 10)
  {
    reportSuccess(true);
    m_frames = 0;
    m_isEnabled = false;
  }
}

SubImageRecognition::ImgRecObject PathTask::getMessage()
{
  SubImageRecognition::ImgRecObject ret;

  if (m_direction == "right")
  {
    int greatest = SHRT_MIN;
    for (std::list<SubImageRecognition::ImgRecObject>::iterator it = m_msgList.begin(); it != m_msgList.end(); it++)
    {
      if (it->center_x > greatest)
      {
        greatest = it->center_x;
        ret = *it;
      }
    }
  }
  else if (m_direction == "left")
  {
    int greatest = SHRT_MAX;
    for (std::list<SubImageRecognition::ImgRecObject>::iterator it = m_msgList.begin(); it != m_msgList.end(); it++)
    {
      if (it->center_x < greatest)
      {
        greatest = it->center_x;
        ret = *it;
      }
    }
  }
  else
  {
    if (m_msgList.size() > 0)
    {
      ret = m_msgList.front();
    }
  }

  return ret;
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
  float dist = getDistance(curWidthPixels, expectedWidthInches);
  return curWidthPixels/dist;
}

/**
 * @brief Calculate the distance to the object
 *
 * @param curObjSize The objects current size
 * @param actualObjSize The objects expected size
 *
 * @return The distance
 */
float PathTask::getDistance(float curObjSize, float actualObjSize)
{
  return (actualObjSize/(2.0f*tan(curObjSize * (M_PI/4.0f)/960.0f))); //45 degrees
}
