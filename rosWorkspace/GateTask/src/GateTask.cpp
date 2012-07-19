#include "GateTask.hpp"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/Point.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <math.h>
#include <float.h>

GateTask::GateTask()
 : m_nodeHandle(),
   m_gateSubscriber(),
   m_taskStateSubscriber(),
   m_centerPointPublisher(),
   m_taskCompletePublisher(),
   m_highLevelMotorPublisher(),
   m_isEnabled(false),
   m_identifiedCenter(false),
   m_haveBothLegs(false),
   m_center(),
   m_zeroth(),
   m_lastId(-1)
{
  m_gateSubscriber = m_nodeHandle.subscribe("img_rec/gate", 10, &GateTask::gateCallback, this);
  m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &GateTask::moduleEnableCallback, this);
  m_centerPointPublisher = m_nodeHandle.advertise<Robosub::Point>("Center_On_Point", 10);
  m_taskCompletePublisher = m_nodeHandle.advertise<std_msgs::String>("Task_Completion", 10);
  m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motion", 10);
}

GateTask::~GateTask()
{

}


void GateTask::run()
{
  while (ros::ok())
  {
    ros::spin();
  }
}

void GateTask::moduleEnableCallback(const Robosub::ModuleEnableMsg& msg)
{
  if (msg.Module == "GateTask")
  {
    if (msg.State == true)
    {
      printf("GateTask: enabled\n");
      m_isEnabled = true;
      m_identifiedCenter = m_haveBothLegs = false;
    }
    else
    {
      printf("GateTask: Disabled\n");
      m_isEnabled = false;
    }
  }
}

void GateTask::gateCallback(const SubImageRecognition::ImgRecObject& msg)
{
  if (m_isEnabled)
  {
    if (m_identifiedCenter && m_lastId == msg.id)
    {
      //then we see a section of the gate. Identify its position in the screen and move it out of screen (turn)
      float dir = 2.0f;
      dir *= msg.center_x > 0.0f ? 1.0f : -1.0f;
      publishMotor("Turn", "Offset", dir);
    }
    else
    {
      if (msg.id == 0)
      {
        m_zeroth = msg;
      }

      if (m_lastId == msg.id)
      {
        //dont have two, turn away
        float dir = 2.0f;
        dir *= msg.center_x > 0.0f ? 1.0f : -1.0f;
        publishMotor("Turn", "Offset", dir);
      }
      else if (msg.id != 0)
      {
        //have both, calculate center
        m_center.center_x = (m_zeroth.center_x + msg.center_x)/2;
        m_center.center_y = m_zeroth.center_y + msg.center_y;
        m_identifiedCenter = true;

        if (m_center.center_x < -10.0f || m_center.center_x > 10.0f)
        {
          printf("Turning by %f\n", m_center.center_x);
          publishMotor("Turn", "Offset", m_center.center_x);
          //possibly turn off forward
        }
        else
        {
          publishMotor("Forward", "Offset", getDistance(msg.width, 3.0f)*1.75);
        }
      }
    }
    m_lastId = msg.id;
  }
}


/**
 * @brief Calcualte the distance the center of the object is off from the center of the camera in inches
 *
 * @param centerDir The current center direction
 * @param width The width of the object
 *
 * @return The distance in inches
 */
float GateTask::calculateDistanceFromCenter(float centerDir, float width)
{
  float pxlPerInch = getPixelsPerInch(width, 3.0f);
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
float GateTask::getPixelsPerInch(float curWidthPixels, float expectedWidthInches)
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
float GateTask::getDistance(float curObjSize, float actualObjSize)
{
  return (actualObjSize/(2.0f*tan(((curObjSize * (M_PI/3.0))/960.0f))));
}

/**
 * @brief plubish to the high level motor controller topic
 *
 * @param direction The direction: Forward, Straf, Turn
 * @param motion The motion type: Manual, Offset
 * @param value The value
 */
void GateTask::publishMotor(std::string direction, std::string motion, float value)
{
  Robosub::HighLevelControl msg;
  msg.Direction = direction;
  msg.MotionType = motion;
  msg.Value = value;

  m_highLevelMotorPublisher.publish(msg);
}
