#include "GateTask.hpp"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/Point.h"
#include "std_msgs/String.h"
//#include "Positioning.hpp"
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
    //               vvvvvv  This only works on gate Id 0
  if (m_isEnabled && msg.id == 0) //It this ID the left gate?
  {


    float pixPerInch = getPixelsPerInch(msg.width, 3.0f);
    float dist = getDistance(msg.width, 3.0f);
    float x = (msg.center_x / pixPerInch) + 36.0f; //add a 3 feet
    float leftMost = msg.center_x - (msg.width/2.0f);
    float rightMost = msg.center_x + (msg.width/2.0f);


    //This code could be replaced by publishing to center on point,
    //if the Navigation Control module were working
    if ((x < -10.0f || x > 10.0f) &&  (leftMost > -230.0f && rightMost < 230.0f)) //if we are too close to the edge, just drive straight
    {
      printf("Strafing by %f\n", x/12.0f);
	publishMotor("Straf", "Offset", x/12.0f);
    }

    if (leftMost > -230.0 && rightMost < 230.0)
    {
	printf("Forward by %f\n", dist + 4);
      publishMotor("Forward", "Offset", dist + 4);
    }
    else
    {
      printf("Disabling\n");
      reportSuccess(true);
    }
  }
}


/**
 * @brief Calculate the distance the center of the object is off from the center of the camera in inches
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
    //FIXIT: What formula is this?
  return (actualObjSize/(2.0f*tan(((curObjSize * (M_PI/4.0))/960.0f))));
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


/**
 * @brief report the success or failure of the module
 *
 * @param success The success or failure
 */
void GateTask::reportSuccess(bool success)
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
