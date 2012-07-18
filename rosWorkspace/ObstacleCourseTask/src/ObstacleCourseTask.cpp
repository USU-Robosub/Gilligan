#include "ros/ros.h"
#include "ObstacleCourseTask.hpp"
#include "SubImageRecognition/ImgRecObject.h"
#include "std_msgs/String.h"


ObstacleCourseTask::ObstacleCourseTask()
 : m_nodeHandle(),
   m_obstacleCourseSubscriber(),
   m_taskStateSubscriber(),
   m_highLevelMotorPublisher(),
   m_taskCompletePublisher(),
   m_taskDownObstacleDetectorSubscriber(),
   m_isEnabled(false),
   m_foundFirst(false),
   m_distanceToObject(0.0),
   m_hbar(),
   m_vbarLeft(),
   m_vbarRight(),
   m_targetCenter_x(0),
   m_targetCenter_y(0),
   m_centerSet(false)
{
  m_obstacleCourseSubscriber = m_nodeHandle.subscribe("img_rec/obstacle_course", 10, &ObstacleCourseTask::obstacleCourseCallback, this);
  m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &ObstacleCourseTask::moduleEnableCallback, this);
  m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motor_Control", 10);
  m_taskCompletePublisher = m_nodeHandle.advertise<std_msgs::String>("Task_Completion", 10);
  m_taskDownObstacleDetectorSubscriber = m_nodeHandle.subscribe("img_rec/obstacle_course_downward", 10, &ObstacleCourseTask::downwardObstacleCallback, this);
}

ObstacleCourseTask::~ObstacleCourseTask()
{

}

void ObstacleCourseTask::run(void)
{
  while (true)
  {
    //do calculations
    m_distanceToObject = getDistance(m_hbar.height, 2.0f);
    float moveX = m_hbar.center_x;
    float moveY = m_hbar.center_y + getPixelsPerInch(m_hbar.height, 2.0f) * 24; // aim for spot 2 feet above center of bar

    if (moveX <= 330 && moveX >= 310 && moveY <= 250 && moveY >= 230)
    {
      //we have centered, drive forward
      printf("Correcting forward by: %f\n", m_distanceToObject);
      publishMotor("Forward", "Offset", m_distanceToObject);
    }
    else if (moveX > 330 || moveX < 310)
    {
      printf("Correcting straf by: %f\n", moveX);
      publishMotor("Straf", "Offset", moveX);
    }
    else if (moveY > 250 || moveY < 230)
    {
      printf("Correcting depth by: %f\n", moveY);
      publishMotor("Depth", "Offset", moveY);
    }

    ros::spinOnce();
  }
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

void ObstacleCourseTask::downwardObstacleCallback(const SubImageRecognition::ImgRecObject& msg)
{
  //send last forward command
  publishMotor("Forward", "Offset", 4.0f);
}

void ObstacleCourseTask::obstacleCourseCallback(const SubImageRecognition::ImgRecObject& msg)
{
  if (m_isEnabled)
  {
    //identify the object
    if (msg.width > msg.height) //horizontal bar
    {
      m_hbar = msg;
    }
    else //vertical Bar
    {
      //determine which bar
      if (msg.center_x > m_hbar.center_x) //right
      {
        m_vbarRight = msg;
      }
      else //left
      {
        m_vbarLeft = msg;
      }
    }

//    //we will receive multiple frames (3) for the 1 object. 2 vertical bars and 1 horizontal
//    m_distanceToObject = computeDistanceToObject(msg.height, msg.width);
//    float moveX = msg.center_x - 320;
//    float moveY = msg.center_y - 240;
//
//    if (moveX <= 330 && moveX >= 310 && moveY <= 250 && moveY >= 230)
//    {
//      //we have centered, drive forward
//      publishMotor("Forward", "Offset", m_distanceToObject);
//      reportSuccess(true);
//    }
//    else if (moveX > 330 || moveX < 310)
//    {
//      printf("Correcting straf by: %f\n", moveX);
//      publishMotor("Straf", "Offset", moveX);
//    }
//    else if (moveY > 250 || moveY < 230)
//    {
//      printf("Correcting depth by: %f\n", moveY);
//      publishMotor("Depth", "Offset", moveY);
//    }
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
float ObstacleCourseTask::calculateDistanceFromCenter(float centerDir, float width)
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
float ObstacleCourseTask::getPixelsPerInch(float curWidthPixels, float expectedWidthInches)
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
float ObstacleCourseTask::getDistance(float curObjSize, float actualObjSize)
{
  return (actualObjSize/(2.0f*tan((curObjSize * 60.0f)/960.0f)));
}

/**
 * @brief report the success or failure of the module
 *
 * @param success The success or failure
 */
void ObstacleCourseTask::reportSuccess(bool success)
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
void ObstacleCourseTask::publishMotor(std::string direction, std::string motion, float value)
{
  Robosub::HighLevelControl msg;
  msg.Direction = direction;
  msg.MotionType = motion;
  msg.Value = value;

  m_highLevelMotorPublisher.publish(msg);
}
