#ifndef _PATH_TASK_HPP
#define _PATH_TASK_HPP

#include <ros/ros.h>
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"

#include <string.h>
#include <stdint.h>

class PathTask
{
  public:
    PathTask();
    ~PathTask();

    void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
    void pathCallback(const SubImageRecognition::ImgRecObject& msg);

  private:
    void performTask(void);

    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_pathSubscriber;
    ros::Subscriber m_taskStateSubscriber;
    ros::Publisher m_highLevelMotorPublisher;
    ros::Publisher m_taskCompletePublisher;
    bool m_isEnabled;
    unsigned int m_frames;


    void reportSuccess(bool success);
    void publishMotor(std::string direction, std::string motion, float value);
    float calculateDistanceFromCenter(float centerDir, float width);
    float getPixelsPerInch(float curWidthPixels, float expectedWidthInches);
    float getDistance(float curObjSize, float actualObjSize);
};

#endif // _PATH_TASK_HPP
