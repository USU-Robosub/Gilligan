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
    int m_mode;


    void reportSuccess(bool success);
    void publishMotor(std::string direction, std::string motion, float value);
};

#endif // _PATH_TASK_HPP
