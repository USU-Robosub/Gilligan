#ifndef _PATH_TASK_HPP
#define _PATH_TASK_HPP

//#define USE_MANUAL 1

#include <ros/ros.h>
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"
#include "std_msgs/String.h"

#include <string.h>
#include <stdint.h>
#include <list>

class PathTask
{
  public:
    PathTask();
    ~PathTask();

    void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
    void pathCallback(const SubImageRecognition::ImgRecObject& msg);
    void pathDirectionCallback(const std_msgs::String& msg);

  private:
    void performTask(void);

    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_pathSubscriber;
    ros::Subscriber m_taskStateSubscriber;
    ros::Subscriber m_pathDirSubscriber;
    ros::Publisher m_highLevelMotorPublisher;
    ros::Publisher m_taskCompletePublisher;
    bool m_isEnabled;
    unsigned int m_frames;
    std::string m_direction;
    std::list<SubImageRecognition::ImgRecObject> m_msgList;
    int m_lastId;


    void manualMode(const SubImageRecognition::ImgRecObject& msg);
    void offsetMode(const SubImageRecognition::ImgRecObject& msg);
    void reportSuccess(bool success);
    void publishMotor(std::string direction, std::string motion, float value);
    float calculateDistanceFromCenter(float centerDir, float width);
    float getPixelsPerInch(float curWidthPixels, float expectedWidthInches);
    float getDistance(float curObjSize, float actualObjSize);
    SubImageRecognition::ImgRecObject getMessage();
};

#endif // _PATH_TASK_HPP
