#ifndef _OBSTACLE_COURSE_TASK_HPP
#define _OBSTACLE_COURSE_TASK_HPP

#include "ros/ros.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

class ObstacleCourseTask
{
  public:
    ObstacleCourseTask();
    ~ObstacleCourseTask();

    void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
    void obstacleCourseCallback(const SubImageRecognition::ImgRecObject& msg);
    void downwardObstacleCallback(const SubImageRecognition::ImgRecObject& msg);
    void run(void);

  private:
    void performTask(void);

    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_obstacleCourseSubscriber;
    ros::Subscriber m_taskStateSubscriber;
    ros::Publisher m_highLevelMotorPublisher;
    ros::Publisher m_taskCompletePublisher;
    ros::Subscriber m_taskDownObstacleDetectorSubscriber;
    bool m_isEnabled;
    bool m_foundFirst;
    float m_distanceToObject;
    SubImageRecognition::ImgRecObject m_hbar;
    SubImageRecognition::ImgRecObject m_vbarLeft;
    SubImageRecognition::ImgRecObject m_vbarRight;
    int16_t m_targetCenter_x;
    int16_t m_targetCenter_y;
    bool m_centerSet;


    float getPixelsPerInch(float curWidthPixels, float expectedWidthInches);
    float calculateDistanceFromCenter(float centerDir, float width);
    float getDistance(float curObjSize, float actualObjSize);
    void reportSuccess(bool success);
    void publishMotor(std::string direction, std::string motion, float value);
};

#endif // _OBSTACLE_COURSE_TASK_HPP
