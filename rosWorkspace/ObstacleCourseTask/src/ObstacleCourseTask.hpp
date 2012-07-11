#ifndef _OBSTACLE_COURSE_TASK_HPP
#define _OBSTACLE_COURSE_TASK_HPP

#include "ros/ros.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"

#include <stdio.h>
#include <stdlib.h>

class ObstacleCourseTask
{
  public:
    ObstacleCourseTask();
    ~ObstacleCourseTask();

    void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
    void obstacleCourseCallback(const SubImageRecognition::ImgRecObject& msg);

  private:
    void performTask(void);

    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_obstacleCourseSubscriber;
    ros::Subscriber m_taskStateSubscriber;
    ros::Publisher m_highLevelMotorPublisher;
    ros::Publisher m_taskCompletePublisher;
    bool m_isEnabled;
    bool m_foundFirst;
    float m_distanceToObject;

    float computeDistanceToObject(float height, float width);
    void reportSuccess(bool success);
};

#endif // _OBSTACLE_COURSE_TASK_HPP
