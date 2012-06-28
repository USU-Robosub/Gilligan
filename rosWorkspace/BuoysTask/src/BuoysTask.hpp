#ifndef _BUOYS_TASK_HPP
#define _BUOYS_TASK_HPP

#include "ros/ros.h"

#include "std_msgs/UInt8.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"

enum BuoyColors
{
    GREEN = 0,
    YELLOW = 1,
    RED = 2
};

struct BuoyData
{
    ros::Time lastSampleTime;
    short centerX;
    short centerY;
    short height;
    short width;
    short confidence;
};

class BuoysTask
{
public:
    BuoysTask(BuoyColors first, BuoyColors second);
    ~BuoysTask();

    void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
    void greenBuoyCallback(const SubImageRecognition::ImgRecObject& msg);
    void yellowBuoyCallback(const SubImageRecognition::ImgRecObject& msg);
    void redBuoyCallback(const SubImageRecognition::ImgRecObject& msg);

private:
    void performTask(void);

    BuoyData m_buoys[3];
    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_greenBuoySubcriber;
    ros::Subscriber m_yellowBuoySubcriber;
    ros::Subscriber m_redBuoySubcriber;
    ros::Subscriber m_taskStateSubscriber;
    ros::Publisher m_highLevelMotorPublisher;
    BuoyColors m_firstToBump;
    BuoyColors m_secondToBump;
    bool m_isEnabled;

};

#endif // _BUOYS_TASK_HPP
