#ifndef _TORPEDO_TASK_HPP
#define _TORPEDO_TASK_HPP

#include <list>
#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"

enum TorpedoColors
{
    RED = 0,
    BLUE = 1
};

class TorpedoTask
{
public:
    TorpedoTask();
    ~TorpedoTask();

    void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
    void redBuoyCallback(const SubImageRecognition::ImgRecObject& msg);
    bool centerOnRedBuoy();
    void fireTorpedo(TorpedoColors toShoot);

private:
    bool performTask(void);

    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_redBuoySubcriber;
    SubImageRecognition::ImgRecObject m_redBuoyLocation;
    ros::Subscriber m_taskStateSubscriber;
    ros::Publisher m_highLevelMotorPublisher;
    ros::Publisher m_centerOnPointPublisher;
    ros::Publisher m_torpedoLaunchPublisher;
    bool m_isEnabled;
};

#endif // _TORPEDO_TASK_HPP
