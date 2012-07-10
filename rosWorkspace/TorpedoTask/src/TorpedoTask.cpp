#include "TorpedoTask.hpp"
#include "Robosub/HighLevelControl.h"
#include "Robosub/Point.h"
#include "std_msgs/UInt8.h"

TorpedoTask::TorpedoTask()
    : m_nodeHandle(),
      m_redBuoySubcriber(),
      m_redBuoyLocation(),
      m_taskStateSubscriber(),
      m_highLevelMotorPublisher(),
      m_centerOnPointPublisher(),
      m_isEnabled(false)
{
    m_redBuoyLocation.height = 0;
    m_redBuoyLocation.width = 0;
    m_redBuoyLocation.center_x = 0;
    m_redBuoyLocation.center_y = 0;
    m_redBuoyLocation.confidence = 0;

    // Subscribe to image recognition of red buoy
    m_redBuoySubcriber = m_nodeHandle.subscribe("image_recognition/buoys/red", 10, &TorpedoTask::redBuoyCallback, this);

    // Subscribe to task state message
    m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &TorpedoTask::moduleEnableCallback, this);

   // Setup publisher for center on point
    m_centerOnPointPublisher = m_nodeHandle.advertise<Robosub::Point>("Center_On_Point", 10);

   // Setup publisher for high level motor control
    m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motor_Control", 10);

    // Setup publisher to fire torpedoes
    m_torpedoLaunchPublisher = m_nodeHandle.advertise<std_msgs::UInt8>("Torpedo_Launch", 10);

   // @todo Setup publisher for center on line
}

TorpedoTask::~TorpedoTask()
{

}

void TorpedoTask::moduleEnableCallback(const Robosub::ModuleEnableMsg& msg)
{
    if (msg.Module == "TorpedoTask")
    {
        if (msg.State == true)
        {
            bool status = false;
            printf("TorpedoTask: Enabled\n");
            m_isEnabled = true;

            status = performTask();

            // @todo Signal task complete on topic with task finish status
        }
        else
        {
            printf("TorpedoTask: Disabled\n");
            m_isEnabled = false;
        }
    }
}

void TorpedoTask::redBuoyCallback(const SubImageRecognition::ImgRecObject& msg)
{
    if (m_isEnabled)
    {
        printf("TorpedoTask: Got red buoy msg x: %i y: %i height: %i width: %i\n", msg.center_x, msg.center_y, msg.height, msg.width);

        //@todo Throw out message below some confidence level?

        m_redBuoyLocation.center_x = msg.center_x;
        m_redBuoyLocation.center_y = msg.center_y;
        m_redBuoyLocation.stamp = msg.stamp;
        m_redBuoyLocation.height = msg.height;
        m_redBuoyLocation.width = msg.width;
        m_redBuoyLocation.confidence = msg.confidence;
    }
}

bool TorpedoTask::performTask(void)
{
    Robosub::HighLevelControl highLevelControlMsg;
    int retries = 0;

    printf("TorpedoTask: Driving Forward to find red buoy...\n");

    // 1. Find the red bouy - keep driving forward
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Manual";
    highLevelControlMsg.Value = 75.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // Get at least 10 samples of the red buoy (easiest to spot) before looking for desired buoy
    retries = 150;  // 15 seconds-ish timeout
    while ((m_redBuoyLocation.height == 0) && (retries > 0))
    {
        usleep(100000);
        retries--;
    }

    if (retries == 0)
    {
        printf("TorpedoTask: Could not identiy red buoy\n");
        return false;
    }

    printf("TorpedoTask: Red buoy identified, attempting to center on buoy\n");
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // Get close to red buoy
    retries = 150;  // 15 seconds-ish timeout
    while ((m_redBuoyLocation.height < 75) && (retries > 0))
    {
        centerOnRedBuoy();
        usleep(100000);
        retries--;
    }

    // Stop moving
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Manual";
    highLevelControlMsg.Value = 0.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // Re-center on buoy
    centerOnRedBuoy();
    centerOnRedBuoy();
    centerOnRedBuoy();
    centerOnRedBuoy();
    centerOnRedBuoy();

    // Fire red torpedo
    fireTorpedo(RED);

    // Re-center on buoy
    centerOnRedBuoy();
    centerOnRedBuoy();
    centerOnRedBuoy();
    centerOnRedBuoy();
    centerOnRedBuoy();

    // Fire blue torpedo
    fireTorpedo(BLUE);

    return true;
}

bool TorpedoTask::centerOnRedBuoy(void)
{
    Robosub::Point pointMsg;
    bool isCentered = false;

    while (!isCentered)
    {
        pointMsg.x = m_redBuoyLocation.center_x;
        pointMsg.y = m_redBuoyLocation.center_y;

        m_centerOnPointPublisher.publish(pointMsg);

        if ((pointMsg.x <= 20) && (pointMsg.x >= -20) && (pointMsg.y <= 20) && (pointMsg.y >= -20))
        {
            isCentered = true;
        }

        usleep(100000);
    }

    return isCentered;
}

void TorpedoTask::fireTorpedo(TorpedoColors toShoot)
{
    std_msgs::UInt8 torpedoMessage;
    torpedoMessage.data = toShoot;

    m_torpedoLaunchPublisher.publish(torpedoMessage);
}


