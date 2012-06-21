#include "BuoysTask.hpp"
#include "Robosub/HighLevelControl.h"

BuoysTask::BuoysTask(BuoyColors first, BuoyColors second)
    : m_nodeHandle(),
      m_greenBuoySubcriber(),
      m_yellowBuoySubcriber(),
      m_redBuoySubcriber(),
      m_taskStateSubscriber(),
      m_highLevelMotorPublisher(),
      m_firstToBump(first),
      m_secondToBump(second),
      m_isEnabled(false)
{
    // Subscribe to image recognition of green buoy
    m_greenBuoySubcriber = m_nodeHandle.subscribe("image_recognition/buoys/green", 10, &BuoysTask::greenBuoyCallback, this);

    // Subscribe to image recognition of red buoy
    m_yellowBuoySubcriber = m_nodeHandle.subscribe("image_recognition/buoys/yellow", 10, &BuoysTask::yellowBuoyCallback, this);

    // Subscribe to image recognition of red buoy
    m_redBuoySubcriber = m_nodeHandle.subscribe("image_recognition/buoys/red", 10, &BuoysTask::redBuoyCallback, this);

    // Subscribe to task state message
    m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &BuoysTask::moduleEnableCallback, this);

   // @todo Setup publisher for center on point

   // @todo Setup publisher for high level motor control
    m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motor_Control", 100);

   // @todo Setup publisher for center on line
}

BuoysTask::~BuoysTask()
{

}

void BuoysTask::moduleEnableCallback(const Robosub::ModuleEnableMsg& msg)
{
    if(msg.Module == "BuoysTask")
    {
        if(msg.State == true)
        {
            m_isEnabled = true;
            performTask();
        }
        else
        {
            m_isEnabled = false;
        }
    }
}

void BuoysTask::greenBuoyCallback(const SubImageRecognition::ImgRecObject& msg)
{
    if (m_isEnabled)
    {
        m_buoys[0].centerX = msg.center_x;
        m_buoys[0].centerY = msg.center_y;
        m_buoys[0].lastSampleTime = msg.stamp;
        m_buoys[0].height = msg.height;
        m_buoys[0].width = msg.width;
        m_buoys[0].confidence = msg.confidence;
    }
}

void BuoysTask::yellowBuoyCallback(const SubImageRecognition::ImgRecObject& msg)
{
    if (m_isEnabled)
    {
        m_buoys[1].centerX = msg.center_x;
        m_buoys[1].centerY = msg.center_y;
        m_buoys[1].lastSampleTime = msg.stamp;
        m_buoys[1].height = msg.height;
        m_buoys[1].width = msg.width;
        m_buoys[1].confidence = msg.confidence;
    }
}

void BuoysTask::redBuoyCallback(const SubImageRecognition::ImgRecObject& msg)
{
    if (m_isEnabled)
    {
        m_buoys[2].centerX = msg.center_x;
        m_buoys[2].centerY = msg.center_y;
        m_buoys[2].lastSampleTime = msg.stamp;
        m_buoys[2].height = msg.height;
        m_buoys[2].width = msg.width;
        m_buoys[2].confidence = msg.confidence;
    }
}

void BuoysTask::performTask(void)
{
    Robosub::HighLevelControl highLevelControlMsg;

    // Initialize buoy values
    for(int i = 0; i < 3; i++)
    {
        m_buoys[i].centerX = 0;
        m_buoys[i].centerY = 0;
        m_buoys[i].lastSampleTime.sec = 0;
        m_buoys[i].lastSampleTime.nsec = 0;
        m_buoys[i].height = 0;
        m_buoys[i].width = 0;
        m_buoys[i].confidence = 0.0f;

    }

//    1. find bouys - keep driving forward
    m_highLevelMotorPublisher.

//    2. Identify first buoy to bump - look at buoy array to determine which to use
//    3. bump buoy - center on point and drive forward
//    4. Reverse - drive backward
//    5. Identify next buoy to bump
//    6. Bump bouy
//    7. Reverse and move above buoys
//    8. move past buoys safely
//    9. Find next path behind buoys
//    10. signal done on topic: Task_Complete
}



