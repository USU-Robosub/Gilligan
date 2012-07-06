#include "BuoysTask.hpp"
#include "Robosub/HighLevelControl.h"
#include "Robosub/Point.h"

BuoysTask::BuoysTask(BuoyColors first, BuoyColors second)
    : m_nodeHandle(),
      m_greenBuoySubcriber(),
      m_yellowBuoySubcriber(),
      m_redBuoySubcriber(),
      m_taskStateSubscriber(),
      m_highLevelMotorPublisher(),
      m_centerOnPointPublisher(),
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

   // Setup publisher for center on point
    m_centerOnPointPublisher = m_nodeHandle.advertise<Robosub::Point>("Center_On_Point", 10);

   // Setup publisher for high level motor control
    m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motor_Control", 10);

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
            printf("BuoysTask: Enabled\n");
            m_isEnabled = true;
            performTask();
        }
        else
        {
            printf("BuoysTask: Disabled\n");
            m_isEnabled = false;
        }
    }
}

void BuoysTask::greenBuoyCallback(const SubImageRecognition::ImgRecObject& msg)
{
    if (m_isEnabled)
    {
        struct BuoyData data;

        data.centerX = msg.center_x;
        data.centerY = msg.center_y;
        data.sampleTime = msg.stamp;
        data.height = msg.height;
        data.width = msg.width;
        data.confidence = msg.confidence;

        m_greenBuoySamples.push_back(data);

        if (m_greenBuoySamples.size() > 50)
        {
            m_greenBuoySamples.pop_front();
        }
    }
}

void BuoysTask::yellowBuoyCallback(const SubImageRecognition::ImgRecObject& msg)
{
    if (m_isEnabled)
    {
        struct BuoyData data;

        data.centerX = msg.center_x;
        data.centerY = msg.center_y;
        data.sampleTime = msg.stamp;
        data.height = msg.height;
        data.width = msg.width;
        data.confidence = msg.confidence;

        m_yellowBuoySamples.push_back(data);

        if (m_yellowBuoySamples.size() > 50)
        {
            m_yellowBuoySamples.pop_front();
        }
    }
}

void BuoysTask::redBuoyCallback(const SubImageRecognition::ImgRecObject& msg)
{
    if (m_isEnabled)
    {
        printf("BuoysTask: Got red buoy msg x: %i y: %i height: %i width: %i\n", msg.center_x, msg.center_y, msg.height, msg.width);

        struct BuoyData data;

        data.centerX = msg.center_x;
        data.centerY = msg.center_y;
        data.sampleTime = msg.stamp;
        data.height = msg.height;
        data.width = msg.width;
        data.confidence = msg.confidence;

        m_redBuoySamples.push_back(data);

        if (m_redBuoySamples.size() > 50)
        {
            m_redBuoySamples.pop_front();
        }
    }
}

void BuoysTask::performTask(void)
{
    Robosub::HighLevelControl highLevelControlMsg;
    int retries = 0;


    printf("BuoysTask: Driving Forward to find buoys...\n");

    // 1. find bouys - keep driving forward
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Manual";
    highLevelControlMsg.Value = 75.0f;

    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // Get at least 10 samples of the red buoy before considering centering on buoy
    retries = 150;  // 15 seconds-ish
    while ((m_redBuoySamples.size() <= 30) && (retries > 0))
    {
        usleep(100000);
        retries--;
    }

    // @todo Determine distance to red buoy, make sure we are sufficiently close instead of just using number of red samples

    // 2. Identify first buoy to bump and center on point
    printf("BuoysTask: Centering on point to bump first buoy\n");
    centerOnBuoy(m_firstToBump);

    // 3. Bump buoy by driving forward
    printf("BuoysTask: Driving Forward to bump first buoy\n");
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = 10.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    sleep(10);

    // 4. Reverse back to sufficient distance to identify next buoy
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = -15.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    sleep(10);

    // 5. Identify next buoy to bump
    centerOnBuoy(m_secondToBump);

    // 6. Bump bouy
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = 10.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    sleep(10);

    // 7. Reverse
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = -5.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    sleep(5);

    // 8. Rise above buoys
    highLevelControlMsg.Direction = "Depth";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = -3.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    sleep(5);

    // 9. Move past buoys safely
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = 10.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // 10. Dive back to nominal depth
    highLevelControlMsg.Direction = "Depth";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = 3.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // 11. Signal done on topic Task_Complete

}

bool BuoysTask::centerOnBuoy(BuoyColors color)
{
    Robosub::Point pointMsg;
    bool isCentered = false;

    while (!isCentered)
    {
        bool shouldSendCenterMsg = false;
        switch (color)
        {
            case RED:
                if (m_redBuoySamples.size() > 10)
                {
                    pointMsg.x = m_redBuoySamples.back().centerX;
                    pointMsg.y = m_redBuoySamples.back().centerY;
                    shouldSendCenterMsg = true;
                }
                break;

            case GREEN:
                if (m_greenBuoySamples.size () > 10)
                {
                    pointMsg.x = m_greenBuoySamples.back().centerX;
                    pointMsg.y = m_greenBuoySamples.back().centerY;
                    shouldSendCenterMsg = true;
                }
                break;

            case YELLOW:
                if (m_yellowBuoySamples.size() > 10)
                {
                    pointMsg.x = m_yellowBuoySamples.back().centerX;
                    pointMsg.y = m_yellowBuoySamples.back().centerY;
                    shouldSendCenterMsg = true;
                }
                break;
        }

        if (shouldSendCenterMsg)
        {
            m_centerOnPointPublisher.publish(pointMsg);

            if ((pointMsg.x <= 10) && (pointMsg.x >= -10) && (pointMsg.y <= 10) && (pointMsg.y >= -10))
            {
                isCentered = true;
            }
        }

        usleep(100000);
    }

    return isCentered;
}



