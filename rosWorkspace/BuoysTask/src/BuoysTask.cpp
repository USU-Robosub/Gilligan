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
    if (msg.Module == "BuoysTask")
    {
        if (msg.State == true)
        {
            bool status = false;
            printf("BuoysTask: Enabled\n");
            m_isEnabled = true;

            status = performTask();

            // @todo Signal task complete on topic with task finish status
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
        printf("BuoysTask: Got green buoy msg x: %i y: %i height: %i width: %i\n", msg.center_x, msg.center_y, msg.height, msg.width);

        //@todo Throw out message below some confidence level?

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
        printf("BuoysTask: Got yellow buoy msg x: %i y: %i height: %i width: %i\n", msg.center_x, msg.center_y, msg.height, msg.width);

        //@todo Throw out message below some confidence level?

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

        //@todo Throw out message below some confidence level?

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

bool BuoysTask::performTask(void)
{
    Robosub::HighLevelControl highLevelControlMsg;
    int retries = 0;

    printf("BuoysTask: Driving Forward to find buoys...\n");

    // 1. Find the bouys - keep driving forward
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Manual";
    highLevelControlMsg.Value = 75.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // Get at least 10 samples of the red buoy (easiest to spot) before looking for desired buoy
    retries = 150;  // 15 seconds-ish timeout
    while ((m_redBuoySamples.size() <= 10) && (retries > 0))
    {
        usleep(100000);
        retries--;
    }

    if (retries == 0)
    {
        printf("BuoyTask: Could not identiy red buoy while trying to initially find buoys\n");
        return false;
    }

    // @todo Determine distance to red buoy, make sure we are sufficiently close to identify the other buoys

    printf("BuoysTask: Red buoy identified, attempting to locate and center on first buoy to bump\n");

    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Manual";
    highLevelControlMsg.Value = 50.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    // 2. Identify first buoy to bump and center on point
    printf("BuoysTask: Centering on point to bump first buoy\n");
    if (centerOnBuoy(m_firstToBump))
    {
        // Allow a little time to stabilize on point
        sleep(3);

        // 3. Bump buoy by driving forward and continuing to center on point
        printf("BuoysTask: Driving Forward to bump first buoy\n");
        bumpBuoy(m_firstToBump);

        // 4. Reverse back to sufficient distance to identify next buoy
        highLevelControlMsg.Direction = "Forward";
        highLevelControlMsg.MotionType = "Offset";
        highLevelControlMsg.Value = -15.0f;
        m_highLevelMotorPublisher.publish(highLevelControlMsg);
        printf("BuoysTask: Backing off to reposition for next buoy bump\n");
        sleep(10);

        // 5. Identify next buoy to bump
        printf("BuoysTask: Centering on point to bump second buoy\n");
        if (centerOnBuoy(m_secondToBump))
        {
            // Allow a little time to stabilize on point
            sleep(3);

            // 6. Bump bouy
            printf("BuoysTask: Driving Forward to bump second buoy\n");
            bumpBuoy(m_secondToBump);

            // 7. Reverse
            printf("BuoysTask: Reversing to get clear of buoys\n");
            highLevelControlMsg.Direction = "Forward";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = -7.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);

            sleep(5);

            // 8. Rise above buoys
            printf("BuoysTask: Rising above buoys\n");
            highLevelControlMsg.Direction = "Depth";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = -3.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);

            sleep(5);

            // 9. Move past buoys safely
            printf("BuoysTask: Moving past buoys\n");
            highLevelControlMsg.Direction = "Forward";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = 10.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);

            sleep(5);

            // 10. Dive back to nominal depth
            printf("BuoysTask: Returning to previous depth\n");
            highLevelControlMsg.Direction = "Depth";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = 3.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);

            sleep(5);

            return true;
        }
        else
        {
            printf("BuoysTask: Failed to center on second buoy\n");
            return false;
        }
    }
    else
    {
        printf("BuoysTask: Failed to center on first buoy\n");
        return false;
    }
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
                if (m_redBuoySamples.size() > 5)
                {
                    pointMsg.x = m_redBuoySamples.back().centerX;
                    pointMsg.y = m_redBuoySamples.back().centerY;
                    shouldSendCenterMsg = true;
                }
                break;

            case GREEN:
                if (m_greenBuoySamples.size () > 5)
                {
                    pointMsg.x = m_greenBuoySamples.back().centerX;
                    pointMsg.y = m_greenBuoySamples.back().centerY;
                    shouldSendCenterMsg = true;
                }
                break;

            case YELLOW:
                if (m_yellowBuoySamples.size() > 5)
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

            if ((pointMsg.x <= 20) && (pointMsg.x >= -20) && (pointMsg.y <= 20) && (pointMsg.y >= -20))
            {
                isCentered = true;
            }
        }

        usleep(100000);
    }

    m_redBuoySamples.clear();
    m_greenBuoySamples.clear();
    m_yellowBuoySamples.clear();

    return isCentered;
}

void BuoysTask::bumpBuoy(BuoyColors color)
{
    Robosub::HighLevelControl highLevelControlMsg;
    bool lostSight = false;
    int retries = 25;

    while ((!lostSight) && (retries > 0))
    {
        // Drive forward for a bit at half thrust
        highLevelControlMsg.Direction = "Forward";
        highLevelControlMsg.MotionType = "Manual";
        highLevelControlMsg.Value = 50.0f;

        m_highLevelMotorPublisher.publish(highLevelControlMsg);

        sleep(1);

        // Try to re-center on buoy while driving forward
        if(!centerOnBuoy(color))
        {
            // Lost sight of buoy, drive another 5 ft forward and hope for a collision
            highLevelControlMsg.Direction = "Forward";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = 5.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);
            sleep(5);

            // Assume buoy has been bumped
            return;
        }

        retries--;
    }
}


