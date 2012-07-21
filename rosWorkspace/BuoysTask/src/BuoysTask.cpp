#include "math.h"
#include "BuoysTask.hpp"
#include "Robosub/HighLevelControl.h"
#include "Robosub/Point.h"
#include "std_msgs/String.h"

BuoysTask::BuoysTask(BuoyColors first, BuoyColors second)
    : m_nodeHandle(),
      m_greenBuoySubcriber(),
      m_yellowBuoySubcriber(),
      m_redBuoySubcriber(),
      m_taskStateSubscriber(),
      m_highLevelMotorPublisher(),
      m_centerOnPointPublisher(),
      m_taskCompletePublisher(),
      m_firstToBump(first),
      m_secondToBump(second),
      m_pFirstBumpList(NULL),
      m_pSecondBumpList(NULL),
      m_isEnabled(false)
{
    // Subscribe to image recognition of green buoy
    m_greenBuoySubcriber = m_nodeHandle.subscribe("img_rec/buoys/green", 10, &BuoysTask::greenBuoyCallback, this);

    // Subscribe to image recognition of red buoy
    m_yellowBuoySubcriber = m_nodeHandle.subscribe("img_rec/buoys/yellow", 10, &BuoysTask::yellowBuoyCallback, this);

    // Subscribe to image recognition of red buoy
    m_redBuoySubcriber = m_nodeHandle.subscribe("img_rec/buoys/red", 10, &BuoysTask::redBuoyCallback, this);

    // Subscribe to task state message
    m_taskStateSubscriber = m_nodeHandle.subscribe("Module_Enable", 10, &BuoysTask::moduleEnableCallback, this);

   // Setup publisher for center on point
    m_centerOnPointPublisher = m_nodeHandle.advertise<Robosub::Point>("Center_On_Point", 10);

    // Setup publisher for high level motor control
    m_highLevelMotorPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motor_Control", 10);

    m_taskCompletePublisher = m_nodeHandle.advertise<std_msgs::String>("Task_Complete", 10);

    switch(m_firstToBump)
    {
        case RED:
            m_pFirstBumpList = &m_redBuoySamples;
            break;
        case GREEN:
            m_pFirstBumpList = &m_greenBuoySamples;
            break;
        case YELLOW:
            m_pFirstBumpList = &m_yellowBuoySamples;
            break;
    }

    switch(m_secondToBump)
    {
        case RED:
            m_pSecondBumpList = &m_redBuoySamples;
            break;
        case GREEN:
            m_pSecondBumpList = &m_greenBuoySamples;
            break;
        case YELLOW:
            m_pSecondBumpList = &m_yellowBuoySamples;
            break;
    }
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

            // Perform task
            status = performTask();

            // Signal task complete on topic with task finish status
            std_msgs::String taskCompleteMsg;

            if (status)
            {
                taskCompleteMsg.data = "BuoysTask Success";
            }
            else
            {
                taskCompleteMsg.data = "BuoysTask Failure";
            }
            m_taskCompletePublisher.publish(taskCompleteMsg);
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
    bool isCentered = false;

    printf("BuoysTask: Driving Forward to find buoys...\n");
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = 2.0f;

    // 1. Find the red bouy - keep driving forward - Get at least 10 samples of buoy before deciding it has been seen
    retries = 200;  // 20 seconds-ish timeout
    while ((m_redBuoySamples.size() <= 10) && (retries > 0))
    {
        m_highLevelMotorPublisher.publish(highLevelControlMsg);

        ros::spinOnce();
        usleep(100000);
        retries--;
    }

    if (retries == 0)
    {
        printf("BuoyTask: Failed to identiy any buoys while trying to initially locate them/n");
        return false;
    }

    printf("BuoysTask: Centering on red buoy to make sure we are heading the right direction\n");

    // Move closer to buoys while centering on the red buoy
    float distanceToRed = getDistanceToBuoy(RED);
    retries = 40;
    while ((retries > 0) && ((distanceToRed > 7.0f) || (distanceToRed == -1.0f)))
    {
        centerOnBuoy(RED);
        m_highLevelMotorPublisher.publish(highLevelControlMsg);

        ros::spinOnce();
        usleep(100000);
        retries--;

        distanceToRed = getDistanceToBuoy(RED);
    }

    // Stop moving forward
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = 0.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    printf("BuoysTask: Hopefully at good distance, attempting to locate and center on first buoy to bump\n");
    // Let a second or so of messages come in
    retries = 10;
    while (retries > 0)
    {
        ros::spinOnce();
        usleep(100000);
        retries--;
    }

    if (!isBuoyVisible(m_firstToBump))
    {
        printf("BuoysTask: First buoy is not visible, looking around a bit...\n");
        searchBuoys(m_firstToBump);
    }

    // 2. Identify first buoy to bump and center on point
    printf("BuoysTask: Centering on point to bump first buoy\n");
    retries = 20;
    while (!isCentered && (retries > 0))
    {
        isCentered = centerOnBuoy(m_firstToBump);
        usleep(100000);
        retries--;
    }

    if (isCentered)
    {
        // 3. Bump buoy by driving forward and continuing to center on point
        printf("BuoysTask: Driving Forward to bump first buoy\n");
        bumpBuoy(m_firstToBump);

        clearPastSamples();

        // 4. Reverse back to sufficient distance to identify next buoy
        highLevelControlMsg.Direction = "Forward";
        highLevelControlMsg.MotionType = "Offset";
        highLevelControlMsg.Value = -2.0f;
        printf("BuoysTask: Backing off to reposition for next buoy bump\n");

        for (int loopDelayCount = 75; loopDelayCount > 0; loopDelayCount--)
        {
            m_highLevelMotorPublisher.publish(highLevelControlMsg);

            usleep(100000);
            ros::spinOnce();
        }

        // 5. Identify next buoy to bump
        // Verify that the next buoy to bump is in view
        if (!isBuoyVisible(m_secondToBump))
        {
            // Rotate a bit each direction and try to see buoy
            printf("BuoysTask: Second buoy is not visible, looking around a bit...\n");
            searchBuoys(m_secondToBump);
        }

        printf("BuoysTask: Centering on point to bump second buoy\n");
        isCentered = false;
        retries = 20;
        while (!isCentered && (retries > 0))
        {
            isCentered = centerOnBuoy(m_secondToBump);
            usleep(100000);
            retries--;
        }

        if (isCentered)
        {
            // 6. Bump bouy
            printf("BuoysTask: Driving Forward to bump second buoy\n");
            bumpBuoy(m_secondToBump);

            clearPastSamples();

            // 7. Reverse
            printf("BuoysTask: Reversing to get clear of buoys\n");
            highLevelControlMsg.Direction = "Forward";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = -7.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);
            ros::spinOnce();

            sleep(5);

            // 8. Rise above buoys
            printf("BuoysTask: Rising above buoys\n");
            highLevelControlMsg.Direction = "Depth";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = -2.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);
            ros::spinOnce();

            sleep(5);

            // 9. Move past buoys safely
            printf("BuoysTask: Moving past buoys\n");
            highLevelControlMsg.Direction = "Forward";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = 7.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);
            ros::spinOnce();

            sleep(5);

            // 10. Dive back to nominal depth
            printf("BuoysTask: Returning to previous depth\n");
            highLevelControlMsg.Direction = "Depth";
            highLevelControlMsg.MotionType = "Offset";
            highLevelControlMsg.Value = 2.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);
            ros::spinOnce();

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
    int retries = 50;

    while (!isCentered && (retries > 0))
    {
        bool shouldSendCenterMsg = false;
        switch (color)
        {
            case RED:
                if (m_redBuoySamples.size() > 2)
                {
                    pointMsg.x = m_redBuoySamples.back().centerX;
                    pointMsg.y = m_redBuoySamples.back().centerY;
                    shouldSendCenterMsg = true;
                }
                break;

            case GREEN:
                if (m_greenBuoySamples.size () > 2)
                {
                    pointMsg.x = m_greenBuoySamples.back().centerX;
                    pointMsg.y = m_greenBuoySamples.back().centerY;
                    shouldSendCenterMsg = true;
                }
                break;

            case YELLOW:
                if (m_yellowBuoySamples.size() > 2)
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
                pointMsg.x = 0;
                pointMsg.y = 0;
                m_centerOnPointPublisher.publish(pointMsg);

                isCentered = true;
            }
        }

        ros::spinOnce();
        usleep(100000);
        retries--;
    }

    clearPastSamples();

    return isCentered;
}

void BuoysTask::bumpBuoy(BuoyColors color)
{
    Robosub::HighLevelControl highLevelControlMsg;
    highLevelControlMsg.Direction = "Forward";
    highLevelControlMsg.MotionType = "Offset";
    highLevelControlMsg.Value = 2.0f;

    for (int loopDelayCount = 50; loopDelayCount > 0; loopDelayCount--)
    {
        m_highLevelMotorPublisher.publish(highLevelControlMsg);
        centerOnBuoy(color);

        usleep(100000);
        ros::spinOnce();
    }

    highLevelControlMsg.Value = 0.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);
}

/**
 * @brief Calculates the approximate distance to a buoy
 *
 * @param width The width of the object
 *
 * @return The distance in inches, or -1.0 if object is not known
 */
float BuoysTask::getDistanceToBuoy(BuoyColors buoy)
{
  float pixelDiameter = 0.0f;
  float distanceInFeet = -1.0f;
  const float buoySizeInches = 9.0f;
  const float pi = 3.14159265f;
  const float fieldOfViewRadians = pi / 4.0;
  const float viewWidthPixels = 480;

  switch (buoy)
  {
    case RED:
      if (m_redBuoySamples.size() >= 1)
      {
        pixelDiameter = (m_redBuoySamples.back().height + m_redBuoySamples.back().width) / 2.0;
      }
      break;
    case GREEN:
      if (m_greenBuoySamples.size() >= 1)
      {
        pixelDiameter = (m_greenBuoySamples.back().height + m_greenBuoySamples.back().width) / 2.0;
      }
      break;
    case YELLOW:
      if (m_yellowBuoySamples.size() >= 1)
      {
        pixelDiameter = (m_yellowBuoySamples.back().height + m_yellowBuoySamples.back().width) / 2.0;
      }
      break;
  }

  if (pixelDiameter != 0.0f)
  {
      float theta = (pixelDiameter * fieldOfViewRadians) / (2.0 * viewWidthPixels);
      distanceInFeet = ((buoySizeInches / (2.0 * tan(theta))) / 12.0);
  }

  return distanceInFeet;
}

void BuoysTask::clearPastSamples(void)
{
    m_redBuoySamples.clear();
    m_greenBuoySamples.clear();
    m_yellowBuoySamples.clear();
}

bool BuoysTask::isBuoyVisible(BuoyColors buoy)
{
    bool isVisible = false;
    switch (buoy)
    {
        case RED:
          if (m_redBuoySamples.size() >= 3)
          {
            isVisible = true;
          }
          break;
        case GREEN:
          if (m_greenBuoySamples.size() >= 3)
          {
            isVisible = true;
          }
          break;
        case YELLOW:
          if (m_yellowBuoySamples.size() >= 3)
          {
            isVisible = true;
          }
          break;
    }

    return isVisible;
}

void BuoysTask::searchBuoys(BuoyColors buoy)
{
    Robosub::HighLevelControl highLevelControlMsg;
    highLevelControlMsg.Direction = "Turn";
    highLevelControlMsg.MotionType = "Offset";

    // Try rotating left 25 degrees, check to see if buoy is in view
    highLevelControlMsg.Value = -25.0f;
    m_highLevelMotorPublisher.publish(highLevelControlMsg);

    int retries = 50;
    while (!isBuoyVisible(buoy) && (retries > 0))
    {
        usleep(100000);
        retries--;
        ros::spinOnce();
    }


    if (isBuoyVisible(buoy))
    {
        // Make sure we stop rotating
        highLevelControlMsg.Value = 0.0f;
        m_highLevelMotorPublisher.publish(highLevelControlMsg);
    }
    else
    {
        // If not, try rotating right 50 degrees and see if it comes into view
        highLevelControlMsg.Value = 50.0f;
        m_highLevelMotorPublisher.publish(highLevelControlMsg);

        retries = 75;
        while (!isBuoyVisible(buoy) && (retries > 0))
        {
          usleep(100000);
          retries--;
          ros::spinOnce();
        }

        if (isBuoyVisible(buoy))
        {
            // Make sure we stop rotating
            highLevelControlMsg.Value = 0.0f;
            m_highLevelMotorPublisher.publish(highLevelControlMsg);
        }
    }
}

