#include <QPoint>
#include<QDesktopWidget>
#include <QImage>
#include <QPixmap>
#include <math.h>

#include "SubConsole.hpp"
#include "ui_SubConsole.h"

/**
 * @brief SubConsole ctor which sets up timers and connect signals/slots
 *
 * @param pParent Poitner to parent widget
 **/
SubConsole::SubConsole(QWidget* pParent)
   : QMainWindow(pParent),
     m_pUi(new Ui::SubConsole),
     m_pJoystickTimer(new QTimer(this)),
     m_pCallbackTimer(new QTimer(this)),
     m_pJoystick(new Joystick()),
     m_nodeHandle(),
     m_motorPublisher(),
     m_imuSubscriber(),
     m_motorControllerTempSubscriber(),
     m_motorCaseTempSubscriber(),
     m_pressureSubscriber(),
     m_motorStateSubscriber(),
     m_forwardCameraSubscriber(),
     m_downwardCameraSubscriber(),
     m_lastXAxisValue(0),
     m_lastYAxisValue(0),
     m_lastThrottleValue(0),
     m_lastTwistValue(0),
     m_pForwardCameraData(NULL),
     m_pDownwardCameraData(NULL),
     m_downPipEnabled(false),
     m_forwardPipEnabled(false)
{
   m_pUi->setupUi(this);
   m_pJoystickTimer->setInterval(JOYSTICK_POLL_INTERVAL_MSEC);
   m_pCallbackTimer->setInterval(CALLBACK_HANDLE_INTERVAL_MSEC);

   connect(m_pJoystickTimer, SIGNAL(timeout()), this, SLOT(readJoystickInput()));
   connect(m_pCallbackTimer, SIGNAL(timeout()), this, SLOT(handleRosCallbacks()));
   connect(m_pUi->connectButton, SIGNAL(clicked()), this, SLOT(joyConnect()));
   connect(m_pUi->downPipButton, SIGNAL(clicked()), this, SLOT(toggleDownwardPiP()));
   connect(m_pUi->forwardPipButton, SIGNAL(clicked()), this, SLOT(toggleForwardPiP()));

   m_pCallbackTimer->start();

   //Attempt to connect to default joystick location
   joyConnect();

   //Center window on screen
   move(qApp->desktop()->availableGeometry(this).center()-rect().center());

   m_pUi->forwardCameraImageThumb->hide();
   m_pUi->downCameraImageThumb->hide();

   m_motorPublisher = m_nodeHandle.advertise<std_msgs::UInt8MultiArray>("sub_motor_driver", 100);

   m_imuSubscriber = m_nodeHandle.subscribe("IMU_Data", 100, &SubConsole::imuDataCallback, this);
   m_motorControllerTempSubscriber = m_nodeHandle.subscribe("Motor_Controller_Temp", 100, &SubConsole::motorControllerTempCallback, this);
   m_motorCaseTempSubscriber = m_nodeHandle.subscribe("Motor_Case_Temp", 100, &SubConsole::motorCaseTempCallback, this);
   m_pressureSubscriber = m_nodeHandle.subscribe("Pressure_Data", 100, &SubConsole::pressureDataCallback, this);
   m_motorStateSubscriber = m_nodeHandle.subscribe("Motor_State", 100, &SubConsole::motorStateCallback, this);
   m_forwardCameraSubscriber = m_nodeHandle.subscribe("/left/image_raw", 100, &SubConsole::forwardCameraCallback, this);
   m_downwardCameraSubscriber = m_nodeHandle.subscribe("/right/image_raw", 100, &SubConsole::downwardCameraCallback, this);

   printf("Finished ROS topic publish and subscription initialization\n");
}

/**
 * @brief SubConsole dtor
 **/
SubConsole::~SubConsole()
{
   delete m_pUi;

   m_pJoystickTimer->stop();
   m_pCallbackTimer->stop();
   delete m_pJoystickTimer;
   delete m_pCallbackTimer;
   delete m_pJoystick;
}

/**
 * @brief Attempts to connect to the joystick
 **/
void SubConsole::joyConnect(void)
{
   m_pJoystickTimer->stop();

   if(m_pJoystick->init(m_pUi->devLineEdit->text().toStdString().c_str()) == 0)
   {
       m_pUi->joystickStatusLabel->setText("Connected");
       m_pJoystickTimer->start();
   }
   else
   {
       m_pUi->joystickStatusLabel->setText("Disconnected");
   }
}

/**
 * @brief Periodically reads joystick states if any states have changed publish topic to motor controllers
 **/
void SubConsole::readJoystickInput(void)
{
   int currentXAxis = m_pJoystick->getAxis(0);
   int currentYAxis = m_pJoystick->getAxis(1);
   int currentTwistAxis = m_pJoystick->getAxis(2);
   int currentThrottleAxis = m_pJoystick->getAxis(3);

   if(m_lastXAxisValue != currentXAxis)   //Strafe
   {
       //Set the horizontal thrusters to opposite thrust to strafe
       int frontTurnSpeed = 127 * (abs(currentXAxis) / (double)JOYSTICK_MAX_VALUE);
       int backTurnSpeed = 127 * (abs(currentXAxis) / (double)JOYSTICK_MAX_VALUE);

       if(currentXAxis > 0)  //Strafe right
       {
          frontTurnSpeed = backTurnSpeed + 128;
       }
       else if(currentXAxis < 0)//Strafe left
       {
          backTurnSpeed = frontTurnSpeed + 128;
       }
       else //Motor should be off
       {
           frontTurnSpeed = 0;
           backTurnSpeed = 0;
       }

       sendMotorSpeedMsg(MOTOR_FRONT_TURN, frontTurnSpeed);
       sendMotorSpeedMsg(MOTOR_BACK_TURN, backTurnSpeed);

       m_lastXAxisValue = currentXAxis;
   }

   if(m_lastYAxisValue != currentYAxis)   //Move forward/backwards
   {
      //Set the forward/reverse thrusters to same value to move forwards or backwards
      int thrusterSpeed = 127 * (abs(currentYAxis) / (double)JOYSTICK_MAX_VALUE);

      //A neg number means the stick is pushed forward, if positive we actually want reverse
      if(currentYAxis > 0)
      {
         thrusterSpeed += 128;
      }
      else if(currentYAxis == 0)
      {
          thrusterSpeed = 0;
      }

      sendMotorSpeedMsg(MOTOR_LEFT_THRUST | MOTOR_RIGHT_THRUST, thrusterSpeed);

      m_lastYAxisValue = currentYAxis;
   }

   if(m_lastTwistValue != currentTwistAxis)  //Turn
   {
      //Set the horizontal thrusters to the same direction/velocity to rotate sub
      int thrusterSpeed = 127 * (abs(currentTwistAxis) / (double)JOYSTICK_MAX_VALUE);

      if(currentTwistAxis > 0)  //Turn right (to turn left leave both set from 0-127)
      {
         thrusterSpeed += 128;
      }
      else if(currentTwistAxis == 0)
      {
          thrusterSpeed = 0;
      }

      sendMotorSpeedMsg(MOTOR_FRONT_TURN | MOTOR_BACK_TURN, thrusterSpeed);

      m_lastTwistValue = currentTwistAxis;
   }

   if(m_lastThrottleValue != currentThrottleAxis)  //Submerge/surface
   {
      //Set the vertical thrusters to the same value to control depth
      int thrusterSpeed = 127 * (abs(currentThrottleAxis) / (double)JOYSTICK_MAX_VALUE);

      if(currentThrottleAxis > 0)
      {
         thrusterSpeed += 128;
      }
      else if(currentThrottleAxis == 0)
      {
          thrusterSpeed = 0;
      }

      sendMotorSpeedMsg(MOTOR_FRONT_DEPTH | MOTOR_BACK_DEPTH, thrusterSpeed);

      m_lastThrottleValue = currentThrottleAxis;
   }
}

/**
 * @brief Periodically allows ROS time to handle received callbacks
 **/
void SubConsole::handleRosCallbacks(void)
{
    ros::spinOnce();
}

/**
 * @brief Publishes a Motor_Driver message
 *
 * @param motorMask Bit mask with the motors to drive at the specified speed
 **/
void SubConsole::sendMotorSpeedMsg(unsigned char motorMask, unsigned char motorSpeed)
{
   std_msgs::UInt8MultiArray motorMsg;

   motorMsg.data.push_back(motorMask);
   motorMsg.data.push_back(motorSpeed);

   m_motorPublisher.publish(motorMsg);

}

/**
 * @brief ROS callback for IMU_Data subscription
 *
 * @param msg The received message
 **/
void SubConsole::imuDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   m_pUi->yawLineEdit->setText(QString::number(msg->data[0]));
   m_pUi->pitchLineEdit->setText(QString::number(msg->data[1]));
   m_pUi->rollLineEdit->setText(QString::number(msg->data[2]));
}

/**
 * @brief ROS callback for Motor_Controller_Temp subscription
 *
 * @param msg The received message
 **/
void SubConsole::motorControllerTempCallback(const std_msgs::Float32::ConstPtr& msg)
{
   float temperature = msg->data;

   m_pUi->motorControllerTempLineEdit->setText(QString::number(temperature));
}

/**
 * @brief ROS callback for Motor_Case_Temp subscription
 *
 * @param msg The received message
 **/
void SubConsole::motorCaseTempCallback(const std_msgs::Float32::ConstPtr& msg)
{
   float temperature = msg->data;

   m_pUi->motorCaseTempLineEdit->setText(QString::number(temperature));
}

/**
 * @brief ROS callback for Pressure_Data subscription
 *
 * @param msg The received message
 **/
void SubConsole::pressureDataCallback(const std_msgs::Float32::ConstPtr& msg)
{
   float pressure = msg->data;

   m_pUi->pressureLineEdit->setText(QString::number(pressure));
}

/**
 * @brief ROS callback for Motor_State subscription
 *
 * @param msg The received message
 **/
void SubConsole::motorStateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   bool motorEnabled = msg->data;

   if(motorEnabled)
   {
      m_pUi->motorStateLineEdit->setText("Enabled");
   }
   else
   {
      m_pUi->motorStateLineEdit->setText("Disabled");
   }
}

/**
 * @brief ROS callback for Mission_State subscription
 *
 * @param msg The received message
 **/
void SubConsole::missionStateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   bool missionEnabled = msg->data;

   if(missionEnabled)
   {
      m_pUi->missionStateLineEdit->setText("Enabled");
   }
   else
   {
      m_pUi->missionStateLineEdit->setText("Disabled");
   }
}

/**
 * @brief ROS callback for Forward_Camera subscription
 *
 * @param msg The received message
 **/
void SubConsole::forwardCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
   int imgHeight = msg->height;
   int imgWidth = msg->width;
   unsigned int step = msg->step;
   QImage image;
   QPixmap pixmap;

   if(m_pForwardCameraData != NULL)
   {
       delete [] m_pForwardCameraData;
   }

   m_pForwardCameraData = new unsigned char[msg->data.size()];
   std::copy(msg->data.begin(), msg->data.end(), m_pForwardCameraData);

   image = QImage(m_pForwardCameraData, imgWidth, imgHeight, step, QImage::Format_RGB888);
   m_pUi->forwardCameraImage->setPixmap(pixmap.fromImage(image, 0));
   m_pUi->forwardCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(144), 0));
}

/**
 * @brief ROS callback for Forward_Camera subscription
 *
 * @param msg The received message
 **/
void SubConsole::downwardCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    int imgHeight = msg->height;
    int imgWidth = msg->width;
    unsigned int step = msg->step;
    QImage image;
    QPixmap pixmap;

    if(m_pDownwardCameraData != NULL)
    {
        delete [] m_pDownwardCameraData;
    }

    m_pDownwardCameraData = new unsigned char[msg->data.size()];
    std::copy(msg->data.begin(), msg->data.end(), m_pDownwardCameraData);

    image = QImage(m_pDownwardCameraData, imgWidth, imgHeight, step, QImage::Format_RGB888);
    m_pUi->downwardCameraImage->setPixmap(pixmap.fromImage(image, 0));
    m_pUi->downCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(144), 0));
}

/**
 * @brief Toggles the visibility of the downward picture-in-picture
 **/
void SubConsole::toggleDownwardPiP(void)
{
    if(m_downPipEnabled)
    {
        m_pUi->downCameraImageThumb->hide();
        m_downPipEnabled = false;
    }
    else
    {
        m_pUi->downCameraImageThumb->show();
        m_downPipEnabled = true;
    }
}

/**
 * @brief Toggles the visibility of the forward camera picture-in-picture
 **/
void SubConsole::toggleForwardPiP(void)
{
    if(m_forwardPipEnabled)
    {
        m_pUi->forwardCameraImageThumb->hide();
        m_forwardPipEnabled = false;
    }
    else
    {
        m_pUi->forwardCameraImageThumb->show();
        m_forwardPipEnabled = true;
    }
}
