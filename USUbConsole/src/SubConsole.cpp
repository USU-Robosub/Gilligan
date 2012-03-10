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
     m_motorDepthPublisher(),
     m_motorDrivePublisher(),
     m_motorTurnPublisher(),
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
     m_turnForwardPercentage(0.8),
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
   connect(m_pUi->turnThrustFwdSlider, SIGNAL(valueChanged(int)), this, SLOT(adjustFwdTurnMax(int)));

   m_pCallbackTimer->start();

   //Attempt to connect to default joystick location
   joyConnect();

   //Center window on screen
   move(qApp->desktop()->availableGeometry(this).center()-rect().center());

   m_pUi->forwardCameraImageThumb->hide();
   m_pUi->downCameraImageThumb->hide();

   m_motorDepthPublisher = m_nodeHandle.advertise<std_msgs::Int16>("Motor_Driver_Depth", 100);
   m_motorDrivePublisher = m_nodeHandle.advertise<std_msgs::Int16>("Motor_Driver_Drive", 100);
   m_motorTurnPublisher = m_nodeHandle.advertise<std_msgs::Int16>("Motor_Driver_Turn", 100);

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
       int turnSpeed = 255 * (abs(currentXAxis) / (double)JOYSTICK_MAX_VALUE);

       if(currentXAxis > 0)  //Strafe right
       {
          sendMotorSpeedMsg(TURN_CONTROLLER, MOTOR_LEFT | MOTOR_FORWARD, turnSpeed * m_turnForwardPercentage);
          sendMotorSpeedMsg(TURN_CONTROLLER, MOTOR_RIGHT | MOTOR_REVERSE, turnSpeed);
       }
       else if(currentXAxis < 0)//Strafe left
       {
          sendMotorSpeedMsg(TURN_CONTROLLER, MOTOR_RIGHT | MOTOR_FORWARD, turnSpeed * m_turnForwardPercentage);
          sendMotorSpeedMsg(TURN_CONTROLLER, MOTOR_LEFT | MOTOR_REVERSE, turnSpeed);
       }
       else //Motor should be off
       {
          sendMotorSpeedMsg(TURN_CONTROLLER, MOTOR_RIGHT | MOTOR_LEFT | MOTOR_FORWARD, turnSpeed);
       }

       m_lastXAxisValue = currentXAxis;
   }

   if(m_lastYAxisValue != currentYAxis)   //Move forward/backwards
   {
      //Set the forward/reverse thrusters to same value to move forwards or backwards
      int thrusterSpeed = 255 * (abs(currentYAxis) / (double)JOYSTICK_MAX_VALUE);

      //A neg number means the stick is pushed forward, if positive we actually want reverse
      if(currentYAxis >= 0)
      {
         sendMotorSpeedMsg(DRIVE_CONTROLLER, MOTOR_LEFT | MOTOR_RIGHT | MOTOR_FORWARD, thrusterSpeed);
      }
      else if(currentYAxis < 0)
      {
         sendMotorSpeedMsg(DRIVE_CONTROLLER, MOTOR_LEFT | MOTOR_RIGHT | MOTOR_REVERSE, thrusterSpeed);
      }

      m_lastYAxisValue = currentYAxis;
   }

   if(m_lastTwistValue != currentTwistAxis)  //Turn
   {
      //Set the horizontal thrusters to the same direction/velocity to rotate sub
      int thrusterSpeed = 255 * (abs(currentTwistAxis) / (double)JOYSTICK_MAX_VALUE);

      if(currentTwistAxis >= 0)  //Turn right, set both thrusters to reverse
      {
          sendMotorSpeedMsg(TURN_CONTROLLER, MOTOR_LEFT | MOTOR_RIGHT | MOTOR_REVERSE, thrusterSpeed);
      }
      else if(currentTwistAxis < 0)    //Turn left, set both thrusters to forward
      {
          sendMotorSpeedMsg(TURN_CONTROLLER, MOTOR_LEFT | MOTOR_RIGHT | MOTOR_FORWARD, thrusterSpeed * m_turnForwardPercentage);
      }

      m_lastTwistValue = currentTwistAxis;
   }

   if(m_lastThrottleValue != currentThrottleAxis)  //Submerge/surface
   {
      //Set the vertical thrusters to the same value to control depth
      int thrusterSpeed = 255 * (abs(currentThrottleAxis) / (double)JOYSTICK_MAX_VALUE);

      if(currentThrottleAxis >= 0)
      {
         sendMotorSpeedMsg(DEPTH_CONTROLLER, MOTOR_LEFT | MOTOR_RIGHT | MOTOR_REVERSE, thrusterSpeed);
      }
      else if(currentThrottleAxis < 0)
      {
         sendMotorSpeedMsg(DEPTH_CONTROLLER, MOTOR_LEFT | MOTOR_RIGHT | MOTOR_FORWARD, thrusterSpeed);
      }

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
void SubConsole::sendMotorSpeedMsg(int motorController, unsigned char motorMask, unsigned char motorSpeed)
{
   std_msgs::Int16 motorMsg;

   motorMsg.data = ((short)motorMask << 8) | motorSpeed;

   //printf("Sending speed 0x%02x to motor mask 0x%02x.  As Int16: 0x%04x\n", motorSpeed, motorMask, motorMsg.data);

   if(motorController == DEPTH_CONTROLLER)
   {
       m_motorDepthPublisher.publish(motorMsg);
   }
   else if(motorController == DRIVE_CONTROLLER)
   {
       m_motorDrivePublisher.publish(motorMsg);
   }
   else if(motorController == TURN_CONTROLLER)
   {
       m_motorTurnPublisher.publish(motorMsg);
   }
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

   m_pUi->motorControllerTempLineEdit->setText(QString::number(temperature/10.0));
}

/**
 * @brief ROS callback for Motor_Case_Temp subscription
 *
 * @param msg The received message
 **/
void SubConsole::motorCaseTempCallback(const std_msgs::Float32::ConstPtr& msg)
{
   float temperature = msg->data;

   m_pUi->motorCaseTempLineEdit->setText(QString::number(temperature/10.0));
}

/**
 * @brief ROS callback for Pressure_Data subscription
 *
 * @param msg The received message
 **/
void SubConsole::pressureDataCallback(const std_msgs::Float32::ConstPtr& msg)
{
   //fresh depth = psi / 0.432
   //salt depth = psi / 0.445
   float pressure = msg->data;

   m_pUi->freshDepthLineEdit->setText(QString::number(pressure / 0.432));
   m_pUi->saltDepthLineEdit->setText(QString::number(pressure / 0.445));
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

   if(m_forwardPipEnabled)
   {
      m_pUi->forwardCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(144), 0));
   }
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

    if(m_downPipEnabled)
    {
        m_pUi->downCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(144), 0));
    }
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

/**
 * @brief Called when the turn thruster fwd thrust percentage slider is moved
 *
 * @param sliderValue The value of the slider
 **/
void SubConsole::adjustFwdTurnMax(int sliderValue)
{
    QString percentString = QString::number(sliderValue);

    percentString += "% Turn Fwd";
    m_turnForwardPercentage = sliderValue / 100.0;
    m_pUi->turnFwdPercentageLabel->setText(percentString);
}
