#include <QPoint>
#include<QDesktopWidget>
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
     m_pJoystick(new Joystick()),
     m_motorPublisher(),
     m_imuSubscriber(),
     m_motorControllerTempSubscriber(),
     m_motorCaseTempSubscriber(),
     m_pressureSubscriber(),
     m_motorStateSubscriber(),
     m_lastXAxisValue(0),
     m_lastYAxisValue(0),
     m_lastThrottleValue(0),
     m_lastTwistValue(0)
{
   m_pUi->setupUi(this);
   m_pJoystickTimer->setInterval(JOYSTICK_POLL_INTERVAL_MSEC);

   connect(m_pJoystickTimer, SIGNAL(timeout()), this, SLOT(readJoystickInput()));
   connect(m_pUi->connectButton, SIGNAL(clicked()), this, SLOT(joyConnect()));

   //Attempt to connect to default joystick location
   joyConnect();

   //Center window on screen
   move(qApp->desktop()->availableGeometry(this).center()-rect().center());

   ros::NodeHandle nodeHandle;

   m_motorPublisher = nodeHandle.advertise<Ui::motorMsg>("Motor_Driver", 100);

   m_imuSubscriber = nodeHandle.subscribe("IMU_Data", 100, &SubConsole::imuDataCallback, this);
   m_motorControllerTempSubscriber = nodeHandle.subscribe("Motor_Controller_Temp", 100, &SubConsole::motorControllerTempCallback, this);
   m_motorCaseTempSubscriber = nodeHandle.subscribe("Motor_Case_Temp", 100, &SubConsole::motorCaseTempCallback, this);
   m_pressureSubscriber = nodeHandle.subscribe("Pressure_Data", 100, &SubConsole::pressureDataCallback, this);
   m_motorStateSubscriber = nodeHandle.subscribe("Motor_State", 100, &SubConsole::motorStateCallback, this);
}

/**
 * @brief SubConsole dtor
 **/
SubConsole::~SubConsole()
{
   delete m_pUi;

   m_pJoystickTimer->stop();
   delete m_pJoystickTimer;
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

   if(m_lastXAxisValue != currentXAxis)
   {
       //Set the horizontal thrusters to opposite thrust to strafe
       int frontTurnSpeed = 127 * (abs(currentXAxis) / (double)JOYSTICK_MAX_VALUE);
       int backTurnSpeed = 127 * (abs(currentXAxis) / (double)JOYSTICK_MAX_VALUE);

       if(currentXAxis >= 0)
       {
          backTurnSpeed = frontTurnSpeed + 128;
       }
       else
       {
          frontTurnSpeed = backTurnSpeed + 128;
       }

       sendMotorSpeedMsg(MOTOR_FRONT_TURN, frontTurnSpeed);
       sendMotorSpeedMsg(MOTOR_BACK_TURN, backTurnSpeed);

       m_lastXAxisValue = currentXAxis;
   }

   if(m_lastYAxisValue != currentYAxis)
   {
      //Set the forward/reverse thrusters to same value to move forwards or backwards
      int thrusterSpeed = 127 * (abs(currentYAxis) / (double)JOYSTICK_MAX_VALUE);

      //A neg number means the stick is pushed forward, if positive we actually want reverse
      if(currentYAxis >= 0)
      {
         thrusterSpeed += 128;
      }

      sendMotorSpeedMsg(MOTOR_LEFT_THRUST | MOTOR_RIGHT_THRUST, thrusterSpeed);

      m_lastYAxisValue = currentYAxis;
   }

   if(m_lastTwistValue != currentTwistAxis)
   {
      //Set the horizontal thrusters to the same direction/velocity to rotate sub
      int thrusterSpeed = 127 * (abs(currentTwistAxis) / (double)JOYSTICK_MAX_VALUE);

      if(currentTwistAxis < 0)
      {
         thrusterSpeed += 128;
      }

      sendMotorSpeedMsg(MOTOR_FRONT_TURN | MOTOR_BACK_TURN, thrusterSpeed);

      m_lastTwistValue = currentTwistAxis;
   }

   if(m_lastThrottleValue != currentThrottleAxis)
   {
      //Set the vertical thrusters to the same value to control depth
      int thrusterSpeed = 127 * (abs(currentThrottleAxis) / (double)JOYSTICK_MAX_VALUE);

      if(currentThrottleAxis < 0)
      {
         thrusterSpeed += 128;
      }

      sendMotorSpeedMsg(MOTOR_FRONT_DEPTH | MOTOR_BACK_DEPTH, thrusterSpeed);

      m_lastThrottleValue = currentThrottleAxis;
   }
}

/**
 * @brief Publishes a Motor_Driver message
 *
 * @param motorMask Bit mask with the motors to drive at the specified speed
 **/
void SubConsole::sendMotorSpeedMsg(unsigned char motorMask, unsigned char motorSpeed)
{
   Ui::motorMsg motorMsg;

   motorMsg.motorMask = motorMask;
   motorMsg.speed = motorSpeed;

   m_motorPublisher.publish(motorMsg);

}

/**
 * @brief ROS callback for IMU_Data subscription
 *
 * @param msg The received message
 **/
void SubConsole::imuDataCallback(const Ui::imuMsg::ConstPtr& msg)
{
   m_pUi->yawLineEdit->setText(QString::number(msg->yaw));
   m_pUi->pitchLineEdit->setText(QString::number(msg->pitch));
   m_pUi->rollLineEdit->setText(QString::number(msg->roll));
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
void SubConsole::motorStateCallback(const std_msgs::Bool::ConstPtr& msg)
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
void SubConsole::missionStateCallback(const std_msgs::Bool::ConstPtr& msg)
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
