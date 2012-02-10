#include <QPoint>
#include<QDesktopWidget>

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
     m_missionTogglePublisher(),
     m_imuSubscriber(),
     m_motorControllerTempSubscriber(),
     m_motorCaseTempSubscriber()
{
   m_pUi->setupUi(this);
   m_pJoystickTimer->setInterval(JOYSTICK_POLL_INTERVAL_MSEC);

   connect(m_pJoystickTimer, SIGNAL(timeout()), this, SLOT(readJoystickInput()));
   connect(m_pUi->connectButton, SIGNAL(clicked()), this, SLOT(joyConnect()));
   connect(m_pUi->missionToggleSlider, SIGNAL(valueChanged(int)), this, SLOT(missionToggle()));

   //Attempt to connect to default joystick location
   joyConnect();

   //Center window on screen
   move(qApp->desktop()->availableGeometry(this).center()-rect().center());

   ros::NodeHandle nodeHandle;

   //@todo create a custom message for the Motor_Driver topic
  //m_motorPublisher = nodeHandle.advertise<std_msgs::String>("Motor_Driver", 100);
   m_missionTogglePublisher = nodeHandle.advertise<std_msgs::Bool>("Motor_State", 100);

   m_imuSubscriber = nodeHandle.subscribe("IMU_Data", 100, &SubConsole::imuDataCallback, this);
   m_motorControllerTempSubscriber = nodeHandle.subscribe("Motor_Controller_Temp", 100, &SubConsole::motorControllerTempCallback, this);
   m_motorCaseTempSubscriber = nodeHandle.subscribe("Motor_Case_Temp", 100, &SubConsole::motorCaseTempCallback, this);
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
   //@todo store last state of each axis, create and send message if changed for each changed axis

   //m_pJoystick->getAxis(0); //x-axis
   //m_pJoystick->getAxis(1); //y-axis
   //m_pJoystick->getAxis(2); //twist
   //m_pJoystick->getAxis(3); //throttle
   //m_pJoystick->getAxis(4); //nub x-axis

   //m_pJoystick->getButton(0); //trigger
}

/**
 * @brief Called when the mission toggle slider is changed
 *
 * @param value The new slider value
 **/
void SubConsole::missionToggle(int value)
{
   std_msgs::Bool msg;
   bool toggle = true;

   if(value != 0)
   {
      toggle = false;
   }

   msg.data = toggle;

   m_missionTogglePublisher.publish(msg);
}

/**
 * @brief ROS callback for IMU_Data subscription
 *
 * @param msg The received message
 **/
void SubConsole::imuDataCallback(const std_msgs::String::ConstPtr& msg)
{
   //@todo pull out roll, pitch, and yaw once an actual message is defined
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
