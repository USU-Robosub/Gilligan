#ifndef SUBCONSOLE_HPP
#define SUBCONSOLE_HPP

#include <QMainWindow>
#include <QTimer>
#include <joystick.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "../msg/imuMsg.h"
#include "../msg/motorMsg.h"

namespace Ui
{
  class SubConsole;
}

class SubConsole : public QMainWindow
{
   Q_OBJECT
public:
   SubConsole(QWidget* pParent = 0);
   ~SubConsole();

private:
   Ui::SubConsole* m_pUi;
   QTimer* m_pJoystickTimer;
   Joystick* m_pJoystick;
   ros::Publisher m_motorPublisher;
   ros::Subscriber m_imuSubscriber;
   ros::Subscriber m_motorControllerTempSubscriber;
   ros::Subscriber m_motorCaseTempSubscriber;
   ros::Subscriber m_pressureSubscriber;
   ros::Subscriber m_motorStateSubscriber;
   ros::Subscriber m_missionStateSubscriber;

   int m_lastXAxisValue;
   int m_lastYAxisValue;
   int m_lastThrottleValue;
   int m_lastTwistValue;

   enum
   {
      JOYSTICK_POLL_INTERVAL_MSEC = 100,
      JOYSTICK_MAX_VALUE = 32767,
      MOTOR_FRONT_TURN = 128,
      MOTOR_BACK_TURN = 64,
      MOTOR_FRONT_DEPTH = 32,
      MOTOR_BACK_DEPTH = 16,
      MOTOR_LEFT_THRUST = 8,
      MOTOR_RIGHT_THRUST = 4
   };

   void imuDataCallback(const Ui::imuMsg::ConstPtr& msg);
   void motorControllerTempCallback(const std_msgs::Float32::ConstPtr& msg);
   void motorCaseTempCallback(const std_msgs::Float32::ConstPtr& msg);
   void pressureDataCallback(const std_msgs::Float32::ConstPtr& msg);
   void motorStateCallback(const std_msgs::Bool::ConstPtr& msg);
   void missionStateCallback(const std_msgs::Bool::ConstPtr& msg);
   void sendMotorSpeedMsg(unsigned char motorMask, unsigned char motorSpeed);

private slots:
   void readJoystickInput(void);
   void joyConnect(void);
};

#endif // SUBCONSOLE_HPP
