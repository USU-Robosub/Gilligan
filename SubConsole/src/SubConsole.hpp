#ifndef SUBCONSOLE_HPP
#define SUBCONSOLE_HPP

#include <QMainWindow>
#include <QTimer>
#include <joystick.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

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
   ros::Publisher m_missionTogglePublisher;
   ros::Subscriber m_imuSubscriber;
   ros::Subscriber m_motorControllerTempSubscriber;
   ros::Subscriber m_motorCaseTempSubscriber;

   enum
   {
      JOYSTICK_POLL_INTERVAL_MSEC = 100
   };

   void imuDataCallback(const std_msgs::String::ConstPtr& msg);
   void motorControllerTempCallback(const std_msgs::Float32::ConstPtr& msg);
   void motorCaseTempCallback(const std_msgs::Float32::ConstPtr& msg);

private slots:
   void readJoystickInput(void);
   void joyConnect(void);
   void missionToggle(int value);
};

#endif // SUBCONSOLE_HPP
