#ifndef SUBCONSOLE_HPP
#define SUBCONSOLE_HPP

#include <QMainWindow>
#include <QTimer>
#include <joystick.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "USUbConsole/imuMsg.h"
#include "USUbConsole/motorMsg.h"
#include "sensor_msgs/Image.h"

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

   void imuDataCallback(const USUbConsole::imuMsg::ConstPtr& msg);
   void motorControllerTempCallback(const std_msgs::Float32::ConstPtr& msg);
   void motorCaseTempCallback(const std_msgs::Float32::ConstPtr& msg);
   void pressureDataCallback(const std_msgs::Float32::ConstPtr& msg);
   void motorStateCallback(const std_msgs::Bool::ConstPtr& msg);
   void missionStateCallback(const std_msgs::Bool::ConstPtr& msg);
   void forwardCameraCallback(const sensor_msgs::Image::ConstPtr& msg);
   void downwardCameraCallback(const sensor_msgs::Image::ConstPtr& msg);
   void tempCallback(const std_msgs::String::ConstPtr& msg);

private:
   Ui::SubConsole* m_pUi;                           //!< Pointer to UI object
   QTimer* m_pJoystickTimer;                        //!< Timer used to poll joystick state
   QTimer* m_pCallbackTimer;                        //!< Timer used to give processing time to ROS to handle callbacks
   Joystick* m_pJoystick;                           //!< Joystick++ library object
   ros::NodeHandle m_nodeHandle;                    //!< ROS node handle
   ros::Publisher m_motorPublisher;                 //!< Publishes the Motor_Driver topic
   ros::Subscriber m_imuSubscriber;                 //!< Subscribes to the IMU_Data topic
   ros::Subscriber m_motorControllerTempSubscriber; //!< Subscribes to the Motor_Controller_Temp topic
   ros::Subscriber m_motorCaseTempSubscriber;       //!< Subscribes to the Motor_Case_Temp topic
   ros::Subscriber m_pressureSubscriber;            //!< Subscribes to the Motor_Controller_Temp topic
   ros::Subscriber m_motorStateSubscriber;          //!< Subscribes to the Pressure_Data topic
   ros::Subscriber m_missionStateSubscriber;        //!< Subscribes to the Mission_State topic
   ros::Subscriber m_forwardCameraSubscriber;       //!< Subscribes to the Forward_Camera topic
   ros::Subscriber m_downwardCameraSubscriber;      //!< Subscribes to the Downward_Camera topic

   int m_lastXAxisValue;      //!< Stores the last joystick x-axis value
   int m_lastYAxisValue;      //!< Stores the last joystick y-axis value
   int m_lastThrottleValue;   //!< Stores the last joystick throttle value
   int m_lastTwistValue;      //!< Stores the last joystick twist value

   unsigned char* m_pForwardCameraData;     //!< Pointer to the the last received forward camera frame
   unsigned char* m_pDownwardCameraData;    //!< Pointer to the the last received downward camera frame

   /**
    * @brief Class constants and mask values
    */
   enum
   {
      JOYSTICK_POLL_INTERVAL_MSEC = 100,
      JOYSTICK_MAX_VALUE = 32767,
      CALLBACK_HANDLE_INTERVAL_MSEC = 20,
      MOTOR_FRONT_TURN = 128,
      MOTOR_BACK_TURN = 64,
      MOTOR_FRONT_DEPTH = 32,
      MOTOR_BACK_DEPTH = 16,
      MOTOR_LEFT_THRUST = 8,
      MOTOR_RIGHT_THRUST = 4
   };

   void sendMotorSpeedMsg(unsigned char motorMask, unsigned char motorSpeed);

private slots:
   void readJoystickInput(void);
   void handleRosCallbacks(void);
   void joyConnect(void);
};

#endif // SUBCONSOLE_HPP