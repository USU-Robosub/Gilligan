#ifndef SUBCONSOLE_HPP
#define SUBCONSOLE_HPP

#include <QMainWindow>
#include <QTimer>
#include <vector>

#include "filter.hpp"
#include "joystick.h"
#include "qwt/qwt_compass.h"
#include "attitude_indicator.h"
#include "ClickableLabel.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CompressedImage.h"
#include "SubImageRecognition/ImgRecAlgorithm.h"
#include "USUbConsole/MotorMessage.h"
#include "qcustomplot.h"

#define AVERAGE_LEN 10
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

   void imuDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
   void motorControllerTempCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
   void moboTempCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
   void pressureDataCallback(const std_msgs::Float32::ConstPtr& msg);
   void depthCallback(const std_msgs::Float32::ConstPtr& msg);
//   void forwardCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
   void leftCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
   void rightCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
   void downwardCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
   void currentVoltageCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
   void errorLogCallback(const std_msgs::String::ConstPtr& msg);
   void motorStatusCallback(const USUbConsole::MotorMessage::ConstPtr& msg);
   //TODO add the a subscriber and callback for the Raw acceleration topic
   void rawAccelCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);


private:
   Ui::SubConsole* m_pUi;                           //!< Pointer to UI object
   QTimer* m_pJoystickTimer;                        //!< Timer used to poll joystick state
   QTimer* m_pCallbackTimer;                        //!< Timer used to give processing time to ROS to handle callbacks
   Joystick* m_pJoystick;                           //!< Joystick++ library object
   ros::NodeHandle m_nodeHandle;                    //!< ROS node handle
   ros::Publisher m_motorDriverPublisher;           //!< Publishes the Motor_Driver_Depth topic
   ros::Publisher m_depthPublisher;                 //!< Publishes the Target_Depth topic
   ros::Publisher m_thresholdBoxPublisher;          //!< Publishes the Threshold_Box topic
   ros::Publisher m_torpedoPublisher;               //!< Publishes the Torpedo_Launch topic
   ros::ServiceClient m_imageRecService;            //!< Publishes to the image recognition topic
   ros::ServiceClient m_listAlgorithmService;       //!< Publishes to the image recognition topic
   ros::Subscriber m_imuSubscriber;                 //!< Subscribes to the IMU_Attitude topic
   ros::Subscriber m_motorControllerTempSubscriber; //!< Subscribes to the Motor_Controller_Temp topic
   ros::Subscriber m_moboTempSubscriber;            //!< Subscribes to the Mobo_Temp topic
   ros::Subscriber m_pressureSubscriber;            //!< Subscribes to the Motor_Controller_Temp topic
   ros::Subscriber m_depthSubscriber;               //!< Subscribes to the Motor_Controller_Temp topic
//   ros::Subscriber m_forwardCameraSubscriber;      //!< Subscribes to the Forward_Camera topic
   ros::Subscriber m_leftCameraSubscriber;          //!< Subscribes to the camera_left topic
   ros::Subscriber m_rightCameraSubscriber;          //!< Subscribes to the camera_right topic
   ros::Subscriber m_downwardCameraSubscriber;      //!< Subscribes to the Downward_Camera topic
   ros::Subscriber m_voltageCurrentSubscriber;      //!< Subscribes to the Computer_Cur_Volt topic
   ros::Subscriber m_errorLogSubscriber;            //!< Subscribes to the Error_Log topic

   ros::Subscriber m_motorStatusSubscriber;         //!< Subscribes to the Motor_Control topic
   ros::Subscriber m_rawAccelSubscriber;            //!< Subscribes to the IMU_Accel_Debug topic

   int m_lastXAxisValue;            //!< Stores the last joystick x-axis value
   int m_lastYAxisValue;            //!< Stores the last joystick y-axis value
   int m_lastThrottleValue;         //!< Stores the last joystick throttle value
   int m_lastTwistValue;            //!< Stores the last joystick twist value
   double m_turnForwardPercentage;  //!< The percentage at which the turn thrusters are utilized in the forward direction for turning/straffing
   double m_rightThrustPercentage;
   double m_leftThrustPercentage;

   short m_leftFwdMotorVal;
   short m_rightFwdMotorVal;
   short m_frontTurnMotorVal;
   short m_rearTurnMotorVal;
   short m_frontDepthMotorVal;
   short m_rearDepthMotorVal;

   Filter m_rollAverage;
   Filter m_pitchAverage;
   Filter m_yawAverage;
   Filter m_depthAverage;
   Filter m_battAverage;
   Filter m_currAverage;

//   unsigned char* m_pForwardCameraData;    //!< Pointer to the the last received forward camera frame
   unsigned char* m_pLeftCameraData;     //!< Pointer to the the last received left camera frame
   unsigned char* m_pRightCameraData;     //!< Pointer to the the last received right camera frame
   unsigned char* m_pDownwardCameraData;    //!< Pointer to the the last received downward camera frame
   bool m_downPipEnabled;                   //!< Flag if downward picture in picture is enabled
   bool m_forwardPipEnabled;                //!< Flag if forward picture in picture is enabled

   //Will be pointing to an actual widget (could be eventually removed, with the required changes)
   QwtCompass* m_pCompass;                   //!< Qwt compass widget
   AttitudeIndicator* m_pPitchIndicator;     //!< Qwt attitude indicator used for pitch
   AttitudeIndicator* m_pRollIndicator;      //!< Qwt attitude indicator used for roll

//Image Record Algorithms
//   ClickableLabel* m_pImageRecBoxLabel;
//   ClickableLabel* m_pImageRecDownBoxLabel;



//   std::vector<SubImageRecognition::ImgRecAlgorithm> m_algorithmSettings;

   /**
    * @brief Class constants and mask values
    */
   enum
   {
      JOYSTICK_POLL_INTERVAL_MSEC = 100,
      JOYSTICK_MAX_VALUE = 32767,
      CALLBACK_HANDLE_INTERVAL_MSEC = 20,
      MOTOR_LEFT_DRIVE = 0x01,
      MOTOR_RIGHT_DRIVE = 0x02,
      MOTOR_FRONT_DEPTH = 0x04,
      MOTOR_REAR_DEPTH = 0x08,
      MOTOR_FRONT_TURN = 0x10,
      MOTOR_REAR_TURN = 0x20,
      MAXIMUM_DEPTH = 14
   };

   void sendMotorSpeedMsg(unsigned char motorMask, short leftDrive, short rightDrive, short frontDepth, short rearDepth, short frontTurn, short rearTurn);
   std::string getSelectedAlgorithm(void);
   void filterData(double data, double* buf, double& result);


private slots:
   void readJoystickInput(void);
   void handleRosCallbacks(void);
   void joyConnect(void);
   void toggleDownwardPiP(void);
   void toggleForwardPiP(void);
//Image Record Algorithms
//   void toggleBoxThresholding(void);
//   void enableViewThresholds(void);
//   void disableViewThresholds(void);
//   void imageRecThresholdBoxDrawn(void);
//   void imageRecDownThresholdBoxDrawn(void);

};

#endif // SUBCONSOLE_HPP
