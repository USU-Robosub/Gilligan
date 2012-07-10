#ifndef _SUB_ATTITUDE_RESOLVER_HPP
#define _SUB_ATTITUDE_RESOLVER_HPP

/**
 * @file SubAttitudeResolver.hpp
 *
 * @brief Header file for the SubAttitudeResolver class
 *
 * Revision History:
 * May 29, 2012  Bryan Hansen   Initial implementation
 */

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "serialib.h"

/**
 * @brief Class responsible for performing a kalman filter to determine attitude and publish it
 */
class SubAttitudeResolver
{
   public:
      SubAttitudeResolver(std::string devName);
      ~SubAttitudeResolver();

      void run();

   private:
      void publishAttitude(double yaw, double pitch, double roll);
      void publishMagDebug(double* pResidual, double* pPr);
      void publishAccelDebug(double* pResidual, double* pPr);
      void kalmanUpdate(double* y_i, double* y_b, double* R, double* Pr, double* residual);
      void kalmanPropagate(void);
      void sampleGyro(short* pRawX, short* pRawY, short* pRawZ);
      void sampleAccel(short* pRawX, short* pRawY, short* pRawZ);
      void sampleMag(short* pRawX, short* pRawY, short* pRawZ);
      void updateOmega(short rawX, short rawY, short rawZ);
      void calculateGyroBias(void);
      void calculateExpectedAccel(void);
      void calculateExpectedMag(void);
      bool syncSerial(char command);

      ros::NodeHandle m_nodeHandle;          //!< ROS node handle
      ros::Publisher m_attitudePublisher;    //!< Publishes the Sub_Attitude topic
      ros::Publisher m_magDebugPublisher;    //!< Publishes the Mag_Debug topic
      ros::Publisher m_accelDebugPublisher;  //!< Publishes the Accel_Debug topic

      double m_q[4];               //!< Attitude Quaternion
      double m_P[6];               //!< Attitude Covariance, 1 degree Uncertainty
      double m_w[3];               //!< Angular Velocity Read from Gyros in Radians/Second
      double m_PrAccel[6];              //!< Output value from kalmanUpdate used to tune uncertainty
      double m_residualAccel[3];        //!< Output value from kalmanUpdate used to tune uncertainty
      double m_PrMag[6];              //!< Output value from kalmanUpdate used to tune uncertainty
      double m_residualMag[3];        //!< Output value from kalmanUpdate used to tune uncertainty
      double m_yaw;                //!< Calculated yaw
      double m_pitch;              //!< Calculated pitch
      double m_roll;               //!< Calculated roll
      double m_biasX;              //!< Calculated bias for gyro x
      double m_biasY;              //!< Calculated bias for gyro y
      double m_biasZ;              //!< Calculated bias for gyro z
      double m_expectedAccel[3];   //!< Expected reading for accelerometer
      double m_expectedMag[3];     //!< Expected reading for magnetometer

      serialib m_serialPort;  //!< Serial port used for communication
      std::string m_devName;  //!< IMU device location to open

      static const double pi = 3.14159265358979;
      static const int gyroFullScale = 285;
      static const double gyroConversion = gyroFullScale / 32767.0;
};

#endif // _SUB_ATTITUDE_RESOLVER_HPP
