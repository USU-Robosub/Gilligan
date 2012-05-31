/**
 * @file SubAttitudeResolver.cpp
 *
 * @brief Implementation file for the SubAttitudeResolver class
 *
 * Revision History:
 * May 29, 2012  Bryan Hansen   Initial implementation
 */

#include <stdio.h>
#include <unistd.h>

#include "SubAttitudeResolver.hpp"
#include "std_msgs/Float64MultiArray.h"

#define BAUD_RATE B57600

/**
 * @brief Constructor
 */
SubAttitudeResolver::SubAttitudeResolver(std::string devName)
  : m_nodeHandle(),
    m_attitudePublisher(),
    m_yaw(0.0),
    m_pitch(0.0),
    m_roll(0.0),
    m_biasX(0.0),
    m_biasY(0.0),
    m_biasZ(0.0),
    m_serialPort(),
    m_devName(devName)
{
    //Initialize values
    m_q[0] = 0;
    m_q[1] = 0;
    m_q[2] = 0;
    m_q[3] = 1;

    m_P[0] = 3.046174197867086e-004;
    m_P[1] = 0.0;
    m_P[2] = 0.0;
    m_P[3] = 3.046174197867086e-004;
    m_P[4] = 0.0;
    m_P[5] = 3.046174197867086e-004;

    m_attitudePublisher = m_nodeHandle.advertise<std_msgs::Float64MultiArray>("IMU_Attitude", 100);
}

/**
 * @brief Destructor
 */
SubAttitudeResolver::~SubAttitudeResolver()
{
    m_serialPort.Close();
}

/**
 * @brief Main run loop, reads the gyro and performs the kalman filter
 */
void SubAttitudeResolver::run()
{
    m_serialPort.Open(m_devName.c_str(), 57600);

    ros::Rate loop_rate(100);
    int count = 0;

    calculateBias();

    while(ros::ok())
    {
        short rawX = 0;
        short rawY = 0;
        short rawZ = 0;

        // Sample gyro
        sampleGyro(&rawX, &rawY, &rawZ);
        updateOmega(rawX, rawY, rawZ);

        kalmanPropagate();

        if((count % 5) == 0)
        {
            //printf("Yaw: %lf, Pitch: %lf, Roll: %lf\n", (m_yaw * 180)/pi, (m_pitch * 180)/pi, (m_roll * 180)/pi);
            publishAttitude(-1*(m_yaw * 180)/pi, -1*(m_pitch * 180)/pi, -1*(m_roll * 180)/pi);
        }

        count++;

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * @brief Publishes the IMU_Attitude message
 */
void SubAttitudeResolver::publishAttitude(double yaw, double pitch, double roll)
{
    std_msgs::Float64MultiArray attitudeMsg;

    attitudeMsg.data.push_back(yaw);
    attitudeMsg.data.push_back(pitch);
    attitudeMsg.data.push_back(roll);

    m_attitudePublisher.publish(attitudeMsg);
}

/**
 * @brief Calculates the gyro bias on startup, IMU should be still during this calculation
 */
void SubAttitudeResolver::calculateBias(void)
{
    short rawX = 0;
    short rawY = 0;
    short rawZ = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    for(int i = 0; i < 64; i++)
    {
        sampleGyro(&rawX, &rawY, &rawZ);

        //printf("Sampled x: %i, y: %i, z: %i\n", rawX, rawY, rawZ);

        sumX += rawX;
        sumY += rawY;
        sumZ += rawZ;
    }

    m_biasX = sumX / 64.0;
    m_biasY = sumY / 64.0;
    m_biasZ = sumZ / 64.0;

    printf("SubAttitudeResolver: Calculated Bias x: %lf, y: %lf, z: %lf\n", m_biasX, m_biasY, m_biasZ);

}

/**
 * @brief Kalman filter propagation
 *
  @brief Thanks to Bryan Bingham!
 */
void SubAttitudeResolver::kalmanPropagate()
{
    double sigma_w2[3] = {3.046174197867087e-008,3.046174197867087e-008,3.046174197867087e-008}; // Gyro Uncertainty (0.01 Degrees/second)
    double dt = 0.01;  // Sampling time (Currently 100 Hz)
    double q_new[4]; // Temporary Propagated Quaternion
    double P_new[6]; // Temporary Propagated Covariance
    double psi[3]; // Used in Quaternion Propagation
    double c; // Used in Quaternion Propagation
    double w_mag; // Magnitude of Angular Velocity Vector
    double q_mag; // Magnitude of Quaternion
    double R13;
    double R21;
    double R22;
    double R23;
    double R33;

    // Combine Terms to Reduce Computations
    double w0w0;
    double w1w1;
    double w2w2;
    double w_magdt05; // w_mag*dt*0.5
    double w1w2;
    double w0w1;
    double w0w2;
    double q0q0;
    double q1q1;
    double q2q2;
    double q3q3;
    double dt2;  // Time Step Squared

    // Combine Terms
    w0w0 = m_w[0]*m_w[0];
    w1w1 = m_w[1]*m_w[1];
    w2w2 = m_w[2]*m_w[2];
    w1w2 = m_w[1]*m_w[2];
    w0w1 = m_w[0]*m_w[1];
    w0w2 = m_w[0]*m_w[2];
    q0q0 = m_q[0]*m_q[0];
    q1q1 = m_q[1]*m_q[1];
    q2q2 = m_q[2]*m_q[2];
    q3q3 = m_q[3]*m_q[3];
    dt2 = dt*dt;

    // Create Direction Cosine Matrix From Quaternion
    R13 = 2.0*m_q[0]*m_q[2] - 2.0*m_q[1]*m_q[3];
    R21 = 2.0*m_q[0]*m_q[1] - 2.0*m_q[2]*m_q[3];
    R22 = -1.0*q0q0 + q1q1 - q2q2 + q3q3;
    R23 = 2.0*m_q[0]*m_q[3] + 2.0*m_q[1]*m_q[2];
    R33 = -1.0*q0q0 - q1q1 + q2q2 + q3q3;

    // Extract Euler Angles (Z X Y Sequence)
    m_yaw = atan2(-1.0*R21,R22); // Z
    m_pitch = asin(R23); // X
    m_roll = atan2(-1.0*R13,R33); // Y

    // Calculate Magnitude of Angular Velocity
    w_mag = sqrt( w0w0 + w1w1 + w2w2 );

    // Propagate Quaternion
    if (w_mag > 1.745329251994330e-004) // 0.01 degrees/second
    {
        w_magdt05 = w_mag*dt*0.5;

        c = cos(w_magdt05);

        psi[0] = sin(w_magdt05)*m_w[0]/w_mag;
        psi[1] = sin(w_magdt05)*m_w[1]/w_mag;
        psi[2] = sin(w_magdt05)*m_w[2]/w_mag;

        q_new[0] = c*m_q[0] + psi[2]*m_q[1] - psi[1]*m_q[2] + psi[0]*m_q[3];
        q_new[1] = c*m_q[1] - psi[2]*m_q[0] + psi[0]*m_q[2] + psi[1]*m_q[3];
        q_new[2] = c*m_q[2] + psi[1]*m_q[0] - psi[0]*m_q[1] + psi[2]*m_q[3];
        q_new[3] = c*m_q[3] - psi[0]*m_q[0] - psi[1]*m_q[1] - psi[2]*m_q[2];

        q_mag = sqrt( q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3] );

        m_q[0] = q_new[0]/q_mag;
        m_q[1] = q_new[1]/q_mag;
        m_q[2] = q_new[2]/q_mag;
        m_q[3] = q_new[3]/q_mag;
    }

    // Propagate Covariance
    P_new[0] = m_P[0] + m_P[5]*dt2*w1w1 - 2.0*dt2*m_P[4]*w1w2 + m_P[3]*dt2*w2w2 - 2.0*dt*m_P[2]*m_w[1] + 2.0*dt*m_P[1]*m_w[2] + dt*sigma_w2[0];
    P_new[1] = m_P[1] - m_P[1]*dt2*w2w2 - m_P[0]*dt*m_w[2] + m_P[2]*dt*m_w[0] + m_P[3]*dt*m_w[2] - m_P[4]*dt*m_w[1] + m_P[2]*dt2*w1w2 + m_P[4]*dt2*w0w2 - m_P[5]*dt2*w0w1;
    P_new[2] = m_P[2] - m_P[2]*dt2*w1w1 + m_P[0]*dt*m_w[1] - m_P[1]*dt*m_w[0] + m_P[4]*dt*m_w[2] - m_P[5]*dt*m_w[1] + m_P[1]*dt2*w1w2 - m_P[3]*dt2*w0w2 + m_P[4]*dt2*w0w1;
    P_new[3] = m_P[3] + m_P[5]*dt2*w0w0 - 2.0*dt2*m_P[2]*w0w2 + m_P[0]*dt2*w2w2 + 2.0*dt*m_P[4]*m_w[0] - 2.0*dt*m_P[1]*m_w[2] + dt*sigma_w2[1];
    P_new[4] = m_P[4] - m_P[4]*dt2*w0w0 + m_P[1]*dt*m_w[1] - m_P[3]*dt*m_w[0] - m_P[2]*dt*m_w[2] + m_P[5]*dt*m_w[0] - m_P[0]*dt2*w1w2 + m_P[1]*dt2*w0w2 + m_P[2]*dt2*w0w1;
    P_new[5] = m_P[5] + m_P[3]*dt2*w0w0 - 2.0*dt2*m_P[1]*w0w1 + m_P[0]*dt2*w1w1 - 2.0*dt*m_P[4]*m_w[0] + 2.0*dt*m_P[2]*m_w[1] + dt*sigma_w2[2];

    m_P[0] = P_new[0];
    m_P[1] = P_new[1];
    m_P[2] = P_new[2];
    m_P[3] = P_new[3];
    m_P[4] = P_new[4];
    m_P[5] = P_new[5];
}

/**
 * @brief Samples the gyro, storing readings in the output parameters
 *
 * @param pRawX Pointer to location to store raw x reading
 * @param pRawY Pointer to location to store raw y reading
 * @param pRawZ Pointer to location to store raw z reading
 */
void SubAttitudeResolver::sampleGyro(short* pRawX, short* pRawY, short* pRawZ)
{
    const unsigned char firstSync = 0xF1;
    const unsigned char secondSync = 0xF5;
    const unsigned char thirdSync = 0xF9;
    bool synched = false;
    unsigned char syncByte;
    char readBuf[6];
    int bytesRead = 0;
    char getGyroCommand = 'G';

    // Read in sync chars
    while(!synched && ros::ok())
    {
        m_serialPort.WriteChar(getGyroCommand);

        bytesRead = m_serialPort.Read(&syncByte, 1, 100);

        if((bytesRead == 1) && (syncByte == firstSync))
        {
            bytesRead = m_serialPort.Read(&syncByte, 1, 10);

            if((bytesRead == 1) && (syncByte == secondSync))
            {
                bytesRead = m_serialPort.Read(&syncByte, 1, 10);

                if((bytesRead == 1) && (syncByte == thirdSync))
                {
                    synched = true;
                }
            }
        }
    }

    bytesRead = 0;

    if(synched)
    {
        while((bytesRead < 6) && (ros::ok()))
        {
            if(m_serialPort.Read(&readBuf[bytesRead], 1, 10) == 1)
            {
                bytesRead++;
            }
        }

        if(bytesRead == 6)
        {
            memcpy(pRawX, &readBuf[0], 2);
            memcpy(pRawY, &readBuf[2], 2);
            memcpy(pRawZ, &readBuf[4], 2);
        }
    }
}

/**
 * @brief Updates omega (converted gyro readings in radians/sec, taking into account bias)
 *
 * @param rawX Raw X gyro reading
 * @param rawY Raw Y gyro reading
 * @param rawZ Raw Z gyro reading
 */
void SubAttitudeResolver::updateOmega(short rawX, short rawY, short rawZ)
{
    m_w[0] = ((rawX - m_biasX) * gyroConversion) * (pi / 180.0);
    m_w[1] = ((rawY - m_biasY) * gyroConversion) * (pi / 180.0);
    m_w[2] = ((rawZ - m_biasZ) * gyroConversion) * (pi / 180.0);
}
