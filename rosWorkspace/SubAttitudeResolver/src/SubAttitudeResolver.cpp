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

/**
 * @brief Constructor
 */
SubAttitudeResolver::SubAttitudeResolver(std::string devName)
  : m_nodeHandle(),
    m_attitudePublisher(),
    m_magDebugPublisher(),
    m_accelDebugPublisher(),
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

    m_PrAccel[0] = 0.0;
    m_PrAccel[1] = 0.0;
    m_PrAccel[2] = 0.0;
    m_PrAccel[3] = 0.0;
    m_PrAccel[4] = 0.0;
    m_PrAccel[5] = 0.0;

    m_residualAccel[0] = 0.0;
    m_residualAccel[1] = 0.0;
    m_residualAccel[2] = 0.0;

    m_PrMag[0] = 0.0;
    m_PrMag[1] = 0.0;
    m_PrMag[2] = 0.0;
    m_PrMag[3] = 0.0;
    m_PrMag[4] = 0.0;
    m_PrMag[5] = 0.0;

    m_residualMag[0] = 0.0;
    m_residualMag[1] = 0.0;
    m_residualMag[2] = 0.0;

    m_attitudePublisher = m_nodeHandle.advertise<std_msgs::Float64MultiArray>("IMU_Attitude", 100);
    m_magDebugPublisher = m_nodeHandle.advertise<std_msgs::Float64MultiArray>("Mag_Debug", 100);
    m_accelDebugPublisher = m_nodeHandle.advertise<std_msgs::Float64MultiArray>("Accel_Debug", 100);
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
    printf("SubAttitudeResolver: Openning %s\n", m_devName.c_str());
    m_serialPort.Open(m_devName.c_str(), 57600);

    ros::Rate loop_rate(100);
    int count = 0;

    short rawGyroX = 0;
    short rawGyroY = 0;
    short rawGyroZ = 0;
    short accelReadings[3];
    short magReadings[3];
    double magR =   0.00005;
    double accelR = 0.00005;

    calculateGyroBias();
    calculateExpectedAccel();
    calculateExpectedMag();

    while(ros::ok())
    {
        // Sample gyro at 100 Hz
        sampleGyro(&rawGyroX, &rawGyroY, &rawGyroZ);
        updateOmega(rawGyroX, rawGyroY, rawGyroZ);

        // Sample accel at 5 Hz
        if((count % 20) == 0)
        {
            double accelReadingsDoubles[3];

            // Sample accelerometer
            sampleAccel(&accelReadings[0], &accelReadings[1], &accelReadings[2]);

            // Convert to doubles
            accelReadingsDoubles[0] = accelReadings[0];
            accelReadingsDoubles[1] = accelReadings[1];
            accelReadingsDoubles[2] = accelReadings[2];

            kalmanUpdate(m_expectedAccel, accelReadingsDoubles, &accelR, m_PrAccel, m_residualAccel); // Accelerometer readings

            //publishAccelDebug(m_residualAccel, m_PrAccel);
        }

//        // Sample mag at 10 Hz
//        if((count % 10) == 0)
//        {
//            double magReadingsDoubles[3];

//            // Sample magnetometer
//            sampleMag(&magReadings[0], &magReadings[1], &magReadings[2]);

//            // Convert to doubles
//            magReadingsDoubles[0] = magReadings[0];
//            magReadingsDoubles[1] = magReadings[1];
//            magReadingsDoubles[2] = magReadings[2];

//            // Run update with magnetometer readings
//            kalmanUpdate(m_expectedMag, magReadingsDoubles, &magR, m_PrMag, m_residualMag);

//            publishMagDebug(m_residualMag, m_PrMag);
//        }

        kalmanPropagate();

        //double test = (180/pi) * 2 * acos(m_q[3]);
        //printf("%lf\n", test)

        // Publish attitude to system at 20 Hz
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
 * @brief Publishes the Mag_Debug message
 */
void SubAttitudeResolver::publishMagDebug(double* pResidual, double* pPr)
{
    std_msgs::Float64MultiArray magDebugMsg;

    magDebugMsg.data.push_back(pResidual[0]);
    magDebugMsg.data.push_back(pResidual[1]);
    magDebugMsg.data.push_back(pResidual[2]);
    magDebugMsg.data.push_back(pPr[0]);
    magDebugMsg.data.push_back(pPr[3]);
    magDebugMsg.data.push_back(pPr[5]);

    m_magDebugPublisher.publish(magDebugMsg);
}

/**
 * @brief Publishes the Accel_Debug message
 */
void SubAttitudeResolver::publishAccelDebug(double* pResidual, double* pPr)
{
    std_msgs::Float64MultiArray accelDebugMsg;

    accelDebugMsg.data.push_back(pResidual[0]);
    accelDebugMsg.data.push_back(pResidual[1]);
    accelDebugMsg.data.push_back(pResidual[2]);
    accelDebugMsg.data.push_back(pPr[0]);
    accelDebugMsg.data.push_back(pPr[3]);
    accelDebugMsg.data.push_back(pPr[5]);

    m_accelDebugPublisher.publish(accelDebugMsg);
}

/**
 * @brief Calculates the gyro bias on startup, IMU should be still during this calculation
 */
void SubAttitudeResolver::calculateGyroBias(void)
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

    printf("SubAttitudeResolver: Calculated Gyro Bias x: %lf, y: %lf, z: %lf\n", m_biasX, m_biasY, m_biasZ);
    fflush(NULL);
}

/**
 * @brief Calculates the expected acceleration readings on startup, IMU should be still during this calculation
 */
void SubAttitudeResolver::calculateExpectedAccel(void)
{
    short rawX = 0;
    short rawY = 0;
    short rawZ = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    for(int i = 0; i < 64; i++)
    {
        sampleAccel(&rawX, &rawY, &rawZ);

        //printf("Sampled x: %i, y: %i, z: %i\n", rawX, rawY, rawZ);

        sumX += rawX;
        sumY += rawY;
        sumZ += rawZ;

        usleep(1000);
    }

    m_expectedAccel[0] = sumX / 64.0;
    m_expectedAccel[1] = sumY / 64.0;
    m_expectedAccel[2] = sumZ / 64.0;

    printf("SubAttitudeResolver: Calculated Expected Accel x: %lf, y: %lf, z: %lf\n", m_expectedAccel[0], m_expectedAccel[1], m_expectedAccel[2]);
    fflush(NULL);
}

/**
 * @brief Calculates the expected magnetometer readings on startup, IMU should be still during this calculation
 */
void SubAttitudeResolver::calculateExpectedMag(void)
{
    short rawX = 0;
    short rawY = 0;
    short rawZ = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    for(int i = 0; i < 5; i++)
    {
        sampleMag(&rawX, &rawY, &rawZ);

        //printf("Sampled x: %i, y: %i, z: %i\n", rawX, rawY, rawZ);

        sumX += rawX;
        sumY += rawY;
        sumZ += rawZ;

        usleep(100000); // Mag should be sampled faster than 0.1 seconds
    }

    m_expectedMag[0] = sumX / 5.0;
    m_expectedMag[1] = sumY / 5.0;
    m_expectedMag[2] = sumZ / 5.0;

    printf("SubAttitudeResolver: Calculated Expected Mag x: %lf, y: %lf, z: %lf\n", m_expectedMag[0], m_expectedMag[1], m_expectedMag[2]);
}

/**
 * @brief Kalman filter propagation
 *
  @brief Thanks to Bryan Bingham!
 */
void SubAttitudeResolver::kalmanPropagate()
{
    //double sigma_w2[3] = {3.046174197867087e-008,3.046174197867087e-008,3.046174197867087e-008}; // Gyro Uncertainty (0.01 Degrees/second)
    double sigma_w2[3] = {1e-006, 1e-006, 1e-006};
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

void SubAttitudeResolver::kalmanUpdate(double* y_i, double* y_b, double* R, double* Pr, double* residual)
{
    double Ry[3]; // Rotate Inertial Vector to Body
    double detPr; // Determinat of Measurement Covariance
    double invPr[6]; // Inverse of Measurement Covariance
    double K[9]; // Kalman Gain
    double dq[3]; // Quaternion Update
    double q_mag; // Magnitude of Quaternion
    double PHT[9]; // P*H*H' Stored to Save Computations
    double q_new[4]; // Updated Quaternion
    double P_new[5]; // Updated Covariance
    double y_mag; // Magnitude of Measurement Vector

    // Combine Terms to Save Computations
    double q0q0;
    double q1q1;
    double q2q2;
    double q3q3;
    double q0q1;
    double q0q2;
    double q0q3;
    double q1q2;
    double q1q3;
    double q2q3;

    // Normalize Vectors
    y_mag = sqrt(y_i[0]*y_i[0] + y_i[1]*y_i[1] + y_i[2]*y_i[2]);
    y_i[0] = y_i[0]/y_mag;
    y_i[1] = y_i[1]/y_mag;
    y_i[2] = y_i[2]/y_mag;

    y_mag = sqrt(y_b[0]*y_b[0] + y_b[1]*y_b[1] + y_b[2]*y_b[2]);
    y_b[0] = y_b[0]/y_mag;
    y_b[1] = y_b[1]/y_mag;
    y_b[2] = y_b[2]/y_mag;

    // Combine Terms to Save Computations
    q0q0 = m_q[0]*m_q[0];
    q1q1 = m_q[1]*m_q[1];
    q2q2 = m_q[2]*m_q[2];
    q3q3 = m_q[3]*m_q[3];
    q0q1 = 2.0*m_q[0]*m_q[1];
    q0q2 = 2.0*m_q[0]*m_q[2];
    q0q3 = 2.0*m_q[0]*m_q[3];
    q1q2 = 2.0*m_q[1]*m_q[2];
    q1q3 = 2.0*m_q[1]*m_q[3];
    q2q3 = 2.0*m_q[2]*m_q[3];

    // Rotation Predicted Measurement Vector to Body
    Ry[0] = y_i[0]*(q0q0 - q1q1 - q2q2 + q3q3) + y_i[1]*(q0q1 + q2q3) + y_i[2]*(q0q2 - q1q3);
    Ry[1] = y_i[0]*(q0q1 - q2q3) - y_i[1]*(q0q0 - q1q1 + q2q2 - q3q3) + y_i[2]*(q0q3 + q1q2);
    Ry[2] = y_i[0]*(q0q2 + q1q3) - y_i[2]*(q0q0 + q1q1 - q2q2 - q3q3) - y_i[1]*(q0q3 - q1q2);

    // Calculate P*H'
    PHT[0] = m_P[2]*Ry[1] - m_P[1]*Ry[2];
    PHT[1] = m_P[0]*Ry[2] - m_P[2]*Ry[0];
    PHT[2] = m_P[1]*Ry[0] - m_P[0]*Ry[1];
    PHT[3] = m_P[4]*Ry[1] - m_P[3]*Ry[2];
    PHT[4] = m_P[1]*Ry[2] - m_P[4]*Ry[0];
    PHT[5] = m_P[3]*Ry[0] - m_P[1]*Ry[1];
    PHT[6] = m_P[5]*Ry[1] - m_P[4]*Ry[2];
    PHT[7] = m_P[2]*Ry[2] - m_P[5]*Ry[0];
    PHT[8] = m_P[4]*Ry[0] - m_P[2]*Ry[1];

    // Calculate Measurement Covariance
    Pr[0] = *R - PHT[3]*Ry[2] + PHT[6]*Ry[1];
    Pr[1] = PHT[7]*Ry[1] - PHT[4]*Ry[2];
    Pr[2] = PHT[8]*Ry[1] - PHT[5]*Ry[2];
    Pr[3] = *R + PHT[1]*Ry[2] - PHT[7]*Ry[0];
    Pr[4] = PHT[2]*Ry[2] - PHT[8]*Ry[0];
    Pr[5] = *R - PHT[2]*Ry[1] + PHT[5]*Ry[0];

    // Calculate Residual
    residual[0] = y_b[0] - Ry[0];
    residual[1] = y_b[1] - Ry[1];
    residual[2] = y_b[2] - Ry[2];

    // Calculate Determinant of Measurement Covariance
    detPr = - Pr[5]*Pr[1]*Pr[1] + 2*Pr[1]*Pr[2]*Pr[4] - Pr[3]*Pr[2]*Pr[2] - Pr[0]*Pr[4]*Pr[4] + Pr[0]*Pr[3]*Pr[5];

    // Calculate Inverse of Measurement Covariance
    invPr[0] = (Pr[3]*Pr[5] - Pr[4]*Pr[4])/detPr;
    invPr[1] = (Pr[2]*Pr[4] - Pr[1]*Pr[5])/detPr;
    invPr[2] = (Pr[1]*Pr[4] - Pr[2]*Pr[3])/detPr;
    invPr[3] = (Pr[0]*Pr[5] - Pr[2]*Pr[2])/detPr;
    invPr[4] = (Pr[1]*Pr[2] - Pr[0]*Pr[4])/detPr;
    invPr[5] = (Pr[0]*Pr[3] - Pr[1]*Pr[1])/detPr;

    // Calculate Kalman Gain
    K[0] = PHT[0]*invPr[0] + PHT[1]*invPr[1] + PHT[2]*invPr[2];
    K[1] = PHT[0]*invPr[1] + PHT[1]*invPr[3] + PHT[2]*invPr[4];
    K[2] = PHT[0]*invPr[2] + PHT[1]*invPr[4] + PHT[2]*invPr[5];
    K[3] = PHT[3]*invPr[0] + PHT[4]*invPr[1] + PHT[5]*invPr[2];
    K[4] = PHT[3]*invPr[1] + PHT[4]*invPr[3] + PHT[5]*invPr[4];
    K[5] = PHT[3]*invPr[2] + PHT[4]*invPr[4] + PHT[5]*invPr[5];
    K[6] = PHT[6]*invPr[0] + PHT[7]*invPr[1] + PHT[8]*invPr[2];
    K[7] = PHT[6]*invPr[1] + PHT[7]*invPr[3] + PHT[8]*invPr[4];
    K[8] = PHT[6]*invPr[2] + PHT[7]*invPr[4] + PHT[8]*invPr[5];

    // Update Covariance
    P_new[0] = m_P[0]*(K[2]*Ry[1] - K[1]*Ry[2] + 1.0) + m_P[1]*(K[0]*Ry[2] - K[2]*Ry[0]) - m_P[2]*(K[0]*Ry[1] - K[1]*Ry[0]);
    P_new[1] = m_P[1]*(K[2]*Ry[1] - K[1]*Ry[2] + 1.0) - m_P[4]*(K[0]*Ry[1] - K[1]*Ry[0]) + m_P[3]*(K[0]*Ry[2] - K[2]*Ry[0]);
    P_new[2] = m_P[2]*(K[2]*Ry[1] - K[1]*Ry[2] + 1.0) - m_P[5]*(K[0]*Ry[1] - K[1]*Ry[0]) + m_P[4]*(K[0]*Ry[2] - K[2]*Ry[0]);
    P_new[3] = m_P[3]*(K[3]*Ry[2] - K[5]*Ry[0] + 1.0) - m_P[4]*(K[3]*Ry[1] - K[4]*Ry[0]) - m_P[1]*(K[4]*Ry[2] - K[5]*Ry[1]);
    P_new[4] = m_P[4]*(K[3]*Ry[2] - K[5]*Ry[0] + 1.0) - m_P[5]*(K[3]*Ry[1] - K[4]*Ry[0]) - m_P[2]*(K[4]*Ry[2] - K[5]*Ry[1]);
    P_new[5] = m_P[5]*(K[7]*Ry[0] - K[6]*Ry[1] + 1.0) - m_P[2]*(K[7]*Ry[2] - K[8]*Ry[1]) + m_P[4]*(K[6]*Ry[2] - K[8]*Ry[0]);

    // Calculate Quaternion Update
    dq[0] = (K[0]*residual[0] + K[1]*residual[1] + K[2]*residual[2])*0.5;
    dq[1] = (K[3]*residual[0] + K[4]*residual[1] + K[5]*residual[2])*0.5;
    dq[2] = (K[6]*residual[0] + K[7]*residual[1] + K[8]*residual[2])*0.5;

    // Update Quaternion
    q_new[0] = m_q[0] + dq[2]*m_q[1] - dq[1]*m_q[2] + dq[0]*m_q[3];
    q_new[1] = m_q[1] - dq[2]*m_q[0] + dq[0]*m_q[2] + dq[1]*m_q[3];
    q_new[2] = m_q[2] + dq[1]*m_q[0] - dq[0]*m_q[1] + dq[2]*m_q[3];
    q_new[3] = m_q[3] - dq[0]*m_q[0] - dq[1]*m_q[1] - dq[2]*m_q[2];

    // Calculate Magnitude of Quaternion
    q_mag = sqrt( q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3] );

    // Normalize Quaternion
    q_new[0] = q_new[0]/q_mag;
    q_new[1] = q_new[1]/q_mag;
    q_new[2] = q_new[2]/q_mag;
    q_new[3] = q_new[3]/q_mag;

    m_P[0] = P_new[0];
    m_P[1] = P_new[1];
    m_P[2] = P_new[2];
    m_P[3] = P_new[3];
    m_P[4] = P_new[4];
    m_P[5] = P_new[5];

    m_q[0] = q_new[0];
    m_q[1] = q_new[1];
    m_q[2] = q_new[2];
    m_q[3] = q_new[3];
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
    bool synched = false;
    char readBuf[6];
    int bytesRead = 0;
    char getGyroCommand = 'G';

    synched = syncSerial(getGyroCommand);

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
 * @brief Samples the accelerometer, storing readings in the output parameters
 *
 * @param pRawX Pointer to location to store raw x reading
 * @param pRawY Pointer to location to store raw y reading
 * @param pRawZ Pointer to location to store raw z reading
 */
void SubAttitudeResolver::sampleAccel(short* pRawX, short* pRawY, short* pRawZ)
{
    bool synched = false;
    char readBuf[6];
    int bytesRead = 0;
    char getAccelCommand = 'A';

    synched = syncSerial(getAccelCommand);

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
 * @brief Samples the magnetometer, storing readings in the output parameters
 *
 * @param pRawX Pointer to location to store raw x reading
 * @param pRawY Pointer to location to store raw y reading
 * @param pRawZ Pointer to location to store raw z reading
 */
void SubAttitudeResolver::sampleMag(short* pRawX, short* pRawY, short* pRawZ)
{
    bool synched = false;
    char readBuf[6];
    int bytesRead = 0;
    char getMagCommand = 'M';

    synched = syncSerial(getMagCommand);

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

bool SubAttitudeResolver::syncSerial(char command)
{
    const unsigned char firstSync = 0xF1;
    const unsigned char secondSync = 0xF5;
    const unsigned char thirdSync = 0xF9;
    unsigned char syncByte;
    bool synched = false;
    int bytesRead = 0;

    // Read in sync chars
    while(!synched && ros::ok())
    {
        m_serialPort.WriteChar(command);

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

    return synched;
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
