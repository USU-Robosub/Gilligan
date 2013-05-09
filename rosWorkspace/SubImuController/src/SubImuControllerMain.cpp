/*
 * SubImuControllerMain.cpp
 *
 *  Created on: Apr 14, 2013
 *      Author: subcrew
 */




#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "common/SerialInterface.hpp"
#include "common/share.hpp"
#include "packets/MipPacket.hpp"
#include "packets/fields/DataFields/DataField.hpp"
#include "packets/fields/DataFields/EulerAngles.hpp"
#include "packets/fields/DataFields/ScaledAccelerometerVector.hpp"
#include "packets/fields/DataFields/ScaledGyroVector.hpp"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

int waitForGoodHeader(SerialInterface& sp, UInt8* pBuf);
void catchSig(int sig);

bool m_running = true;

int main(int argc, char** argv)
{
  UInt32 baudRate = 115200;
  Int32 maxSize = 115200;
  UInt8 aBuf[maxSize];
  SerialInterface sp("/dev/controller_Imu", baudRate);
  signal(SIGINT, catchSig);

  ros::init(argc, argv, "SubImuController");
  ros::NodeHandle nh;

  ros::Publisher imuAttitudePub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Attitude", 1000);
  ros::Publisher imuAccelPub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Accel_Debug", 1000);
  ros::Publisher imuGyroPub = nh.advertise<std_msgs::Float32MultiArray>("IMU_Gyro_Debug", 1000);

  ros::Rate loop_rate(1);

  //TODO redo the whole reading and writing
  while (ros::ok() && m_running)
  {
    int size = waitForGoodHeader(sp, aBuf);

    if (size > 0)
    {
      UInt8 expectedSize = MipPacket::MIP_PACKET_HEADER_SIZE + MipPacket::MIP_PACKET_FOOTER_SIZE + aBuf[MipPacket::PAYLOAD_LENGTH_OFFSET];

      while (size < expectedSize && ros::ok() && m_running)
      {
        int tmpSize = sp.recv(&aBuf[size],1);

        if (tmpSize > 0)
        {
          size += tmpSize;
        }
        else if (tmpSize < 0)
        {
          printf("Error\n");
        }
      }

      if (size == expectedSize)
      {

        if (aBuf[MipPacket::DESCRIPTOR_OFFSET] == 0x80)
        {
          MipPacket packet;
          packet.deserialize(aBuf, size);

          for (MipFieldNode* pIt = packet.getIterator(); pIt != NULL; pIt = pIt->m_pNext)
          {
             if (pIt->m_pData->getFieldDescriptor() == DataField::DATA_FIELD_EULER_ANGLES_SET)
             {
               EulerAngles* pData = static_cast<EulerAngles*>(pIt->m_pData);

               std_msgs::Float32MultiArray rawMsg;
               rawMsg.data.push_back(pData->getYaw());
               rawMsg.data.push_back(pData->getPitch());
               rawMsg.data.push_back(pData->getRoll());
               imuAttitudePub.publish(rawMsg);
             }
             else if (pIt->m_pData->getFieldDescriptor() == DataField::DATA_FIELD_SCALED_ACCELEROMETER_VECTOR_SET)
             {
               ScaledAccelerometerVector* pData = static_cast<ScaledAccelerometerVector*>(pIt->m_pData);

               std_msgs::Float32MultiArray rawMsg;
               rawMsg.data.push_back(pData->getX());
               rawMsg.data.push_back(pData->getY());
               rawMsg.data.push_back(pData->getZ());
               imuAccelPub.publish(rawMsg);
             }
             else if (pIt->m_pData->getFieldDescriptor() == DataField::DATA_FIELD_SCALED_GYRO_VECTOR_SET)
             {
               ScaledGyroVector* pData = static_cast<ScaledGyroVector*>(pIt->m_pData);

               std_msgs::Float32MultiArray rawMsg;
               rawMsg.data.push_back(pData->getX());
               rawMsg.data.push_back(pData->getY());
               rawMsg.data.push_back(pData->getZ());
               imuGyroPub.publish(rawMsg);
             }
          }
        }
      }
      else
      {
        printf("bad packet\n");
      }
    }
  }
}

int waitForGoodHeader(SerialInterface& sp, UInt8* pBuf)
{
  while (ros::ok() && m_running)
  {
    int size = 0;
    int tmp = 0;
    while ((tmp = sp.recv(&pBuf[0], 1)) <= 0)
    {
      if (tmp < 0)
      {
        printf("Error\n");
      }
    }

    if (pBuf[size] == MipPacket::SYNC_BYTE_1_VALUE)
    {
      size++;

      while ((tmp = sp.recv(&pBuf[size], 1)) <= 0 && ros::ok() && m_running)
      {
        if (tmp < 0)
        {
          printf("Error\n");
        }
      }

      if (pBuf[size] == MipPacket::SYNC_BYTE_2_VALUE)
      {
        size++;

        while (size < MipPacket::MIP_PACKET_HEADER_SIZE + MipPacket::MIP_PACKET_FOOTER_SIZE && ros::ok() && m_running)
        {
          int tmpSize = sp.recv(&pBuf[size], MipPacket::MIP_PACKET_HEADER_SIZE + MipPacket::MIP_PACKET_FOOTER_SIZE - size);

          if (tmpSize > 0)
          {
            size += tmpSize;
          }
          else if (tmpSize < 0)
          {
            printf("Error\n");
          }
        }
        return size;
      }
    }
    else
    {
      printf("Bad sync byte of %x\n", pBuf[0]);
    }
  }
  return 0;
}

void catchSig(int sig)
{
  m_running = false;
}
