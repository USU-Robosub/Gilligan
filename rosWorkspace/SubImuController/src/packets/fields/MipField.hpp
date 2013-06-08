/*
 * MipField.hpp
 *
 *  Created on: Mar 14, 2013
 *      Author: bholdaway
 */

#ifndef MIPFIELD_HPP_
#define MIPFIELD_HPP_

#include <stdlib.h>
#include <string>

#include "../../common/share.hpp"

class MipField
{
  public:
    MipField();
    ~MipField();

    UInt8 getFieldLength();
    UInt8 getFieldDescriptor();
    void setFieldDescriptor(UInt8 descriptor);

    virtual UInt8 serialize(UInt8* pBuf, UInt8 size) = 0;
    UInt8 serializeHeader(UInt8* pBuf, UInt8 size);
    virtual UInt8 deserialize(UInt8* pBuf, UInt8 size) = 0;
    UInt8 deserializeHeader(UInt8* pBuf, UInt8 size);
    virtual std::string toString() = 0;

    enum
    {
      FIELD_LENGTH_OFFSET = 0,
      FIELD_DESCRIPTOR_OFFSET = 1,
      FIELD_PAYLOAD_OFFSET = 2
    };

    enum
    {
      MIN_MIP_FIELD_HEADER_SIZE = 2
    };

    enum BaseCommands
    {
      MIP_PING_FIELD_DESCRIPTOR = 0x1, //Ping command, reply is 0xf1
      MIP_SET_IDLE_COMMAND_FIELD_DESCRIPTOR = 0x2, //Set the imu to idle, reply 0xf1
      MIP_GET_DEVICE_INFO_FIELD_DESCRIPTOR = 0x3, //request the device info, reply 0xf1 then 0x81
      MIP_GET_DEVICE_DESCRIPTOR_SETS_FIELD_DESCRIPTOR = 0x4, //get the device descriptor sets, reply 0xf1 then 0x82
      MIP_SELF_TEST_FIELD_DESCRIPTOR = 0x5, //perform a self test, reply 0xf1 then 0x83
      MIP_RESUME_COMMAND_FIELD_DESCRIPTOR = 0x6, //Set the imu to resume, reply 0xf1
      MIP_DATA_FIELD_DESCRIPTOR = 0x4,
      MIP_MESSAGE_FORMAT_COMMAND_FIELD_DESCRIPTOR = 0x08,

      MIP_STREAM_COMMAND_FIELD_DESCRIPTOR = 0x11, //used to enable/disable IMU/AHRS stream

      MIP_GPS_TIME_CORRELATION_UPDATE_FIELD_DESCRIPTOR = 0x72, //updates the gx3 internal gps time correlation registers, reply 0x84 then 0x85
      MIP_DEVICE_RESET_FIELD_DESCRIPTOR = 0x7E, //reset the device, reply 0xf1
      MIP_REPLY_FIELD_DESCRIPTOR = 0xf1
    };

  protected:
    UInt8 m_fieldLength;
    UInt8 m_fieldDescriptor;
};

#endif /* MIPFIELD_HPP_ */
