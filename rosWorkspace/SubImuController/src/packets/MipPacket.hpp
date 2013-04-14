/*
 * MipPacket.hpp
 *
 *  Created on: Mar 14, 2013
 *      Author: bholdaway
 */

#ifndef MIPPACKET_HPP_
#define MIPPACKET_HPP_

#include "../common/share.hpp"
#include "MipFieldList.hpp"

class MipPacket {
public:
  MipPacket();
  ~MipPacket();

  /**
   * @brief Offsets
   */
  enum
  {
    SYNC_1_OFFSET = 0,
    SYNC_2_OFFSET = 1,
    DESCRIPTOR_OFFSET = 2,
    PAYLOAD_LENGTH_OFFSET = 3,
    PAYLOAD_OFFSET = 4
  };

  /**
   * @brief default values
   */
  enum
  {
    SYNC_BYTE_1_VALUE = 0x75,
    SYNC_BYTE_2_VALUE = 0x65,

    MIP_PACKET_HEADER_SIZE = 4,
    MIP_PACKET_FOOTER_SIZE = 2
  };

  enum DescriptorSet
  {
    DESCRIPTOR_SET_COMMANDS = 0x01,
    DESCRIPTOR_SET_3DM_COMMAND = 0x0c,
    DESCRIPTOR_SET_SYSTEM_COMMAND = 0x7f,
    DESCRIPTOR_SET_IMU_DATA = 0x80
  };

  Int32 serialize(UInt8*& rpBuf) const;
  UInt8 deserialize(UInt8* pBuf, Int32 size);

  bool isChecksumGood() const;
  UInt8 getDescriptor() const;
  UInt32 getMipFieldListSize();
  void addMipField(MipField* pField); //force each child class to expose their own
  MipField* getMipField(Int32 pos);
  MipFieldNode* getIterator() const;
  UInt16 calculateChecksum(UInt8* pBuf, Int32 size) const;

protected:
  UInt8 m_sync1;
  UInt8 m_sync2;
  UInt8 m_descriptorSet;
  UInt8 m_payloadLength;
  MipFieldList m_fieldList;
  UInt8 m_checksumMsb;
  UInt8 m_checksumLsb;
  bool m_goodChecksum;


};

#endif /* MIPPACKET_HPP_ */
