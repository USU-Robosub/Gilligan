/*
 * MipPacket.cpp
 *
 *  Created on: Mar 14, 2013
 *      Author: bholdaway
 */

#include "MipPacket.hpp"
#include "../common/BufferUtils.hpp"

MipPacket::MipPacket()
 :  m_sync1(SYNC_BYTE_1_VALUE),
    m_sync2(SYNC_BYTE_2_VALUE),
    m_descriptorSet(0),
    m_payloadLength(0),
    m_fieldList(),
    m_checksumMsb(0),
    m_checksumLsb(0)
{

}

MipPacket::~MipPacket()
{
  //empty
}

Int32 MipPacket::serialize(UInt8*& rpBuf) const
{
  Int32 size = MIP_PACKET_HEADER_SIZE + PAYLOAD_LENGTH_OFFSET + MIP_PACKET_FOOTER_SIZE;
  rpBuf = new UInt8[size];

  Int32 pos = 0;
  rpBuf[SYNC_1_OFFSET] = SYNC_BYTE_1_VALUE;
  pos++;
  rpBuf[SYNC_2_OFFSET] = SYNC_BYTE_2_VALUE;
  pos++;
  rpBuf[DESCRIPTOR_OFFSET] = m_descriptorSet;
  pos++;
  rpBuf[PAYLOAD_LENGTH_OFFSET] = m_payloadLength;
  pos++;

  for (MipFieldNode* pIt = m_fieldList.getIterator(); pIt != NULL; pIt = pIt->m_pNext)
  {
    if (pIt->m_pData)
    {
      pIt->m_pData->serialize(&rpBuf[pos], size - pos);
      pos += pIt->m_pData->getFieldLength();
    }
  }

  UInt16 checksum = calculateChecksum(rpBuf, pos);
  rpBuf[pos++] = (UInt8)(checksum >> 8);
  rpBuf[pos++] = (UInt8)(checksum & 0xf);

  return pos;
}

UInt8 MipPacket::deserialize(UInt8* pBuf, Int32 size)
{
  Int32 ret = 0;

  m_sync1 = pBuf[SYNC_1_OFFSET];
  m_sync2 = pBuf[SYNC_2_OFFSET];
  m_descriptorSet = pBuf[DESCRIPTOR_OFFSET];
  m_payloadLength = pBuf[PAYLOAD_LENGTH_OFFSET];

  m_fieldList.deserialize(m_descriptorSet, &pBuf[PAYLOAD_OFFSET], size - MIP_PACKET_HEADER_SIZE - MIP_PACKET_FOOTER_SIZE);

  UInt16 checksum = calculateChecksum(pBuf, size - MIP_PACKET_FOOTER_SIZE);
  UInt16 testChecksum = getUInt16(&pBuf[size - MIP_PACKET_FOOTER_SIZE]);
  m_goodChecksum = (checksum == testChecksum);

  return ret;
}

bool MipPacket::isChecksumGood() const
{
  return  m_goodChecksum;
}

UInt8 MipPacket::getDescriptor() const
{
  return m_descriptorSet;
}

void MipPacket::addMipField(MipField* pField)
{
  m_fieldList.add(pField);
}

MipField* MipPacket::getMipField(Int32 pos)
{
  return m_fieldList.getAtPos(pos);
}

UInt32 MipPacket::getMipFieldListSize()
{
  return m_fieldList.getCount();
}

UInt16 MipPacket::calculateChecksum(UInt8* pBuf, Int32 size) const
{
  UInt8 byte1 = 0;
  UInt8 byte2 = 0;
  UInt16 checksum = 0;

  for (Int32 i = 0; i < size; i++)
  {
    byte1 += pBuf[i];
    byte2 += byte1;
  }

  checksum = (UInt16)(byte1 << 8) | byte2;
  return checksum;
}

MipFieldNode* MipPacket::getIterator() const
{
  return m_fieldList.getIterator();
}
