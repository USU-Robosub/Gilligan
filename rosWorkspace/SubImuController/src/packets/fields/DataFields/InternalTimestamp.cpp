/*
 * InternalTimestamp.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include <sstream>
#include "InternalTimestamp.hpp"
#include "../../../common/BufferUtils.hpp"

InternalTimestamp::InternalTimestamp()
  : DataField(),
    m_timestamp(0)
{
  //empty
}

InternalTimestamp::~InternalTimestamp()
{
  //empty
}

UInt8 InternalTimestamp::serialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= INTERNAL_TIMESTAMP_STATIC_SIZE)
  {
    serializeHeader(pBuf, size);
    putFloat32(&pBuf[INTERNAL_TIMESTAMP_TIMESTAMP_OFFSET], m_timestamp);

    ret = m_fieldLength;
  }

  return ret;
}

UInt8 InternalTimestamp::deserialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    deserializeHeader(pBuf, size);

    if (size >= m_fieldLength && m_fieldLength == INTERNAL_TIMESTAMP_STATIC_SIZE)
    {
      m_timestamp = getUInt32(&pBuf[INTERNAL_TIMESTAMP_TIMESTAMP_OFFSET]);

      ret = m_fieldLength;
    }
  }

  return ret;
}

/**
 * @brief Get the timestamp
 *
 * @return The time in 16 microsecond increments
 */
UInt32 InternalTimestamp::getTimestamp()
{
  return m_timestamp;
}

/**
 * @brief Get the timestamp in micro seconds
 *
 * @return The time in micro microseconds
 */
UInt32 InternalTimestamp::getTimestampAsMicroSeconds()
{
  return m_timestamp*16;
}

void InternalTimestamp::setTimestamp(UInt32 timestamp)
{
  m_timestamp = timestamp;
}

std::string InternalTimestamp::toString()
{
  std::ostringstream ret;
  ret << "InternalTimestamp: Timestamp(" << m_timestamp << ")";

  return ret.str();
}
