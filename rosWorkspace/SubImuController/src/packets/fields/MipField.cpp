/*
 * MipField.cpp
 *
 *  Created on: Mar 14, 2013
 *      Author: bholdaway
 */

#include <string.h>
#include "MipField.hpp"

MipField::MipField()
 :  m_fieldLength(0),
    m_fieldDescriptor(0)
{
  //empty
}

MipField::~MipField()
{
  //empty
}

UInt8 MipField::getFieldLength()
{
  return m_fieldLength;
}

UInt8 MipField::getFieldDescriptor()
{
  return m_fieldDescriptor;
}

void MipField::setFieldDescriptor(UInt8 descriptor)
{
  m_fieldDescriptor = descriptor;
}

UInt8 MipField::serializeHeader(UInt8* pBuf, UInt8 size)
{
  UInt8 retSize = 0;
  if (size >= MIN_MIP_FIELD_HEADER_SIZE)
  {
    retSize = MIN_MIP_FIELD_HEADER_SIZE;
    pBuf[FIELD_LENGTH_OFFSET] = m_fieldLength;
    pBuf[FIELD_DESCRIPTOR_OFFSET] = m_fieldDescriptor;
  }

  return retSize;
}

UInt8 MipField::deserializeHeader(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;
  if (size >= MIN_MIP_FIELD_HEADER_SIZE)
  {
    m_fieldDescriptor = pBuf[FIELD_DESCRIPTOR_OFFSET];
    m_fieldLength = pBuf[FIELD_LENGTH_OFFSET];
    ret = m_fieldLength;
  }

  return ret;
}

