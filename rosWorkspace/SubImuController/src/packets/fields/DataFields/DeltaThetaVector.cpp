/*
 * DeltaThetaVector.cpp
 *
 *  Created on: Apr 16, 2013
 *      Author: bholdaway
 */

#include <sstream>
#include "DeltaThetaVector.hpp"
#include "../../../common/BufferUtils.hpp"

DeltaThetaVector::DeltaThetaVector()
  : DataField(),
    m_x(0.0),
    m_y(0.0),
    m_z(0.0)
{
  //empty
}

DeltaThetaVector::~DeltaThetaVector()
{
  //empty
}


UInt8 DeltaThetaVector::serialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= DELTA_THETA_VECTOR_STATIC_SIZE)
  {
    serializeHeader(pBuf, size);
    putFloat32(&pBuf[DELTA_THETA_VECTOR_X_OFFSET], m_x);
    putFloat32(&pBuf[DELTA_THETA_VECTOR_Y_OFFSET], m_y);
    putFloat32(&pBuf[DELTA_THETA_VECTOR_Z_OFFSET], m_z);

    ret = m_fieldLength;
  }

  return ret;
}

UInt8 DeltaThetaVector::deserialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    deserializeHeader(pBuf, size);

    if (size >= m_fieldLength && m_fieldLength == DELTA_THETA_VECTOR_STATIC_SIZE)
    {
      m_x = getFloat32(&pBuf[DELTA_THETA_VECTOR_X_OFFSET]);
      m_y = getFloat32(&pBuf[DELTA_THETA_VECTOR_Y_OFFSET]);
      m_z = getFloat32(&pBuf[DELTA_THETA_VECTOR_Z_OFFSET]);

      ret = m_fieldLength;
    }
  }

  return ret;
}

Float32 DeltaThetaVector::getX()
{
  return m_x;
}

Float32 DeltaThetaVector::getY()
{
  return m_y;
}

Float32 DeltaThetaVector::getZ()
{
  return m_z;
}

void DeltaThetaVector::setX(Float32 x)
{
  m_x = x;
}

void DeltaThetaVector::setY(Float32 y)
{
  m_y = y;
}

void DeltaThetaVector::setZ(Float32 z)
{
  m_z = z;
}

std::string DeltaThetaVector::toString()
{
  std::ostringstream ret;
  ret << "DeltaThetaVector: X(" << m_x << "), Y(" << m_y << "), Z(" << m_z << ")";

  return ret.str();
}
