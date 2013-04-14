/*
 * ScaledGyroVector.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include <sstream>
#include "ScaledGyroVector.hpp"
#include "../../../common/BufferUtils.hpp"

ScaledGyroVector::ScaledGyroVector()
{
  // TODO Auto-generated constructor stub

}

ScaledGyroVector::~ScaledGyroVector()
{
  // TODO Auto-generated destructor stub
}

UInt8 ScaledGyroVector::serialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= SCALED_GYRO_VECTOR_STATIC_SIZE)
  {
    serializeHeader(pBuf, size);
    putFloat32(&pBuf[SCALED_GYRO_VECTOR_X_OFFSET], m_x);
    putFloat32(&pBuf[SCALED_GYRO_VECTOR_Y_OFFSET], m_y);
    putFloat32(&pBuf[SCALED_GYRO_VECTOR_Z_OFFSET], m_z);

    ret = m_fieldLength;
  }

  return ret;
}

UInt8 ScaledGyroVector::deserialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    deserializeHeader(pBuf, size);

    if (size >= m_fieldLength && m_fieldLength == SCALED_GYRO_VECTOR_STATIC_SIZE)
    {
      m_x = getFloat32(&pBuf[SCALED_GYRO_VECTOR_X_OFFSET]);
      m_y = getFloat32(&pBuf[SCALED_GYRO_VECTOR_Y_OFFSET]);
      m_z = getFloat32(&pBuf[SCALED_GYRO_VECTOR_Z_OFFSET]);

      ret = m_fieldLength;
    }
  }

  return ret;
}

Float32 ScaledGyroVector::getX()
{
  return m_x;
}

Float32 ScaledGyroVector::getY()
{
  return m_y;
}

Float32 ScaledGyroVector::getZ()
{
  return m_z;
}

void ScaledGyroVector::setX(Float32 x)
{
  m_x = x;
}

void ScaledGyroVector::setY(Float32 y)
{
  m_y = y;
}

void ScaledGyroVector::setZ(Float32 z)
{
  m_z = z;
}

std::string ScaledGyroVector::toString()
{
  std::ostringstream ret;
  ret << "ScaledGyroVector: X(" << m_x << "), Y(" << m_y << "), Z(" << m_z << ")";

  return ret.str();
}
