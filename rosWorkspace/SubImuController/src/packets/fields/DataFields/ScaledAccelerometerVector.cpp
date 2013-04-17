/*
 * ScaledAccelerometerVector.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include <sstream>
#include "ScaledAccelerometerVector.hpp"
#include "../../../common/BufferUtils.hpp"

ScaledAccelerometerVector::ScaledAccelerometerVector()
  : DataField(),
    m_x(0.0),
    m_y(0.0),
    m_z(0.0)
{
  // empty
}

ScaledAccelerometerVector::~ScaledAccelerometerVector()
{
  // empty
}

UInt8 ScaledAccelerometerVector::serialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= SCALED_ACCELEROMETER_VECTOR_STATIC_SIZE)
  {
    serializeHeader(pBuf, size);
    putFloat32(&pBuf[SCALED_ACCELEROMETER_VECTOR_X_OFFSET], m_x);
    putFloat32(&pBuf[SCALED_ACCELEROMETER_VECTOR_Y_OFFSET], m_y);
    putFloat32(&pBuf[SCALED_ACCELEROMETER_VECTOR_Z_OFFSET], m_z);

    ret = m_fieldLength;
  }

  return ret;
}

UInt8 ScaledAccelerometerVector::deserialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    deserializeHeader(pBuf, size);

    if (size >= m_fieldLength && m_fieldLength == SCALED_ACCELEROMETER_VECTOR_STATIC_SIZE)
    {
      m_x = getFloat32(&pBuf[SCALED_ACCELEROMETER_VECTOR_X_OFFSET]);
      m_y = getFloat32(&pBuf[SCALED_ACCELEROMETER_VECTOR_Y_OFFSET]);
      m_z = getFloat32(&pBuf[SCALED_ACCELEROMETER_VECTOR_Z_OFFSET]);

      ret = m_fieldLength;
    }
  }

  return ret;
}

Float32 ScaledAccelerometerVector::getX()
{
  return m_x;
}

Float32 ScaledAccelerometerVector::getY()
{
  return m_y;
}

Float32 ScaledAccelerometerVector::getZ()
{
  return m_z;
}

void ScaledAccelerometerVector::setX(Float32 x)
{
  m_x = x;
}

void ScaledAccelerometerVector::setY(Float32 y)
{
  m_y = y;
}

void ScaledAccelerometerVector::setZ(Float32 z)
{
  m_z = z;
}

std::string ScaledAccelerometerVector::toString()
{
  std::ostringstream ret;
  ret << "ScaledAccelerometerVector: X(" << m_x << "), Y(" << m_y << "), Z(" << m_z << ")";

  return ret.str();
}
