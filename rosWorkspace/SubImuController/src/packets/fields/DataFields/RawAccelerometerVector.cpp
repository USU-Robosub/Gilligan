/*
 * RawAccelerometerVector.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include <sstream>
#include "RawAccelerometerVector.hpp"
#include "../../../common/BufferUtils.hpp"

RawAccelerometerVector::RawAccelerometerVector()
  : DataField(),
    m_x(0.0),
    m_y(0.0),
    m_z(0.0)
{
  // empty
}

RawAccelerometerVector::~RawAccelerometerVector()
{
  // empty
}

UInt8 RawAccelerometerVector::serialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= RAW_ACCELEROMETER_VECTOR_STATIC_SIZE)
  {
    serializeHeader(pBuf, size);
    putFloat32(&pBuf[RAW_ACCELEROMETER_VECTOR_X_OFFSET], m_x);
    putFloat32(&pBuf[RAW_ACCELEROMETER_VECTOR_Y_OFFSET], m_y);
    putFloat32(&pBuf[RAW_ACCELEROMETER_VECTOR_Z_OFFSET], m_z);

    ret = m_fieldLength;
  }

  return ret;
}

UInt8 RawAccelerometerVector::deserialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    deserializeHeader(pBuf, size);

    if (size >= m_fieldLength && m_fieldLength == RAW_ACCELEROMETER_VECTOR_STATIC_SIZE)
    {
      m_x = getFloat32(&pBuf[RAW_ACCELEROMETER_VECTOR_X_OFFSET]);
      m_y = getFloat32(&pBuf[RAW_ACCELEROMETER_VECTOR_Y_OFFSET]);
      m_z = getFloat32(&pBuf[RAW_ACCELEROMETER_VECTOR_Z_OFFSET]);

      ret = m_fieldLength;
    }
  }

  return ret;
}

/**
 * @brief Get the X raw accelerometer vector
 *
 * @returns the X raw accelerometer vector
 */
Float32 RawAccelerometerVector::getX()
{
  return m_x;
}

/**
 * @brief Get the Y raw accelerometer vector
 *
 * @returns The Y raw accelerometer vector
 */
Float32 RawAccelerometerVector::getY()
{
  return m_y;
}

/**
 * @brief Get the Z raw accelerometer vector
 *
 * @returns The Z raw accelerometer vector
 */
Float32 RawAccelerometerVector::getZ()
{
  return m_z;
}

/**
 * @brief Set the X raw accelerometer vector
 *
 * @param x The X raw accelerometer vector
 */
void RawAccelerometerVector::setX(Float32 x)
{
  m_x = x;
}

/**
 * @brief Get the Y raw accelerometer vector
 *
 * @param y The Y raw accelerometer vector
 */
void RawAccelerometerVector::setY(Float32 y)
{
  m_y = y;
}

/**
 * @brief Get the Z raw accelerometer vector
 *
 * @param z The Z raw accelerometer vector
 */
void RawAccelerometerVector::setZ(Float32 z)
{
  m_z = z;
}

std::string RawAccelerometerVector::toString()
{
  std::ostringstream ret;
  ret << "RawAccelerometerVector: X(" << m_x << "), Y(" << m_y << "), Z(" << m_z << ")";

  return ret.str();
}
