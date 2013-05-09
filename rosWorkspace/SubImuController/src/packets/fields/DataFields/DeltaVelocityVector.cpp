/*
 * DeltaVelocityVector.cpp
 *
 *  Created on: Apr 16, 2013
 *      Author: bholdaway
 */

#include <sstream>
#include "DeltaVelocityVector.hpp"
#include "../../../common/BufferUtils.hpp"

DeltaVelocityVector::DeltaVelocityVector()
  : DataField(),
    m_x(0.0),
    m_y(0.0),
    m_z(0.0)
{
  //empty
}

DeltaVelocityVector::~DeltaVelocityVector()
{
  //empty
}


UInt8 DeltaVelocityVector::serialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= DELTA_VELOCITY_VECTOR_STATIC_SIZE)
  {
    serializeHeader(pBuf, size);
    putFloat32(&pBuf[DELTA_VELOCITY_VECTOR_X_OFFSET], m_x);
    putFloat32(&pBuf[DELTA_VELOCITY_VECTOR_Y_OFFSET], m_y);
    putFloat32(&pBuf[DELTA_VELOCITY_VECTOR_Z_OFFSET], m_z);

    ret = m_fieldLength;
  }

  return ret;
}

UInt8 DeltaVelocityVector::deserialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    deserializeHeader(pBuf, size);

    if (size >= m_fieldLength && m_fieldLength == DELTA_VELOCITY_VECTOR_STATIC_SIZE)
    {
      m_x = getFloat32(&pBuf[DELTA_VELOCITY_VECTOR_X_OFFSET]);
      m_y = getFloat32(&pBuf[DELTA_VELOCITY_VECTOR_Y_OFFSET]);
      m_z = getFloat32(&pBuf[DELTA_VELOCITY_VECTOR_Z_OFFSET]);

      ret = m_fieldLength;
    }
  }

  return ret;
}

/**
 * @brief Get the X Delta Velocity
 *
 * @return The X delta velocity in g's a second
 */
Float32 DeltaVelocityVector::getX()
{
  return m_x;
}

/**
 * @brief Get the Y Delta Velocity
 *
 * @return The Y delta velocity in g's a second
 */
Float32 DeltaVelocityVector::getY()
{
  return m_y;
}

/**
 * @brief Get the Z Delta Velocity
 *
 * @return The Z delta velocity in g's a second
 */
Float32 DeltaVelocityVector::getZ()
{
  return m_z;
}

/**
 * @brief Set the X Delta Velocity
 *
 * @param x The X delta velocity in g's a second
 */
void DeltaVelocityVector::setX(Float32 x)
{
  m_x = x;
}

/**
 * @brief Set the Y Delta Velocity
 *
 * @param y The Y delta velocity in g's a second
 */
void DeltaVelocityVector::setY(Float32 y)
{
  m_y = y;
}

/**
 * @brief Set the Z Delta Velocity
 *
 * @param z The Z delta velocity in g's a second
 */
void DeltaVelocityVector::setZ(Float32 z)
{
  m_z = z;
}

std::string DeltaVelocityVector::toString()
{
  std::ostringstream ret;
  ret << "DeltaVelocityVector: X(" << m_x << "), Y(" << m_y << "), Z(" << m_z << ")";

  return ret.str();
}
