/*
 * EulerAngles.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include <sstream>
#include "EulerAngles.hpp"
#include "../../../common/BufferUtils.hpp"

EulerAngles::EulerAngles()
  : DataField(),
    m_yaw(0.0),
    m_pitch(0.0),
    m_roll(0.0)
{
  // Empty
}

EulerAngles::~EulerAngles()
{
  // Empty
}

UInt8 EulerAngles::serialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= EULER_ANGLES_STATIC_SIZE)
  {
    serializeHeader(pBuf, size);
    putFloat32(&pBuf[EULER_ANGLES_YAW_OFFSET], m_yaw);
    putFloat32(&pBuf[EULER_ANGLES_PITCH_OFFSET], m_pitch);
    putFloat32(&pBuf[EULER_ANGLES_ROLL_OFFSET], m_roll);

    ret = m_fieldLength;
  }

  return ret;
}

UInt8 EulerAngles::deserialize(UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    deserializeHeader(pBuf, size);

    if (size >= m_fieldLength && m_fieldLength == EULER_ANGLES_STATIC_SIZE)
    {
      m_yaw = getFloat32(&pBuf[EULER_ANGLES_YAW_OFFSET]);
      m_pitch = getFloat32(&pBuf[EULER_ANGLES_PITCH_OFFSET]);
      m_roll = getFloat32(&pBuf[EULER_ANGLES_ROLL_OFFSET]);

      ret = m_fieldLength;
    }
  }

  return ret;
}

Float32 EulerAngles::getYaw()
{
  return m_yaw;
}

Float32 EulerAngles::getPitch()
{
  return m_pitch;
}

Float32 EulerAngles::getRoll()
{
  return m_roll;
}

void EulerAngles::setYaw(Float32 yaw)
{
  m_yaw = yaw;
}

void EulerAngles::setPitch(Float32 pitch)
{
  m_pitch = pitch;
}

void EulerAngles::setRoll(Float32 roll)
{
  m_roll = roll;
}

std::string EulerAngles::toString()
{
  std::ostringstream ret;
  ret << "EulerAngles: Yaw(" << m_yaw << "), Pitch(" << m_pitch << "), Roll(" << m_roll << ")";

  return ret.str();
}
