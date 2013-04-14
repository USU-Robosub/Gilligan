/*
 * EulerAngles.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef EULERANGLES_HPP_
#define EULERANGLES_HPP_

#include "DataField.hpp"

class EulerAngles : public DataField
{
  public:
    EulerAngles();
    ~EulerAngles();

    virtual UInt8 serialize(UInt8* pBuf, UInt8 size);
    virtual UInt8 deserialize(UInt8* pBuf, UInt8 size);

    Float32 getYaw();
    Float32 getPitch();
    Float32 getRoll();

    void setYaw(Float32 yaw);
    void setPitch(Float32 pitch);
    void setRoll(Float32 roll);

    std::string toString();

    enum
    {
      EULER_ANGLES_YAW_OFFSET = MipField::MIN_MIP_FIELD_HEADER_SIZE,
      EULER_ANGLES_PITCH_OFFSET = EULER_ANGLES_YAW_OFFSET + sizeof(Float32),
      EULER_ANGLES_ROLL_OFFSET = EULER_ANGLES_PITCH_OFFSET + sizeof(Float32),

      EULER_ANGLES_STATIC_SIZE = EULER_ANGLES_ROLL_OFFSET + sizeof(Float32)
    };

  private:
    Float32 m_yaw;
    Float32 m_pitch;
    Float32 m_roll;
};

#endif /* EULERANGLES_HPP_ */
