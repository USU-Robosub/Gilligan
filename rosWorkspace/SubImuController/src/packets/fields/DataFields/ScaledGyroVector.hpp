/*
 * ScaledGyroVector.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef SCALEDGYROVECTOR_HPP_
#define SCALEDGYROVECTOR_HPP_

#include "DataField.hpp"

class ScaledGyroVector : public DataField
{
  public:
    ScaledGyroVector();
    ~ScaledGyroVector();

    virtual UInt8 serialize(UInt8* pBuf, UInt8 size);
    virtual UInt8 deserialize(UInt8* pBuf, UInt8 size);

    Float32 getX();
    Float32 getY();
    Float32 getZ();

    void setX(Float32 x);
    void setY(Float32 y);
    void setZ(Float32 z);

    std::string toString();

    enum
    {
      SCALED_GYRO_VECTOR_X_OFFSET = MipField::MIN_MIP_FIELD_HEADER_SIZE,
      SCALED_GYRO_VECTOR_Y_OFFSET = SCALED_GYRO_VECTOR_X_OFFSET + sizeof(Float32),
      SCALED_GYRO_VECTOR_Z_OFFSET = SCALED_GYRO_VECTOR_Y_OFFSET + sizeof(Float32),

      SCALED_GYRO_VECTOR_STATIC_SIZE = SCALED_GYRO_VECTOR_Z_OFFSET + sizeof(Float32)
    };

  private:
    Float32 m_x;
    Float32 m_y;
    Float32 m_z;
};

#endif /* SCALEDGYROVECTOR_HPP_ */
