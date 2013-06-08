/*
 * ScaledAccelerometerVector.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef SCALEDACCELEROMETERVECTOR_HPP_
#define SCALEDACCELEROMETERVECTOR_HPP_

#include <stdlib.h>
#include <string.h>
#include "DataField.hpp"

class ScaledAccelerometerVector : public DataField
{
  public:
    ScaledAccelerometerVector();
    ~ScaledAccelerometerVector();

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
      SCALED_ACCELEROMETER_VECTOR_X_OFFSET = MipField::MIN_MIP_FIELD_HEADER_SIZE,
      SCALED_ACCELEROMETER_VECTOR_Y_OFFSET = SCALED_ACCELEROMETER_VECTOR_X_OFFSET + sizeof(Float32),
      SCALED_ACCELEROMETER_VECTOR_Z_OFFSET = SCALED_ACCELEROMETER_VECTOR_Y_OFFSET + sizeof(Float32),

      SCALED_ACCELEROMETER_VECTOR_STATIC_SIZE = SCALED_ACCELEROMETER_VECTOR_Z_OFFSET + sizeof(Float32)
    };

  private:
    Float32 m_x;
    Float32 m_y;
    Float32 m_z;
};

#endif /* SCALEDACCELEROMETERVECTOR_HPP_ */
