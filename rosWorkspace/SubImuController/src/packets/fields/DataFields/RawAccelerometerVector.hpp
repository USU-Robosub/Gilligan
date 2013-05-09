/*
 * RawAccelerometerVector.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef RAW_ACCELEROMETER_VECTOR_HPP_
#define RAW_ACCELEROMETER_VECTOR_HPP_

#include <stdlib.h>
#include <string.h>
#include "DataField.hpp"

class RawAccelerometerVector : public DataField
{
  public:
    RawAccelerometerVector();
    ~RawAccelerometerVector();

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
      RAW_ACCELEROMETER_VECTOR_X_OFFSET = MipField::MIN_MIP_FIELD_HEADER_SIZE,
      RAW_ACCELEROMETER_VECTOR_Y_OFFSET = RAW_ACCELEROMETER_VECTOR_X_OFFSET + sizeof(Float32),
      RAW_ACCELEROMETER_VECTOR_Z_OFFSET = RAW_ACCELEROMETER_VECTOR_Y_OFFSET + sizeof(Float32),

      RAW_ACCELEROMETER_VECTOR_STATIC_SIZE = RAW_ACCELEROMETER_VECTOR_Z_OFFSET + sizeof(Float32)
    };

  private:
    Float32 m_x;
    Float32 m_y;
    Float32 m_z;
};

#endif /* RAW_ACCELEROMETER_VECTOR_HPP_ */
