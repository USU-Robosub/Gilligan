/*
 * DataField.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef DATAFIELD_HPP_
#define DATAFIELD_HPP_

#include "../MipField.hpp"

class DataField : public MipField
{
  public:
    DataField();
    ~DataField();

    virtual UInt8 serialize(UInt8* pBuf, UInt8 size);
    virtual UInt8 deserialize(UInt8* pBuf, UInt8 size);
    static UInt8 deserializeToField(MipField*& rpField, UInt8* pBuf, UInt8 size);

    virtual std::string toString();

    enum DataFieldSet
    {
      DATA_FIELD_RAW_ACCELEROMETER_VECTOR_SET = 0x1,
      DATA_FIELD_RAW_GYRO_VECTOR_SET = 0x2,
      DATA_FIELD_MAGNETOMETER_VECTOR_SET = 0x3,
      DATA_FIELD_SCALED_ACCELEROMETER_VECTOR_SET = 0x4,
      DATA_FIELD_SCALED_GYRO_VECTOR_SET = 0x5,
      DATA_FIELD_SCALED_MAGNETOMETER_VECTOR_SET = 0x6,
      DATA_FIELD_DELTA_THETA_VECTOR_SET = 0x7,
      DATA_FIELD_DELTA_VELOCITY_VECTOR_SET = 0x8,
      DATA_FIELD_ORIENTATION_MATRIX_SET = 0x9,
      DATA_FIELD_QUATERNION_SET = 0xa,
      DATA_FIELD_ORIENTATION_UPDATE_MATRIX_SET = 0xb,
      DATA_FIELD_EULER_ANGLES_SET = 0xc,
      DATA_FIELD_INTERNAL_TIMESTAMP_SET = 0xe,
      DATA_FIELD_BEACONED_TIMESTAMP_SET = 0xf,
      DATA_FIELD_STABILIZED_MAG_VECTOR_SET = 0x10, //NORTH
      DATA_FIELD_STABILIZED_ACCEL_VECTOR = 0x11, //UP
      DATA_FIELD_GPS_CORRELATION_TIMESTAMP_SET = 0x12,
      DATA_FIELD_WRAPPED_RAW_SET = 0x82 //for GX3-15 and 25 data packet
    };
};

#endif /* DATAFIELD_HPP_ */
