/*
 * DataField.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include <stdio.h>

#include "DataField.hpp"
#include "EulerAngles.hpp"
#include "ScaledAccelerometerVector.hpp"
#include "ScaledGyroVector.hpp"

DataField::DataField()
  : MipField()
{
  //empty
}

DataField::~DataField()
{
  // TODO Auto-generated destructor stub
}

UInt8 DataField::serialize(UInt8* pBuf, UInt8 size)
{
  return serializeHeader(pBuf, size);
}

UInt8 DataField::deserialize(UInt8* pBuf, UInt8 size)
{
  return deserializeHeader(pBuf, size);
}

UInt8 DataField::deserializeToField(MipField*& rpField, UInt8* pBuf, UInt8 size)
{
  UInt8 ret = 0;

  if (size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    switch (pBuf[MipField::FIELD_DESCRIPTOR_OFFSET])
    {
      case DATA_FIELD_RAW_ACCELEROMETER_VECTOR_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_RAW_GYRO_VECTOR_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_MAGNETOMETER_VECTOR_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_SCALED_ACCELEROMETER_VECTOR_SET:
      {
        ScaledAccelerometerVector* pField = new ScaledAccelerometerVector();
        ret = pField->deserialize(pBuf, size);
        rpField = pField;
      }
      break;

      case DATA_FIELD_SCALED_GYRO_VECTOR_SET:
      {
        ScaledGyroVector* pField = new ScaledGyroVector();
        ret = pField->deserialize(pBuf, size);
        rpField = pField;
      }
      break;

      case DATA_FIELD_SCALED_MAGNETOMETER_VECTOR_SET:
      {

      }
      break;

      case DATA_FIELD_DELTA_THETA_VECTOR_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_DELTA_VELOCITY_VECTOR_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_ORIENTATION_MATRIX_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_QUATERNION_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_ORIENTATION_UPDATE_MATRIX_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_EULER_ANGLES_SET:
      {
        EulerAngles* pField = new EulerAngles();
        ret = pField->deserialize(pBuf, size);
        rpField = pField;
      }
      break;

      case DATA_FIELD_INTERNAL_TIMESTAMP_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_BEACONED_TIMESTAMP_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_STABILIZED_MAG_VECTOR_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_STABILIZED_ACCEL_VECTOR:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_GPS_CORRELATION_TIMESTAMP_SET:
      {
        //TODO later
      }
      break;

      case DATA_FIELD_WRAPPED_RAW_SET:
      {
        //TODO later
      }
      break;

      default:
        printf("Unhandled DataField 0x%x\n", pBuf[MipField::FIELD_DESCRIPTOR_OFFSET]);
        DataField* pField = new DataField();
        ret = pField->deserialize(pBuf, size);
        rpField = pField;
      break;
    }
  }

  return ret;
}

std::string DataField::toString()
{
  std::string ret = "DataField";

  return ret;
}
