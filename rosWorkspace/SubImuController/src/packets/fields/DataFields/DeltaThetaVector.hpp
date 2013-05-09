/*
 * DeltaThetaVector.hpp
 *
 *  Created on: Apr 16, 2013
 *      Author: bholdaway
 */

#ifndef DELTA_THETA_VECTOR_HPP_
#define DELTA_THETA_VECTOR_HPP_

#include "DataField.hpp"

class DeltaThetaVector : public DataField
{
  public:
    DeltaThetaVector();
    ~DeltaThetaVector();

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
      DELTA_THETA_VECTOR_X_OFFSET = MipField::MIN_MIP_FIELD_HEADER_SIZE,
      DELTA_THETA_VECTOR_Y_OFFSET = DELTA_THETA_VECTOR_X_OFFSET + sizeof(Float32),
      DELTA_THETA_VECTOR_Z_OFFSET = DELTA_THETA_VECTOR_Y_OFFSET + sizeof(Float32),

      DELTA_THETA_VECTOR_STATIC_SIZE = DELTA_THETA_VECTOR_Z_OFFSET + sizeof(Float32)
    };

  private:
    Float32 m_x;
    Float32 m_y;
    Float32 m_z;
};

#endif /* DELTA_THETA_VECTOR_HPP_ */
