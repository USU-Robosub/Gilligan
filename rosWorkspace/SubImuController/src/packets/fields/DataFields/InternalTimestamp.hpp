/*
 * InternalTimestamp.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef INTERNALTIMESTAMP_HPP_
#define INTERNALTIMESTAMP_HPP_

#include "DataField.hpp"

class InternalTimestamp : public DataField
{
  public:
    InternalTimestamp();
    ~InternalTimestamp();

    virtual UInt8 serialize(UInt8* pBuf, UInt8 size);
    virtual UInt8 deserialize(UInt8* pBuf, UInt8 size);

    UInt32 getTimestamp();
    UInt32 getTimestampAsMicroSeconds();

    void setTimestamp(UInt32 timestamp);

    std::string toString();

    enum
    {
      INTERNAL_TIMESTAMP_TIMESTAMP_OFFSET = MipField::MIN_MIP_FIELD_HEADER_SIZE,

      INTERNAL_TIMESTAMP_STATIC_SIZE = INTERNAL_TIMESTAMP_TIMESTAMP_OFFSET + sizeof(UInt32)
    };

  private:
    UInt32 m_timestamp;
};

#endif /* INTERNALTIMESTAMP_HPP_ */
