/*
 * BufferUtils.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef BUFFERUTILS_HPP_
#define BUFFERUTILS_HPP_


#include "PortableTypes.hpp"

void putUInt8(UInt8* pBuf, UInt8 data);
UInt8 getUInt8(UInt8* pBuf);

void putUInt16(UInt8* pBuf, UInt16 data);
UInt16 getUInt16(UInt8* pBuf);

void putUInt32(UInt8* pBuf, UInt32 data);
UInt32 getUInt32(UInt8* pBuf);


void putInt8(UInt8* pBuf, Int8 data);
Int8 getInt8(UInt8* pBuf);

void putInt16(UInt8* pBuf, Int16 data);
Int16 getInt16(UInt8* pBuf);

void putInt32(UInt8* pBuf, Int32 data);
Int32 getInt32(UInt8* pBuf);


void putFloat32(UInt8* pBuf, Float32 data);
Float32 getFloat32(UInt8* pBuf);

void putFloat64(UInt8* pBuf, Float64 data);
Float64 getFloat64(UInt8* pBuf);



#endif /* BUFFERUTILS_HPP_ */
