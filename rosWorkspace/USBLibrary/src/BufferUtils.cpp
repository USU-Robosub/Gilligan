/*
 * BufferUtils.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include "../include/USBLibrary/BufferUtils.hpp"

//todo update with endianness

void putUInt8(UInt8* pBuf, UInt8 data)
{
  pBuf[0] = data;
}

UInt8 getUInt8(UInt8* pBuf)
{
  UInt8 ret = pBuf[0];

  return ret;
}

void putUInt16(UInt8* pBuf, UInt16 data)
{
  pBuf[0] = ((UInt8*)&data)[0];
  pBuf[1] = ((UInt8*)&data)[1];
}

UInt16 getUInt16(UInt8* pBuf)
{
  UInt16 ret = ((UInt16)pBuf[0] << 8) | ((UInt16)pBuf[1]);

  return ret;
}

void putUInt32(UInt8* pBuf, UInt32 data)
{
  pBuf[0] = ((UInt8*)&data)[0];
  pBuf[1] = ((UInt8*)&data)[1];
  pBuf[2] = ((UInt8*)&data)[2];
  pBuf[3] = ((UInt8*)&data)[3];
}

UInt32 getUInt32(UInt8* pBuf)
{
  UInt32 ret = ((UInt32)pBuf[0] << 24) | ((UInt32)pBuf[1] << 16) | ((UInt32)pBuf[2] << 8) | ((UInt32)pBuf[3]);

  return ret;
}

void putInt8(UInt8* pBuf, Int8 data)
{
  pBuf[0] = data;
}

Int8 getInt8(UInt8* pBuf)
{
  Int8 ret = (Int8)pBuf[0];

  return ret;
}

void putInt16(UInt8* pBuf, Int16 data)
{
  pBuf[0] = ((UInt8*)&data)[0];
  pBuf[1] = ((UInt8*)&data)[1];
}

Int16 getInt16(UInt8* pBuf)
{
  Int16 ret = ((Int16)pBuf[0] << 8) | ((Int16)pBuf[1]);

  return ret;
}

void putInt32(UInt8* pBuf, Int32 data)
{
  pBuf[0] = ((UInt8*)&data)[0];
  pBuf[1] = ((UInt8*)&data)[1];
  pBuf[2] = ((UInt8*)&data)[2];
  pBuf[3] = ((UInt8*)&data)[3];
}

Int32 getInt32(UInt8* pBuf)
{
  Int32 ret = ((Int32)pBuf[0] << 24) | ((Int32)pBuf[1] << 16) | ((Int32)pBuf[2] << 8) | ((Int32)pBuf[3]);

  return ret;
}

void putFloat32(UInt8* pBuf, Float32 data)
{
  pBuf[0] = ((UInt8*)&data)[3];
  pBuf[1] = ((UInt8*)&data)[2];
  pBuf[2] = ((UInt8*)&data)[1];
  pBuf[3] = ((UInt8*)&data)[0];
}

Float32 getFloat32(UInt8* pBuf)
{
  Float32 ret = 0;
  UInt8* pFloat = (UInt8*)&ret;

  pFloat[0] = pBuf[3];
  pFloat[1] = pBuf[2];
  pFloat[2] = pBuf[1];
  pFloat[3] = pBuf[0];

  return ret;
}

void putFloat64(UInt8* pBuf, Float64 data)
{
  pBuf[0] = ((UInt8*)&data)[7];
  pBuf[1] = ((UInt8*)&data)[6];
  pBuf[2] = ((UInt8*)&data)[5];
  pBuf[3] = ((UInt8*)&data)[4];
  pBuf[0] = ((UInt8*)&data)[3];
  pBuf[1] = ((UInt8*)&data)[2];
  pBuf[2] = ((UInt8*)&data)[1];
  pBuf[3] = ((UInt8*)&data)[0];
}

Float64 getFloat64(UInt8* pBuf)
{
  Float64 ret = 0;
  UInt8* pFloat = (UInt8*)&ret;

  pFloat[0] = pBuf[7];
  pFloat[1] = pBuf[6];
  pFloat[2] = pBuf[5];
  pFloat[3] = pBuf[4];
  pFloat[4] = pBuf[3];
  pFloat[5] = pBuf[2];
  pFloat[6] = pBuf[1];
  pFloat[7] = pBuf[0];

  return ret;
}
