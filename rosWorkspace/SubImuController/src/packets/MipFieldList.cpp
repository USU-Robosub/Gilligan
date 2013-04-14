/*
 * MipFieldList.cpp
 *
 *  Created on: Mar 15, 2013
 *      Author: bholdaway
 */

#include "MipFieldList.hpp"
#include "MipPacket.hpp"
#include "fields/DataFields/DataField.hpp"

MipFieldNode::MipFieldNode()
 :  m_pData(NULL),
    m_pNext(NULL)
{
  //Empty
}

MipFieldList::MipFieldList()
 :  m_pHead(NULL),
    m_pTail(NULL),
    m_count(0)
{
  //Empty
}

MipFieldList::~MipFieldList()
{
  clear();
}

void MipFieldList::add(MipField* pField)
{
  MipFieldNode* pNode = new MipFieldNode();
  pNode->m_pData = pField;
  pNode->m_pNext = NULL;

  if (m_pTail != NULL)
  {
    m_pTail->m_pNext = pNode;
  }

  m_pTail = pNode;

  if (m_pHead == NULL)
  {
    m_pHead = pNode;
  }
  m_count++;
}

MipField* MipFieldList::remove()
{
  MipField* pNode = NULL;

  if (m_pHead != NULL)
  {
    m_pHead = m_pHead->m_pNext;
    pNode = m_pHead->m_pData;
    m_count--;
  }

  return pNode;
}

MipField* MipFieldList::getAtPos(Int32 pos)
{
  MipField* pRet = NULL;

  if (pos < m_count)
  {
    for (MipFieldNode* pIt = m_pHead; pIt != NULL && pos >= 0; pIt = pIt->m_pNext)
    {
      pRet = pIt->m_pData;
      pos--;
    }
  }

  return pRet;
}

UInt32 MipFieldList::getCount()
{
  return m_count;
}

MipFieldNode* MipFieldList::getIterator() const
{
  return m_pHead;
}

void MipFieldList::clear()
{
  while (m_pHead != NULL)
  {
    MipFieldNode* pToDel = m_pHead;
    m_pHead = m_pHead->m_pNext;

    if (pToDel->m_pData != NULL)
    {
      delete pToDel->m_pData;
    }
    delete pToDel;
  }
}

void MipFieldList::deserialize(UInt8 descriptorSet, UInt8* pBuf, Int32 size)
{
  clear();

  //figure this out
  Int32 pos = 0;
  while (pos < size && size >= MipField::MIN_MIP_FIELD_HEADER_SIZE)
  {
    UInt8 ret = 0;
    switch (descriptorSet)
    {
      case MipPacket::DESCRIPTOR_SET_COMMANDS:
      {
        //TODO finish later
      }
      break;

      case MipPacket::DESCRIPTOR_SET_3DM_COMMAND:
      {
        //TODO finish later
      }
      break;

      case MipPacket::DESCRIPTOR_SET_SYSTEM_COMMAND:
      {
        //TODO finish later
      }
      break;

      case MipPacket::DESCRIPTOR_SET_IMU_DATA:
      {
        MipField* pField = NULL;
        ret = DataField::deserializeToField(pField, &pBuf[pos], size);

        if (pField != NULL)
        {
          add(pField);
        }
      }
      break;
    }

    pos += ret;
  }
}
