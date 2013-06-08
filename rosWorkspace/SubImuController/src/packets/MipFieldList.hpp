/*
 * MipFieldList.hpp
 *
 *  Created on: Mar 15, 2013
 *      Author: bholdaway
 */

#ifndef MIPFIELDLIST_HPP_
#define MIPFIELDLIST_HPP_

#include "fields/MipField.hpp"

class MipFieldNode
{
  public:
    MipFieldNode();

    MipField* m_pData;
    MipFieldNode* m_pNext;
};

class MipFieldList
{
  public:
    MipFieldList();
    ~MipFieldList();

    void add(MipField* pField);
    MipField* remove();
    MipField* getAtPos(Int32 pos);
    MipFieldNode* getIterator() const;
    UInt32 getCount();
    void clear();

    void deserialize(UInt8 descriptorSet, UInt8* pBuf, Int32 size);

  private:
    MipFieldNode* m_pHead;
    MipFieldNode*m_pTail;
    Int32 m_count;
};

#endif /* MIPFIELDLIST_HPP_ */
