/*
 * SerialInterface.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef SERIALINTERFACE_HPP_
#define SERIALINTERFACE_HPP_

#include <string>
#include "share.hpp"

class SerialInterface
{
  public:
    SerialInterface(std::string port);
    ~SerialInterface();

    Int32 recv(UInt8* pBuf, UInt32 size);
    Int32 send(UInt8* pBuf, UInt32 size);

    bool isGood();
    bool openInterface();

  private:
    int m_fd;
    std::string m_port;
    bool m_isPortGood;

    void configure();

};

#endif /* SERIALINTERFACE_HPP_ */
