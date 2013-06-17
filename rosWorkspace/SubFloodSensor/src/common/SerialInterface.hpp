/*
 * SerialInterface.hpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#ifndef SERIALINTERFACE_HPP_
#define SERIALINTERFACE_HPP_

#include <string>
#include <termios.h>
#include "PortableTypes.hpp"

class SerialInterface
{
  public:
    SerialInterface(std::string port, UInt32 baudRate);
    ~SerialInterface();

    Int32 recv(UInt8* pBuf, UInt32 size);
    Int32 send(UInt8* pBuf, UInt32 size);

    bool isGood();
    bool openInterface();
    speed_t convertSpeed(UInt32 baudRate);
    void closeInterface();

  private:
    int m_fd;
    std::string m_port;
    UInt32 m_baudRate;
    bool m_isPortGood;

    void configure();

};

#endif /* SERIALINTERFACE_HPP_ */
