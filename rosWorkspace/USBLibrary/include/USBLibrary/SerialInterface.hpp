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
#include "share.hpp"

class SerialInterface
{
  public:
    SerialInterface(std::string port, UInt32 baudRate);
    ~SerialInterface();

    Int32 recv(UInt8* pBuf, UInt32 size);
    Int32 send(UInt8* pBuf, UInt32 size);

    bool isGood();
    bool openInterface(int waitTimes=0, int minBytes=0);
    speed_t convertSpeed(UInt32 baudRate);

  private:
    int m_fileDescr;
    std::string m_port;
    UInt32 m_baudRate;
    bool m_isPortGood;

    void configure(int waitTimes=0, int minBytes=0);

};

#endif /* SERIALINTERFACE_HPP_ */
