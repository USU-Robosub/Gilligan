/*
 * SerialInterface.cpp
 *
 *  Created on: Apr 13, 2013
 *      Author: bholdaway
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <cstring>
#include "../include/USBLibrary/SerialInterface.hpp"

SerialInterface::SerialInterface(std::string port, UInt32 baudRate)
  : m_fileDescr(0),//file descriptor
    m_port(port),
    m_baudRate(baudRate),
    m_isPortGood(false)
{
  //empty
}

SerialInterface::~SerialInterface()
{
  close(m_fileDescr);
}


Int32 SerialInterface::recv(UInt8* pBuf, UInt32 size)
{
  size_t ret = -1;

  if (!m_isPortGood)
  {
    sleep(1);
    openInterface();
  }

  if (m_fileDescr > 0)
  {
    fd_set rdfs;
    struct timeval timeout;

    FD_ZERO(&rdfs);
    FD_SET(m_fileDescr, &rdfs);

    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    int sel = 0;
    if ((sel = select(m_fileDescr+1, &rdfs, NULL, NULL, &timeout)) > 0)
    {
      ret = read(m_fileDescr, pBuf, 1);
    }

    if (ret < 0 || sel < 0)
    {
      m_isPortGood = false;
      close(m_fileDescr);
      ret = -1;
      printf("Closing port\n");
    }
    else if (sel == 0)
    {
      printf("timeout\n");
    }
  }

  return ret;
}

Int32 SerialInterface::send(UInt8* pBuf, UInt32 size)
{
  size_t ret = -1;

  if (!m_isPortGood)
  {
    sleep(1);
    openInterface();
  }

  if (m_fileDescr > 0)
  {
    fd_set rdfs;
    struct timeval timeout;

    FD_ZERO(&rdfs);
    FD_SET(m_fileDescr, &rdfs);

    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (select(m_fileDescr+1, NULL, &rdfs, NULL, &timeout) > 0)
    {
      ret = write(m_fileDescr, pBuf, size);

      if (ret < 0)
      {
        m_isPortGood = false;
        close(m_fileDescr);
        printf("Closing port\n");
        ret = -1;
      }
    }
  }

  return ret;
}

void SerialInterface::configure(int waitTime /*= 0*/, int minBytes /*= 0*/)
{
  fcntl(m_fileDescr, F_SETFL, FNDELAY);
  struct termios port_settings;      // structure to store the port settings in
  bzero(&port_settings, sizeof(port_settings));

  speed_t speed = convertSpeed(m_baudRate);

  cfsetispeed(&port_settings, speed);    // set baud rates
  cfsetospeed(&port_settings, speed);

  port_settings.c_cflag |= ( CLOCAL | CREAD |  CS8); //set ignore ctrl lines READ, // Configure the device : 8 bits, no parity, no control
  port_settings.c_iflag |= ( IGNPAR | IGNBRK );

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  port_settings.c_cc[VMIN]  = minBytes;
  port_settings.c_cc[VTIME] = waitTime; //==20 in subSense & MotorControll make it variable?

  if(tcsetattr(m_fileDescr, TCSANOW, &port_settings) < 0)    // apply the settings to the port
  {
    printf("Error: Failed to set serial settings\n");
  }
}

bool SerialInterface::isGood()
{
  return m_isPortGood;
}

bool SerialInterface::openInterface(int waitTime, int minBytes /*= 0*/)
{
  printf("trying to open Serial port\n");
  m_fileDescr = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (m_fileDescr != -1)
  {
    configure(waitTime,minBytes);
    m_isPortGood = true;
    printf("Serial port open\n");
  }
  else
  {
    m_fileDescr = 0;
  }
}

speed_t SerialInterface::convertSpeed(UInt32 baudRate)
{
  speed_t speed;

  switch (baudRate)
  {
    case 57600:
    {
      speed = B57600;
    }
    break;

    case 230400:
    {
      speed = B230400;
    }
    break;

    case 460800:
    {
      speed = B460800;
    }
    break;

    case 500000:
    {
      speed = B500000;
    }
    break;

    case 576000:
    {
      speed = B576000;
    }
    break;

    case 921600:
    {
      speed = B921600;
    }
    break;

    case 1000000:
    {
      speed = B1000000;
    }
    break;

    case 1152000:
    {
      speed = B1152000;
    }
    break;

    case 1500000:
    {
      speed = B1500000;
    }
    break;

    case 2000000:
    {
      speed = B2000000;
    }
    break;

    case 2500000:
    {
      speed = B2500000;
    }
    break;

    case 3000000:
    {
      speed = B3000000;
    }
    break;

    case 3500000:
    {
      speed = B3500000;
    }
    break;

    case 4000000:
    {
      speed = B4000000;
    }
    break;

    case 115200:
    default:
    {
      speed = B115200;
    }
  }

  return speed;
}
