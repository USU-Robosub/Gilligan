#ifndef _SERIAL_DEVICE_HPP
#define _SERIAL_DEVICE_HPP

// Windows specific includes
#if defined WIN32
#include <windows.h>
#endif

// Linux specific includes
#ifdef __linux__
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#endif

/**
 * @brief SerialDevice interface used for lowest-level communication on a serial port
 */
class SerialDevice
{
  public:
    SerialDevice();
    ~SerialDevice();

    int openDevice(const char* pDevice, const unsigned int baudRate, unsigned int timeoutMs);
    void closeDevice();

    int writeSerial(const unsigned char* pBuffer, const unsigned int numBytes);
    int readSerial(unsigned char* pBuffer, unsigned int bufSize);

  private:
#ifdef WIN32
    HANDLE m_deviceHandle;          //!< Serial port handle (Windows)
    COMMTIMEOUTS m_timeouts;        //!< Struct containing timeout information (Windows)
#elif defined (__linux__)
    int m_deviceFd;                 //!< Serial port file descriptor (Linux)
    struct termios m_portSettings;  //!< Serial port settings (Linux)
#endif
};

#endif // _SERIAL_DEVICE_HPP
