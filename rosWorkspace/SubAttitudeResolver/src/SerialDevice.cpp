#include <stdio.h>
#include "SerialDevice.hpp"

/**
 * @brief SerialDevice constructor
 */
SerialDevice::SerialDevice()
#ifdef WIN32
  : m_deviceHandle(NULL),
  m_timeouts()
#elif defined (__linux__)
  : m_deviceFd(0),
  m_portSettings()
#endif
{
  // Empty
}

/**
 * @brief SerialDevice desstructor
 */
SerialDevice::~SerialDevice()
{
  // Empty
}

/**
 * @brief Opens the serial device
 *
 * @param pDevice Pointer to string containing device name to open
 * @param baudRate The baud rate to open, valid values are 9600, 38400, 57600, 115200
 * @param timeoutMs Read timeout in milliseconds
 *
 * @return Status of open, 0 on success, -1 on failure to open
 */
int SerialDevice::openDevice(const char* pDevice, const unsigned int baudRate, unsigned int timeoutMs)
{
  #ifdef WIN32

  // Open serial port on Windows
  m_deviceHandle = CreateFileA(pDevice, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
  if (m_deviceHandle == INVALID_HANDLE_VALUE)
  {
    if(GetLastError() == ERROR_FILE_NOT_FOUND)
    {
      printf("SerialDevice: Error, serial device not found\n");
    }
    else
    {
      printf("SerialDevice: Error while opening serial device\n");
    }

    return -1;
  }

  // Set parameters
  DCB dcbSerialParams = { 0 };
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

  // Get the port parameters
  if (!GetCommState(m_deviceHandle, &dcbSerialParams))
  {
    printf("SerialDevice: Error, could not get serial port params\n");
    return -1;
  }

  switch (baudRate)
  {
    case 9600:
      dcbSerialParams.BaudRate = CBR_9600;
      break;
    case 38400:
      dcbSerialParams.BaudRate = CBR_38400;
      break;
    case 57600:
      dcbSerialParams.BaudRate = CBR_57600;
      break;
    case 115200:
      dcbSerialParams.BaudRate = CBR_115200;
      break;
    default:
      printf("SerialDevice: Invalid baud rate specified: %i\n", baudRate);
      return -1;
  }

  dcbSerialParams.ByteSize = 8;                 // 8 data bits
  dcbSerialParams.StopBits = ONESTOPBIT;        // One stop bit
  dcbSerialParams.Parity = NOPARITY;            // No parity

  if (!SetCommState(m_deviceHandle, &dcbSerialParams))
  {
    printf("SerialDevice: Error setting CommState\n");
    return -1;
  }

  // Set Timeout
  m_timeouts.ReadIntervalTimeout = 0;
  m_timeouts.ReadTotalTimeoutConstant = (DWORD)timeoutMs; // Set the Timeout in Milliseconds
  m_timeouts.ReadTotalTimeoutMultiplier = 0;
  m_timeouts.WriteTotalTimeoutConstant = MAXDWORD;
  m_timeouts.WriteTotalTimeoutMultiplier = 0;

  if(!SetCommTimeouts(m_deviceHandle, &m_timeouts))
  {
      printf("SerialDevice: Error setting CommTimeouts\n");
      return -1;
  }
#endif

#ifdef __linux__
    speed_t speedSetting;

    // Open device
    m_deviceFd = open(pDevice, O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_deviceFd == -1)
    {
        return -1;
    }

    // Open the device in nonblocking mode
    fcntl(m_deviceFd, F_SETFL, FNDELAY);

    // Set parameters
    tcgetattr(m_deviceFd, &m_portSettings);		// Get the current options of the port
    bzero(&m_portSettings, sizeof(m_portSettings));		// Clear all the options

    switch (baudRate)
    {
      case 9600:
        speedSetting = B9600;
        break;
      case 38400:
        speedSetting = B38400;
        break;
      case 57600:
        speedSetting = B57600;
        break;
      case 115200:
        speedSetting = B115200;
        break;
      default:
        printf("SerialDevice: Invalid baud rate specified: %i\n", baudRate);
        return -1;
    }
    cfsetispeed(&m_portSettings, speedSetting);
    cfsetospeed(&m_portSettings, speedSetting);
    m_portSettings.c_cflag |= (CLOCAL | CREAD |  CS8);
    m_portSettings.c_iflag |= (IGNPAR | IGNBRK );
    m_portSettings.c_cc[VTIME] = timeoutMs / 100;   // Timeout is in 1/10th second increments
    m_portSettings.c_cc[VMIN] = 0;
    tcsetattr(m_deviceFd, TCSANOW, &m_portSettings);
#endif
    return 0;
}

/**
 * @brief Closes the serial device
 */
void SerialDevice::closeDevice()
{
#ifdef WIN32
  if(m_deviceHandle != NULL)
  {
    CloseHandle(m_deviceHandle);
  }
#endif

#ifdef __linux__
    close(m_deviceFd);
#endif
}

/**
 * @brief Writes a buffer using the serial device
 *
 * @param pBuffer Pointer to buffer of bytes to write
 * @param numBytes Number of bytes to write
 *
 * @return Number of bytes written, or -1 on error
 */
int SerialDevice::writeSerial(const unsigned char* pBuffer, const unsigned int numBytes)
{
#ifdef WIN32
  DWORD bytesWritten = 0;

  if (!WriteFile(m_deviceHandle, pBuffer, numBytes, &bytesWritten, NULL))
  {
    bytesWritten = -1;
  }
  else
  {
    bytesWritten = numBytes;
  }
#endif
#ifdef __linux__
  int bytesWritten = write(m_deviceFd, pBuffer, numBytes);
#endif

  return bytesWritten;
}

/**
 * @brief Reads a buffer using the serial device
 *
 * @param pBuffer Pointer to buffer into which bytes should be read
 * @param bufSize Maximum number of bytes to read
 *
 * @return Number of bytes read, or -1 on error
 */
int SerialDevice::readSerial(unsigned char* pBuffer, unsigned int bufSize)
{
#ifdef WIN32
  DWORD bytesRead = 0;

  if (!SetCommTimeouts(m_deviceHandle, &m_timeouts))
  {
      return -1;
  }

  if (!ReadFile(m_deviceHandle, pBuffer, (DWORD)bufSize, &bytesRead, NULL))
  {
     return -1;
  }
#endif

#ifdef __linux__
  int bytesRead = read(m_deviceFd, pBuffer, bufSize);
#endif

  return bytesRead;
}
