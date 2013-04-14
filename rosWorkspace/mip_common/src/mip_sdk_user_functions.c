/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_user_functions.c 
//! @author  Nathan Miller
//! @version 1.0
//
//! @description Target-Specific Functions Required by the MIP SDK
//
// External dependencies:
//
//  mip.h
// 
//! @copyright 2011 Microstrain. 
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN SHALL NOT BE HELD LIABLE 
//! FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY 
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY 
//! CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH 
//! THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////


#include "mip_common/mip_sdk_user_functions.h"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_open(void *port_handle, int port_num, int baudrate)
//
//! @section DESCRIPTION
//! Target-Specific communications port open. 
//
//! @section DETAILS
//!
//! @param [out] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in] int port_num       - port number (as recognized by the operating system.)
//! @param [in] int baudrate       - baudrate of the com port.
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem opening the port.\n
//! @retval MIP_USER_FUNCTION_OK     The open was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_open(void **port_handle, int port_num, int baudrate)
{
  s32 ret;
  *port_handle = malloc(sizeof(struct serial_fd));
  struct serial_fd* sfd = (struct serial_fd*)*port_handle;

  sfd->pBuffer = (u8*)malloc(sizeof(u8) * MAX_BUFFER_SIZE);
  sfd->position = 0;
  sfd->size = 0;
  sfd->state = STATE_WAITING_ON_FD;
  sfd->portNumber = port_num;
  sfd->baudrate = baudrate;

  memset(sfd->pBuffer, 0, sizeof(u8) * MAX_BUFFER_SIZE);

  ret = -1;
  while ((ret = openPort(sfd)) < 0)
  {
    sleep(1);
  }

  printf("Opened serial port\n");

  return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_close(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific port close function 
//
//! @section DETAILS
//!
//! @param [in] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem closing the port.\n
//! @retval MIP_USER_FUNCTION_OK     The close was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_close(void *port_handle)
{
  if(port_handle == NULL)
   return MIP_USER_FUNCTION_ERROR;

  struct serial_fd* fd = (struct serial_fd*)port_handle;

 close(fd->fd);
 free(fd->pBuffer);
 free(fd);

 return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function. 
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_written - the number of bytes actually written to the port
//! @param [in]  u32 timeout_ms     - the write timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The write was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
{
  struct serial_fd* fd;
  size_t ret;

  if(port_handle == NULL)
  {
    printf("mip_sdk_port_write error: handle is null\n");
    return MIP_USER_FUNCTION_ERROR;
  }

  fd = (struct serial_fd*)port_handle;

  if (fd->state == STATE_WAITING_ON_FD)
  {
    openPort(fd);
  }
  else
  {
    //TODO maybe put this in a while look that lasts only until the timeout
    ret = write(fd->fd, buffer, num_bytes);

    if (ret >= 0)
    {
      *bytes_written = ret;
      //printf("Wrote %i bytes of %u\n", ret, num_bytes);
      return MIP_USER_FUNCTION_OK;
    }
    else
    {
      *bytes_written = 0;
      tempClosePort(fd);
      //printf("mip_sdk_port_write Error: Failed to write %u bytes\n", num_bytes);
    }
  }

  return MIP_USER_FUNCTION_ERROR;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function. 
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_read    - the number of bytes actually read from the device
//! @param [in]  u32 timeout_ms     - the read timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The read was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
{
  fd_set rdfs;
  struct timeval timeout;
  *bytes_read = 0;

  if(port_handle == NULL)
  {
    return MIP_USER_FUNCTION_ERROR;
  }

  struct serial_fd* fd = (struct serial_fd*)port_handle;

  if (num_bytes > 0 && fd->size > 0)
  {
    u32 newSize = MIN(num_bytes, fd->size);
    memcpy(buffer, &fd->pBuffer[fd->position], newSize);

    num_bytes -= newSize;
    fd->position += newSize;
    fd->size -= newSize;
    *bytes_read += newSize;
    //printf("read: buffer size is %u\n", fd->size);
  }

  //grab the rest of the bytes
  if (num_bytes > 0)
  {
    if (fd->state == STATE_WAITING_ON_FD)
    {
      openPort(fd);
    }
    else
    {
      FD_ZERO(&rdfs);
      FD_SET(fd->fd, &rdfs);

      timeout.tv_sec = timeout_ms / 1000;
      timeout.tv_usec = (timeout_ms % 1000) * 1000;

      if (select(fd->fd+1, &rdfs, NULL, NULL, &timeout) > 0)
      {
        size_t bytesRead = read(fd->fd, fd->pBuffer, MAX_BUFFER_SIZE);
        fd->position = 0;

        if (bytesRead >= 0)
        {
          fd->size = bytesRead;

          u32 newSize = MIN(num_bytes, fd->size);
          memcpy(&buffer[*bytes_read], fd->pBuffer, newSize);

          num_bytes -= newSize;
          fd->position += newSize;
          fd->size -= newSize;
          *bytes_read += newSize;
        }
        else
        {
          tempClosePort(fd);
          *bytes_read = 0;
          //printf("READ ERROR: received %i\n", bytesRead);
          return MIP_USER_FUNCTION_ERROR;
        }
      }
    }
  }

 return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_port_read_count(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific Function to Get the Number of Bytes Waiting on the Port.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Number of bytes waiting on the port,\n
//!           0, if there is an error.
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_port_read_count(void *port_handle)
{
  fd_set rdfs;
  struct timeval timeout;

  if(port_handle == NULL)
  {
    printf("mip_sdk_read_count error: handle is null\n");
    return MIP_USER_FUNCTION_ERROR;
  }

  struct serial_fd* fd = (struct serial_fd*)port_handle;

  if (fd->state == STATE_WAITING_ON_FD)
  {
    openPort(fd);
  }
  else if (fd->size < MAX_BUFFER_SIZE)
  {
    FD_ZERO(&rdfs);
    FD_SET(fd->fd, &rdfs);

    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;

    if (select(fd->fd+1, &rdfs, NULL, NULL, &timeout) > 0)
    {
      u32 i;

      //move the bytes down
      for (i = 0; i < fd->size && fd->position > 0; i++)
      {
        fd->pBuffer[i] = fd->pBuffer[fd->position + i];
      }
      fd->position = 0;

      size_t bytesRead = read(fd->fd, &fd->pBuffer[fd->size], MAX_BUFFER_SIZE - fd->size);

      if (bytesRead > 0)
      {
        fd->size += bytesRead;
        //printf("read_count: buffer size is %u\n", fd->size);
      }
      else
      {
        tempClosePort(fd);
      }
    }
  }

  return fd->size;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_get_time_ms()
//
//! @section DESCRIPTION
//! Target-Specific Call to get the current time in milliseconds.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Current time in milliseconds.
//
//! @section NOTES
//! 
//!   1) This value should no roll-over in short periods of time (e.g. minutes)\n
//!   2) Most systems have a millisecond counter that rolls-over every 32 bits\n
//!      (e.g. 49.71 days roll-over period, with 1 millisecond LSB)\n
//!   3) An absolute reference is not required since this function is\n
//!      used for relative time-outs.\n
//!   4) The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//!      edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_get_time_ms()
{
  struct timeval  tv;
  gettimeofday(&tv, NULL);
  u32 ms = tv.tv_sec * 1000 + (tv.tv_usec / 1000);
  return ms;
}

s32 openPort(struct serial_fd* sfd)
{
  printf("openport\n");
  s32 ret;
  ret = 0;

  sfd->fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);

  if (sfd->fd == -1)
  {
    ret = -1;
  }
  else
  {
    fcntl(sfd->fd, F_SETFL, 0);
    struct termios port_settings;      // structure to store the port settings in

    speed_t speed = getSpeed(sfd->baudrate);

    cfsetispeed(&port_settings, speed);    // set baud rates
    cfsetospeed(&port_settings, speed);

    port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

    port_settings.c_cflag &= ~CRTSCTS;

    port_settings.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    port_settings.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    port_settings.c_cc[VMIN]  = 0;
    port_settings.c_cc[VTIME] = 20;

    if(tcsetattr(sfd->fd, TCSANOW, &port_settings) < 0)    // apply the settings to the port
    {
      //printf("Error: Failed to set serial settings\n");
      ret = -1;
    }
    else
    {
      sfd->state = STATE_RUNNING;
      //sleep(5);
    }
  }

  return ret;
}

s32 tempClosePort(struct serial_fd* sfd)
{
  close(sfd->fd);
  sfd->state =STATE_WAITING_ON_FD;

  memset(sfd->pBuffer, 0, MAX_BUFFER_SIZE);
  sfd->size = 0;
  sfd->position = 0;
}

speed_t getSpeed(u32 speed)
{
  speed_t retSpeed;

  switch (speed)
  {
    case 115200:
    {
      speed = B115200;
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

    case 300000:
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

    default:
    {
      speed = B115200;
    }
    break;
  }

  return retSpeed;
}
