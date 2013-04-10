/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_user_functions.h 
//! @author  Nathan Miller
//! @version 1.0
//
//! @description Target-Specific, User-Defined Functions Definitions
//
// External dependencies:
//
//  
// 
//! @copyright 2011 Microstrain. 
//
//!@section CHANGES
//! 
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

#ifndef _MIP_SDK_USER_FUNCTIONS_H
#define _MIP_SDK_USER_FUNCTIONS_H

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip.h"
#include <termios.h>

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////
//! @def 

#define MIP_USER_FUNCTION_OK    0
#define MIP_USER_FUNCTION_ERROR 1

#define MIP_COM_PORT_BUFFER_SIZE 0x200
#define MAX_BUFFER_SIZE 115200

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

enum SerialState
{
  STATE_WAITING_ON_FD = 0,
  STATE_RUNNING = 1
};

struct serial_fd
{
    int fd;
    u32 position;
    u32 size;
    u8* pBuffer;
    u32 baudrate;
    s32 portNumber;
    s32 state;
};
////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////


u16 mip_sdk_port_open(void **port_handle, int port_num, int baudrate);
u16 mip_sdk_port_close(void *port_handle);

u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms);
u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms);

u32 mip_sdk_port_read_count(void *port_handle);

u32 mip_sdk_get_time_ms();

s32 openPort(struct serial_fd* sfd);
s32 tempClosePort(struct serial_fd* sfd);
speed_t getSpeed(u32 speed);

#endif
