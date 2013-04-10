/*
 * mpi_gx3_15.h
 *
 *  Created on: Mar 31, 2013
 *      Author: bholdaway
 */

#ifndef MIP_GX3_15_H_
#define MIP_GX3_15_H_

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip.h"

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////
//! @def




////////////////////////////////////////////////////////////////////////////////
// GX3-15 PARAMETERS
////////////////////////////////////////////////////////////////////////////////

//GX3-15 Status parameters

#define GX3_15_MODEL_NUMBER     6227





////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

///
// Device Basic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = 3DM_GX3_35_BASIC_STATUS_SEL)
///

#define GX3_15_BASIC_STATUS_SEL   1

typedef struct _gx3_15_basic_status_field
{
 u16 device_model;    // always 3DM_GX3_15_MODEL_NUMBER
 u8  status_selector; // always 3DM_GX3_15_BASIC_STATUS_SEL
 u8  com_mode;
 u8  com_device;
 u32 settings_flags;

 u16 com1_port_state;
 u32 com1_port_baudrate;
}gx3_15_basic_status_field;

///
// Device Diagnostic Status (returned in MIP_REPLY_DESC_3DM_DEVICE_STATUS field
// when status_selector = 3DM_GX3_35_DIAGNOSTICS_STATUS_SEL)
///

#define GX3_15_DIAGNOSTICS_STATUS_SEL 2

typedef struct _gx3_15_diagnostic_status_field
{
 u16 device_model;    // always 3DM_GX3_15_MODEL_NUMBER
 u8  status_selector; // always 3DM_GX3_15_DIAGNOSTICS_STATUS_SEL
 u8  com_mode;
 u8  com_device;
 u32 settings_flags;

 u16 com1_port_state;
 u32 com1_port_baudrate;
 u32 com1_port_bytes_written,  com1_port_bytes_read;
 u32 com1_port_write_overruns, com1_port_read_overruns;

 u16 usb_port_state;
 u32 usb_port_bytes_written,  usb_port_bytes_read;
 u32 usb_port_write_overruns, usb_port_read_overruns;

 u16 gps_driver_state;
 u16 gps_port_state;
 u32 gps_port_bytes_written,  gps_port_bytes_read;
 u32 gps_port_write_overruns, gps_port_read_overruns;
 u32 gps_messages_processed;
 u32 gps_messages_delayed;

 u16 imu_driver_state;
 u16 imu_port_state;
 u32 imu_port_bytes_written,  imu_port_bytes_read;
 u32 imu_port_write_overruns, imu_port_read_overruns;
 u32 imu_messages_processed;
 u32 imu_messages_delayed;
}gx3_15_device_status_field;


#pragma pack()


#endif /* MIP_GX3_15_H_ */
