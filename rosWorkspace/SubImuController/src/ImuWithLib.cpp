//============================================================================
// Name        : ImuWithLib.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <math.h>

#include "mip_sdk.h"
#include "mip_gx3_15.h"
#include "byteswap_utilities.h"
/////////////////////

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds


//Help Functions
void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

void convertRadiansToDegrees(mip_ahrs_euler_angles *angles);


// Globals
//The primary device interface structure
mip_interface device_interface;

//Packet Counters (valid, timeout, and checksum errors)
u32 ahrs_valid_packet_count = 0;
u32 ahrs_timeout_packet_count = 0;
u32 ahrs_checksum_error_packet_count = 0;

//AHRS
mip_ahrs_scaled_gyro  curr_ahrs_gyro;
mip_ahrs_scaled_accel curr_ahrs_accel;
mip_ahrs_euler_angles curr_ahrs_euler;

void sigHandler(int sig);

u8 m_running = 1;

int main(int argc, char** argv)
{
  u32 com_port, baudrate;
  base_device_info_field device_info;
  u8  temp_string[20] = {0};
  s16 i;
  u16 device_descriptors[128]  = {0};
  u16 device_descriptors_size  = 128*2;
  u32 bit_result;
  u8  com_mode = 0;

  signal(SIGINT, sigHandler);

  if(argc != NUM_COMMAND_LINE_ARGUMENTS)
  {
   //print_command_line_usage();
   return -1;
  }


  printf("Initializing interface...");
  fflush(stdout);
  if(mip_interface_init(com_port, baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
   return -1;
  printf("done\n");


 if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback) != MIP_INTERFACE_OK)
  return -1;


     ///
     //Wait for packets to arrive
     ///

     printf("in the while loop\n");
     while(m_running > 0)
     {
      //Update the parser (this function reads the port and parses the bytes
      mip_interface_update(&device_interface);

      //Be nice to other programs
      //usleep(10);
     }


   printf("Closing interface...");
   fflush(stdout);
   if (mip_interface_close(&device_interface) != MIP_INTERFACE_OK)
   {
     printf("Failed to close the interface\n");
   }
   printf("done\n");

  return 0;
}

void sigHandler(int sig)
{
  m_running = 0;
}

// AHRS Packet Callback
void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;
 float percent;

 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  //Handle valid packets
  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   ahrs_valid_packet_count++;

   //Loop through all of the data fields
   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {
    switch(field_header->descriptor)
    {
      case MIP_AHRS_DATA_EULER_ANGLES:
      {
        //mip_ahrs_euler_angles_byteswap(mip_ahrs_euler_angles *euler_angles
        memcpy(&curr_ahrs_euler, field_data, sizeof(mip_ahrs_euler_angles));

        mip_ahrs_euler_angles_byteswap(&curr_ahrs_euler);
        convertRadiansToDegrees(&curr_ahrs_euler);
      }break;

     case MIP_AHRS_DATA_ACCEL_SCALED:
     {
      memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

      //For little-endian targets, byteswap the data field
      mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);
     }break;

     case MIP_AHRS_DATA_GYRO_SCALED:
     {
      memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

      //For little-endian targets, byteswap the data field
      mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);
     }break;

     default:
     {
       printf("Unknown case: %u\n", field_header->descriptor);
     }break;
    }
   }
//   printf("%lf, %lf, %lf\t%lf, %lf, %lf\t%lf, %lf, %lf\n", curr_ahrs_euler.yaw, curr_ahrs_euler.pitch, curr_ahrs_euler.roll, curr_ahrs_gyro.scaled_gyro[0],
//     curr_ahrs_gyro.scaled_gyro[1], curr_ahrs_gyro.scaled_gyro[2], curr_ahrs_accel.scaled_accel[0], curr_ahrs_accel.scaled_accel[1], curr_ahrs_accel.scaled_accel[2]);
  }break;

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   ahrs_checksum_error_packet_count++;
   percent = ((float)(ahrs_checksum_error_packet_count)/ (float)(ahrs_checksum_error_packet_count + ahrs_valid_packet_count + ahrs_timeout_packet_count))*100;
   fprintf(stderr, "FAILED: error count %u: percent %f\n", ahrs_checksum_error_packet_count, percent);
  }break;

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   ahrs_timeout_packet_count++;
  }break;
  default: break;
 }

}

void convertRadiansToDegrees(mip_ahrs_euler_angles *angles)
{
  angles->pitch *= 180.0/M_PI;
  angles->yaw *= 180.0/M_PI;
  angles->roll *= 180.0/M_PI;
}
