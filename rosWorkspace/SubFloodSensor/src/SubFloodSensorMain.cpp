#include <signal.h>

#include "ros/ros.h"
#include "SubFloodSensor/FloodMsg.h"
#include "common/SerialInterface.hpp" //replace with a library
#include "common/PortableTypes.hpp" //replace with a library
#include "common/BufferUtils.hpp" //replace with a library

#define MIN_PACKET_SIZE 3
#define MAX_PACKET_SIZE 255 + MIN_PACKET_SIZE
#define PACKET_BYTES_PER_READING_OFFSET 2
#define PACKET_SIZE_OFFSET 1
#define PACKET_DATA_OFFSET 3

void catchSig(int sig);
Int32 recvPacket(SerialInterface* pIface, UInt8* pBuf, Int32 size);
Int32 recvHeader(SerialInterface* pIface, UInt8* pBuf, Int32 size);
void printBuffer(UInt8* pBuf, Int32 size);
UInt32 getSeconds();

bool m_running = true;

int main(int argc, char** argv)
{
  std::string enclosure = "unspecified";
  UInt32 timeOut = 30;
  UInt32 numberOfSensors = 2;
  UInt32 tripCurrentValue = 600;
  UInt8* pBuf = new UInt8[MAX_PACKET_SIZE];
  int32_t baudRate = 115200;
  SerialInterface sp("/dev/ttyUSB0", baudRate);
  signal(SIGINT, catchSig);

  if (!sp.openInterface())
  {
    printf("Failed to open the serial port\n");
    exit(0);
  }

  ros::init(argc, argv, "SubImuController");
  ros::NodeHandle nh;

  ros::Publisher floodPub = nh.advertise<SubFloodSensor::FloodMsg>("FloodState", 1000);

  ros::Rate loop_rate(1);

  UInt32 aLastSeen[numberOfSensors];
  for (UInt32 i = 0; i < numberOfSensors; i++)
  {
    aLastSeen[i] = getSeconds();
  }

  while (ros::ok() && m_running)
  {
    if (sp.isGood())
    {
      Int32 size = recvPacket(&sp, pBuf, MAX_PACKET_SIZE);

      if (size > 0)
      {
        //received packet
        UInt8 packetSize = pBuf[PACKET_SIZE_OFFSET];
        UInt8 bytesPerReading = pBuf[PACKET_BYTES_PER_READING_OFFSET];

        switch(bytesPerReading)
        {
          case sizeof(UInt8):
          {
            for (UInt8 i = 0; i < packetSize && i < numberOfSensors; i++)
            {
              UInt8 value = pBuf[PACKET_DATA_OFFSET+i];
              printf("node %i had value of %u\n", i, value);
              //TODO
            }
          }
          break;

          case sizeof(UInt16):
          {
            for (UInt8 i = 0; i < packetSize && (i/sizeof(UInt16)) < numberOfSensors; i += sizeof(UInt16))
            {
              UInt16 value = getUInt16(&pBuf[PACKET_DATA_OFFSET+i]);
              UInt32 pos = i/sizeof(UInt16);
              aLastSeen[pos] = getSeconds();

              SubFloodSensor::FloodMsg msg;
              msg.enclosure = enclosure;
              msg.sensorNumber = pos;
              msg.state = (value > tripCurrentValue ? 1 : 0);
              floodPub.publish(msg);
            }
          }
          break;

          case sizeof(UInt32):
          {
            for (UInt8 i = 0; i < packetSize && (i/sizeof(UInt32)) < numberOfSensors; i += sizeof(UInt32))
            {
              UInt32 value = getUInt32(&pBuf[PACKET_DATA_OFFSET+i]);
              printf("node %i had value of %u\n", i/sizeof(UInt32), value);
              //TODO
            }
          }
          break;
        }
      }
      else
      {
        //printf("Received 0\n");
      }
    }
    else
    {
      sp.closeInterface();
      printf("SerialPort is bad or not open, opening\n");
      if (!sp.openInterface())
      {
        sleep(1);
      }
    }

    //check all devices
    for (UInt32 i = 0; i < numberOfSensors; i++)
    {
      if (aLastSeen[i] + timeOut < getSeconds())
      {
        //send error
        SubFloodSensor::FloodMsg msg;
        msg.enclosure = enclosure;
        msg.sensorNumber = i;
        msg.state = 2; //unknown state

        printf("Have not seen sensor %u, sending error message\n", i);
        floodPub.publish(msg);
        aLastSeen[i] = 0xffffffff - timeOut; //never to be seen from again
      }
    }
  }

  return 1;
}

Int32 recvHeader(SerialInterface* pIface, UInt8* pBuf, Int32 size)
{
  Int32 ret = 0;

  Int32 maxRetries = 100;

  //wait for 'S'
  while (ret < 1 && maxRetries > 0 && pIface->isGood())
  {
    Int32 size = pIface->recv(&pBuf[ret], 1);
    if (size > 0)
    {
      ret += size;
    }
    else
    {
      maxRetries--;
    }
  }

  if (size > 0 && pBuf[0] == 'S')
  {
    while (ret < MIN_PACKET_SIZE && maxRetries > 0 && pIface->isGood())
    {
      Int32 size = pIface->recv(&pBuf[ret], MIN_PACKET_SIZE - ret);
      if (size > 0)
      {
        ret += size;
      }
      else
      {
        maxRetries--;
      }
    }
  }
  else
  {
    ret = 0;
  }

  return ret;
}

Int32 recvPacket(SerialInterface* pIface, UInt8* pBuf, Int32 size)
{
  //get header
  Int32 packetSize = 0;
  Int32 maxRetries = 100;

  memset(pBuf, 0, size);

  packetSize = recvHeader(pIface, pBuf, size);

  if (pBuf[0] == 'S' && packetSize == MIN_PACKET_SIZE)
  {
    //read until we receive an 'E'
    Int32 currentSize = MIN_PACKET_SIZE;
    Int32 newSize = pBuf[PACKET_SIZE_OFFSET];
    printf("Grabbing %i more bytes\n", newSize);
    while (currentSize < newSize + MIN_PACKET_SIZE && maxRetries > 0 && pIface->isGood())
    {
      Int32 ret = pIface->recv(&pBuf[currentSize], (newSize + MIN_PACKET_SIZE) - currentSize);
      if (ret > 0)
      {
        currentSize += ret;
      }
      else
      {
        maxRetries--;
      }
    }

    //verify packet
    if (currentSize > MIN_PACKET_SIZE && maxRetries > 0)
    {
      UInt8 newSize = (UInt8)pBuf[PACKET_SIZE_OFFSET];
      UInt8 bytesPerReading = pBuf[PACKET_BYTES_PER_READING_OFFSET];

      if (currentSize == newSize + MIN_PACKET_SIZE && newSize % bytesPerReading == 0)
      {
        packetSize = currentSize;
      }
      else
      {
        printf("Throwing away %i bytes2. %i vs %u. bytes per reading %u\n", currentSize, currentSize, newSize, bytesPerReading);
        printBuffer(pBuf, currentSize);
      }
    }
    else
    {
      printf("Throwing away %i bytes\n", currentSize);
    }
  }
  else
  {
    packetSize = 0;
  }

  return packetSize;
}

void catchSig(int sig)
{
  m_running = false;
  printf("SubFloodSensor: Closing\n");
}

void printBuffer(UInt8* pBuf, Int32 size)
{
  printf("Buffer:\n");
  for (Int32 i = 0; i < size; i++)
  {
    printf("%x ", pBuf[i]);
  }
  printf("\n");
}

UInt32 getSeconds()
{
  UInt32 seconds = 0;

  struct timeval tv;
  struct timezone tz;

  if (gettimeofday(&tv, &tz) == 0)
  {
    seconds = tv.tv_sec;
  }

  return seconds;
}
