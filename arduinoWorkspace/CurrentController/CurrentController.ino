#include "TinyWireS.h"

#define I2C_SLAVE_ADDR  0x23

void setup()
{
  TinyWireS.begin(I2C_SLAVE_ADDR);
}

void loop()
{
  if (TinyWireS.available())
  {
    byte cmdByte = TinyWireS.receive();
    
    switch (cmdByte)
    {
      case 0x1: //
        TinyWireS.send(0xaa);
        break;
        
      case 0x2:
        TinyWireS.send(0xbb);
        break;
    }
  }
}
