#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "types.h"
#include "defs.h"
#include "i2c.h"

#define STATUS_LED 5 //stat LED is on PB5

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define ITG3200_R 0xD1	// ADD pin is pulled low
#define ITG3200_W 0xD0 

void i2cInit(void);
void gyro_init(void);
void magnetometer_init(void);
void accelerometer_init(void);
int16_t x_accel(void);
int16_t y_accel(void);
int16_t z_accel(void);
int16_t x_gyro(void);
int16_t y_gyro(void);
int16_t z_gyro(void);
void sampleMagnetometer(void);

///============Global Vars=========/////////////////
int16_t x_mag, y_mag, z_mag; //x, y, and z magnetometer values

void setup()
{
  //1 = output, 0 = input
    DDRB = 0b01100000; //PORTB4, B5 output for stat LED
    DDRC = 0b00010000; //PORTC4 (SDA), PORTC5 (SCL), PORTC all others are inputs
    DDRD = 0b00000010; //PORTD (TX output on PD1)
    PORTC = 0b00110000; //pullups on the I2C bus
	
    cbi(PORTB, 5);
    
    i2cInit();
    accelerometer_init();
    magnetometer_init();
    gyro_init();
    
    Serial.begin(57600);
    Serial.setTimeout(1000);
}

void loop()
{
  char command;
               
  if(Serial.readBytes(&command, 1) == 1)
  {
    if(command == 'G')
    {                  
      readGyro();
    }
    else if(command == 'A')
    {
      readAccelerometer();
    }
    else if(command == 'M')
    {
      readMagnetometer();
    }
  }
}

void readGyro(void)
{   
  uint16_t xGyro = x_gyro();
  uint16_t yGyro = y_gyro();
  uint16_t zGyro = z_gyro();
  unsigned char gyroBuf[9];
  
  gyroBuf[0] = 0xF1;
  gyroBuf[1] = 0xF5;
  gyroBuf[2] = 0xF9;
  memcpy(&gyroBuf[3], &xGyro, 2); 
  memcpy(&gyroBuf[5], &yGyro, 2); 
  memcpy(&gyroBuf[7], &zGyro, 2); 

  Serial.write(gyroBuf, 9);
}

void readAccelerometer(void)
{	
  uint16_t xAccel = x_accel();
  uint16_t yAccel = y_accel();
  uint16_t zAccel = z_accel();
  unsigned char accelBuf[9];
  
  accelBuf[0] = 0xF1;
  accelBuf[1] = 0xF5;
  accelBuf[2] = 0xF9;
  memcpy(&accelBuf[3], &xAccel, 2); 
  memcpy(&accelBuf[5], &yAccel, 2); 
  memcpy(&accelBuf[7], &zAccel, 2); 

  Serial.write(accelBuf, 9);
}

void readMagnetometer(void)
{
  unsigned char magBuf[9];
  
  sampleMagnetometer();
  
  magBuf[0] = 0xF1;
  magBuf[1] = 0xF5;
  magBuf[2] = 0xF9;
  memcpy(&magBuf[3], &x_mag, 2); 
  memcpy(&magBuf[5], &y_mag, 2); 
  memcpy(&magBuf[7], &z_mag, 2); 

  Serial.write(magBuf, 9);
}

void gyro_init(void)
{
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write 0xB4
	i2cWaitForComplete();
	i2cSendByte(0x3E);	// write register address
	i2cWaitForComplete();
	i2cSendByte(0x80);
	i2cWaitForComplete();
	i2cSendStop();
	
	delay_ms(10);
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write 0xB4
	i2cWaitForComplete();
	i2cSendByte(0x16);	// write register address
	i2cWaitForComplete();
	i2cSendByte(0x02);  // DLPF_CFG = 2 FS_SEL = 0  00010
	i2cWaitForComplete();
	i2cSendStop();	
	
	delay_ms(10);

        i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write 0xB4
	i2cWaitForComplete();
	i2cSendByte(0x15);	// write register address
	i2cWaitForComplete();
	i2cSendByte(0x09);      // Internal sampling rate of 100 Hz
	i2cWaitForComplete();
	i2cSendStop();	
	
	delay_ms(10);
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write 0xB4
	i2cWaitForComplete();
	i2cSendByte(0x3E);	// write register address
	i2cWaitForComplete();
	i2cSendByte(0x01);      //Use x-axis for clk
	i2cWaitForComplete();
	i2cSendStop();
}

void accelerometer_init(void)
{

	//initialize
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x2D);    //power register
	i2cWaitForComplete();
	i2cSendByte(0x08);    //measurement mode
	i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x31);    //data format
	i2cWaitForComplete();
	i2cSendByte(0x08);    //full resolution
	i2cWaitForComplete();
	i2cSendStop();

}

void sampleMagnetometer(void)
{
	/*
		The magnetometer values must be read consecutively
		in order to move the magnetometer pointer. Therefore the x, y, and z
		outputs need to be kept in this function. To read the magnetometer 
		values, call the function magnetometer(), then global vars 
		x_mag, y_mag, z_mag.
	*/
	
	magnetometer_init();
	
	uint8_t xh, xl, yh, yl, zh, zl;
	
	//must read all six registers plus one to move the pointer back to 0x03
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0x3D);    //write to HMC
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	xh = i2cGetReceivedByte();	//x high byte
	i2cWaitForComplete();
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	xl = i2cGetReceivedByte();	//x low byte
	i2cWaitForComplete();
	x_mag = xl|(xh << 8);
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	zh = i2cGetReceivedByte();	
	i2cWaitForComplete();      //z high byte
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	zl = i2cGetReceivedByte();	//z low byte
	i2cWaitForComplete();
	z_mag = zl|(zh << 8);
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	yh = i2cGetReceivedByte();	//y high byte
	i2cWaitForComplete();
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	yl = i2cGetReceivedByte();	//y low byte
	i2cWaitForComplete();
	y_mag = yl|(yh << 8);
	
	i2cSendByte(0x3D);         //must reach 0x09 to go back to 0x03
	i2cWaitForComplete();
	
	i2cSendStop();	
}

void magnetometer_init(void)
{
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0x3C);    //write to HMC
	i2cWaitForComplete();
	i2cSendByte(0x00);    //mode register
	i2cWaitForComplete();
	i2cSendByte(0x70);    //8 average, 15Hz, normal measurement
	i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0x3C);    //write to HMC
	i2cWaitForComplete();
	i2cSendByte(0x01);    //mode register
	i2cWaitForComplete();
	i2cSendByte(0xA0);    //gain = 5
	i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0x3C);    //write to HMC
	i2cWaitForComplete();
	i2cSendByte(0x02);    //mode register
	i2cWaitForComplete();
	i2cSendByte(0x00);    //continuous measurement mode
	i2cWaitForComplete();
	i2cSendStop();
}

int16_t x_gyro(void)
{
	uint16_t  xh, xl;
        int16_t data;
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write 
	i2cWaitForComplete();
	i2cSendByte(0x1D);	   // x high address
	i2cWaitForComplete();
	i2cSendStart();
	
	i2cSendByte(ITG3200_R);	// read
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	
	xh = i2cGetReceivedByte();	// Get MSB result
	i2cWaitForComplete();
	i2cSendStop();
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write
	i2cWaitForComplete();
	i2cSendByte(0x1E);	    // x low address
	i2cWaitForComplete();
	i2cSendStart();
	
	i2cSendByte(ITG3200_R);	// read
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	
	xl = i2cGetReceivedByte();	// Get LSB result
	i2cWaitForComplete();
	i2cSendStop();
	
	data = xl|(xh << 8);
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	return data;
}

int16_t y_gyro(void)
{
	uint16_t  xh, xl;
        int16_t data;
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write
	i2cWaitForComplete();
	i2cSendByte(0x1F);	// y high address
	i2cWaitForComplete();
	i2cSendStart();
	
	i2cSendByte(ITG3200_R);	 // read
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	
	xh = i2cGetReceivedByte();	// Get MSB result
	i2cWaitForComplete();
	i2cSendStop();
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write
	i2cWaitForComplete();
	i2cSendByte(0x20);	// y low address
	i2cWaitForComplete();
	i2cSendStart();
	
	i2cSendByte(ITG3200_R);	// read
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	
	xl = i2cGetReceivedByte();	// Get LSB result
	i2cWaitForComplete();
	i2cSendStop();
	
	data = xl|(xh << 8);
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	return data;
}

int16_t z_gyro(void)
{
	uint16_t  xh, xl;
        int16_t data;
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write
	i2cWaitForComplete();
	i2cSendByte(0x21);	// z high address
	i2cWaitForComplete();
	i2cSendStart();
	
	i2cSendByte(ITG3200_R);	// read
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	
	xh = i2cGetReceivedByte();	// Get MSB result
	i2cWaitForComplete();
	i2cSendStop();
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(ITG3200_W);	// write
	i2cWaitForComplete();
	i2cSendByte(0x22);	// z low address
	i2cWaitForComplete();
	i2cSendStart();
	
	i2cSendByte(ITG3200_R);	// read
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	
	xl = i2cGetReceivedByte();	// Get LSB result
	i2cWaitForComplete();
	i2cSendStop();
	
	data = xl|(xh << 8);
	
	cbi(TWCR, TWEN);	// Disable TWI
	sbi(TWCR, TWEN);	// Enable TWI
	
	return data;
}

int16_t x_accel(void)
{		
	//0xA6 for a write
	//0xA7 for a read
	
	uint8_t dummy, xh, xl;
	int16_t xo;
	
	//0x32 data registers
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x32);    //X0 data register
	i2cWaitForComplete();
	
	i2cSendStop();		 //repeat start
	i2cSendStart();

	i2cWaitForComplete();
	i2cSendByte(0xA7);    //read from ADXL
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	xl = i2cGetReceivedByte();	//x low byte
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	dummy = i2cGetReceivedByte();	//must do a multiple byte read?
	i2cWaitForComplete();
	i2cSendStop();	
	
	//0x33 data registers
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x33);    //X1 data register
	i2cWaitForComplete();
	
	i2cSendStop();		 //repeat start
	i2cSendStart();

	i2cWaitForComplete();
	i2cSendByte(0xA7);    //read from ADXL
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	xh = i2cGetReceivedByte();	//x high byte
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	dummy = i2cGetReceivedByte();	//must do a multiple byte read?
	i2cWaitForComplete();
	i2cSendStop();
	xo = xl|(xh << 8);
	return xo;
}

int16_t y_accel(void)
{		
	//0xA6 for a write
	//0xA7 for a read
	
	uint8_t dummy, yh, yl;
	int16_t yo;
	
	//0x34 data registers
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x34);    //Y0 data register
	i2cWaitForComplete();
	
	i2cSendStop();		 //repeat start
	i2cSendStart();

	i2cWaitForComplete();
	i2cSendByte(0xA7);    //read from ADXL
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	yl = i2cGetReceivedByte();	//x low byte
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	dummy = i2cGetReceivedByte();	//must do a multiple byte read?
	i2cWaitForComplete();
	i2cSendStop();	
	
	//0x35 data registers
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x35);    //Y1 data register
	i2cWaitForComplete();
	
	i2cSendStop();		 //repeat start
	i2cSendStart();

	i2cWaitForComplete();
	i2cSendByte(0xA7);    //read from ADXL
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	yh = i2cGetReceivedByte();	//y high byte
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	dummy = i2cGetReceivedByte();	//must do a multiple byte read?
	i2cWaitForComplete();
	i2cSendStop();
	yo = yl|(yh << 8);
	return yo;
}

int16_t z_accel(void)
{	
	//0xA6 for a write
	//0xA7 for a read
	
	uint8_t dummy, zh, zl;
	int16_t zo;
	
	//0x36 data registers
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x36);    //Z0 data register
	i2cWaitForComplete();
	
	i2cSendStop();		 //repeat start
	i2cSendStart();

	i2cWaitForComplete();
	i2cSendByte(0xA7);    //read from ADXL
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	zl = i2cGetReceivedByte();	//z low byte
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	dummy = i2cGetReceivedByte();	//must do a multiple byte read?
	i2cWaitForComplete();
	i2cSendStop();	
	
	//0x37 data registers
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0xA6);    //write to ADXL
	i2cWaitForComplete();
	i2cSendByte(0x37);    //Z1 data register
	i2cWaitForComplete();
	
	i2cSendStop();		 //repeat start
	i2cSendStart();

	i2cWaitForComplete();
	i2cSendByte(0xA7);    //read from ADXL
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	zh = i2cGetReceivedByte();	//z high byte
	i2cWaitForComplete();
	i2cReceiveByte(FALSE);
	i2cWaitForComplete();
	dummy = i2cGetReceivedByte();	//must do a multiple byte read?
	i2cWaitForComplete();
	i2cSendStop();
	zo = zl|(zh << 8);	
	return zo;
}

