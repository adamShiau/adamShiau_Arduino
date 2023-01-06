/*
  This file is part of the Arduino_ASM330LHH library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "ASM330LHH.h"

#define ASM330LHH_ADDRESS            0x6A

#define ASM330LHH_WHO_AM_I_REG       0X0F
#define ASM330LHH_CTRL1_XL           0X10
#define ASM330LHH_CTRL2_G            0X11

#define ASM330LHH_STATUS_REG         0X1E

#define ASM330LHH_CTRL6_C            0X15
#define ASM330LHH_CTRL7_G            0X16
#define ASM330LHH_CTRL8_XL           0X17

#define ASM330LHH_OUTX_L_G           0X22
#define ASM330LHH_OUTX_H_G           0X23
#define ASM330LHH_OUTY_L_G           0X24
#define ASM330LHH_OUTY_H_G           0X25
#define ASM330LHH_OUTZ_L_G           0X26
#define ASM330LHH_OUTZ_H_G           0X27

#define ASM330LHH_OUTX_L_XL          0X28
#define ASM330LHH_OUTX_H_XL          0X29
#define ASM330LHH_OUTY_L_XL          0X2A
#define ASM330LHH_OUTY_H_XL          0X2B
#define ASM330LHH_OUTZ_L_XL          0X2C
#define ASM330LHH_OUTZ_H_XL          0X2D

/*** CTRL1_XL register parameters***/
#define ODR_104		0x40
#define ODR_208		0x50
#define ODR_416		0x60
#define FS_2G		0b0000
#define FS_4G		0b1000
#define FS_8G		0b1100
#define FS_16G		0b0100
#define BW_50Hz		0b0011
#define BW_100Hz	0b0010
#define BW_200Hz	0b0001
#define BW_400Hz	0b0000

/*** CTRL2_G register parameters***/
#define FS_250DPS	0b0000
#define FS_500DPS	0b0100
#define FS_1000DPS	0b1000
#define FS_2000DPS	0b1100

#define SENS_AXLM_4G 0.000122
#define SENS_GYRO_250 0.00875

ASM330LHHClass::ASM330LHHClass(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress)
{
}

ASM330LHHClass::ASM330LHHClass(SPIClass& spi, int csPin, int spi_clock) :
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  // _irqPin(irqPin),
  _spiSettings(spi_clock, MSBFIRST, SPI_MODE0)
{
}

ASM330LHHClass::~ASM330LHHClass()
{
}

int ASM330LHHClass::begin()
{
  if (_spi != NULL) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    // _spi->begin();
	Serial.println("SPI MODE");
  } else {
    _wire->begin();
	Serial.println("I2C MODE");
  }

  if (readRegister(ASM330LHH_WHO_AM_I_REG) != 0x6B) {
	Serial.println(readRegister(ASM330LHH_WHO_AM_I_REG), HEX);
    end();
    return 0;
  }

  //set the gyroscope control register to work at 416 Hz, 250 dps and in bypass mode
  writeRegister(ASM330LHH_CTRL2_G, ODR_416|FS_250DPS);

  // Set the Accelerometer control register to work at 416 Hz, +/-4G,and in bypass mode and enable ODR/4
  // low pass filter(check figure9 of ASM330LHH's datasheet)
  writeRegister(ASM330LHH_CTRL1_XL, ODR_416|FS_4G|BW_100Hz);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(ASM330LHH_CTRL7_G, 0x00);

  // Set the ODR config register to ODR/4
  writeRegister(ASM330LHH_CTRL8_XL, 0x09);

  return 1;
}

void ASM330LHHClass::end()
{
  if (_spi != NULL) {
    _spi->end();
    digitalWrite(_csPin, LOW);
    pinMode(_csPin, INPUT);
  } else {
    writeRegister(ASM330LHH_CTRL2_G, 0x00);
    writeRegister(ASM330LHH_CTRL1_XL, 0x00);
    _wire->end();
  }
}

void ASM330LHHClass::print_AccelerationData(unsigned char *temp_a, int& x, int& y, int& z, unsigned int t_new, unsigned int& t_old)
{
	x = temp_a[1]<<8 | temp_a[0];
	if((x>>15) == 1) x = x - (1<<16);
	y = temp_a[3]<<8 | temp_a[2];
	if((y>>15) == 1) y = y - (1<<16);
	z = temp_a[5]<<8 | temp_a[4];
	if((z>>15) == 1) z = z - (1<<16);
	
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	// Serial.print(wx, HEX);
	Serial.print((float)x*SENS_AXLM_4G);
	Serial.print('\t');
	// Serial.print(wy, HEX);
	Serial.print((float)y*SENS_AXLM_4G);
	Serial.print('\t');
	// Serial.println(wz, HEX);
	Serial.println((float)z*SENS_AXLM_4G);
	t_old = t_new;
}

int ASM330LHHClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(ASM330LHH_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }
/*** +/- 4g w/ 16bit ***/
  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int ASM330LHHClass::readFakeAcceleration(int& x, int& y, int& z)
{
  int16_t data[3];

  x = 1;
  y = 2;
  z = 3;

  return 1;
}

int ASM330LHHClass::readFakeAcceleration(unsigned char data[6])
{

	data[0] = 1;
	data[1] = 0;
	data[2] = 2;
	data[3] = 0;
	data[4] = 3;
	data[5] = 0;

  return 1;
}

int ASM330LHHClass::readAcceleration(unsigned char data[6])
{
	while(!accelerationAvailable());
	if (!readRegisters(ASM330LHH_OUTX_L_XL, (uint8_t*)data, 6)) {
		return 0;
	}
	return 1;
}

int ASM330LHHClass::readAcceleration(int& x, int& y, int& z)
{
	int16_t data[3];

	while(!accelerationAvailable());
	if (!readRegisters(ASM330LHH_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
	x = NAN;
	y = NAN;
	z = NAN;

	return 0;
	}
	/*** +/- 4g w/ 16bit ***/
	x = (int)data[0];
	y = (int)data[1];
	z = (int)data[2];

	return 1;
}

int ASM330LHHClass::accelerationAvailable()
{
  if (readRegister(ASM330LHH_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float ASM330LHHClass::accelerationSampleRate()
{
  uint8_t data;
  float odr = 0;
  readRegisters(ASM330LHH_CTRL1_XL, &data, sizeof(data));
  switch (data>>4)
  {
  case 1:
    odr = 12.5;
    break;
  case 2:
    odr = 26.0;
    break;
  case 3:
    odr = 52.0;
    break;
  case 4:
    odr = 104.0;
    break;
  case 5:
    odr = 208.0;
    break;
  case 6:
    odr = 416.0;
    break;
  case 7:
    odr = 833.0;
    break;
  case 8:
    odr = 1660.0;
    break;
  case 9:
    odr = 3330.0;
    break;
  case 10:
    odr = 6660.0;
    break;
  
  default:
    odr = 0.0;
    break;
  }
  return odr;
}

void ASM330LHHClass::print_GyroData(unsigned char *temp_a, int& x, int& y, int& z, unsigned int t_new, unsigned int& t_old)
{
	x = temp_a[1]<<8 | temp_a[0];
	if((x>>15) == 1) x = x - (1<<16);
	y = temp_a[3]<<8 | temp_a[2];
	if((y>>15) == 1) y = y - (1<<16);
	z = temp_a[5]<<8 | temp_a[4];
	if((z>>15) == 1) z = z - (1<<16);
	
	t_new = micros();
	Serial.print(t_new - t_old);
	Serial.print('\t');
	// Serial.print(wx, HEX);
	Serial.print((float)x*SENS_GYRO_250);
	Serial.print('\t');
	// Serial.print(wy, HEX);
	Serial.print((float)y*SENS_GYRO_250);
	Serial.print('\t');
	// Serial.println(wz, HEX);
	Serial.println((float)z*SENS_GYRO_250);
	t_old = t_new;
}

int ASM330LHHClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(ASM330LHH_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }
/*** +/- 2000 w/ 16bit ***/

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int ASM330LHHClass::readFakeGyroscope(int& x, int& y, int& z)
{
	int16_t data[3];


	x = 4;
	y = 5;
	z = 6;

	return 1;
}

int ASM330LHHClass::readFakeGyroscope(unsigned char data[6])
{
	data[0] = 1;
	data[1] = 0;
	data[2] = 2;
	data[3] = 0;
	data[4] = 3;
	data[5] = 0;

	return 1;
}

int ASM330LHHClass::readGyroscope(unsigned char data[6])
{
	while(!gyroscopeAvailable());
	// Serial.println(sizeof(data));
	if (!readRegisters(ASM330LHH_OUTX_L_G, (uint8_t*)data, 6)) {
	// x = NAN;
	// y = NAN;
	// z = NAN;

	return 0;
	}
	return 1;
}

int ASM330LHHClass::readGyroscope(int& x, int& y, int& z)
{
	int16_t data[3];

	while(!gyroscopeAvailable());
	// Serial.println(sizeof(data));
	if (!readRegisters(ASM330LHH_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
	x = NAN;
	y = NAN;
	z = NAN;

	return 0;
	}

	x = (int)data[0];
	y = (int)data[1];
	z = (int)data[2];

	return 1;
}

int ASM330LHHClass::gyroscopeAvailable()
{
  if (readRegister(ASM330LHH_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float ASM330LHHClass::gyroscopeSampleRate()
{
  uint8_t data;
  float odr = 0;
  readRegisters(ASM330LHH_CTRL2_G, &data, sizeof(data));
  switch (data>>4)
  {
  case 1:
    odr = 12.5;
    break;
  case 2:
    odr = 26.0;
    break;
  case 3:
    odr = 52.0;
    break;
  case 4:
    odr = 104.0;
    break;
  case 5:
    odr = 208.0;
    break;
  case 6:
    odr = 416.0;
    break;
  case 7:
    odr = 833.0;
    break;
  case 8:
    odr = 1660.0;
    break;
  
  default:
    odr = 0.0;
    break;
  }
  return odr;
}

int ASM330LHHClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int ASM330LHHClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(0x80 | address);
    _spi->transfer(data, length);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    if (_wire->endTransmission(false) != 0) {
      return -1;
    }

    if (_wire->requestFrom(_slaveAddress, length) != length) {
      return 0;
    }

    for (size_t i = 0; i < length; i++) {
      *data++ = _wire->read();
    }
  }
  return 1;
}

int ASM330LHHClass::writeRegister(uint8_t address, uint8_t value)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(address);
    _spi->transfer(value);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
      return 0;
    }
  }
  return 1;
}

// #ifdef ARDUINO_AVR_UNO_WIFI_REV2
// ASM330LHHClass IMU(SPI, SPIIMU_SS, SPIIMU_INT);
// #else
// ASM330LHHClass IMU(Wire, ASM330LHH_ADDRESS);
// #endif
