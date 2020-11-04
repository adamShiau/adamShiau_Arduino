/* Arduino I2cMaster Library
 * Copyright (C) 2010 by William Greiman
 *
 * This file is part of the Arduino I2cMaster Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino I2cMaster Library.  If not, see
 * <http://www.gnu.org/licenses/>.*/



#include <Arduino.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

 #ifndef I2CMASTER_H
 #define I2CMASTER_H

/** Delay used for software I2C */
uint8_t const I2C_DELAY_USEC = 2;

/** Bit to or with address for read start and read restart */
uint8_t const I2C_READ = 1;

/** Bit to or with address for write start and write restart */
uint8_t const I2C_WRITE = 0;
//------------------------------------------------------------------------------
// Status codes in TWSR - names are from Atmel TWSR.h with TWSR_ added


/**
 * \class SoftI2cMaster
 * \brief Software I2C master class
 */
class SoftI2cMaster {
 public:
  SoftI2cMaster();
  void init(uint8_t sdaPin, uint8_t sclPin);
  uint8_t read(uint8_t last);
  bool restart(uint8_t addressRW);
  bool start(uint8_t addressRW);
  void stop(void);
  bool write(uint8_t b);
 private:
  //SoftI2cMaster() {}
  uint8_t sdaPin_, sdabit, sdaport;
  uint8_t sclPin_, sclbit, sclport;
  uint8_t timer, oldSREG;
  void turnOffPWM(uint8_t timer);
  volatile uint8_t *sdareg, *sdaout, *sclreg, *sclout;
};

#endif
//------------------------------------------------------------------------------


