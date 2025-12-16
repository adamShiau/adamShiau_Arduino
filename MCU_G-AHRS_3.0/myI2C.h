#pragma once
#include <Wire.h>

// I2C 裝置與 API（僅宣告）
extern TwoWire myWire;
void SERCOM0_Handler();
void myI2C_init(void);

// EEPROM driver
#include "EEPROM_24AA32A_I2C.h"
// #include "EEPROM_24AA32A_I2C.h"
extern EEPROM_24AA32A_I2C eeprom;
