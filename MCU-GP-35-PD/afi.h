#ifndef AFI_H
#define AFI_H

#include "IMU_PIG_DEFINE.h"
#include "EEPROM_MANAGE.h"
#include "wiring_private.h"
#include "crcCalculator.h"
#include <TinyGPSPlus.h>
#include "myI2C.h"
#include "mySPI.h"
#include "myUART.h"
#include "myPWM.h"
#include "myWDT.h"

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  uint32_t ulong_val;
}my_time_t;

typedef union
{
  float float_val[3];
  uint8_t bin_val[12];
  int int_val[3];
}my_acc_t;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  int int_val;
}my_float_t;

#endif