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

typedef union
{
  struct {
    float ax;
    float ay;
    float az;
    float a11;
    float a12;
    float a13;
    float a21;
    float a22;
    float a23;
    float a31;
    float a32;
    float a33;
    float gx;
    float gy;
    float gz;
    float g11;
    float g12;
    float g13;
    float g21;
    float g22;
    float g23;
    float g31;
    float g32;
    float g33;
  } _f; 
  struct {
    int ax;
    int ay;
    int az;
    int a11;
    int a12;
    int a13;
    int a21;
    int a22;
    int a23;
    int a31;
    int a32;
    int a33;
    int gx;
    int gy;
    int gz;
    int g11;
    int g12;
    int g13;
    int g21;
    int g22;
    int g23;
    int g31;
    int g32;
    int g33;
  } _d; 
}
my_misalignment_cali_t;

#endif