#ifndef GP1Z_H
#define GP1Z_H

#include "myCLK.h"
#include "myRESCUE.h"
#include "IMU_PIG_DEFINE.h"
#include "myCALIBRATION.h"
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

#define PRINT_SELECT_FN(x) \
  Serial.println("\nprint SELECT_FN"); \
  if (x == SEL_DEFAULT) { \
      Serial.println("---->>>SELECT SEL_DEFAULT"); \
  } else if (x == SEL_RST) { \
      Serial.println("---->>>SELECT SEL_RST"); \
  } else if (x == SEL_FOG_1) { \
      Serial.println("---->>>SELECT SEL_FOG_1"); \
  } else if (x == SEL_FOG_2) { \
      Serial.println("---->>>SELECT SEL_FOG_2"); \
  } else if (x == SEL_FOG_3) { \
      Serial.println("---->>>SELECT SEL_FOG_3"); \
  } else if (x == SEL_IMU) { \
      Serial.println("---->>>SELECT SEL_IMU"); \
  } else if (x == SEL_NMEA) { \
      Serial.println("---->>>SELECT SEL_NMEA"); \
  } else if (x == SEL_ATT_NMEA) { \
      Serial.println("---->>>SELECT SEL_ATT_NMEA"); \
  } else if (x == SEL_FOG_PARA) { \
      Serial.println("---->>>SELECT SEL_FOG_PARA"); \
  } else if (x == SEL_HP_TEST) { \
      Serial.println("---->>>SELECT SEL_HP_TEST"); \ 
  }\
  else { \
      Serial.println("---->>>SELECT_FN out of range!"); \
  }

#define PRINT_OUTPUT_REG(x) \
    Serial.println("\nprint OUTPUT_REG"); \
    switch(x) { \
        case INT_SYNC: \
            Serial.println("---->>>INT_SYNC"); \
            break; \
        case EXT_SYNC: \
            Serial.println("---->>>EXT_SYNC"); \
            break; \
        case STOP_SYNC: \
            Serial.println("---->>>STOP_SYNC"); \
            break; \
        case NMEA_MODE: \
            Serial.println("---->>>NMEA_MODE"); \
            break; \
        case HP_TEST: \
            Serial.println("---->>>HP_TEST"); \
            break; \
        default: \
            Serial.println("---->>>Reg value out of range!"); \
            break; \
    }


#define PRINT_OUTPUT_MODE(x) \
Serial.println("\nprint OUTPUT_MODE"); \
if (x == MODE_RST) { \
    Serial.println("---->>>MODE_RST"); \
} else if (x == MODE_FOG) { \
    Serial.println("---->>>MODE_FOG"); \
} else if (x == MODE_IMU) { \
    Serial.println("---->>>MODE_IMU"); \
} else if (x == MODE_FOG_HP_TEST) { \
    Serial.println("---->>>MODE_FOG_HP_TEST"); \
} else if (x == MODE_NMEA) { \
    Serial.println("---->>>MODE_NMEA"); \
} else if (x == MODE_ATT_NMEA) { \
    Serial.println("---->>>MODE_ATT_NMEA"); \
} else if (x == MODE_FOG_PARAMETER) { \
    Serial.println("---->>>MODE_FOG_PARAMETER"); \
}\
else { \
    Serial.println("---->>>OUTPUT_MODE out of range!"); \
}

#define PRINT_MUX_FLAG(x) \
  Serial.println("\nprint mux_flag"); \
  if(x == 0) Serial.println("---->>>MUX_OUTPUT"); \
  else if(x == 1) Serial.println("---->>>MUX_PARAMETER"); \
  else if(x == 2) Serial.println("---->>>MUX_ESCAPE"); 
  
#endif