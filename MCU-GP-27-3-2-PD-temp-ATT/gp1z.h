#ifndef GP1Z_H
#define GP1Z_H

#include "IMU_PIG_DEFINE.h"
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

#define PRINT_SELECT_FN(x) \
  Serial.println("\nprint SELECT_FN"); \
  if (x == 0) { \
      Serial.println("SELECT SEL_DEFAULT"); \
  } else if (x == 1) { \
      Serial.println("SELECT SEL_RST"); \
  } else if (x == 2) { \
      Serial.println("SELECT SEL_FOG_1"); \
  } else if (x == 3) { \
      Serial.println("SELECT SEL_FOG_2"); \
  } else if (x == 4) { \
      Serial.println("SELECT SEL_FOG_3"); \
  } else if (x == 5) { \
      Serial.println("SELECT SEL_IMU"); \
  } else if (x == 6) { \
      Serial.println("SELECT SEL_NMEA"); \
  } else if (x == 7) { \
      Serial.println("SELECT SEL_FOG_PARA"); \
  } else if (x == 8) { \
      Serial.println("SELECT SEL_HP_TEST"); \
  }\
  else { \
      Serial.println("SELECT_FN out of range!"); \
  }

#define PRINT_OUTPUT_REG(x) \
    Serial.println("\nprint OUTPUT_REG"); \
    switch(x) { \
        case INT_SYNC: \
            Serial.println("INT_SYNC"); \
            break; \
        case EXT_SYNC: \
            Serial.println("EXT_SYNC"); \
            break; \
        case STOP_SYNC: \
            Serial.println("STOP_SYNC"); \
            break; \
        case NMEA_MODE: \
            Serial.println("NMEA_MODE"); \
            break; \
        case HP_TEST: \
            Serial.println("HP_TEST"); \
            break; \
        default: \
            Serial.println("Reg value out of range!"); \
            break; \
    }


#define PRINT_OUTPUT_MODE(x) \
Serial.println("\nprint OUTPUT_MODE"); \
if (x == MODE_RST) { \
    Serial.println("MODE_RST"); \
} else if (x == MODE_FOG) { \
    Serial.println("MODE_FOG"); \
} else if (x == MODE_IMU) { \
    Serial.println("MODE_IMU"); \
} else if (x == MODE_FOG_HP_TEST) { \
    Serial.println("MODE_FOG_HP_TEST"); \
} else if (x == MODE_NMEA) { \
    Serial.println("MODE_NMEA"); \
} else if (x == MODE_FOG_PARAMETER) { \
    Serial.println("MODE_FOG_PARAMETER"); \
}\
else { \
    Serial.println("OUTPUT_MODE out of range!"); \
}

#define PRINT_MUX_FLAG(x) \
  Serial.println("\nprint mux_flag"); \
  if(x == 0) Serial.println("MUX_OUTPUT"); \
  else if(x == 1) Serial.println("MUX_PARAMETER"); \
  else if(x == 2) Serial.println("MUX_ESCAPE"); 
#endif