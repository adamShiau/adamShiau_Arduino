#line 1 "C:\\Users\\user\\Documents\\Arduino\\MCU-GP-28-3-PD\\src\\MySensorSetting.h"
#ifndef MYSENSORSETTING_h
#define MYSENSORSETTING_h

#include <LSM6DS3Sensor.h>
#include "Arduino_BMI270_BMM150.h"


const unsigned char HEADER[2] = {0xFA, 0xFF};

typedef union
{
  float float_val[3];
  uint8_t bin_val[12];
  int int_val[3];
}my_data3;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  int int_val;
}my_data1;

typedef union
{
  float float_val;
  uint8_t bin_val[4];
  uint32_t ulong_val;
}my_time_t;

class Nano33IOT{
    private:
        LSM6DS3Sensor AccGyr;
    public:
        Nano33IOT();
        void init();
        void getIMUData(float (&omg)[3], float (&acc)[3]);
};

#endif