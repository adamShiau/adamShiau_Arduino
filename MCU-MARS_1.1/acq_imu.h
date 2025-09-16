#pragma once
#include <Arduino.h>
#include "common.h"
#include "IMU_PIG_DEFINE.h"
#include "myUART.h" 
#include "myI2C.h"  
#include "EEPROM_MANAGE.h"
#include "myWDT.h"
#include "src/MadgwickAHRS_IMU.h"
#include "mySPI.h"
#include "myI2C.h"
#include "crcCalculator.h"

extern unsigned int CtrlReg;
extern  bool run_fog_flag;
extern unsigned long data_cnt;
extern volatile bool ISR_PEDGE;
extern my_time_t mcu_time;
extern unsigned int t_previous; 
extern const unsigned char KVH_HEADER[4];
extern const unsigned char MARS_PD_TEMP[4];
extern const unsigned char PIG_HEADER[2];
extern crcCal myCRC;
extern my_attitude_cali_t attitude_cali_coe;

extern Madgwick ahrs_attitude;
// extern void reset_SYNC()

// 對外提供 acq_imu 的原型
void acq_imu(byte &select_fn, unsigned int value, byte ch);
