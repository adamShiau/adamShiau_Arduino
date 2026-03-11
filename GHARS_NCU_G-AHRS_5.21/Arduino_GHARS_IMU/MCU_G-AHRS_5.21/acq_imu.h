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
#include "crcCalculator.h"


// ================================
// IMU packet layout constants
// ================================
// 2026/03/06, Send output data parameters setting.

// Packet format:
// [0:3]   Header
// [4:15]  Gyro xyz (3 x float)
// [16:27] Acc xyz  (3 x float)
// [28:31] PD temp  (1 x float)
// [32:35] MCU time (1 x uint32/float, depends on definition)
// [36:47] Attitude pitch/roll/yaw (3 x float)
// [48]    GNSS status (1 byte)
// [49:52] CRC32

constexpr uint8_t IMU_PACKET_TOTAL_LEN = 53;
constexpr uint8_t IMU_PACKET_NOCRC_LEN = 49;

constexpr uint8_t IMU_GYRO_OFFSET = 4;
constexpr uint8_t IMU_ACCL_OFFSET = 16;
constexpr uint8_t IMU_TEMP_OFFSET = 28;
constexpr uint8_t IMU_TIME_OFFSET = 32;
constexpr uint8_t IMU_ATT_OFFSET  = 36;
constexpr uint8_t IMU_GPS_OFFSET  = 48;
constexpr uint8_t IMU_CRC_OFFSET  = 49;

constexpr size_t FOG_PD_TEMP_OFFSET   = 12;
// ================================


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
