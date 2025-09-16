#pragma once
#include <Arduino.h>
#include "common.h"
#include "IMU_PIG_DEFINE.h"
// #include "myUART.h" 


extern unsigned int CtrlReg;
extern  bool run_fog_flag;

// 對外提供 acq_imu 的原型
void acq_imu(byte &select_fn, unsigned int value, byte ch);
