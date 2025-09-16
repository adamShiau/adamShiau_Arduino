#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <ASM330LHHSensor.h>

// I2C 參數
#define ADXL355_ADDR        0x1D
#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000

// 全域感測器物件：只宣告
extern ASM330LHHSensor IMU;

// 初始化函式：只宣告
void myI2C_init(void);
