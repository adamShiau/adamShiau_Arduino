#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <ASM330LHHSensor.h>

// SPI 速度常數（用 constexpr 避免巨集汙染）
constexpr uint32_t SPI_CLOCK_8M = 8000000;
constexpr uint32_t SPI_CLOCK_1M = 1000000;

// 片選腳位（你原本是 2；若外部已定義就沿用）
#ifndef CHIP_SELECT_PIN
#define CHIP_SELECT_PIN 2
#endif

// 只做 extern 宣告
extern SPIClassSAMD mySPI;
extern ASM330LHHSensor IMU;

// 初始化 API
void mySPI_init(void);

// （選擇性）如果你想在切換到 SPI 版本時保留 IMU 這個名字：
// #ifdef USE_SPI_IMU_AS_DEFAULT
// #define IMU IMU_SPI
// #endif
