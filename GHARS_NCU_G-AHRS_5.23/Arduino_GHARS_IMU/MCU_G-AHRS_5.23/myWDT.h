#pragma once
#include <Arduino.h>

// EXT WDT pins (如需共用，留在 header OK)
#define WDI         5
#define EXT_WDT_EN  4

namespace MYWDT {
  extern bool wdi_status;   // 只宣告，不定義
}

// 函式原型（只宣告）
void resetWDT();
void systemReset();
void setupWDT(uint8_t period);
void disableWDT();

void reset_EXT_WDI(char wdi_pin);
void disable_EXT_WDT(char en_pin);
void enable_EXT_WDT(char en_pin);

void myWDT_init();
