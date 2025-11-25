#include <Arduino.h>
#include "myWDT.h"

// 只在 .cpp include 實作所需的相依，避免汙染其他翻譯單元
#include "EEPROM_MANAGE.h"  // EEPROM_EXTWDT, SET_EXTWDT_ENABLE
#include "IMU_PIG_DEFINE.h" // SET_EXTWDT_ENABLE
#include "myUART.h"         // msg_out()

namespace MYWDT {
  bool wdi_status = false;   // 唯一一次定義
}

// 內部小工具函式不需要暴露到外面
static inline void WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1) { /* wait */ }
}

//============= resetWDT =====================================================
void resetWDT() {
  WDT->CLEAR.reg = 0xA5; // reset the WDT
  WDTsync();
}

//============= systemReset ==================================================
void systemReset() {
  WDT->CLEAR.reg = 0x00; // system reset via WDT
  WDTsync();
}

//============= setupWDT =====================================================
void setupWDT(uint8_t period) {
  WDT->CTRL.reg = 0;          // disable watchdog
  WDTsync();

  WDT->CONFIG.reg = min<uint8_t>(period, 11); // 0..11
  WDT->CTRL.reg = WDT_CTRL_ENABLE;            // enable watchdog
  WDTsync();

  Serial.println("setupWDT");
}

//============= disableWDT ===================================================
void disableWDT() {
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync();
  Serial.println("disableWDT");
}

/*** * EXT Watch dog  * ***/
void reset_EXT_WDI(char wdi_pin) {
  MYWDT::wdi_status = !MYWDT::wdi_status;
  digitalWrite(wdi_pin, MYWDT::wdi_status);
}

void disable_EXT_WDT(char en_pin) {
  Serial.println("disable_EXT_WDT");
  digitalWrite(en_pin, HIGH);
}

void enable_EXT_WDT(char en_pin) {
  Serial.print("EEPROM_EXTWDT:");
  Serial.println(EEPROM_EXTWDT);
  if (EEPROM_EXTWDT == SET_EXTWDT_ENABLE) {
    msg_out((char*)"EXT_WDT is enable.");
    digitalWrite(en_pin, LOW);
  } else {
    msg_out((char*)"EXT_WDT is not enable, check EEPROM_EXTWDT setting.");
  }
}

void myWDT_init() {
  pinMode(WDI, OUTPUT);
  pinMode(EXT_WDT_EN, OUTPUT);

  disableWDT();
  disable_EXT_WDT(EXT_WDT_EN);
}
