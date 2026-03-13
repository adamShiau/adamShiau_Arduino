#include <Arduino.h>
#include "myI2C.h"
#include <wiring_private.h>  // 提供 pinPeripheral/PIO_SERCOM

#define I2C_STANDARD_MODE   100000
#define I2C_FAST_MODE       400000
#define I2C_FAST_MODE_PLUS  1000000

// Pins: SDA=27, SCL=20 (照你原本)
TwoWire myWire(&sercom0, 27, 20);

// 注意：這裡才「定義」 eeprom，全案只此一份
EEPROM_24AA32A_I2C eeprom = EEPROM_24AA32A_I2C(myWire);

void SERCOM0_Handler() {
  myWire.onService();
}

void myI2C_init(void) {
  myWire.begin();
  myWire.setClock(I2C_FAST_MODE);
  pinPeripheral(27, PIO_SERCOM);
  pinPeripheral(20, PIO_SERCOM);
}
