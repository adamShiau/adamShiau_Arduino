#include "mySPI.h"
#include <wiring_private.h>   // 提供 pinPeripheral/PIO_SERCOM_ALT

// 注意順序：SPIClassSAMD(SERCOM*, pinMISO, pinSCK, pinMOSI, TXPad, RXPad)
// 下面沿用你原先的腳位：MISO=3, SCK=23, MOSI=22，SERCOM4
SPIClassSAMD mySPI(&sercom4, 3, 23, 22, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

// SPI 版 ASM330LHH（避免和 I2C 版 IMU 撞名）
ASM330LHHSensor IMU(&mySPI, CHIP_SELECT_PIN);

void mySPI_init(void)
{
    mySPI.begin();

    // 把 3/22/23 切到 SERCOM ALT 功能
    pinPeripheral(3,  PIO_SERCOM_ALT);  // MISO
    pinPeripheral(22, PIO_SERCOM_ALT);  // MOSI
    pinPeripheral(23, PIO_SERCOM_ALT);  // SCK

    // CS 預設拉高
    pinMode(CHIP_SELECT_PIN, OUTPUT);
    digitalWrite(CHIP_SELECT_PIN, HIGH);
}
