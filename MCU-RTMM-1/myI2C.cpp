#include <Arduino.h>
#include "myI2C.h"

// 在這裡「定義」一次 IMU 物件
ASM330LHHSensor IMU(&Wire, 0xD5U);  // 保持你原來的位址

void myI2C_init(void)
{
    Wire.begin();
    Wire.setClock(I2C_FAST_MODE);

    // 如果未來改回自定義 SERCOM I2C（myWire），
    // 把 TwoWire 與 pinPeripheral() 的東西移到這裡，
    // 並 #include <wiring_private.h> 只在 .cpp 使用。
}
