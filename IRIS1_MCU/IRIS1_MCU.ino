#include "wiring_private.h"
#include "myUART.h"


void setup() {
  myUART_init();
  
}
byte cnt = 0;

void loop() { 
  unsigned long now = millis();

 // 從 PC 收到資料，轉發到 FPGA
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial4.write(c);

    // 顯示 HEX
    // Serial.print("SEND: ");
    // Serial.print("0x");
    // if (c < 0x10) Serial.print("0"); // 補零對齊
    // Serial.print(c, HEX);
    // Serial.print(" ");

    // 顯示到 Serial Monitor
    // Serial.print("0x");
    if (c < 0x10) Serial.print("0"); // 補零對齊
    if ((uint8_t)c == 0x56) { 
      Serial.print(c, HEX);
      Serial.println();  // 換行
    } else {
      Serial.print(c, HEX);
      Serial.print(" ");
    }
  }

  // 從 FPGA 收到資料，轉發到 PC
  while (Serial4.available()) {
    char c = Serial4.read();
    Serial1.write(c);

    // 直接輸出字串內容
    Serial.write(c);
  }

}



