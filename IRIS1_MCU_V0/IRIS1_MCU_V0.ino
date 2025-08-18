#include "wiring_private.h"
#include "myUART.h"

const byte sendData[] = {0xAB, 0xBA, 0x66, 0x00, 0x00, 0x00, 0x02, 0x01, 0x55, 0x56};
const unsigned long sendInterval = 500; // 0.5 秒
unsigned long lastSendTime = 0;

void setup() {
  myUART_init();
  
}
byte cnt = 0;

void loop() { 
  unsigned long now = millis();

  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;

    // 送出 HEX 資料
    Serial4.write(sendData, sizeof(sendData));

    // Debug 顯示送出的內容
    Serial.print("Send: ");
    for (size_t i = 0; i < sizeof(sendData); i++) {
      Serial.print("0x");
      if (sendData[i] < 0x10) Serial.print("0");
      Serial.print(sendData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

 // 從 PC 收到資料，轉發到 FPGA
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial4.write(c);

    // 顯示 HEX
    // Serial.print("0x");
    if (c < 0x10) Serial.print("0"); // 補零對齊
    Serial.print(c, HEX);
    Serial.print(" ");
  }

  // 從 FPGA 收到資料，轉發到 PC
  while (Serial4.available()) {
    char c = Serial4.read();
    Serial1.write(c);

    // 直接輸出字串內容
    Serial.write(c);

    // 同時也輸出 HEX（可註解掉，如果不需要）
    // Serial.print(" [0x");
    // if (c < 0x10) Serial.print("0");
    // Serial.print(c, HEX);
    // Serial.print("] ");
  }


}



