#include "wiring_private.h"
#include "myUART.h"

void setup() {
  myUART_init();
  
}
byte cnt = 0;

void loop() { 

 // 從 PC 收到資料，轉發到 FPGA
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial4.write(c);
  }

  // 從 FPGA 收到資料，轉發到 PC
  while (Serial4.available()) {
    char c = Serial4.read();
    Serial1.write(c);
  }

  // Serial1.write(cnt++);
  // delay(100);
}



