#include "wiring_private.h"
#include "myUART.h"

int cnt=0;
void setup() {
  myUART_init();
  
}


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


}



