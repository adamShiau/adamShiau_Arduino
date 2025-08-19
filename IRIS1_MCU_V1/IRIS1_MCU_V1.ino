#include "src/IRIS_MCU.h"


uint32_t try1 = 0, try4 = 0;

void setup() {
  myUART_init();
  
}

void loop() { 
  get_uart_cmd(readDataDynamic(&try1), &my_cmd);


}



