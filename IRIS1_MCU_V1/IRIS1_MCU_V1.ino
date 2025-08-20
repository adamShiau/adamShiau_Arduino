#include "src/IRIS_MCU.h"

#define INT_SYNC 1
#define EXT_SYNC 2
#define STOP_RUN 4

uint32_t try1 = 0, try4 = 0;

extern Uart Serial4;

void setup() {
  myUART_init();
  
}

void loop() { 
  get_uart_cmd(readDataDynamic(&try1), &my_cmd);
  cmd_mux(&my_cmd);
  fog_parameter(&my_cmd, &fog_params);

  if(my_cmd.mux == MUX_OUTPUT) {
    my_cmd.mux = MUX_ESCAPE;
    if(my_cmd.cmd == MODE_IMU) {
      my_cmd.select_fn = SEL_IMU; // set select_fn to SEL_IMU
    }
  }

  if(my_cmd.select_fn == SEL_IMU) {
    my_cmd.select_fn = SEL_IDLE; // clear select_fn

    if(my_cmd.value == EXT_SYNC) { 
      my_cmd.run = 1; // set run flag
      sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 2, 2);
    }
    else if(my_cmd.value == STOP_RUN) { 
      my_cmd.run = 0; // set run flag
      sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 4, 2);
    }
  }

  if(my_cmd.run == 1) {
    while (Serial4.available()) {
    char c = Serial4.read();
    Serial1.write(c);
    Serial.write(c);
  }
  }

  
}



