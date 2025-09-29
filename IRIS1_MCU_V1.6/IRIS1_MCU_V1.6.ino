#include "src/IRIS_MCU.h"


// #define INT_SYNC 1
// #define EXT_SYNC 2
// #define STOP_RUN 4

#define SYNC_50HZ  1e6
#define SYNC_100HZ 5e5
#define SYNC_200HZ 2.5e5

uint32_t try_cnt = 0;

uint8_t data_cnt = 0;

extern Uart Serial4;

/*** Attitude calculation*/
// Navigation::ComplementaryFilter my_cpf;
Madgwick ahrs_attitude;

void setup() {
  myUART_init();
  crc32_init_table();
  delay(1000); // wait FPGA init done before boot_capture_all()
  DEBUG_PRINT("Boot capture all parameters from FPGA...\n");

  // delay(2000);
  // Serial.println("IRIS1_MCU_V1.2");
  // DEBUG_PRINT("Boot capture all parameters from FPGA...\n");
  boot_capture_all(&fog_params);
  delay(100);
  set_data_rate(SYNC_100HZ); 
  delay(10);
  ahrs_attitude.init(100.0f); // sample rate

  // my_cpf.setIMUError(IRIS, 100);
  // my_cpf.setThreshold(0.00028, 0.00028, 0.00028); // dps, 0905 HP指示調整成~ 1 dph
                      
}

void loop() { 
  get_uart_cmd(readDataDynamic(&try_cnt), &my_cmd);
  cmd_mux(&my_cmd);
  fog_parameter(&my_cmd, &fog_params);
  output_mode_setting(&my_cmd, &output_fn, &auto_rst);
  output_fn(&my_cmd, &fog_params);
}



