#include "src/app/app_state.h"         // g_cmd / g_fog_params / g_output_fn / g_auto_rst / ahrs_attitude
#include "src/myUART.h"               // myUART_init / readDataDynamic
#include "src/common.h"               
#include "src/output_mode_setting.h"  // output_mode_setting
#include "src/domain/protocol/cmd_codec_v1.h"
#include "src/utils/crc32.h"
#include "src/usecase/cmd_dispatch.h"

// #define INT_SYNC 1
// #define EXT_SYNC 2
// #define STOP_RUN 4

#define SYNC_50HZ  1e6
#define SYNC_100HZ 5e5
#define SYNC_200HZ 2.5e5

// uint32_t try_cnt = 0;

uint8_t data_cnt = 0;

extern Uart Serial4;



void setup() {
  myUART_init();
  crc32_init_table();
  delay(100);
  DEBUG_PRINT("Boot capture all parameters from FPGA...\n");

  // delay(2000);
  // Serial.println("IRIS1_MCU_V1.2");
  // DEBUG_PRINT("Boot capture all parameters from FPGA...\n");
  boot_capture_all(&g_fog_params);
  delay(100);
  set_data_rate(SYNC_100HZ); 
  delay(10);
  ahrs_attitude.init(100.0f); // sample rate                
}

void loop() { 
  static uint32_t try_cnt = 0;

  uint8_t* buf = readDataDynamic(&try_cnt);

  if (buf) {
    decode_cmd_v1(buf, &g_cmd);
    cmd_dispatch(&g_cmd, &g_fog_params, &g_output_fn, &g_auto_rst);
  }
  
  if (g_output_fn) {
    g_output_fn(&g_cmd, &g_fog_params);
  }

}



