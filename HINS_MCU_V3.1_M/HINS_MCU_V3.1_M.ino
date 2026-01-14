#include "src/app/app_state.h"         // g_cmd / g_fog_params / g_output_fn / g_auto_rst / ahrs_attitude
#include "src/myUART.h"               // myUART_init / readDataDynamic
#include "src/common.h"               
#include "src/usecase/output_service/output_mode_setting.h"  // output_mode_setting
#include "src/domain/protocol/cmd_codec_v1.h"
#include "src/utils/crc32.h"
#include "src/usecase/cmd_dispatch.h"
#include "src/usecase/configuration_service.h"

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
  delay(1000);
  apply_configuration_from_container(&g_fog_params);
  delay(10);
  ahrs_attitude.init(100.0f); // sample rate                
}

void loop() { 
  static uint32_t try_cnt = 0;
  uint8_t* buf = NULL;

  if (Serial.available() > 0) {
    buf = readDataDynamic(&Serial, &try_cnt); // 傳入 Stream 物件 
    if (buf) {
      g_p_output_stream = &Serial; // 成功解析，切換輸出路徑至 USB
    }
  }

  if (!buf && Serial2.available() > 0) {
    buf = readDataDynamic(&Serial2, &try_cnt); // 傳入 Serial2 [cite: 4, 5]
    if (buf) {
      g_p_output_stream = &Serial2; // 成功解析，切換輸出路徑至 Serial2
    }
  }

  // uint8_t* buf = readDataDynamic(&try_cnt);

  if (buf) {
    decode_cmd_v1(buf, &g_cmd);
    cmd_dispatch(&g_cmd, &g_fog_params, &g_output_fn, &g_auto_rst);
  }

  if (g_output_fn) {
    g_output_fn(&g_cmd, &g_fog_params);
  }

}



