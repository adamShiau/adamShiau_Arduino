#include "src/IRIS_MCU.h"

#define INT_SYNC 1
#define EXT_SYNC 2
#define STOP_RUN 4

#define DATA_DELAY_CNT 5

#define SYNC_50HZ  1e6
#define SYNC_100HZ 5e5
#define SYNC_200HZ 2.5e5

// 預期 payload 長度（11 個 float × 4 bytes）
#define SENSOR_PAYLOAD_LEN 44

uint32_t try1 = 0, try4 = 0;

uint8_t data_cnt = 0;

extern Uart Serial4;


void setup() {
  myUART_init();
  crc32_init_table();
  delay(100);
  DEBUG_PRINT("Boot capture all parameters from FPGA...\n");
  boot_capture_all(&fog_params);
  delay(100);
  // dump_fog_param(&fog_params, 1);
  // delay(100);
  // dump_fog_param(&fog_params, 2);
  // delay(100);
  // dump_fog_param(&fog_params, 3);
  // delay(100);
  // dump_misalignment_param(&fog_params);
  // delay(100);
  set_data_rate(SYNC_100HZ); 
  delay(10);
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
      delay(10);
      reset_FPGA_timer();
    }
    else if(my_cmd.value == STOP_RUN) { 
      my_cmd.run = 0; // set run flag
      data_cnt = 0;
      sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 4, 2);
      sensor_raw = {}; // reset sensor_raw
      sensor_cali = {}; // reset sensor_cali
    }
  }

  if (my_cmd.run == 1) {
  uint8_t* pkt = readDataBytewise(HDR_ABBA, 2, TRL_5556, 2, SENSOR_PAYLOAD_LEN, &try4);

  if (pkt) {
    data_cnt++;
    // 1) 解析 raw
    if (update_raw_data(pkt, &sensor_raw) == 0) {

      // dumpPkt(pkt, SENSOR_PAYLOAD_LEN); //<-- for debug monitor

      // 2) 校正 → 輸出到 sensor_cali
      sensor_data_cali(&sensor_raw, &sensor_cali, &fog_params);

      // 3) 用 sensor_cali 打包成 44 bytes
      uint8_t out[SENSOR_PAYLOAD_LEN];
      pack_sensor_payload_from_cali(&sensor_cali, out);
      // pack_sensor_payload_from_cali(&sensor_raw, out);

      // 4) 以 (KVH_HEADER + out) 產生 CRC
      uint8_t crc[4];
      gen_crc32(KVH_HEADER, out, SENSOR_PAYLOAD_LEN, crc);

      // （可選）debug
      // DEBUG_PRINT("CRC32 = %02X %02X %02X %02X\r\n", crc[0],crc[1],crc[2],crc[3]);

      // 5) 依序送出：Header + Calibrated Payload + CRC
      if(data_cnt >= DATA_DELAY_CNT) {
        Serial1.write(KVH_HEADER, sizeof(KVH_HEADER));
        Serial1.write(out, SENSOR_PAYLOAD_LEN);
        Serial1.write(crc, 4);
      }
      
    }
  }
}


  
}



