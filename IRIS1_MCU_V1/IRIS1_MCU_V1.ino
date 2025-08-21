#include "src/IRIS_MCU.h"

#define INT_SYNC 1
#define EXT_SYNC 2
#define STOP_RUN 4

// 預期 payload 長度（11 個 float × 4 bytes）
#define SENSOR_PAYLOAD_LEN 44

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
    uint8_t* pkt = readDataBytewise(HDR_ABBA, 2, TRL_5556, 2, SENSOR_PAYLOAD_LEN, &try4);

    if (pkt) {
      

      if (update_raw_data(pkt, &sensor_data_raw) == 0) dumpPkt(pkt, SENSOR_PAYLOAD_LEN);

      // int idx = 0;
      // memcpy(sensor_data_raw.fog.fogx.step.bin_val,  &pkt[idx], 4); idx += 4;
      // memcpy(sensor_data_raw.fog.fogy.step.bin_val,  &pkt[idx], 4); idx += 4;
      // memcpy(sensor_data_raw.fog.fogz.step.bin_val,  &pkt[idx], 4); idx += 4;

      // memcpy(sensor_data_raw.adxl357.ax.bin_val,     &pkt[idx], 4); idx += 4;
      // memcpy(sensor_data_raw.adxl357.ay.bin_val,     &pkt[idx], 4); idx += 4;
      // memcpy(sensor_data_raw.adxl357.az.bin_val,     &pkt[idx], 4); idx += 4;

      // memcpy(sensor_data_raw.temp.tempx.bin_val,     &pkt[idx], 4); idx += 4;
      // memcpy(sensor_data_raw.temp.tempy.bin_val,     &pkt[idx], 4); idx += 4;
      // memcpy(sensor_data_raw.temp.tempz.bin_val,     &pkt[idx], 4); idx += 4;

      // memcpy(sensor_data_raw.adxl357.temp.bin_val,   &pkt[idx], 4); idx += 4;

      // memcpy(sensor_data_raw.time.time.bin_val,      &pkt[idx], 4); idx += 4;

      Serial1.write(HDR_OUT, sizeof(HDR_OUT));   // 先送 header
      Serial1.write(pkt, 44);             // 再送 payload
   
  }

  }

  
}



