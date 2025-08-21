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
  crc32_init_table();
  
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

      uint8_t crc[4];
      gen_crc32(KVH_HEADER, pkt, SENSOR_PAYLOAD_LEN, crc);

      // Debug 印 CRC
      DEBUG_PRINT("CRC32 = %02X %02X %02X %02X\r\n",
                  crc[0], crc[1], crc[2], crc[3]);

      // 依序送出
      Serial1.write(KVH_HEADER, sizeof(KVH_HEADER));   // 送 header (4B)
      Serial1.write(pkt, SENSOR_PAYLOAD_LEN);          // 送 payload (44B)
      Serial1.write(crc, 4);  

      // Serial1.write(KVH_HEADER, sizeof(KVH_HEADER));   // 先送 header
      // Serial1.write(pkt, 44);             // 再送 payload

      // alt_u8* imu_data = (alt_u8*)malloc(48); // KVH_HEADER:4 + fog:12 + accl:12 + fog_temp:12 + accl_temp:4 + time:4 
			// alt_u8 CRC32[4];

      // memcpy(imu_data, KVH_HEADER, 4);
      // memcpy(imu_data+4, gyro_misalign_calibrated.x.bin_val, 4); 
      // memcpy(imu_data+8, gyro_misalign_calibrated.y.bin_val, 4); 
      // memcpy(imu_data+12, gyro_misalign_calibrated.z.bin_val, 4); 
      // memcpy(imu_data+16, accl_misalign_calibrated.x.bin_val, 4); 
      // memcpy(imu_data+20, accl_misalign_calibrated.y.bin_val, 4); 
      // memcpy(imu_data+24, accl_misalign_calibrated.z.bin_val, 4); 
      // memcpy(imu_data+28, data.temp.tempx.bin_val, 4); 
      // memcpy(imu_data+32, data.temp.tempy.bin_val, 4); 
      // memcpy(imu_data+36, data.temp.tempz.bin_val, 4); 
      // memcpy(imu_data+40, data.adxl357.temp.bin_val, 4); 
      // memcpy(imu_data+44, data.time.time.bin_val, 4);              
      // crc_32(imu_data, 48, CRC32);
      // free(imu_data);
   
  }

  }

  
}



