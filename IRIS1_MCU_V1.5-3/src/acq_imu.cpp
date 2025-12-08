#include "common.h"
#include "output_mode_config.h"
#include "myUART.h"

#define INT_SYNC 1
#define EXT_SYNC 2
#define STOP_RUN 4

#define DATA_DELAY_CNT 5

// 讀取來自 FPGA payload 長度（11 個 float × 4 bytes），不含 header 與 CRC
#define SENSOR_PAYLOAD_LEN 44

static my_sensor_t sensor_raw = {}, sensor_cali = {};

static uint32_t t_start = 0;
static uint8_t data_cnt = 0;
static uint32_t try_cnt = 0;

void acq_imu (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    if(rx->select_fn == SEL_IMU) {
        rx->select_fn = SEL_IDLE; //clear select_fn
        DEBUG_PRINT("-> select acq_imu mode\n");

        if(rx->value == INT_SYNC || rx->value == EXT_SYNC) {
            DEBUG_PRINT("acq_imu start\n");
            rx->run = 1;
            sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 2, 2);
            delay(10);
            reset_FPGA_timer();
        }
        else if(rx->value == STOP_RUN) {
            DEBUG_PRINT("acq_imu select stop\n");
            rx->run = 0;
            data_cnt = 0;
            sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 4, 2);
            sensor_raw = {}; // reset sensor_raw
            sensor_cali = {}; // reset sensor_cali
        }
    }

    if (rx -> run == 1) {

        uint8_t* pkt = readDataStream(HDR_ABBA, 2, TRL_5556, 2, SENSOR_PAYLOAD_LEN, &try_cnt);

        if (pkt) {
            if(data_cnt < DATA_DELAY_CNT) data_cnt++;
            // 1) 解析 raw
            if (update_raw_data(pkt, &sensor_raw) == 0) {

                // dumpPkt(pkt, SENSOR_PAYLOAD_LEN); //<-- for debug monitor

                // 2) 校正 → 輸出到 sensor_cali
                sensor_data_cali(&sensor_raw, &sensor_cali, fog_parameter);
                
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
