#include "common.h"
#include "output_mode_config.h"
#include "myUART.h"

#define INT_SYNC 1
#define EXT_SYNC 2
#define STOP_RUN 4

#define DATA_DELAY_CNT 5

// 讀取來自 FPGA payload 長度（11 個 float × 4 bytes），不含 header 與 CRC
#define SENSOR_PAYLOAD_LEN 44
// 姿態 3 floats = 12 bytes
#define ATT_PAYLOAD_LEN    12
// 合併後總長度
#define TOTAL_PAYLOAD_LEN  (SENSOR_PAYLOAD_LEN + ATT_PAYLOAD_LEN)

static my_sensor_t sensor_raw = {}, sensor_cali = {};

static uint32_t t_start = 0;
static uint8_t data_cnt = 0;
static uint32_t try_cnt = 0;
static my_att_t my_att, my_GYRO_cali, my_ACCL_cali;

void acq_ahrs (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    if(rx->select_fn == SEL_AHRS) {
        rx->select_fn = SEL_IDLE; //clear select_fn
        DEBUG_PRINT("-> select acq_ahrs mode\n");

        if(rx->value == INT_SYNC || rx->value == EXT_SYNC) {
            DEBUG_PRINT("acq_ahrs start\n");
            rx->run = 1;
            sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 2, 2);
            delay(10);
            reset_FPGA_timer();
        }
        else if(rx->value == STOP_RUN) {
            DEBUG_PRINT("acq_ahrs select stop\n");
            rx->run = 0;
            data_cnt = 0;
            sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 4, 2);
            sensor_raw = {}; // reset sensor_raw
            sensor_cali = {}; // reset sensor_cali
            my_cpf.resetEuler(0,0,0);
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
                my_GYRO_cali.float_val[0] = sensor_cali.fog.fogx.step.float_val;
                my_GYRO_cali.float_val[1] = sensor_cali.fog.fogy.step.float_val;
                my_GYRO_cali.float_val[2] = sensor_cali.fog.fogz.step.float_val;
                my_ACCL_cali.float_val[0] = sensor_cali.adxl357.ax.float_val;
                my_ACCL_cali.float_val[1] = sensor_cali.adxl357.ay.float_val;
                my_ACCL_cali.float_val[2] = sensor_cali.adxl357.az.float_val;

                // 3) 先用校正後資料跑姿態（時間、角速、加速度）
                my_cpf.run(sensor_raw.time.time.float_val,
                           my_GYRO_cali.float_val,
                           my_ACCL_cali.float_val);

                // 取回 Euler (roll, pitch, yaw) → 存到 my_att.float_val[3]
                my_cpf.getEularAngle(my_att.float_val);

                // 4) 建立新的 payload：前 44B 是感測打包，後 12B 是姿態
                uint8_t out[TOTAL_PAYLOAD_LEN];

                // 4-1) 先打包原本 44B 感測 payload
                pack_sensor_payload_from_cali(&sensor_cali, out);

                // 4-2) 再把 12B 姿態 append 在後面
                memcpy(out + SENSOR_PAYLOAD_LEN, my_att.bin_val, ATT_PAYLOAD_LEN);

                // 5) 用 (KVH_HEADER + out[0..TOTAL_PAYLOAD_LEN-1]) 算 CRC32
                uint8_t crc[4];
                gen_crc32(KVH_HEADER, out, TOTAL_PAYLOAD_LEN, crc);

                // 6) 依序送出：Header + 新 payload(56B) + CRC(4B)
                if(data_cnt >= DATA_DELAY_CNT) {
                    Serial1.write(KVH_HEADER, sizeof(KVH_HEADER));
                    Serial1.write(out, TOTAL_PAYLOAD_LEN);
                    Serial1.write(crc, 4);
                }

                // Serial.print("IMU attitude: ");
                // Serial.print(my_att.float_val[0], 3); Serial.print(", ");
                // Serial.print(my_att.float_val[1], 3); Serial.print(", ");
                // Serial.println(my_att.float_val[2], 3);
            }
        }
    }

}
