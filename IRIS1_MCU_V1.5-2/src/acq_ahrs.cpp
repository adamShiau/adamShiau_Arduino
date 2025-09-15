#include "common.h"
#include "output_mode_config.h"
#include "myUART.h"

#define INT_SYNC 1
#define EXT_SYNC 2 
#define STOP_RUN 4 
// ===== 參數設定（可調整）=====    
#define ACC_MIN   (0.05f*9.80665f)       // Accel 最低閥值 (m/s^2)
#define ACC_MAX   (16.0f  * 9.80665f)    // Accel 飽和值 m/s^2
#define GYRO_MIN_DPS (0.01f)          // Gyro 最低閥值 (dps)
#define GYRO_MAX_DPS (660.0f)           // Gyro 飽和值 (dps)

#define DATA_DELAY_CNT 5

// 讀取來自 FPGA payload 長度（11 個 float × 4 bytes），不含 header 與 CRC
#define SENSOR_PAYLOAD_LEN 44
// 姿態 3 floats = 12 bytes
#define ATT_PAYLOAD_LEN    12
// 合併後總長度
#define TOTAL_PAYLOAD_LEN  (SENSOR_PAYLOAD_LEN + ATT_PAYLOAD_LEN)

// ===== 靜止偵測參數（與 Library 內預設相近，視場景微調）=====
#define G0_MPS2              (9.80665f)
#define GYRO_STILL_THRESH_DPS  (1.0f)   // |gx|+|gy|+|gz| 的和門檻
#define ACC_STILL_TOL_G        (0.06f)  // |‖a‖-1g|/1g 的容忍

// ===== 加速學習設定 =====
#define BIAS_ALPHA_BOOST       (0.010f) // 靜止剛發生時臨時加大（原本大約 0.002）
#define BIAS_ALPHA_BOOST_MS    (800u)   // 提升持續時間 (ms)

// ===== 狀態機 =====
static uint8_t   g_just_still = 0;
static uint32_t  g_still_ts_ms = 0;
static float     g_bias_alpha_base = 0.0f;  // 進入 boost 前先記住原值

static my_sensor_t sensor_raw = {}, sensor_cali = {};

static uint32_t t_start = 0;
static uint8_t data_cnt = 0;
static uint32_t try_cnt = 0;
static my_att_t my_att, my_GYRO_cali, my_ACCL_cali;
static float ax_lp=0, ay_lp=0, az_lp=0; //Low-Pass for Accel


static float g_w_prev_dps[3] = {0,0,0};
static float g_dt_prev_s = 0.0f;
static uint8_t g_have_prev = 0;
static uint32_t g_last_ts_us = 0;


// 死區 + 飽和（不依賴 copysignf）
// x: 輸入值
// min_abs: 最低閥值 (|x| < min_abs → 輸出 0)
// max_abs: 最大絕對值 (|x| > max_abs → 輸出 ±max_abs)
static inline float apply_deadband_and_sat(float x, float min_abs, float max_abs) {
    if (!isfinite(x)) return 0.0f; // 防呆：NaN/Inf → 0

    float ax = fabsf(x);
    if (ax < min_abs) return 0.0f; // 死區內視為 0

    if (ax > max_abs) {            // 飽和處理
        if (x > 0.0f) return max_abs;
        else          return -max_abs;
    }

    return x; // 正常範圍內直接輸出
}

static inline bool is_still_app(float gx_dps, float gy_dps, float gz_dps,
                                float ax_mps2, float ay_mps2, float az_mps2) {
    float gsum = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);
    if (gsum > GYRO_STILL_THRESH_DPS) return false;
    float anorm = sqrtf(ax_mps2*ax_mps2 + ay_mps2*ay_mps2 + az_mps2*az_mps2);
    float dev   = fabsf(anorm - G0_MPS2) / G0_MPS2;
    return (dev <= ACC_STILL_TOL_G);
}

// 兩樣本 Coning 合成（小角近似）
// in:  w1_dps, w2_dps  [deg/s]；dt1, dt2 [s]
// out: weq_dps        [deg/s]
static inline void coning_two_sample_dps(const float w1_dps[3], float dt1,
                                         const float w2_dps[3], float dt2,
                                         float weq_dps[3]) {
    const float D2R = 0.01745329252f;
    const float R2D = 57.295779513f;

    // Δθ1, Δθ2（rad）
    float d1[3] = { (w1_dps[0]*D2R)*dt1, (w1_dps[1]*D2R)*dt1, (w1_dps[2]*D2R)*dt1 };
    float d2[3] = { (w2_dps[0]*D2R)*dt2, (w2_dps[1]*D2R)*dt2, (w2_dps[2]*D2R)*dt2 };

    // 0.5 * (Δθ1 × Δθ2)
    float cx[3] = {
        0.5f * (d1[1]*d2[2] - d1[2]*d2[1]),
        0.5f * (d1[2]*d2[0] - d1[0]*d2[2]),
        0.5f * (d1[0]*d2[1] - d1[1]*d2[0])
    };

    // 合成角增量
    float dc[3] = { d1[0]+d2[0]+cx[0], d1[1]+d2[1]+cx[1], d1[2]+d2[2]+cx[2] };

    // 平均到等效角速率（再轉回 dps）
    float dt = dt1 + dt2;
    if (dt < 1e-9f) { weq_dps[0]=w2_dps[0]; weq_dps[1]=w2_dps[1]; weq_dps[2]=w2_dps[2]; return; }

    float weq_rad[3] = { dc[0]/dt, dc[1]/dt, dc[2]/dt };
    weq_dps[0] = weq_rad[0]*R2D;
    weq_dps[1] = weq_rad[1]*R2D;
    weq_dps[2] = weq_rad[2]*R2D;
}



void acq_ahrs (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    if(rx->select_fn == SEL_AHRS) {
        rx->select_fn = SEL_IDLE; //clear select_fn
        DEBUG_PRINT("-> select acq_ahrs mode\n");

        if(rx->value == INT_SYNC || rx->value == EXT_SYNC) {
            DEBUG_PRINT("acq_ahrs start\n");
            ahrs_attitude.captureYawZeroLocalCase(); // 重設 yaw0
            rx->run = 1;
            sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 2, 2);
            delay(10);
            reset_FPGA_timer();
        }
        else if(rx->value == STOP_RUN) {
            DEBUG_PRINT("acq_ahrs select stop\n");
            ahrs_attitude.resetAttitude(true); // reset attitude and yaw0
            rx->run = 0;
            data_cnt = 0;
            sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 4, 2);
            sensor_raw = {}; // reset sensor_raw
            sensor_cali = {}; // reset sensor_cali
            // my_cpf.resetEuler(0,0,0);
        }
    }

    if (rx -> run == 1) {

        uint8_t* pkt = readDataStream(HDR_ABBA, 2, TRL_5556, 2, SENSOR_PAYLOAD_LEN, &try_cnt);

        if (pkt) {
            if(data_cnt < DATA_DELAY_CNT) data_cnt++;

            // 1) 解析 raw，需配合實際安裝軸向做調整
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

                // 3) 先用校正後資料跑姿態, 考慮閥值篩選功能
                
                // 3-1) 閥值與飽和值篩選
                my_att_t my_GYRO_att_calculate, my_ACCL_att_calculate;
            
                for (int i = 0; i < 3; ++i) {
                    // Gyro: 死區 + 飽和
                    my_GYRO_att_calculate.float_val[i] =
                    apply_deadband_and_sat(my_GYRO_cali.float_val[i],
                                    GYRO_MIN_DPS, GYRO_MAX_DPS);

                    // Accel: 死區 + 飽和
                    my_ACCL_att_calculate.float_val[i] =
                    apply_deadband_and_sat(my_ACCL_cali.float_val[i],
                                    ACC_MIN, ACC_MAX);
                }

                const float ACC_LP_ALPHA = 0.2f;  // 0.1~0.3 視振動調
                if (ax_lp==0 && ay_lp==0 && az_lp==0) {
                    ax_lp = my_ACCL_att_calculate.float_val[0];
                    ay_lp = my_ACCL_att_calculate.float_val[1];
                    az_lp = my_ACCL_att_calculate.float_val[2];
                } else {
                    ax_lp = (1-ACC_LP_ALPHA)*ax_lp + ACC_LP_ALPHA*my_ACCL_att_calculate.float_val[0];
                    ay_lp = (1-ACC_LP_ALPHA)*ay_lp + ACC_LP_ALPHA*my_ACCL_att_calculate.float_val[1];
                    az_lp = (1-ACC_LP_ALPHA)*az_lp + ACC_LP_ALPHA*my_ACCL_att_calculate.float_val[2];
}


                // ---- (NEW) 靜止偵測 + 快速度偏置學習 ----
                // 用「原始校正後」的感測值做偵測（權重判斷要敏感，不要用低通/飽和後）
                bool still_now = is_still_app(
                    my_GYRO_cali.float_val[0], my_GYRO_cali.float_val[1], my_GYRO_cali.float_val[2],
                    my_ACCL_cali.float_val[0], my_ACCL_cali.float_val[1], my_ACCL_cali.float_val[2]
                );

                uint32_t now_ms = millis();
                if (still_now) {
                    if (!g_just_still) {
                        // 由「動」→「靜」的瞬間：記下原本 alpha，拉高 alpha 一小段時間
                        g_just_still   = 1;
                        g_still_ts_ms  = now_ms;
                        g_bias_alpha_base = ahrs_attitude.getGyroBiasAlpha(); // 記原值
                        ahrs_attitude.setGyroBiasAlpha(BIAS_ALPHA_BOOST);     // 拉高學習速率
                        // （可選）也可短暫把 beta 拉高一些，加速 roll/pitch 回正：提供 setter 再做
                    } else {
                        // 持續靜止：到時間就還原
                        if ((uint32_t)(now_ms - g_still_ts_ms) > BIAS_ALPHA_BOOST_MS) {
                            ahrs_attitude.setGyroBiasAlpha(g_bias_alpha_base);
                            g_just_still = 0; // 保持「靜止」但 boost 結束
                        }
                    }
                } else {
                    // 一旦又動了：立刻還原 alpha，重置狀態
                    if (g_just_still) {
                        ahrs_attitude.setGyroBiasAlpha(g_bias_alpha_base);
                        g_just_still = 0;
                    }
                }
                // ---- (END) 靜止偵測 + 快速度偏置學習 ----
                
                // 3-2) 計算姿態
                ahrs_attitude.updateIMU_dualAccel(
                    my_GYRO_att_calculate.float_val[0],
                    my_GYRO_att_calculate.float_val[1],
                    my_GYRO_att_calculate.float_val[2],
                    // 低通版：給姿態修正
                    ax_lp, ay_lp, az_lp,
                    // 原始版（未低通；但可保留你「死區/飽和」的清潔）：給權重/動態判斷
                    my_ACCL_att_calculate.float_val[0],
                    my_ACCL_att_calculate.float_val[1],
                    my_ACCL_att_calculate.float_val[2]
                );



                my_att.float_val[0] =  ahrs_attitude.getLocalCasePitch(); // pitch
                my_att.float_val[1] =  ahrs_attitude.getLocalCaseRoll();  // roll
                my_att.float_val[2] =  ahrs_attitude.getLocalCaseYaw();   // yaw


                // 3-3) IMU data sensor frame 轉 Case frame, 更新至 sensor_cali 結構
                my_att_t my_GYRO_case_frame, my_ACCL_case_frame;
                
                ahrs_attitude.sensorVecToCase(my_GYRO_cali.float_val,  my_GYRO_case_frame.float_val);
                ahrs_attitude.sensorVecToCase(my_ACCL_cali.float_val,  my_ACCL_case_frame.float_val);
                sensor_cali.fog.fogx.step.float_val = my_GYRO_case_frame.float_val[0];
                sensor_cali.fog.fogy.step.float_val = my_GYRO_case_frame.float_val[1];
                sensor_cali.fog.fogz.step.float_val = my_GYRO_case_frame.float_val[2];
                sensor_cali.adxl357.ax.float_val   = my_ACCL_case_frame.float_val[0];
                sensor_cali.adxl357.ay.float_val   = my_ACCL_case_frame.float_val[1];
                sensor_cali.adxl357.az.float_val   = my_ACCL_case_frame.float_val[2];

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


