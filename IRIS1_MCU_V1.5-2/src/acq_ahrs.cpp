#include "common.h"
#include "output_mode_config.h"
#include "myUART.h"
#include <math.h>   // fabsf, sqrtf, isfinite

#define INT_SYNC 1
#define EXT_SYNC 2
#define STOP_RUN 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD ( (float)M_PI / 180.0f )
#define RAD2DEG ( 180.0f / (float)M_PI )

// ===== 參數設定（可調整）=====
#define ACC_MIN        (0.05f * 9.80665f)   // Accel 最低閥值 (m/s^2)
#define ACC_MAX        (16.0f * 9.80665f)   // Accel 飽和值 (m/s^2)
#define GYRO_MIN_DPS   (0.01f)              // Gyro 最低閥值 (dps)
#define GYRO_MAX_DPS   (660.0f)             // Gyro 飽和值 (dps)
#define ACC_LP_ALPHA   (0.2f)               // Acc 低通係數 0.1~0.3
                                            // 100 Hz 時等效 τ ≈ (1-α)/α * Ts ≈ 0.04 s，fc ~ 4 Hz
                                            // 若殘留振動多，可試 0.12~0.16（fc ~2–3 Hz）；太鈍則往 0.25 調

#define DATA_DELAY_CNT 5

// 讀取來自 FPGA payload 長度（11 個 float × 4 bytes），不含 header 與 CRC
#define SENSOR_PAYLOAD_LEN 44
// 姿態 3 floats = 12 bytes
#define ATT_PAYLOAD_LEN    12
// 合併後總長度
#define TOTAL_PAYLOAD_LEN  (SENSOR_PAYLOAD_LEN + ATT_PAYLOAD_LEN)

// ===== 靜止偵測參數 =====
#define G0_MPS2               (9.80665f)
#define GYRO_STILL_THRESH_DPS (2.0f)   // |gx|+|gy|+|gz| 的和門檻
#define ACC_STILL_TOL_G       (0.06f)  // |‖a‖-1g|/1g 的容忍

// ===== 加速學習設定（Gyro bias EMA；fs = 100 Hz）=====
// 公式：一階 EMA  b_k = (1-α) b_{k-1} + α x_k
// 時間常數 τ ≈ Ts / α ；截止頻率 fc ≈ α / (2π·Ts) = α·fs / (2π)
// 在 fs=100 Hz (Ts=0.01 s) 時：
//   α=0.001 → τ≈10.0 s，fc≈0.016 Hz
//   α=0.002 → τ≈ 5.0 s，fc≈0.032 Hz  ← 建議 base
//   α=0.003 → τ≈ 3.33 s，fc≈0.048 Hz
//   α=0.005 → τ≈ 2.0  s，fc≈0.080 Hz
//   α=0.010 → τ≈ 1.0  s，fc≈0.159 Hz  ← still 加速
//   α=0.020 → τ≈ 0.5  s，fc≈0.318 Hz  ← 更快收斂
// BIAS_ALPHA_BOOST_MS 為加速學習維持時間（毫秒）
#define BIAS_ALPHA_BASE     (0.002f)  // 慢速學習 α；100 Hz 時 fc ≈ 0.032 Hz
#define BIAS_ALPHA_BOOST    (0.010f)  // still 瞬間啟用的 α（τ≈1.0 s，fc≈0.159 Hz @ 100 Hz）
#define BIAS_ALPHA_BOOST_MS (800u)    // 加速學習維持時間（ms），可調 500~1500 ms

// ===== 奇異區遲滯（Pitch 接近 ±90°）=====
#define GL_ENTER_DEG  (88.0f)   // 進入奇異區門檻
#define GL_EXIT_DEG   (86.0f)   // 離開奇異區門檻（要小於 ENTER，形成遲滯）

// ===== 狀態機 =====
static uint8_t   g_just_still = 0;
static uint32_t  g_still_ts_ms = 0;
static float     g_bias_alpha_base = 0.0f;  // 進入 boost 前先記住原值

// ---- 姿態角輸出移動平均設定 ----
#define ATT_MA_N  5   // 移動平均的群延遲 ≈ (N-1)/2 * Ts ≈ 20 ms @ 100 Hz 
                      // N=5  → 延遲 ≈ 20  ms（目前）
                      // N=21 → 延遲 ≈ 100 ms（目前）
                      // N=51 → 延遲 ≈ 250 ms（目前）

static float att_ma_buf[3][ATT_MA_N];  // [axis][k]，for pitch/roll 線性平均
static float att_ma_sum[3] = {0,0,0};
static uint8_t att_ma_idx   = 0;
static uint8_t att_ma_count = 0;       // 未滿窗時用它做除數

// yaw 使用圓形平均（sin/cos）
static float yaw_sin_buf[ATT_MA_N] = {0};
static float yaw_cos_buf[ATT_MA_N] = {0};
static float yaw_ma_sin_sum = 0.0f;
static float yaw_ma_cos_sum = 0.0f;

static my_sensor_t sensor_raw = {}, sensor_cali = {};

static uint8_t  data_cnt = 0;
static uint32_t try_cnt = 0;
static my_att_t my_att, my_GYRO_cali, my_ACCL_cali;
static float    ax_lp=0, ay_lp=0, az_lp=0; // Acc 低通狀態

// Coning 狀態
static float    g_w_prev_dps[3] = {0,0,0};
static float    g_dt_prev_s = 0.0f;
static uint8_t  g_have_prev = 0;
static uint32_t g_last_ts_us = 0;

// 奇異區鎖定與投影積分 yaw
static bool  yaw_gl_locked = false;
static float yaw_hold_deg  = 0.0f;

// 名目取樣時間（100 Hz）
static const float Ts = 1.0f / 100.0f;

// ---- 小工具：死區 + 飽和 ----
static inline float apply_deadband_and_sat(float x, float min_abs, float max_abs) {
    if (!isfinite(x)) return 0.0f; // 防呆：NaN/Inf → 0
    float ax = fabsf(x);
    if (ax < min_abs) return 0.0f; // 死區
    if (ax > max_abs) return (x > 0.0f ? max_abs : -max_abs); // 飽和
    return x;
}

// ---- 靜止偵測（與庫內一致邏輯；用原始校正後數據）----
static inline bool is_still_app(float gx_dps, float gy_dps, float gz_dps,
                                float ax_mps2, float ay_mps2, float az_mps2) {
    float gsum = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);
    if (gsum > GYRO_STILL_THRESH_DPS) return false;
    float anorm = sqrtf(ax_mps2*ax_mps2 + ay_mps2*ay_mps2 + az_mps2*az_mps2);
    float dev   = fabsf(anorm - G0_MPS2) / G0_MPS2;
    return (dev <= ACC_STILL_TOL_G);
}

// ---- 兩樣本 Coning 合成（輸入/輸出 dps）----
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

// ---- 四元數轉世界←機體 3×3 矩陣 ----
static inline void quat_to_Rwb(float w,float x,float y,float z, float R[9]) {
    float ww=w*w, xx=x*x, yy=y*y, zz=z*z;
    float wx=w*x, wy=w*y, wz=w*z, xy=x*y, xz=x*z, yz=y*z;
    R[0]=ww+xx-yy-zz; R[1]=2*(xy-wz);    R[2]=2*(xz+wy);
    R[3]=2*(xy+wz);   R[4]=ww-xx+yy-zz; R[5]=2*(yz-wx);
    R[6]=2*(xz-wy);   R[7]=2*(yz+wx);   R[8]=ww-xx-yy+zz;
}

void acq_ahrs (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    if(rx->select_fn == SEL_AHRS) {
        rx->select_fn = SEL_IDLE; //clear select_fn
        DEBUG_PRINT("-> select acq_ahrs mode\n");

        if(rx->value == INT_SYNC || rx->value == EXT_SYNC) {
            DEBUG_PRINT("acq_ahrs start\n");
            ahrs_attitude.captureYawZeroLocalCase(); // 重設 yaw0（相對零位）
            rx->run = 1;
            sendCmd(Serial4, HDR_ABBA, TRL_5556, 2, 2, 2);
            delay(10);
            // —— 初始化（開跑 or setup 時做一次）——
            ahrs_attitude.setGyroBiasAlpha(BIAS_ALPHA_BASE);
            g_bias_alpha_base = BIAS_ALPHA_BASE;
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

            // 也重置本檔案內部狀態
            ax_lp=ay_lp=az_lp=0;
            g_have_prev = 0; g_last_ts_us = 0; g_dt_prev_s = 0;
            yaw_gl_locked = false; yaw_hold_deg = 0.0f;

            // 清空 MA 緩衝
            memset(att_ma_buf, 0, sizeof(att_ma_buf));
            att_ma_sum[0]=att_ma_sum[1]=att_ma_sum[2]=0.0f;
            att_ma_idx=0;
            att_ma_count=0;

            // 清空 yaw 的圓形平均狀態
            yaw_ma_sin_sum = 0.0f;
            yaw_ma_cos_sum = 0.0f;
            memset(yaw_sin_buf, 0, sizeof(yaw_sin_buf));
            memset(yaw_cos_buf, 0, sizeof(yaw_cos_buf));
        }
    }

    if (rx->run == 1) {
        uint8_t* pkt = readDataStream(HDR_ABBA, 2, TRL_5556, 2, SENSOR_PAYLOAD_LEN, &try_cnt);
        if (pkt) {
            if(data_cnt < DATA_DELAY_CNT) data_cnt++;

            // 1) 解析 raw，需配合實際安裝軸向做調整
            if (update_raw_data(pkt, &sensor_raw) == 0) {

                // 2) 校正 → 輸出到 sensor_cali
                sensor_data_cali(&sensor_raw, &sensor_cali, fog_parameter);
                my_GYRO_cali.float_val[0] = sensor_cali.fog.fogx.step.float_val;
                my_GYRO_cali.float_val[1] = sensor_cali.fog.fogy.step.float_val;
                my_GYRO_cali.float_val[2] = sensor_cali.fog.fogz.step.float_val;
                my_ACCL_cali.float_val[0] = sensor_cali.adxl357.ax.float_val;
                my_ACCL_cali.float_val[1] = sensor_cali.adxl357.ay.float_val;
                my_ACCL_cali.float_val[2] = sensor_cali.adxl357.az.float_val;

                // 3-1) 閥值與飽和值篩選
                my_att_t my_GYRO_att_calculate, my_ACCL_att_calculate;
                for (int i = 0; i < 3; ++i) {
                    // Gyro: 死區 + 飽和
                    my_GYRO_att_calculate.float_val[i] =
                        apply_deadband_and_sat(my_GYRO_cali.float_val[i], GYRO_MIN_DPS, GYRO_MAX_DPS);

                    // Accel: 死區 + 飽和
                    my_ACCL_att_calculate.float_val[i] =
                        apply_deadband_and_sat(my_ACCL_cali.float_val[i], ACC_MIN, ACC_MAX);
                }

                // 3-1.5) Acc 低通（姿態修正用；權重判斷用原始）
                if (ax_lp==0 && ay_lp==0 && az_lp==0) {
                    ax_lp = my_ACCL_att_calculate.float_val[0];
                    ay_lp = my_ACCL_att_calculate.float_val[1];
                    az_lp = my_ACCL_att_calculate.float_val[2];
                } else {
                    ax_lp = (1-ACC_LP_ALPHA)*ax_lp + ACC_LP_ALPHA*my_ACCL_att_calculate.float_val[0];
                    ay_lp = (1-ACC_LP_ALPHA)*ay_lp + ACC_LP_ALPHA*my_ACCL_att_calculate.float_val[1];
                    az_lp = (1-ACC_LP_ALPHA)*az_lp + ACC_LP_ALPHA*my_ACCL_att_calculate.float_val[2];
                }

                // ---- 靜止偵測 + 快速度偏置學習 ----
                bool still_now = is_still_app(
                    my_GYRO_cali.float_val[0], my_GYRO_cali.float_val[1], my_GYRO_cali.float_val[2],
                    my_ACCL_cali.float_val[0], my_ACCL_cali.float_val[1], my_ACCL_cali.float_val[2]
                );

                uint32_t now_ms = millis();
                if (still_now) {
                    if (!g_just_still) {
                        g_just_still   = 1;
                        g_still_ts_ms  = now_ms;
                        g_bias_alpha_base = ahrs_attitude.getGyroBiasAlpha();
                        ahrs_attitude.setGyroBiasAlpha(BIAS_ALPHA_BOOST);
                    } else {
                        if ((uint32_t)(now_ms - g_still_ts_ms) > BIAS_ALPHA_BOOST_MS) {
                            ahrs_attitude.setGyroBiasAlpha(g_bias_alpha_base);
                            g_just_still = 0;
                        }
                    }
                } else {
                    if (g_just_still) {
                        ahrs_attitude.setGyroBiasAlpha(g_bias_alpha_base);
                        g_just_still = 0;
                    }
                }
                // ---- (END) 靜止偵測 ----

                // 3-2) 計算 dt（秒）
                uint32_t now_us = micros();
                float dt_curr;
                if (g_last_ts_us == 0) {
                    dt_curr = Ts;                 // 第一筆：名目取樣時間
                } else {
                    uint32_t du = now_us - g_last_ts_us;   // 自動處理 micros 溢位
                    dt_curr = (float)du * 1e-6f;
                }
                g_last_ts_us = now_us;

                // 夾一下 dt（避免偶發卡頓或 0）
                if (dt_curr < 1e-5f) dt_curr = 1e-5f;  // 10 µs 下限保護

                // 若超過較寬鬆上限（建議 50 ms），丟棄上一筆，避免錯誤的兩樣本外積
                if (dt_curr > 0.05f) {   // > 50 ms 視為掉拍
                    dt_curr = Ts;        // 重置為名目取樣時間
                    g_have_prev = 0;     // 不做兩樣本合成，下一筆重新開始
                }

                // 3-3) 兩樣本 coning 合成 → 等效角速率 weq_dps
                float w_eq_dps[3];
                if (g_have_prev) {
                    coning_two_sample_dps(g_w_prev_dps, g_dt_prev_s,
                                          my_GYRO_att_calculate.float_val, dt_curr,
                                          w_eq_dps);
                } else {
                    w_eq_dps[0] = my_GYRO_att_calculate.float_val[0];
                    w_eq_dps[1] = my_GYRO_att_calculate.float_val[1];
                    w_eq_dps[2] = my_GYRO_att_calculate.float_val[2];
                    g_have_prev = 1;
                }

                // 3-4) 姿態更新（低通 acc 做修正；原始 acc 做權重判斷）
                ahrs_attitude.updateIMU_dualAccel(
                    w_eq_dps[0], w_eq_dps[1], w_eq_dps[2],
                    ax_lp, ay_lp, az_lp,                                         // 低通 acc → 姿態修正
                    my_ACCL_att_calculate.float_val[0],                          // 原始 acc → 權重判斷
                    my_ACCL_att_calculate.float_val[1],
                    my_ACCL_att_calculate.float_val[2]
                );

                // 3-5) 產出 Pitch / Roll（照舊）
                my_att.float_val[0] = ahrs_attitude.getLocalCasePitch(); // pitch
                my_att.float_val[1] = ahrs_attitude.getLocalCaseRoll();  // roll

                // 3-6) 奇異區遲滯 + 投影積分 yaw
                float pitch_deg_now = my_att.float_val[0];
                if (!yaw_gl_locked && fabsf(pitch_deg_now) >= GL_ENTER_DEG) {
                    yaw_gl_locked = true;
                    yaw_hold_deg = ahrs_attitude.getLocalCaseYaw();
                }
                if (yaw_gl_locked && fabsf(pitch_deg_now) <= GL_EXIT_DEG) {
                    yaw_gl_locked = false;
                }

                if (yaw_gl_locked) {
                    // 取 q_WS，投影 ω_b 到世界座標，積分 z 分量
                    float q0,q1,q2,q3; ahrs_attitude.getQuatWS(q0,q1,q2,q3);
                    float Rwb[9]; quat_to_Rwb(q0,q1,q2,q3, Rwb);
                    float omega_b_dps[3] = { w_eq_dps[0], w_eq_dps[1], w_eq_dps[2] };
                    float omega_w_dps[3] = {
                        Rwb[0]*omega_b_dps[0] + Rwb[1]*omega_b_dps[1] + Rwb[2]*omega_b_dps[2],
                        Rwb[3]*omega_b_dps[0] + Rwb[4]*omega_b_dps[1] + Rwb[5]*omega_b_dps[2],
                        Rwb[6]*omega_b_dps[0] + Rwb[7]*omega_b_dps[1] + Rwb[8]*omega_b_dps[2]
                    };
                    yaw_hold_deg += omega_w_dps[2] * dt_curr; // dps × s = deg
                    my_att.float_val[2] = yaw_hold_deg;
                } else {
                    // 正常使用庫內 yaw
                    my_att.float_val[2] = ahrs_attitude.getLocalCaseYaw();
                    yaw_hold_deg = my_att.float_val[2]; // 讓持有值跟上
                }

                // --- 滾動 coning 狀態 ---
                g_w_prev_dps[0] = my_GYRO_att_calculate.float_val[0];
                g_w_prev_dps[1] = my_GYRO_att_calculate.float_val[1];
                g_w_prev_dps[2] = my_GYRO_att_calculate.float_val[2];
                g_dt_prev_s = dt_curr;

                // ---- (NEW) 姿態角輸出移動平均 ----
                // pitch / roll：線性平均；yaw：圓形平均
                for (int axis = 0; axis < 2; ++axis) { // 先處理 pitch(0)、roll(1)
                    float v = my_att.float_val[axis];
                    if (!isfinite(v)) v = 0.0f;
                    att_ma_sum[axis] -= att_ma_buf[axis][att_ma_idx];
                    att_ma_buf[axis][att_ma_idx] = v;
                    att_ma_sum[axis] += v;
                }
                // yaw：用 sin/cos 環形緩衝平滑，避免跨 ±180° 出錯
                {
                    float yaw_deg = my_att.float_val[2];
                    if (!isfinite(yaw_deg)) yaw_deg = 0.0f;
                    float yaw_rad = yaw_deg * DEG2RAD;

                    // 先移除該槽舊貢獻（首次為 0）
                    yaw_ma_sin_sum -= yaw_sin_buf[att_ma_idx];
                    yaw_ma_cos_sum -= yaw_cos_buf[att_ma_idx];

                    // 寫入新貢獻
                    float s = sinf(yaw_rad);
                    float c = cosf(yaw_rad);
                    yaw_sin_buf[att_ma_idx] = s;
                    yaw_cos_buf[att_ma_idx] = c;

                    // 累加新貢獻
                    yaw_ma_sin_sum += s;
                    yaw_ma_cos_sum += c;
                }

                // 更新有效樣本數與循環索引
                if (att_ma_count < ATT_MA_N) att_ma_count++;
                att_ma_idx = (att_ma_idx + 1) % ATT_MA_N;

                // 取平均（未滿窗時用 att_ma_count 做除數）
                float ma_pitch = att_ma_sum[0] / att_ma_count;
                float ma_roll  = att_ma_sum[1] / att_ma_count;

                // yaw 圓形平均（防 atan2(0,0)）
                float avg_sin = yaw_ma_sin_sum / att_ma_count;
                float avg_cos = yaw_ma_cos_sum / att_ma_count;
                float ma_yaw;
                if (avg_sin*avg_sin + avg_cos*avg_cos < 1e-6f) {
                    ma_yaw = my_att.float_val[2]; // 回退用當下 yaw
                } else {
                    ma_yaw = atan2f(avg_sin, avg_cos) * RAD2DEG;
                }

                // 覆蓋輸出
                my_att.float_val[0] = ma_pitch;
                my_att.float_val[1] = ma_roll;
                my_att.float_val[2] = ma_yaw;

                // 3-7) IMU data sensor frame 轉 Case frame, 更新至 sensor_cali 結構（照舊）
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
            }
        }
    }
}
