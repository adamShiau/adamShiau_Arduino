#include "common.h"
#include "output_mode_config.h"
#include "myUART.h"
#include <math.h>   // fabsf, sqrtf, isfinite
#include "app/app_state.h"
#include "utils/crc32.h"

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
#define GYRO_MAX_DPS   (1000.0f)            // Gyro 飽和值 (dps), 2025/12/08 change code.
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


// Start -------------------------------- sin/cos Moving Average filter setting --------------------------------
// 2025/12/05, added: 對「角度(度)」做圓形 Moving Average (sin/cos) 濾波
// ---- 姿態角輸出移動平均設定 ----
#define ATT_MA_N  5   // (filter window size) 移動平均的群延遲 ≈ (N-1)/2 * Ts ≈ 20 ms @ 100 Hz 
                      // N=5  → 延遲 ≈ 20  ms（目前）
                      // N=21 → 延遲 ≈ 100 ms（目前）
                      // N=51 → 延遲 ≈ 250 ms（目前）

// pitch sin/cos Moving Average filter parameter setting
static float pitch_sin_buf[ATT_MA_N] = {0};
static float pitch_cos_buf[ATT_MA_N] = {0};
static float pitch_sin_sum = 0.0f;
static float pitch_cos_sum = 0.0f;
static int   pitch_idx     = 0;
static int   pitch_count   = 0;

// roll sin/cos Moving Average filter parameter setting
static float roll_sin_buf[ATT_MA_N] = {0};
static float roll_cos_buf[ATT_MA_N] = {0};
static float roll_sin_sum = 0.0f;
static float roll_cos_sum = 0.0f;
static int   roll_idx     = 0;
static int   roll_count   = 0;

// yaw sin/cos Moving Average filter parameter setting
static float yaw_sin_buf[ATT_MA_N] = {0};
static float yaw_cos_buf[ATT_MA_N] = {0};
static float yaw_sin_sum = 0.0f;
static float yaw_cos_sum = 0.0f;
static int   yaw_idx     = 0;
static int   yaw_count   = 0;

// End -------------------------------- sin/cos Moving Average filter setting --------------------------------




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


// Start -------------------------------- sin/cos Moving Average filter function implement --------------------------------
// 2025/12/05, added: 對「角度(度)」做圓形 Moving Average (sin/cos) 濾波
static inline float circle_ma_filter_deg(float angle_deg,
                         float *buf_sin, float *buf_cos,
                         float *sum_sin, float *sum_cos,
                         int *idx, int *count,
                         int N)
{
    // 防呆：NaN / Inf 處理
    if (!isfinite(angle_deg)) {
        angle_deg = 0.0f;
    }

    float rad = angle_deg * DEG2RAD;

    // 新值
    float new_s = sinf(rad);
    float new_c = cosf(rad);

    // 取出這個槽位舊值（初始為 0）
    float old_s = buf_sin[*idx];
    float old_c = buf_cos[*idx];

    // 更新 running sum：加上 (新 - 舊)
    *sum_sin += (new_s - old_s);
    *sum_cos += (new_c - old_c);

    // 寫回新值到 buffer
    buf_sin[*idx] = new_s;
    buf_cos[*idx] = new_c;

    // 更新樣本數與索引
    if (*count < N) {
        (*count)++;           // 未滿窗，count 慢慢增加
    }
    *idx = (*idx + 1) % N;    // 環形索引

    // 防守：如果 count == 0（理論上不會），先回 raw
    if (*count == 0) {
        return angle_deg;
    }

    // 計算平均向量
    float avg_s = (*sum_sin) / (float)(*count);
    float avg_c = (*sum_cos) / (float)(*count);

    // 向量長度太小 → 方向不明 → 回退使用當下 raw angle
    float norm2 = avg_s * avg_s + avg_c * avg_c;
    if (norm2 < 1e-6f) {
        return angle_deg;
    }

    // atan2 還原成角度（度）
    float filt_rad = atan2f(avg_s, avg_c);
    float filt_deg = filt_rad * RAD2DEG;
    return filt_deg;
}

// End -------------------------------- sin/cos Moving Average filter function implement --------------------------------

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
            sendCmd(Serial1, HDR_ABBA, TRL_5556, 2, 2, 2);
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
            sendCmd(Serial1, HDR_ABBA, TRL_5556, 2, 4, 2);
            
            sensor_raw = {}; // reset sensor_raw
            sensor_cali = {}; // reset sensor_cali

            // 也重置本檔案內部狀態
            ax_lp=ay_lp=az_lp=0;
            g_have_prev = 0; g_last_ts_us = 0; g_dt_prev_s = 0;
            yaw_gl_locked = false; yaw_hold_deg = 0.0f;

            // Start -------------------------------- sin/cos Moving Average filter parameters setting --------------------------------
            // 2025/12/08, added: 對「角度(度)」做圓形 Moving Average (sin/cos) 濾波

            // ---- pitch ----
            memset(pitch_sin_buf, 0, sizeof(pitch_sin_buf));
            memset(pitch_cos_buf, 0, sizeof(pitch_cos_buf));
            pitch_sin_sum = 0.0f;
            pitch_cos_sum = 0.0f;
            pitch_idx     = 0;
            pitch_count   = 0;

            // ---- roll ----
            memset(roll_sin_buf, 0, sizeof(roll_sin_buf));
            memset(roll_cos_buf, 0, sizeof(roll_cos_buf));
            roll_sin_sum = 0.0f;
            roll_cos_sum = 0.0f;
            roll_idx     = 0;
            roll_count   = 0;

            // ---- yaw ----
            memset(yaw_sin_buf, 0, sizeof(yaw_sin_buf));
            memset(yaw_cos_buf, 0, sizeof(yaw_cos_buf));
            yaw_sin_sum = 0.0f;
            yaw_cos_sum = 0.0f;
            yaw_idx     = 0;
            yaw_count   = 0;
            
            // End -------------------------------- sin/cos Moving Average filter parameters setting --------------------------------



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

                
                // Start -------------------------------- sin/cos Moving Average filter -------------------------------- 
                // 2025/12/05, added: 對「角度(度)」做圓形 Moving Average (sin/cos) 濾波
                // pitch circle MA filter.

                // read raw attitude.
                float raw_pitch = my_att.float_val[0];
                float raw_roll  = my_att.float_val[1];
                float raw_yaw   = my_att.float_val[2];

                // pitch circle MA filter.
                float ma_pitch = circle_ma_filter_deg(raw_pitch,
                                                pitch_sin_buf, pitch_cos_buf,
                                                &pitch_sin_sum, &pitch_cos_sum,
                                                &pitch_idx, &pitch_count,
                                                ATT_MA_N);

                // roll circle MA filter.
                float ma_roll = circle_ma_filter_deg(raw_roll,
                                            roll_sin_buf, roll_cos_buf,
                                            &roll_sin_sum, &roll_cos_sum,
                                            &roll_idx, &roll_count,
                                            ATT_MA_N);

                // yaw circle MA filter.
                float ma_yaw  = circle_ma_filter_deg(raw_yaw,
                                            yaw_sin_buf, yaw_cos_buf,
                                            &yaw_sin_sum, &yaw_cos_sum,
                                            &yaw_idx, &yaw_count,
                                            ATT_MA_N);

                // update filtered attitude.
                my_att.float_val[0] = ma_pitch;
                my_att.float_val[1] = ma_roll;
                my_att.float_val[2] = ma_yaw;

                // End -------------------------------- sin/cos Moving Average filter -------------------------------- 


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
                // pack_sensor_payload_from_cali(&sensor_cali, out);
                pack_sensor_payload_from_cali(&sensor_raw, out); // test

                // 4-2) 再把 12B 姿態 append 在後面
                memcpy(out + SENSOR_PAYLOAD_LEN, my_att.bin_val, ATT_PAYLOAD_LEN);

                // 5) 用 (KVH_HEADER + out[0..TOTAL_PAYLOAD_LEN-1]) 算 CRC32
                uint8_t crc[4];
                gen_crc32(KVH_HEADER, out, TOTAL_PAYLOAD_LEN, crc);

                // 6) 依序送出：Header + 新 payload(56B) + CRC(4B)
                if(data_cnt >= DATA_DELAY_CNT) {
                    Serial2.write(KVH_HEADER, sizeof(KVH_HEADER));
                    Serial2.write(out, TOTAL_PAYLOAD_LEN);
                    Serial2.write(crc, 4);
                }
            }
        }
    }
}
