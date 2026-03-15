#include "acq_imu.h"
#include "src/packet_parser.h"
#include "EEPROM_MANAGE.h"

// GNSS-IMU Yaw融合變數
static float heading_shift_offset = 0.0f;      // YAW偏移量
static bool yaw_first_run = true;              // 首次運行標記
static float first_imu_yaw = 0.0f;             // 第一個IMU yaw值
static float original_imu_yaw = 0.0f;          // 當前原始IMU YAW值
static uint32_t last_yaw_print_time = 0;      // 上次YAW打印時間
static float gnss_heading_deg = 0.0f;         // 最新GNSS航向
static bool gnss_heading_valid = false;       // GNSS航向有效性
static uint32_t last_gnss_update_time = 0;    // 最後GNSS更新時間
static DataStatus last_gnss_status = DATA_INVALID;  // 最後收到的GNSS狀態
static float IEEE_754_INT2F(int in);\
static int IEEE_754_F2INT(float in);
static float sf_temp_comp_1st(float temp, int slope, int offset, bool print); 
static float bias_temp_comp_1st_3t(float temp,
                                    int T1, int T2,
                                    int s1, int o1,
                                    int s2, int o2,
                                    int s3, int o3, bool print);
static float convert_PD_temp_f(uint8_t msb, uint8_t lsb);
static FirstOrderLPF3D gyroLPF;
static FirstOrderLPF3D accelLPF;
static uint8_t g_lpf_idx = 2; // Gyro 預設 10Hz
static uint8_t a_lpf_idx = 2; // Accel 預設 5Hz (加速度計通常震動較大，建議濾波強一點)

void reset_SYNC(); // define  in .ino
void acc_cali(float acc_cli[3], float acc[3]); // define  in .ino
void gyro_cali(float gyro_cli[3], float gyro[3]); // define  in .ino
void print_imu_data(bool on, float acc[3], float gyro[3]); // define  in .ino
void clear_SEL_EN(byte &select_fn); // define  in .ino
void sendGpsPacketKVH(const GnssData& gnss_data); // GPS 位置封包輸出 (KVH 格式)





// GNSS航向更新函數
void updateGNSSHeading(float heading, DataStatus status) {
    // 保存最新的GNSS狀態供顯示使用
    last_gnss_status = status;

    // 只有在GPS航向有效時才更新
    if (status == DATA_ALL_VALID) {  // 只有航向有效時才融合
        gnss_heading_deg = heading;
        gnss_heading_valid = true;
        last_gnss_update_time = millis();

        // 使用當前IMU值計算shift，讓融合後的YAW等於GNSS heading
        if (original_imu_yaw != 0.0f) {
            float new_shift = heading - original_imu_yaw;

            // 角度包裝到 [-180, 180] 範圍
            while (new_shift > 180.0f) new_shift -= 360.0f;
            while (new_shift < -180.0f) new_shift += 360.0f;

            heading_shift_offset = new_shift;

            // 限制更新訊息為5秒一次
            static uint32_t last_shift_update_time = 0;
            uint32_t current_time = millis();
            if (current_time - last_shift_update_time >= 5000) {
                Serial.print("📡 GNSS更新shift: GPS=");
                Serial.print(heading, 1);
                Serial.print("° 當前IMU=");
                Serial.print(original_imu_yaw, 1);
                Serial.print("° 新shift=");
                Serial.print(heading_shift_offset, 1);
                Serial.println("°");
                last_shift_update_time = current_time;
            }
        }
    } else {
        // GPS狀態無效，標記為無效但不清除數據
        gnss_heading_valid = false;
    }
}

// GNSS-IMU Yaw融合函數
float applyGNSSIMUYawFusion(float imu_yaw) {
    // 保存原始IMU yaw值
    original_imu_yaw = imu_yaw;

    // 初始化處理：第一次運行時設定初始shift
    if (yaw_first_run) {
        first_imu_yaw = imu_yaw;
        heading_shift_offset = -first_imu_yaw;  // 設定shift讓起始值為0°
        yaw_first_run = false;

        Serial.print("🎯 初始YAW校正：第一個IMU YAW=");
        Serial.print(first_imu_yaw, 1);
        Serial.print("° 設定shift=");
        Serial.print(heading_shift_offset, 1);
        Serial.println("° (校正到0度起始)");
    }

    // 最終結果 = IMU_yaw + heading_shift_offset
    float corrected_yaw = imu_yaw + heading_shift_offset;

    // 包裝到 [0, 360] 範圍
    while (corrected_yaw < 0.0f) corrected_yaw += 360.0f;
    while (corrected_yaw >= 360.0f) corrected_yaw -= 360.0f;

    // 每5秒打印一次YAW計算結果
    uint32_t current_time = millis();
    if (current_time - last_yaw_print_time >= 5000) {
        last_yaw_print_time = current_time;

        Serial.print("YAW融合: IMU=");
        Serial.print(imu_yaw, 1);
        Serial.print("° + shift=");
        Serial.print(heading_shift_offset, 1);
        Serial.print("° = ");
        Serial.print(corrected_yaw, 1);
        Serial.print("°");

        // 根據封包status欄位顯示GNSS狀態
        Serial.print(" | GNSS=");
        Serial.print(gnss_heading_deg, 1);
        Serial.print("° (狀態:0x");
        Serial.print(last_gnss_status, HEX);
        Serial.print(" ");

        switch(last_gnss_status) {
            case DATA_ALL_VALID:
                Serial.print("航向有效");
                break;
            case DATA_POS_ONLY:
                Serial.print("僅位置有效");
                break;
            case DATA_NO_FIX:
                Serial.print("無定位信號");
                break;
            case DATA_UNSTABLE:
                Serial.print("數據不穩定");
                break;
            case DATA_INVALID:
            default:
                Serial.print("數據無效");
                break;
        }
        Serial.print(")");
        Serial.println();
    }

    return corrected_yaw;
}





#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD ( (float)M_PI / 180.0f )
#define RAD2DEG ( 180.0f / (float)M_PI )

// ===== 參數設定（可調整）=====
#define ACC_MIN        (0.0f * 9.80665f)   // Accel 最低閥值 (m/s^2)
#define ACC_MAX        (16.0f * 9.80665f)   // Accel 飽和值 (m/s^2)
#define GYRO_MIN_DPS   (0.01f)              // Gyro 最低閥值 (dps) => 此處是由記憶體內數值決定
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


// // Start -------------------------------- Unwrapped Moving Average filter setting --------------------------------
#define ATT_MA_N 5    // window size = 5, 
// delay = (dt * (window size - 1)/2)
// window size = 5, dt = 10 ms, delay = 20 ms.

// For Roll, Yaw unwrapped moving average using.
struct AttMaState {
    float buf[ATT_MA_N];
    float sum;
    uint8_t idx;
    uint8_t count;
    float prevRaw;
    float unwrapped;
    bool inited;
};
AttMaState rollMaState;    // For Roll unwrapped moving average using.
AttMaState yawMaState;     // For Yaw unwrapped moving average using.

// Pitch Original Moving Average filter parameter setting
static float pitch_buf[ATT_MA_N] = {0};
static float pitch_sum     = 0.0f;
static int   pitch_idx     = 0;
static int   pitch_count   = 0;
// // End -------------------------------- Unwrapped Moving Average filter setting --------------------------------


// static my_sensor_t sensor_raw = {}, sensor_cali = {};

// static uint8_t  data_cnt = 0;
// static uint32_t try_cnt = 0;
// static my_acc_t my_att, my_GYRO_cali, my_ACCL_cali;
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

// parameters for ahrs calculation time.
static unsigned long sum_dt_getIMU_data = 0;
static unsigned long sum_dt_ahrs = 0;
static unsigned long sum_dt_euler_transform = 0;
static unsigned long sum_dt_outputData = 0;
static int count = 0;


// // Start -------------------------------- Unwrapped Moving Average filter setting --------------------------------
// 將 angle 轉回指定範圍 (-halfPeriod, +halfPeriod]
static inline float wrapToRangeDeg(float angleDeg, float halfPeriodDeg) {
    float fullPeriodDeg = 2.0f * halfPeriodDeg;
    while (angleDeg >  halfPeriodDeg) angleDeg -= fullPeriodDeg;
    while (angleDeg <= -halfPeriodDeg) angleDeg += fullPeriodDeg;
    return angleDeg;
}

float attMovingAverageDeg(AttMaState &state, float rawDeg, float halfPeriodDeg, bool reset) {
    if (reset || !state.inited) {
        state.sum = 0.0f;
        state.idx = 0;
        state.count = 0;

        state.prevRaw = rawDeg;
        state.unwrapped = rawDeg;
        state.inited = true;

        for (uint8_t i = 0; i < ATT_MA_N; i++) state.buf[i] = 0.0f;
        // reset 後第一筆直接輸出 raw（或你也可輸出 0）
        return wrapToRangeDeg(rawDeg, halfPeriodDeg);
    }

    // -------- 1) unwrap：修正跨界跳變（不使用三角函數）--------
    float fullPeriodDeg = 2.0f * halfPeriodDeg;
    float delta = rawDeg - state.prevRaw;

    // 例如 yaw/roll: +179 -> -179，delta = -358，小於 -180，補 +360 => +2
    if (delta >  halfPeriodDeg)  delta -= fullPeriodDeg;
    else if (delta < -halfPeriodDeg) delta += fullPeriodDeg;
    state.unwrapped += delta;
    state.prevRaw = rawDeg;

    // -------- 2) moving average（running sum + ring buffer，O(1)）--------
    if (state.count < ATT_MA_N) {
        state.buf[state.idx] = state.unwrapped;
        state.sum += state.unwrapped;
        state.count++;
        state.idx = (uint8_t)((state.idx + 1) % ATT_MA_N);
        float mean = state.sum / (float)state.count;
        return wrapToRangeDeg(mean, halfPeriodDeg);
    } 
    else {
        state.sum -= state.buf[state.idx];
        state.buf[state.idx] = state.unwrapped;
        state.sum += state.unwrapped;
        state.idx = (uint8_t)((state.idx + 1) % ATT_MA_N);
        float mean = state.sum / (float)ATT_MA_N;
        return wrapToRangeDeg(mean, halfPeriodDeg);
    }
}

static inline void resetAttMaState(AttMaState &state, float initDeg) {
    state.sum = 0.0f;
    state.idx = 0;
    state.count = 0;
    state.prevRaw = initDeg;
    state.unwrapped = initDeg;
    state.inited = true;
    for (uint8_t i = 0; i < ATT_MA_N; i++) state.buf[i] = initDeg;
}

static inline void resetPitchLinearMA(float initDeg) {
    pitch_sum = initDeg * (float)ATT_MA_N;
    pitch_idx = 0;
    pitch_count = 0;
    for (uint8_t i = 0; i < ATT_MA_N; i++) pitch_buf[i] = initDeg;
}
// End -------------------------------- Unwrapped Moving Average filter function implement --------------------------------


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

static byte *reg_fog;
static byte cnt_tt = 0;


// ==========================================================================================================================================================================
// 2026/03/02, The AHRS algorithm is split into two calculation areas. (parameters setting)
// Start, state machine setting. Target: IMU data at 200 Hz, Euler angle at 100 Hz.
// Phase = PH_AHRS: Execute the AHRS algorithm + Pitch (case frame) derived from quaternion, k=1,3,5, ...
// Phase = PH_EULER: Euler angle derived from quaternion (case frame) + Unwrapped Moving Average filter. k=2,4,6, ...
enum Phase { PH_AHRS = 0, PH_EULER = 1 };
static Phase phase = PH_AHRS;

// static float pitch_AHRS_phase = 0.0f;                         // Store "pitch_AHRS_phase" computed in the AHRS phase for use in the Euler phase.
static float w_eq_dps_AHRS_phase[3] = {0.0f, 0.0f, 0.0f};     // Store "w_eq_dps" computed in the AHRS phase for use in the Euler phase.

// IMU measurement rate: 200 Hz
// AHRS computation rate: 100 Hz
// Define: Ts = 1.0f / 100.0f;
static float dt_AHRS_phase = Ts;                              // Store "dt_AHRS_phase" computed in the AHRS phase for use in the Euler phase. 

// End, state machine setting.
// ==========================================================================================================================================================================


void acq_imu(byte &select_fn, unsigned int value, byte ch)
{
    my_acc_t my_memsGYRO;
    // my_float_t pd_temp;
    static my_float_t PD_temp, step_H, step_L, step_cnt;
    static my_acc_t my_memsXLM, my_ACCL_cali;
    static my_acc_t my_GYRO, my_GYRO_cali, my_att;

    byte *fog;
	uint8_t CRC32[4];

    if(select_fn&SEL_IMU)
	{
        CtrlReg = value;

        if(ch==1) run_fog_flag = sp13.setSyncMode(CtrlReg);
        else if(ch==2) run_fog_flag = sp14.setSyncMode(CtrlReg);
        else if(ch==3) run_fog_flag = sp9.setSyncMode(CtrlReg);

        switch(CtrlReg){
            case INT_SYNC:
                data_cnt = 0;
                EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
                setupWDT(11);
                enable_EXT_WDT(EXT_WDT_EN);
                reset_EXT_WDI(WDI);
                
            break;
            case EXT_SYNC:
                data_cnt = 0;
                Serial.println("Enter EXT_SYNC mode");
                Serial.println("Set EXTT to CHANGE");

                ahrs_attitude.captureYawZeroLocalCase();
                // —— 初始化（開跑 or setup 時做一次）——
                ahrs_attitude.setGyroBiasAlpha(BIAS_ALPHA_BASE);
                gyroLPF.initAndPrint("Gyro", 200.0f, g_lpf_idx);
                accelLPF.initAndPrint("Accel", 200.0f, a_lpf_idx);

                EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
                setupWDT(11);
                enable_EXT_WDT(EXT_WDT_EN);
                reset_EXT_WDI(WDI);
                // 設定 t_previous 為當前時間，實現計時歸零
                t_previous = millis();
            break;

            case EXT_SYNC2:
                data_cnt = 0;
                Serial.println("Enter EXT_SYNC2 mode");
                Serial.println("Set EXTT to CHANGE");
                EIC->CONFIG[1].bit.SENSE7 = 3; ////set interrupt condition to Both
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 1);
                setupWDT(11);
                enable_EXT_WDT(EXT_WDT_EN);
                reset_EXT_WDI(WDI);
            break;

            case STOP_SYNC:
                reset_SYNC();
                data_cnt = 0;
                ahrs_attitude.resetAttitude(true);
                gyroLPF.reset();
                accelLPF.reset();
                EIC->CONFIG[1].bit.SENSE7 = 0; //set interrupt condition to None
                eeprom.Write(EEPROM_ADDR_FOG_STATUS, 0);
                disableWDT();
                disable_EXT_WDT(EXT_WDT_EN);

                // 2026/02/26, added ma reset.
                // reset Euler moving average filter memory
                resetAttMaState(rollMaState, 0.0f);
                resetAttMaState(yawMaState,  0.0f);
                resetPitchLinearMA(0.0f);
            break;

            default:
            break;
        }
    }
    
    if(run_fog_flag) {

    /*** GNSS decoder section */
    // GPS 5Hz 控制 - 獨立於ISR_PEDGE
    static bool cmd1_sent = false;
    static bool cmd2_sent = false;
    static uint8_t packet_buffer[64];
    static uint16_t buffer_index = 0;
    static bool expecting_packet = false;
    // 移除定時器相關變數 (改為收到即發送)
    // static unsigned long gps_last_time = 0;
    // static bool gps_timer_enabled = false;
    // static GnssData last_valid_gps_data;
    // static bool has_valid_gps_data = false;

    // 初始化GPS通訊 (只執行一次)
    if (!cmd1_sent) {
        cmd1_sent = true;
        byte data1[] = {0xAB, 0xBA, 0x01, 0x00, 0x00, 0x00, 0x05, 0x01, 0x55, 0x56};
        Serial2.write(data1, sizeof(data1));
        Serial.println("📤 發送CMD 1: 設定MINS為5Hz");
        expecting_packet = true;
    }

    // 延遲發送CMD 2 (只執行一次)
    if (!cmd2_sent && data_cnt > 10) {
        cmd2_sent = true;
        byte data2[] = {0xAB, 0xBA, 0x02, 0x00, 0x00, 0x00, 0x05, 0x02, 0x55, 0x56};
        Serial2.write(data2, sizeof(data2));
        Serial.println("📤 發送CMD 2: 設定MINS為5Hz");
        expecting_packet = true;
    }

    // 持續接收和解析GPS數據 (不受5Hz限制)
    if (Serial2.available()) {
        while (Serial2.available()) {
            uint8_t byte_received = Serial2.read();

            // Look for FA FF pattern in the data stream
            if (buffer_index == 0 && byte_received == 0xFA) {
                // Found potential start of packet
                packet_buffer[buffer_index++] = byte_received;
            } else if (buffer_index == 1 && byte_received == 0xFF) {
                // Found complete header FA FF
                packet_buffer[buffer_index++] = byte_received;
            } else if (buffer_index > 1 && buffer_index < sizeof(packet_buffer)) {
                // Continue collecting packet data
                packet_buffer[buffer_index++] = byte_received;

                // Check if we have enough data to determine packet size (新格式: [Header] [CMD][status][Size] [Payload] [Checksum])
                if (buffer_index >= 6) {  // 需要6個bytes才能讀到完整的header
                    uint16_t data_size = packet_buffer[4] | (packet_buffer[5] << 8);  // Size在位置4-5
                    uint16_t total_size = 6 + data_size;  // Header size = 6

                    // Check if we have the complete packet
                    if (buffer_index >= total_size) {
                        // Parse the packet
                        GnssData gnss_data;
                        if (PacketParser::parsePacket(packet_buffer, total_size, &gnss_data)) {
                            // 更新GNSS航向數據（只有包含航向的封包類型）
                            if (gnss_data.packet_type == PKT_HEADING || gnss_data.packet_type == PKT_POS_HEADING) {
                                updateGNSSHeading(gnss_data.heading, gnss_data.status);
                            }

                            // 收到GPS位置封包立即發送 (不使用定時器控制)
                            if (gnss_data.packet_type == PKT_POSITION) {
                                #ifdef UART_RS422_CMD
                                sendGpsPacketKVH(gnss_data);
                                #endif
                            }
                        }

                        // Reset buffer for next packet
                        buffer_index = 0;
                        memset(packet_buffer, 0, sizeof(packet_buffer));
                        break;
                    }
                }
            } else {
                // Reset if we get unexpected data or buffer overflow
                buffer_index = 0;
                memset(packet_buffer, 0, sizeof(packet_buffer));

                // Check if this byte could be start of new packet
                if (byte_received == 0xFA) {
                    packet_buffer[buffer_index++] = byte_received;
                }
            }
        }
    }

    // GPS封包輸出已改為收到即發送，不再使用定時器控制
    /*** End of GNSS decoder section */
    
    /*** FOG data decoder section */
    fog = sp14.readData(header, sizeofheader, &try_cnt);

    if(fog) 
    {    
        reg_fog = fog;

        // 取得來自FPGA之FOG數據
        // myfog_GYRO.bin_val[0] = reg_fog[8];
        // myfog_GYRO.bin_val[1] = reg_fog[9];
        // myfog_GYRO.bin_val[2] = reg_fog[10];
        // myfog_GYRO.bin_val[3] = reg_fog[11];
        //   Serial.println(myfog_GYRO.int_val);

        // 計算 PD_Temp
        PD_temp.float_val = convert_PD_temp_f(reg_fog[12], reg_fog[13]);


        const eeprom_obj *ptr = &eeprom_x;

        // SF 補償參數
        int sf0 = ptr->EEPROM_SF0; 
        int sf1 = ptr->EEPROM_SF1;

        // Bias 補償參數
        int t1 = ptr->EEPROM_BIAS_COMP_T1; 
        int t2 = ptr->EEPROM_BIAS_COMP_T2; 
        int s1 = ptr->EEPROM_SFB_1_SLOPE;  
        int o1 = ptr->EEPROM_SFB_1_OFFSET; 
        int s2 = ptr->EEPROM_SFB_2_SLOPE;  
        int o2 = ptr->EEPROM_SFB_2_OFFSET; 
        int s3 = ptr->EEPROM_SFB_3_SLOPE;  
        int o3 = ptr->EEPROM_SFB_3_OFFSET; 

        /*** 處理累積平均 */
        step_H.bin_val[0] = reg_fog[0];
        step_H.bin_val[1] = reg_fog[1];
        step_H.bin_val[2] = reg_fog[2];
        step_H.bin_val[3] = reg_fog[3];

        step_L.bin_val[0] = reg_fog[4];
        step_L.bin_val[1] = reg_fog[5];
        step_L.bin_val[2] = reg_fog[6];
        step_L.bin_val[3] = reg_fog[7];

        step_cnt.bin_val[0] = reg_fog[8];
        step_cnt.bin_val[1] = reg_fog[9];
        step_cnt.bin_val[2] = reg_fog[10];
        step_cnt.bin_val[3] = reg_fog[11];

        // 1. 組合 64-bit 整數
        int64_t full_step_64;
        full_step_64 = ((int64_t)step_H.int_val << 32) | (uint32_t)step_L.int_val;

        // 2. 使用 double 確保中間運算不丟失任何細節
        double averaged_step;
        
        if (step_cnt.int_val > 0) {
            averaged_step = (double)full_step_64 / (double)step_cnt.int_val;
        } else {
            averaged_step = 0.0f; 
        }
        /***  結束累積平均           */

        /*** debug print PD_temp */
        
        // Serial.print("Temp| "); Serial.print(PD_temp.float_val, 2); Serial.print(", "); 

        /*** debug print step */
        // Serial.print("Step| "); Serial.print(myfog_GYRO.int_val); Serial.print(", ");
        // Serial.print("Step_H| "); Serial.print(step_H.int_val); Serial.print(", ");
        // Serial.print("Step_L| "); Serial.print(step_L.int_val); Serial.print(", ");
        // Serial.print("Step_cnt| "); Serial.print(step_cnt.int_val); Serial.print("\n");

        // Serial.print("Step| "); 
        // Serial.print(myfog_GYRO.bin_val[0], HEX); Serial.print(", "); 
        // Serial.print(myfog_GYRO.bin_val[1], HEX); Serial.print(", ");
        // Serial.print(myfog_GYRO.bin_val[2], HEX); Serial.print(", "); 
        // Serial.print(myfog_GYRO.bin_val[3], HEX); Serial.print("\n"); 
        // Serial.print("Err| "); 
        // Serial.print(reg_fog[7], HEX); Serial.print(", "); 
        // Serial.print(reg_fog[6], HEX); Serial.print(", ");
        // Serial.print(reg_fog[5], HEX); Serial.print(", "); 
        // Serial.print(reg_fog[4], HEX); Serial.print("\n");
        
        /*** SF first order temperature compensation */
        float SF_FOG = sf_temp_comp_1st(PD_temp.float_val, sf0, sf1, 0);

        /*** Bias three section temperature compensation */
        float BS_FOG = bias_temp_comp_1st_3t(PD_temp.float_val,
                                    t1, t2, s1, o1, s2, o2, s3, o3, 0);
        
        // Serial.print("SF: "); Serial.print(SF_FOG, 5); Serial.print(", ");
        // Serial.print("  |BS: "); Serial.print(BS_FOG, 5); Serial.print("\n");

        // if(ISR_PEDGE)
        // {
        data_cnt++;
        mcu_time.ulong_val = millis() - t_previous;

        ISR_PEDGE = false;

        /*** get sensor raw data*/
        // IMU.Get_X_Axes_g_f(my_memsXLM.float_val);// get mems XLM data in m/s^2
        /*** ------get xlm raw data -----***/
        IMU.Get_X_Axes_f(my_memsXLM.float_val);// get mems XLM data in g
        // 在 Cali 之前套用 Accel 濾波
        accelLPF.apply(my_memsXLM.float_val, a_lpf_idx);
        /*** ------mis-alignment calibration xlm raw data -----***/
        acc_cali(my_ACCL_cali.float_val, my_memsXLM.float_val);

        /*** ------get gyro raw data -----***/
        IMU.Get_G_Axes_f(my_memsGYRO.float_val);// get mems GYRO data in degree/s
        my_GYRO.float_val[0] = my_memsGYRO.float_val[0]; 
        my_GYRO.float_val[1] = my_memsGYRO.float_val[1];
        my_GYRO.float_val[2] = (float)(averaged_step * SF_FOG + BS_FOG);
        // 在 Cali 之前套用 Gyro 濾波
        gyroLPF.apply(my_GYRO.float_val, g_lpf_idx);
        /*** ------mis-alignment calibration gyro raw data -----***/
        gyro_cali(my_GYRO_cali.float_val, my_GYRO.float_val);

        // Serial.print("Temp| "); Serial.print(PD_temp.float_val, 2); Serial.print(", "); 
        // Serial.print("FOG| "); Serial.print(my_GYRO.float_val[2], 2); Serial.print("\n"); 

        switch (phase) 
        {    // switch, phase, start.
            case PH_AHRS:
                {    // case, PH_AHRS, start.
                // Phase = PH_AHRS: Execute the AHRS algorithm + Pitch (case frame) derived from quaternion, k=1,3,5, ...
            
                // 3-1) 閥值與飽和值篩選
                my_acc_t my_GYRO_att_calculate, my_ACCL_att_calculate;
                
                    // Gyro: 死區 + 飽和
                    my_GYRO_att_calculate.float_val[0] =
                    apply_deadband_and_sat(my_GYRO_cali.float_val[0], attitude_cali_coe._f.std_wx, GYRO_MAX_DPS);
                    my_GYRO_att_calculate.float_val[1] =
                    apply_deadband_and_sat(my_GYRO_cali.float_val[1], attitude_cali_coe._f.std_wy, GYRO_MAX_DPS);
                    my_GYRO_att_calculate.float_val[2] =
                    apply_deadband_and_sat(my_GYRO_cali.float_val[2], attitude_cali_coe._f.std_wz, GYRO_MAX_DPS);

                for (int i = 0; i < 3; ++i) {
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
                    my_ACCL_att_calculate.float_val[2] );

                // --- 滾動 coning 狀態 ---
                g_w_prev_dps[0] = my_GYRO_att_calculate.float_val[0];
                g_w_prev_dps[1] = my_GYRO_att_calculate.float_val[1];
                g_w_prev_dps[2] = my_GYRO_att_calculate.float_val[2];
                g_dt_prev_s = dt_curr;

                // pitch_AHRS_phase = ahrs_attitude.getLocalCasePitch();            // Store "pitch_AHRS_phase" computed in the AHRS phase for use in the Euler phase.
                w_eq_dps_AHRS_phase[0] = w_eq_dps[0];                            // Store "w_eq_dps" computed in the AHRS phase for use in the Euler phase.
                w_eq_dps_AHRS_phase[1] = w_eq_dps[1];
                w_eq_dps_AHRS_phase[2] = w_eq_dps[2];
                dt_AHRS_phase = dt_curr;                                         // Store "dt_curr" computed in the AHRS phase for use in the Euler phase.
                
                // Serial.println(dt_curr, 6);                                   // The "dt_curr" value should be about 0.01. (AHRS calculation rate: 100 Hz.)

                phase = PH_EULER;
                break;    // case PH_AHRS break.
                }         // case, PH_AHRS, end.

            case PH_EULER:
                {    // case, PH_EULER, start.
                // Phase = PH_EULER: Euler angle derived from quaternion (case frame) + Unwrapped Moving Average filter. k=2,4,6, ...
                float roll_deg, pitch_deg, yaw_deg;
                ahrs_attitude.getLocalCaseEuler(roll_deg, pitch_deg, yaw_deg);
                my_att.float_val[0] = pitch_deg;
                my_att.float_val[1] = roll_deg;

                // 3-6) 奇異區遲滯 + 投影積分 yaw
                float pitch_deg_now = my_att.float_val[0];
                if (!yaw_gl_locked && fabsf(pitch_deg_now) >= GL_ENTER_DEG) {
                    yaw_gl_locked = true;
                    yaw_hold_deg = yaw_deg;     // 2026/03/05 changed, "ahrs_attitude.getLocalCaseYaw()" -> "yaw_deg"
                }
                if (yaw_gl_locked && fabsf(pitch_deg_now) <= GL_EXIT_DEG) {
                    yaw_gl_locked = false;
                }

                if (yaw_gl_locked) {
                    // 取 q_WS，投影 ω_b 到世界座標，積分 z 分量
                    float q0,q1,q2,q3; ahrs_attitude.getQuatWS(q0,q1,q2,q3);
                    float Rwb[9]; quat_to_Rwb(q0,q1,q2,q3, Rwb);
                    
                    // 2026/03/04: Change
                    // Use "w_eq_dps" stored in the PH_AHRS phase. ("w_eq_dps_AHRS_phase")
                    float omega_b_dps[3] = { w_eq_dps_AHRS_phase[0], w_eq_dps_AHRS_phase[1], w_eq_dps_AHRS_phase[2] };

                    float omega_w_dps[3] = {
                        Rwb[0]*omega_b_dps[0] + Rwb[1]*omega_b_dps[1] + Rwb[2]*omega_b_dps[2],
                        Rwb[3]*omega_b_dps[0] + Rwb[4]*omega_b_dps[1] + Rwb[5]*omega_b_dps[2],
                        Rwb[6]*omega_b_dps[0] + Rwb[7]*omega_b_dps[1] + Rwb[8]*omega_b_dps[2]
                    };

                    // 2026/03/04: Change
                    // Use "dt_curr" stored in the PH_AHRS phase.  ("dt_AHRS_phase")
                    yaw_hold_deg += omega_w_dps[2] * dt_AHRS_phase; // dps × s = deg

                    my_att.float_val[2] = yaw_hold_deg;
                } else {
                    // 正常使用庫內 yaw
                    my_att.float_val[2] = yaw_deg;             // 2026/03/05 changed, "ahrs_attitude.getLocalCaseYaw()" -> "yaw_deg"
                    yaw_hold_deg = my_att.float_val[2];        // 讓持有值跟上
                }

                my_att.float_val[2] = applyGNSSIMUYawFusion(my_att.float_val[2]); // GNSS-IMU yaw融合

                phase = PH_AHRS;
                break;    // case PH_EULER break.
                }         // case, PH_EULER, end.
        }                 // switch, phase, end.


        // Send output data to PC.
        // --- 座標旋轉至輸出IMU顯示正確 --- 
        my_acc_t my_GYRO_case_frame, my_memsXLM_case_frame;
        ahrs_attitude.sensorVecToCase(my_GYRO_cali.float_val,     my_GYRO_case_frame.float_val);
        ahrs_attitude.sensorVecToCase(my_ACCL_cali.float_val,  my_memsXLM_case_frame.float_val);

        // Serial.print("case: "); 
        // Serial.print(my_GYRO_case_frame.float_val[0], 2); Serial.print(", "); 
        // Serial.print(my_GYRO_case_frame.float_val[1], 2); Serial.print(", "); 
        // Serial.print(my_GYRO_case_frame.float_val[2], 2); Serial.print("\n"); 
        

        //   print_imu_data(false, my_ACCL_cali.float_val, my_GYRO_cali.float_val);

        // // Prepare to send the calculation results.
        uint8_t imu_packet[IMU_PACKET_TOTAL_LEN];
        uint8_t CRC32[4];


        // packet the data to send.
        memcpy(imu_packet + 0,               KVH_HEADER, 4);
        memcpy(imu_packet + IMU_GYRO_OFFSET, my_GYRO_case_frame.bin_val, 12);
        memcpy(imu_packet + IMU_ACCL_OFFSET, my_memsXLM_case_frame.bin_val, 12);
        // memcpy(imu_packet + IMU_TEMP_OFFSET, PD_temp.bin_val, 4);
        imu_packet[IMU_TEMP_OFFSET + 0] = PD_temp.bin_val[3];
        imu_packet[IMU_TEMP_OFFSET + 1] = PD_temp.bin_val[2];
        imu_packet[IMU_TEMP_OFFSET + 2] = PD_temp.bin_val[1];
        imu_packet[IMU_TEMP_OFFSET + 3] = PD_temp.bin_val[0];
        memcpy(imu_packet + IMU_TIME_OFFSET, mcu_time.bin_val, 4);
        memcpy(imu_packet + IMU_ATT_OFFSET,  my_att.bin_val, 12);
        imu_packet[IMU_GPS_OFFSET] = last_gnss_status;
        // Calculate CRC. (CRC32 for bytes [0..48])
        myCRC.crc_32(imu_packet, IMU_PACKET_NOCRC_LEN, CRC32);
        memcpy(imu_packet + IMU_CRC_OFFSET, CRC32, 4);

        if (data_cnt >= DELAY_CNT)
        {
            Serial1.write(imu_packet, IMU_PACKET_TOTAL_LEN);
        }

        resetWDT(); 
        reset_EXT_WDI(WDI); 
    }
    // }
	}
	clear_SEL_EN(select_fn);
}

// GPS 位置封包輸出函數 (KVH 格式，標頭 0x82)
void sendGpsPacketKVH(const GnssData& gnss_data) {
    // 定義 GPS 標頭（直接在函數內定義）
    const unsigned char GPS_HEADER[4] = {0xFE, 0x82, 0xFF, 0x55};

    // 引用外部變數
    extern const unsigned char MARS_PD_TEMP[4];
    extern my_time_t mcu_time;
    extern crcCal myCRC;

    // CRC32 變數（局部定義）
    uint8_t CRC32[4];

    // GPS 位置封包結構 (53 bytes 總長度)
    uint8_t* gps_packet = (uint8_t*)malloc(53);

    // GPS 標頭 (4 bytes): 0xFE 0x82 0xFF 0x55
    memcpy(gps_packet, GPS_HEADER, 4);

    // GPS 位置數據 (24 bytes): 經度(8) + 緯度(8) + 海拔(4) + 時間戳(4)
    union { double d; uint8_t b[8]; } lat_union, lon_union;
    union { float f; uint8_t b[4]; } alt_union, time_union;

    lat_union.d = gnss_data.latitude;
    lon_union.d = gnss_data.longitude;
    alt_union.f = gnss_data.altitude;
    time_union.f = (float)millis(); // 使用 MCU 時間戳

    memcpy(gps_packet + 4, lat_union.b, 8);   // 緯度
    memcpy(gps_packet + 12, lon_union.b, 8);  // 經度
    memcpy(gps_packet + 20, alt_union.b, 4);  // 海拔
    memcpy(gps_packet + 24, time_union.b, 4); // 時間戳

    // UTC 時間數據 (8 bytes): 時(1) 分(1) 秒(1) 毫秒(2) 日(1) 月(1) 年(2)
    if (gnss_data.time_valid) {
        gps_packet[28] = gnss_data.hour;
        gps_packet[29] = gnss_data.minute;
        gps_packet[30] = gnss_data.second;
        gps_packet[31] = gnss_data.millisecond & 0xFF;
        gps_packet[32] = (gnss_data.millisecond >> 8) & 0xFF;
        gps_packet[33] = gnss_data.day;
        gps_packet[34] = gnss_data.month;
        gps_packet[35] = gnss_data.year & 0xFF;
        gps_packet[36] = (gnss_data.year >> 8) & 0xFF;
    } else {
        memset(gps_packet + 28, 0, 9); // 無效時間，填0
    }

    // 溫度數據 (4 bytes)
    memcpy(gps_packet + 37, MARS_PD_TEMP, 4);

    // MCU 時間 (4 bytes)
    mcu_time.ulong_val = millis();
    memcpy(gps_packet + 41, mcu_time.bin_val, 4);

    // GPS 狀態 (1 byte)
    gps_packet[45] = (uint8_t)gnss_data.status;

    // 保留位 (3 bytes)
    memset(gps_packet + 46, 0, 3);

    // 計算 CRC32 (4 bytes)
    myCRC.crc_32(gps_packet, 49, CRC32);
    memcpy(gps_packet + 49, CRC32, 4);

    // 發送完整封包
    Serial1.write(gps_packet, 53);

    free(gps_packet);
}

static float sf_temp_comp_1st(float temp, int slope, int offset, bool print=0) {
    float slope_f  = IEEE_754_INT2F(slope);
    float offset_f = IEEE_754_INT2F(offset);
    if(print) {
        Serial.print("----SF cali----\n");
        Serial.print("T: "); Serial.print(temp, 2); Serial.print(", ");
        Serial.print("Slope: ");  Serial.print(slope_f*10000, 2);  Serial.print(", ");
        Serial.print("offset: "); Serial.print(offset_f*10000, 2); Serial.print("\n");
    }
  return slope_f * temp + offset_f;
}

static float bias_temp_comp_1st_3t(float temp,
                                    int T1, int T2,
                                    int s1, int o1,
                                    int s2, int o2,
                                    int s3, int o3, bool print) {
  float slope, offset;
  float T1_f = IEEE_754_INT2F(T1); float T2_f = IEEE_754_INT2F(T2); float s1_f = IEEE_754_INT2F(s1); float o1_f = IEEE_754_INT2F(o1);
  float s2_f = IEEE_754_INT2F(s2); float o2_f = IEEE_754_INT2F(o2); float s3_f = IEEE_754_INT2F(s3); float o3_f = IEEE_754_INT2F(o3);
  if      (temp < T1_f) { slope = s1_f; offset = o1_f; }
  else if (temp < T2_f) { slope = s2_f; offset = o2_f; }
  else                  { slope = s3_f; offset = o3_f; }
  if(print) {
    Serial.print("----Bias cali----\n");
    Serial.print("T1: "); Serial.print(T1_f, 2); Serial.print(", ");
    Serial.print("T2: "); Serial.print(T2_f, 2); Serial.print(", ");
    Serial.print("s1: "); Serial.print(s1_f*10000, 4); Serial.print(", ");
    Serial.print("o1: "); Serial.print(o1_f*10000, 4); Serial.print(", ");
    Serial.print("s2: "); Serial.print(s2_f*10000, 4); Serial.print(", ");
    Serial.print("o2: "); Serial.print(o2_f*10000, 4); Serial.print(", ");
    Serial.print("s3: "); Serial.print(s3_f*10000, 4); Serial.print(", ");
    Serial.print("o3: "); Serial.print(o3_f*10000, 4); Serial.print("\n");
}
  return slope * temp + offset;
}

static float IEEE_754_INT2F(int in)
{
	my_float_t temp;
	temp.int_val = in;

	return temp.float_val;
}

static int IEEE_754_F2INT(float in)
{
	my_float_t temp;
	temp.float_val = in;

	return temp.int_val;
}

/**
 * 將 DS1775 的 16-bit 原始資料 (MSB, LSB) 轉換為攝氏度浮點數
 * 支援 12-bit 解析度 (0.0625°C) 並處理 2 補數正負號
 */
static float convert_PD_temp_f(uint8_t msb, uint8_t lsb) {
    // 1. 合併為 16-bit 帶正負號整數 (int16_t)
    // MSB 移至高位，LSB 放在低位
    int16_t raw_temp = (int16_t)((msb << 8) | lsb);

    // 2. 轉換為浮點數
    // 根據 DS1775 手冊，12-bit 模式下 LSB 的 Bit 4 代表 0.0625°C [cite: 235, 238, 250]
    // 直接除以 256.0f 即可完整保留所有小數位元 [cite: 250]
    return (float)raw_temp / 256.0f;
}