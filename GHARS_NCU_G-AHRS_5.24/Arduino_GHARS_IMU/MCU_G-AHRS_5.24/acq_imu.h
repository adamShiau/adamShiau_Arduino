#pragma once
#include <Arduino.h>
#include "common.h"
#include "IMU_PIG_DEFINE.h"
#include "myUART.h" 
#include "myI2C.h"  
#include "EEPROM_MANAGE.h"
#include "myWDT.h"
#include "src/MadgwickAHRS_IMU.h"
#include "mySPI.h"
#include "crcCalculator.h"


// ================================
// IMU packet layout constants
// ================================
// 2026/03/06, Send output data parameters setting.

// Packet format:
// [0:3]   Header
// [4:15]  Gyro xyz (3 x float)
// [16:27] Acc xyz  (3 x float)
// [28:31] PD temp  (1 x float)
// [32:35] MCU time (1 x uint32/float, depends on definition)
// [36:47] Attitude pitch/roll/yaw (3 x float)
// [48]    GNSS status (1 byte)
// [49:52] CRC32

constexpr uint8_t IMU_PACKET_TOTAL_LEN = 53;
constexpr uint8_t IMU_PACKET_NOCRC_LEN = 49;

constexpr uint8_t IMU_GYRO_OFFSET = 4;
constexpr uint8_t IMU_ACCL_OFFSET = 16;
constexpr uint8_t IMU_TEMP_OFFSET = 28;
constexpr uint8_t IMU_TIME_OFFSET = 32;
constexpr uint8_t IMU_ATT_OFFSET  = 36;
constexpr uint8_t IMU_GPS_OFFSET  = 48;
constexpr uint8_t IMU_CRC_OFFSET  = 49;

constexpr size_t FOG_PD_TEMP_OFFSET   = 12;
// ================================


extern unsigned int CtrlReg;
extern  bool run_fog_flag;
extern unsigned long data_cnt;
extern volatile bool ISR_PEDGE;
extern my_time_t mcu_time;
extern unsigned int t_previous; 
extern const unsigned char KVH_HEADER[4];
extern const unsigned char MARS_PD_TEMP[4];
extern const unsigned char PIG_HEADER[2];
extern crcCal myCRC;
extern my_attitude_cali_t attitude_cali_coe;

extern Madgwick ahrs_attitude;
// extern void reset_SYNC()

// 對外提供 acq_imu 的原型
void acq_imu(byte &select_fn, unsigned int value, byte ch);



/* * First-Order LPF Alpha Index Table
 * Formula: alpha = (2*pi*fc*Ts) / (2*pi*fc*Ts + 1)
 * -------------------------------------------------------------------------
 * Index | Target BW (fc) | DR = 100 Hz (Ts=0.01) | DR = 200 Hz (Ts=0.005)
 * -------------------------------------------------------------------------
 * 0   |    Disable     |        1.0000         |        1.0000
 * 1   |     50 Hz      |        0.7584         |        0.6110
 * 2   |     20 Hz      |        0.5568         |        0.3860
 * 3   |     10 Hz      |        0.3858         |        0.2391 (Recommended)
 * 4   |      5 Hz      |        0.2391         |        0.1358
 * 5   |      1 Hz      |        0.0591         |        0.0305
 * -------------------------------------------------------------------------
 */
/* ---------------------------------------------------------------------
 * y[n]   : 目前的輸出 (Filtered Value)
 * y[n-1] : 上一次的輸出 (Last State)
 * x[n]   : 目前的輸入 (Raw Input)
 * alpha  : 平滑係數 (Smoothing Factor), 範圍 0 < alpha <= 1
 * * 2. 程式實作變體 (Implementation Formula):
 * y[n] = y[n-1] + alpha * (x[n] - y[n-1])  <-- 結構體內部的 apply() 邏輯
 * * 3. 係數與頻寬轉換公式 (Coefficient vs. Bandwidth):
 * [設計用] alpha = (2*pi*fc*Ts) / (2*pi*fc*Ts + 1)
 * [分析用] fc = (alpha * fs) / (2*pi * (1 - alpha))
 * ---------------------------------------------------------------------
 * fc : 截止頻率 (Cutoff Frequency, BW)
 * fs : 採樣頻率 (Sampling Rate, e.g., 200Hz)
 * Ts : 採樣週期 (Sampling Period, 1/fs)
 * * 4. 係數對照表 (fs = 200 Hz, Ts = 0.005s):
 * - Index 0: Disable  (alpha = 1.0000)
 * - Index 1: ~62 Hz   (alpha = 0.6610) <-- 高動態模式
 * - Index 3: 10 Hz    (alpha = 0.2391) <-- 推薦穩定模式
 * =========================================================================
 */
struct FirstOrderLPF3D {
    float state[3];
    bool inited = false;
    const char* lpf_name = "Unknown"; // 用於顯示辨識

    // 定義 200Hz 下對應的 Alpha 數值陣列 [cite: 271, 331]
    const float ALPHAS[6] = {1.0000f, 0.6110f, 0.3860f, 0.2391f, 0.1358f, 0.0305f};

    void apply(float* data, uint8_t idx) {
        if (idx >= 6) return;
        float alpha = ALPHAS[idx];
        if (alpha >= 1.0f) return;

        if (!inited) {
            for (int i = 0; i < 3; i++) state[i] = data[i];
            inited = true;
        } else {
            for (int i = 0; i < 3; i++) {
                state[i] += alpha * (data[i] - state[i]);
                data[i] = state[i];
            }
        }
    }

    void initAndPrint(const char* name, float dr, uint8_t idx) {
        this->lpf_name = name;
        this->inited = false;
        if (idx >= 6) return;

        float alpha = ALPHAS[idx];
        Serial.print("\n--- "); Serial.print(lpf_name); Serial.println(" LPF Initialized ---");
        Serial.print("Data Rate: "); Serial.print(dr, 0); Serial.println(" Hz");
        Serial.print("Alpha Index: ["); Serial.print(idx); Serial.println("]");
        
        if (alpha >= 1.0f) {
            Serial.println("LPF Status: DISABLED");
        } else {
            float fc = (alpha * dr) / (2.0f * 3.14159f * (1.0f - alpha));
            Serial.print("Estimated BW: "); Serial.print(fc, 1); Serial.println(" Hz");
        }
        Serial.println("----------------------------");
    }

    void reset() {
        if (inited) {
            Serial.print(lpf_name); Serial.println(" LPF Reset (STOP_SYNC).");
        }
        this->inited = false;
    }
};

mcu_fletcher16(const uint8_t* data, size_t len);
