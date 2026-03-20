#pragma once
#include <Arduino.h>

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

/**
 * @brief 預定義 Alpha 數值陣列 (對應 100Hz/200Hz 等頻率下的濾波強度)
 */
static const float LPF_ALPHAS_TABLE[6] = {1.0000f, 0.6110f, 0.3859f, 0.2391f, 0.1358f, 0.0305f};

struct FirstOrderLPF_1D {
    float state = 0.0f;
    bool inited = false;
    const char* tag = "LPF_1D";

    // 初始化與顯示：在 ahrs_start_stream 中呼叫
    void init(const char* name, float dr, uint8_t idx) {
        this->tag = name;
        this->inited = false; // 標記為未初始化，讓 apply 的第一筆資料進來時自動設定 state
        if (idx >= 6) {
            Serial.print("\n[Filter] "); Serial.print(tag);
            Serial.println(" Warning: Invalid Index (Bypass Mode)");
            return; // 在印完警告後才返回
        }

        float alpha = LPF_ALPHAS_TABLE[idx];
        Serial.print("\n--- [Filter]  "); Serial.print(tag); Serial.println(" LPF Initialized ---");
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

    // 核心運算：由 ahrs_run_tick 持續呼叫
    inline void apply(float* data, uint8_t alpha_idx) {
        if (alpha_idx >= 6 || LPF_ALPHAS_TABLE[alpha_idx] >= 1.0f) return;
        
        if (!inited) {
            state = *data; // 第一次執行時自動初始化狀態
            inited = true;
        } else {
            state += LPF_ALPHAS_TABLE[alpha_idx] * (*data - state);
            *data = state;
        }
    }

    // 重置：在 ahrs_stop_stream 或系統重啟時呼叫
    void reset() { 
        inited = false; 
        Serial.print(tag); Serial.println(" LPF Reset (STOP_SYNC).");
    }
};

// 3D 版本以此類推...