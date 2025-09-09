#pragma once
#include <cstdint>
#include <algorithm>

/**
 * @brief 時間對齊器 - 將 IMU/GNSS 時間對齊到系統 micros()
 * 
 * 解決 IMU/GNSS 時間基準與 MCU micros() 不同步的問題
 * 使用線性映射: t_sys = scale * t_device + offset
 */
class TimeAligner {
public:
    /**
     * @brief 構造函數
     * @param tick_us IMU raw tick 轉 μs 的倍率
     */
    explicit TimeAligner(double tick_us = 1.0)
      : tick_us_(tick_us), have_ref_(false), offset_(0.0), scale_(1.0), last_est_(0.0) {}

    /**
     * @brief 將設備原始時間對齊到系統 μs
     * @param device_raw_ts 設備原始時間戳
     * @param sys_now_us 當前系統時間 (micros())
     * @return 對齊後的系統時間 (μs)
     */
    uint64_t toSystemMicros(uint64_t device_raw_ts, uint64_t sys_now_us) {
        double device_us = device_raw_ts * tick_us_;
        
        if (!have_ref_) {
            // 首次建立對齊：offset = sys - device_us
            offset_ = static_cast<double>(sys_now_us) - device_us;
            have_ref_ = true;
            last_est_ = static_cast<double>(sys_now_us);
            return sys_now_us;
        }
        
        // 線性映射
        double est = scale_ * device_us + offset_;
        
        // 防守：不回退（單調遞增）
        if (est < last_est_) {
            est = last_est_;
        }
        last_est_ = est;
        
        return static_cast<uint64_t>(est);
    }

    /**
     * @brief 細微校正 scale/offset（抑制長期漂移）
     * @param device_raw_ts 設備原始時間戳
     * @param sys_now_us 當前系統時間
     * @param alpha_scale_ppm scale 調整系數 (ppm 級)
     * @param alpha_off offset 調整系數
     */
    void refine(uint64_t device_raw_ts, uint64_t sys_now_us, 
                double alpha_scale_ppm = 0.001, double alpha_off = 0.01) {
        if (!have_ref_) return;
        
        double device_us = device_raw_ts * tick_us_;
        double est = scale_ * device_us + offset_;
        double err = static_cast<double>(sys_now_us) - est; // μs 誤差
        
        // 調整 offset（快速響應）
        offset_ += alpha_off * err;
        
        // 調整 scale（慢速，ppm 級）
        if (device_us > 1.0) {
            double ppm_step = alpha_scale_ppm * (err / device_us);
            scale_ += ppm_step;
        }
    }

    /**
     * @brief 設置 tick 到微秒的轉換倍率
     * @param tick_us 每個 tick 對應的微秒數
     */
    void setTickMicros(double tick_us) { tick_us_ = tick_us; }
    
    /**
     * @brief 獲取當前縮放係數
     */
    double scale() const { return scale_; }
    
    /**
     * @brief 獲取當前偏移量
     */
    double offset() const { return offset_; }
    
    /**
     * @brief 檢查是否已建立參考
     */
    bool ready() const { return have_ref_; }

private:
    double tick_us_;        // tick 到微秒的轉換倍率
    bool have_ref_;         // 是否已建立時間參考
    double offset_;         // 時間偏移量 (μs)
    double scale_;          // 時間縮放係數
    double last_est_;       // 上次估算的時間，用於防止回退
};