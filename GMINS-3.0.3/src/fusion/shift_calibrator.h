#pragma once

#include "../data/data_types.h"
#include <stdint.h>

enum class CalibrationState {
    IDLE,
    COLLECTING_PLSHD,
    COLLECTING_MTI,
    CALCULATING_OFFSET,
    CALIBRATED,
    ERROR
};

enum class CalibrationMode {
    STANDARD,      // 四元數模式
    FUSION         // 歐拉角模式
};

struct ShiftCalibrationConfig {
    uint8_t plshd_samples_required = 20;      // PLSHD 樣本數
    uint8_t mti_samples_required = 50;        // MTI 樣本數  
    uint32_t setup_timeout_ms = 120000;       // 初始化超時 (2分鐘)
    bool fusion_mode_enabled = true;         // 預設 Fusion 模式
    float max_shift_change_rate = 1.0f;       // 最大變化率 °/s
    float outlier_threshold = 10.0f;          // 異常值閾值 °
    float min_valid_heading = 0.0f;           // 最小有效航向
    float max_valid_heading = 360.0f;         // 最大有效航向
    uint32_t dynamic_update_interval_ms = 1000; // 動態更新間隔
};

struct ShiftCalibrationStatus {
    CalibrationState state;
    CalibrationMode mode;
    
    uint8_t plshd_samples_collected;
    uint8_t mti_samples_collected;
    
    float plshd_average;                      // PLSHD 平均值
    float mti_average;                        // MTI 平均值  
    float current_offset;                     // 當前校正偏移量
    
    bool is_calibrated;
    uint32_t calibration_start_time;
    uint32_t last_update_time;
    uint32_t total_calibration_time;
    
    float last_gnss_heading;
    float last_imu_yaw;
    uint32_t last_gnss_timestamp;
    uint32_t last_imu_timestamp;
};

struct ShiftCalibrationStats {
    uint32_t setup_calibrations_performed;
    uint32_t dynamic_updates_applied;
    uint32_t nan_recoveries;
    uint32_t outliers_rejected;
    float max_offset_recorded;
    float min_offset_recorded;
};

class ShiftCalibrator {
public:
    ShiftCalibrator();
    ~ShiftCalibrator();
    
    bool initialize(const ShiftCalibrationConfig& config);
    void update();
    void reset();
    
    // 校正流程控制
    bool startSetupCalibration();
    bool isSetupComplete() const;
    void abortCalibration();
    
    // 資料輸入
    bool addPLSHDSample(float heading_deg, uint32_t timestamp);
    bool addMTISample(float yaw_deg, uint32_t timestamp);
    
    // 動態校正
    void updateDynamicCorrection(float gnss_heading, float imu_yaw, uint32_t timestamp);
    
    // 校正應用
    float applyShiftCorrection(float raw_yaw_deg) const;
    void applyCorrectionToQuaternion(float quaternion[4]) const;
    void applyCorrectionToEulerAngles(float euler[3]) const;
    
    // 狀態查詢
    ShiftCalibrationStatus getStatus() const { return status_; }
    ShiftCalibrationStats getStats() const { return stats_; }
    float getCurrentOffset() const { return status_.current_offset; }
    
    // 模式控制
    void setFusionMode(bool enabled);
    bool isFusionModeEnabled() const { return config_.fusion_mode_enabled; }
    CalibrationMode getCurrentMode() const { return status_.mode; }
    
    // 回調函數設定
    void setCalibrationCompleteCallback(void (*callback)(float offset));
    void setShiftUpdateCallback(void (*callback)(float offset, float gnss_heading, float imu_yaw));
    void setErrorCallback(void (*callback)(const char* error_message));
    
    // 診斷和除錯
    void printDetailedStatus() const;
    void printCalibrationResults() const;
    bool validateConfiguration() const;
    
    // 配置更新
    void updateConfig(const ShiftCalibrationConfig& config);
    ShiftCalibrationConfig getConfig() const { return config_; }
    
private:
    ShiftCalibrationConfig config_;
    ShiftCalibrationStatus status_;
    ShiftCalibrationStats stats_;
    
    // 樣本緩衝區
    float* plshd_buffer_;
    float* mti_buffer_;
    
    // 回調函數指標
    void (*calibration_complete_callback_)(float offset);
    void (*shift_update_callback_)(float offset, float gnss_heading, float imu_yaw);
    void (*error_callback_)(const char* error_message);
    
    // 內部方法
    bool collectPLSHDSamples();
    bool collectMTISamples();
    void calculateInitialOffset();
    
    bool validatePLSHDSample(float heading) const;
    bool validateMTISample(float yaw) const;
    bool isOutlier(float value, const float* buffer, uint8_t count) const;
    
    float calculateMean(const float* buffer, uint8_t count) const;
    float calculateStandardDeviation(const float* buffer, uint8_t count) const;
    
    float normalizeAngle(float angle) const;
    float angleDifference(float angle1, float angle2) const;
    
    void updateStatistics();
    void handleCalibrationError(const char* error_message);
    bool isDataStale(uint32_t timestamp, uint32_t max_age_ms) const;
    
    // 四元數和歐拉角轉換
    void quaternionToEuler(const float q[4], float euler[3]) const;
    void eulerToQuaternion(const float euler[3], float q[4]) const;
    void applyYawRotationToQuaternion(float quaternion[4], float yaw_offset_deg) const;
};