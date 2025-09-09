#pragma once

#include "data_types.h"
#include "coord_transform.h"

struct ProcessingConfig {
    // Filter parameters
    float lowpass_cutoff_freq;
    float highpass_cutoff_freq;
    bool enable_outlier_rejection;
    float outlier_threshold_sigma;
    
    // Calibration parameters
    bool enable_auto_calibration;
    uint32_t calibration_samples;
    float calibration_threshold;
    
    // Validation parameters
    bool enable_range_checking;
    bool enable_continuity_checking;
    float max_delta_threshold;
};

class DataProcessor {
public:
    DataProcessor();
    ~DataProcessor();
    
    bool initialize(const ProcessingConfig& config);
    
    // Data preprocessing
    bool processIMUData(IMUData& imu_data);
    bool processGNSSData(GNSSData& gnss_data);
    
    // Data validation
    bool validateIMUData(const IMUData& imu_data);
    bool validateGNSSData(const GNSSData& gnss_data);
    
    // Data quality assessment
    float assessIMUQuality(const IMUData& imu_data);
    float assessGNSSQuality(const GNSSData& gnss_data);
    
    // Outlier detection
    bool isIMUOutlier(const IMUData& imu_data);
    bool isGNSSOutlier(const GNSSData& gnss_data);
    
    // Time synchronization helpers
    SensorDataPacket synchronizeSensorData(const IMUData& imu_data, const GNSSData& gnss_data);
    bool isDataTemporallyAligned(const IMUData& imu_data, const GNSSData& gnss_data, uint32_t max_offset_ms);
    
    // Statistics
    struct ProcessingStats {
        uint32_t imu_samples_processed;
        uint32_t gnss_samples_processed;
        uint32_t outliers_rejected;
        uint32_t validation_failures;
        float avg_imu_quality;
        float avg_gnss_quality;
    };
    
    ProcessingStats getStats() const { return stats_; }
    void resetStats();
    
private:
    ProcessingConfig config_;
    ProcessingStats stats_;
    
    // Historical data for filtering and validation
    IMUData imu_history_[10];
    GNSSData gnss_history_[10];
    uint8_t imu_history_index_;
    uint8_t gnss_history_index_;
    uint8_t imu_history_size_;
    uint8_t gnss_history_size_;
    
    // Calibration state
    bool is_calibrating_;
    uint32_t calibration_sample_count_;
    float accel_bias_[3];
    float gyro_bias_[3];
    
    // Filter states
    float imu_lowpass_state_[6];  // 3 accel + 3 gyro
    float gnss_lowpass_state_[6]; // 3 pos + 3 vel
    
    // Processing functions
    bool applyCalibratedBiasCorrection(IMUData& imu_data);
    bool applyLowPassFilter(float* data, float* state, int channels, float alpha);
    bool performOutlierRejection(const float* current, const float* history, int samples, float threshold);
    bool checkDataContinuity(const float* current, const float* previous, float max_delta);
    
    void updateHistory(const IMUData& imu_data);
    void updateHistory(const GNSSData& gnss_data);
    
    float calculateStandardDeviation(const float* data, int count);
    float calculateMean(const float* data, int count);
};