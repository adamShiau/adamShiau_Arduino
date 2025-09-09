#pragma once

#include "../data/data_types.h"
#include "ekf_position.h"
#include "ekf_orientation.h"

struct FusionConfig {
    // Time synchronization
    uint32_t max_time_diff_ms;
    bool enable_time_sync;
    
    // Dynamic weights
    float gnss_position_weight;
    float gnss_velocity_weight;
    float imu_acceleration_weight;
    float imu_angular_weight;
    
    // Quality thresholds
    float min_gnss_hdop;
    uint8_t min_satellite_count;
    float max_acceleration_noise;
    float max_gyro_noise;
    
    // Fusion parameters
    float prediction_dt;
    bool enable_adaptive_weights;
    bool enable_outlier_rejection;
};

struct FusionState {
    NavigationState nav_state;
    float gnss_quality;
    float imu_quality;
    uint32_t last_gnss_time;
    uint32_t last_imu_time;
    bool is_initialized;
    uint32_t fusion_count;
};

class FusionEngine {
public:
    FusionEngine();
    ~FusionEngine();
    
    bool initialize(const FusionConfig& config);
    void reset();
    
    bool processSensorData(const SensorDataPacket& sensor_packet);
    NavigationState getNavigationState() const;
    FusionState getFusionState() const { return fusion_state_; }
    
    void updateConfig(const FusionConfig& config);
    FusionConfig getConfig() const { return config_; }
    
    bool isInitialized() const { return fusion_state_.is_initialized; }
    float getDataQuality() const;
    
private:
    FusionConfig config_;
    FusionState fusion_state_;
    
    EKFPosition position_filter_;
    EKFOrientation orientation_filter_;
    
    bool processGNSSData(const GNSSData& gnss_data);
    bool processIMUData(const IMUData& imu_data);
    
    void updateDynamicWeights();
    bool performTimeSync(const SensorDataPacket& packet);
    bool validateDataQuality(const SensorDataPacket& packet);
    
    float calculateGNSSQuality(const GNSSData& gnss_data);
    float calculateIMUQuality(const IMUData& imu_data);
    
    void predictState(float dt);
    void updateWithGNSS();
    void updateWithIMU();
};