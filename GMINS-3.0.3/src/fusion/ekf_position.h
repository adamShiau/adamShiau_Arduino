#pragma once

#include "../data/data_types.h"
#include <stdint.h>

struct EKFPositionState {
    // State vector: [x, y, z, vx, vy, vz, ax, ay, az]
    float state[9];
    
    // Covariance matrix (9x9)
    float covariance[9][9];
    
    // Innovation and residuals
    float innovation[6];  // [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
    float residual_norm;
    
    uint32_t last_update_time;
    bool is_initialized;
};

struct EKFPositionConfig {
    // Process noise
    float process_noise_position;
    float process_noise_velocity;
    float process_noise_acceleration;
    
    // Measurement noise
    float gnss_position_noise;
    float gnss_velocity_noise;
    float imu_acceleration_noise;
    
    // Initialization
    float initial_position_uncertainty;
    float initial_velocity_uncertainty;
    float initial_acceleration_uncertainty;
};

class EKFPosition {
public:
    EKFPosition();
    ~EKFPosition();
    
    bool initialize(const EKFPositionConfig& config);
    void reset();
    
    void predict(float dt, const float acceleration[3]);
    void updateWithGNSS(const float position[3], const float velocity[3], 
                       const float position_noise[3], const float velocity_noise[3]);
    
    void getState(float position[3], float velocity[3], float acceleration[3]) const;
    void getCovariance(float covariance[9][9]) const;
    
    float getPositionUncertainty() const;
    float getVelocityUncertainty() const;
    bool isConverged() const;
    
    EKFPositionState getFullState() const { return state_; }
    
private:
    EKFPositionConfig config_;
    EKFPositionState state_;
    
    // Working matrices for calculations
    float F_[9][9];  // State transition matrix
    float H_[6][9];  // Observation matrix
    float Q_[9][9];  // Process noise covariance
    float R_[6][6];  // Measurement noise covariance
    float K_[9][6];  // Kalman gain
    
    void predictStateTransition(float dt);
    void predictCovariance(float dt);
    void updateKalmanGain();
    void updateState();
    void updateCovariance();
    
    void buildStateTransitionMatrix(float dt);
    void buildProcessNoiseMatrix(float dt);
    void buildObservationMatrix();
};