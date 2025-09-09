#pragma once

#include "../data/data_types.h"
#include <stdint.h>

struct EKFOrientationState {
    // State vector: [qw, qx, qy, qz, wx, wy, wz, bx, by, bz]
    float state[10];
    
    // Covariance matrix (10x10)
    float covariance[10][10];
    
    // Innovation and residuals
    float innovation[7];  // [quat_error[3], gyro_error[3], mag_error[1]]
    float residual_norm;
    
    uint32_t last_update_time;
    bool is_initialized;
};

struct EKFOrientationConfig {
    // Process noise
    float process_noise_quaternion;
    float process_noise_gyro_bias;
    float process_noise_angular_velocity;
    
    // Measurement noise
    float gyro_noise;
    float accel_noise;
    float mag_noise;
    
    // Initialization
    float initial_attitude_uncertainty;
    float initial_gyro_bias_uncertainty;
    
    // Gravity and magnetic field reference
    float gravity_magnitude;
    float magnetic_declination;
};

class EKFOrientation {
public:
    EKFOrientation();
    ~EKFOrientation();
    
    bool initialize(const EKFOrientationConfig& config);
    void reset();
    
    void predict(float dt, const float angular_velocity[3]);
    void updateWithIMU(const float acceleration[3], const float angular_velocity[3],
                      const float gyro_noise[3], const float accel_noise[3]);
    void updateWithMagnetometer(const float magnetometer[3], const float mag_noise[3]);
    
    void getQuaternion(float quaternion[4]) const;
    void getEulerAngles(float& roll, float& pitch, float& yaw) const;
    void getAngularVelocity(float angular_velocity[3]) const;
    void getGyroBias(float bias[3]) const;
    
    float getAttitudeUncertainty() const;
    bool isConverged() const;
    
    EKFOrientationState getFullState() const { return state_; }
    
private:
    EKFOrientationConfig config_;
    EKFOrientationState state_;
    
    // Working matrices for calculations
    float F_[10][10];  // State transition matrix
    float H_[7][10];   // Observation matrix
    float Q_[10][10];  // Process noise covariance
    float R_[7][7];    // Measurement noise covariance
    float K_[10][7];   // Kalman gain
    
    void predictStateTransition(float dt, const float angular_velocity[3]);
    void predictCovariance(float dt);
    void updateKalmanGain();
    void updateState();
    void updateCovariance();
    
    void normalizeQuaternion();
    void buildStateTransitionMatrix(float dt, const float angular_velocity[3]);
    void buildProcessNoiseMatrix(float dt);
    void buildObservationMatrix();
    
    // Quaternion utilities
    void quaternionMultiply(const float q1[4], const float q2[4], float result[4]);
    void quaternionToEuler(const float quaternion[4], float& roll, float& pitch, float& yaw);
    void eulerToQuaternion(float roll, float pitch, float yaw, float quaternion[4]);
};