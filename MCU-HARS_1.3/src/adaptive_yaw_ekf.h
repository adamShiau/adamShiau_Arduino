#pragma once
#include <Arduino.h>

// EKF 狀態定義
enum EKF_Status {
    EKF_DISABLED = 0,
    EKF_ACTIVE = 1
};

class AdaptiveYawEKF {
public:
    AdaptiveYawEKF();

    // 主要更新函數
    void update(uint8_t gps_status, float gnss_heading, float imu_yaw, float imu_yaw_rate);

    // 取得融合後的航向角
    float getFusedHeading() const { return fused_heading; }

    // 取得 EKF 狀態
    bool isActive() const { return (ekf_status == EKF_ACTIVE); }

    // 重置 EKF
    void reset();

private:
    // EKF 狀態
    EKF_Status ekf_status;
    float fused_heading;

    // 狀態管理
    bool shouldActivateEKF(uint8_t gps_status);
    bool shouldDeactivateEKF(uint8_t gps_status);

    // EKF 初始化
    void initializeEKF(float initial_heading, float initial_imu_yaw);

    // 基本融合 (EKF 未啟動時使用)
    void runBasicFusion(float gnss_heading, float imu_yaw);

    // EKF 核心算法 (預留，待實作)
    void runEKFStep(float gnss_heading, float imu_yaw_rate);

    // 調試輸出
    void printStatus(const char* message);
};