#include "adaptive_yaw_ekf.h"

AdaptiveYawEKF::AdaptiveYawEKF() {
    ekf_status = EKF_DISABLED;
    fused_heading = 0.0;
}

void AdaptiveYawEKF::update(uint8_t gps_status, float gnss_heading, float imu_yaw, float imu_yaw_rate) {
    // 狀態轉換判定
    if (!isActive() && shouldActivateEKF(gps_status)) {
        // 啟動 EKF
        initializeEKF(gnss_heading, imu_yaw);
        ekf_status = EKF_ACTIVE;
        printStatus("EKF activated");
    }
    else if (isActive() && shouldDeactivateEKF(gps_status)) {
        // 停用 EKF
        ekf_status = EKF_DISABLED;
        printStatus("EKF deactivated - using basic fusion");
    }

    // 執行相應的融合算法
    if (isActive()) {
        // 使用 EKF 融合
        runEKFStep(gnss_heading, imu_yaw_rate);
    } else {
        // 使用基本融合
        runBasicFusion(gnss_heading, imu_yaw);
    }
}

bool AdaptiveYawEKF::shouldActivateEKF(uint8_t gps_status) {
    // 簡化判定：只有 GPS 狀態為全部有效時才啟動
    return (gps_status == 0x00);  // DATA_ALL_VALID
}

bool AdaptiveYawEKF::shouldDeactivateEKF(uint8_t gps_status) {
    // 簡化判定：GPS 狀態不是全部有效就停用
    return (gps_status != 0x00);  // 不是 DATA_ALL_VALID
}

void AdaptiveYawEKF::initializeEKF(float initial_heading, float initial_imu_yaw) {
    // 使用 GNSS 航向初始化融合結果
    fused_heading = initial_heading;

    // TODO: 初始化 EKF 狀態向量和協方差矩陣
    // state = [yaw, yaw_rate, gnss_reliability]
    // P = initial_covariance_matrix

    printStatus("EKF initialized");
}

void AdaptiveYawEKF::runBasicFusion(float gnss_heading, float imu_yaw) {
    // 基本權重融合 (EKF 未啟動時的後備方案)
    float gnss_weight = 0.7;
    float imu_weight = 0.3;

    fused_heading = gnss_weight * gnss_heading + imu_weight * imu_yaw;
}

void AdaptiveYawEKF::runEKFStep(float gnss_heading, float imu_yaw_rate) {
    // TODO: 實作 EKF 核心算法
    // 1. Prediction Step
    // 2. Update Step
    // 3. Adaptive Noise Adjustment

    // 暫時使用簡單融合
    fused_heading = 0.8 * gnss_heading + 0.2 * fused_heading;
}

void AdaptiveYawEKF::reset() {
    ekf_status = EKF_DISABLED;
    fused_heading = 0.0;
    printStatus("EKF reset");
}

void AdaptiveYawEKF::printStatus(const char* message) {
    Serial.print("AdaptiveYawEKF: ");
    Serial.print(message);
    Serial.print(" | Status: ");
    Serial.print(isActive() ? "ACTIVE" : "DISABLED");
    Serial.print(" | Heading: ");
    Serial.println(fused_heading);
}