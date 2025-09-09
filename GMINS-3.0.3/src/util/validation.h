#pragma once

/**
 * GMINS 數據驗證模組
 * 提供所有感測器數據和導航狀態的驗證功能
 */

#include "../data/data_types.h"
#include "math_utils.h"

namespace DataValidation {

// =============================================================================
// 🧪 IMU 數據驗證
// =============================================================================

/**
 * 驗證 IMU 數據的有效性
 * @param data IMU 數據結構
 * @return 驗證結果和詳細錯誤信息
 */
struct IMUValidationResult {
    bool is_valid;
    uint32_t error_flags;
    
    // 錯誤標誌定義
    static constexpr uint32_t ERROR_TIMESTAMP_ZERO     = 0x00000001;
    static constexpr uint32_t ERROR_ACCEL_MAGNITUDE    = 0x00000002;
    static constexpr uint32_t ERROR_GYRO_MAGNITUDE     = 0x00000004;
    static constexpr uint32_t ERROR_QUATERNION_INVALID = 0x00000008;
    static constexpr uint32_t ERROR_TEMPERATURE_RANGE  = 0x00000010;
    static constexpr uint32_t ERROR_NAN_VALUES         = 0x00000020;
    static constexpr uint32_t ERROR_FLAGS_INCONSISTENT = 0x00000040;
};

IMUValidationResult validateIMUData(const IMUData& data);

/**
 * 檢查 IMU 數據中的 NaN 或 Inf 值
 */
bool hasInvalidValues(const IMUData& data);

/**
 * 驗證加速度計數據合理性
 * @param accel_x, accel_y, accel_z 加速度數據（m/s²）
 * @return 是否在合理範圍內
 */
bool isAccelMagnitudeValid(float accel_x, float accel_y, float accel_z);

/**
 * 驗證陀螺儀數據合理性
 * @param gyro_x, gyro_y, gyro_z 角速度數據（rad/s）
 * @return 是否在合理範圍內
 */
bool isGyroMagnitudeValid(float gyro_x, float gyro_y, float gyro_z);

// =============================================================================
// 📡 GNSS 數據驗證
// =============================================================================

/**
 * 驗證 GNSS 數據的有效性
 */
struct GNSSValidationResult {
    bool is_valid;
    uint32_t error_flags;
    
    // 錯誤標誌定義
    static constexpr uint32_t ERROR_TIMESTAMP_ZERO     = 0x00000001;
    static constexpr uint32_t ERROR_LATITUDE_RANGE     = 0x00000002;
    static constexpr uint32_t ERROR_LONGITUDE_RANGE    = 0x00000004;
    static constexpr uint32_t ERROR_ALTITUDE_RANGE     = 0x00000008;
    static constexpr uint32_t ERROR_ACCURACY_POOR      = 0x00000010;
    static constexpr uint32_t ERROR_SATELLITE_COUNT    = 0x00000020;
    static constexpr uint32_t ERROR_FIX_TYPE_INVALID   = 0x00000040;
    static constexpr uint32_t ERROR_VELOCITY_EXCESSIVE = 0x00000080;
    static constexpr uint32_t ERROR_NAN_VALUES         = 0x00000100;
};

GNSSValidationResult validateGNSSData(const GNSSData& data);

/**
 * 驗證 WGS84 位置數據
 */
bool isWGS84PositionValid(double latitude, double longitude, float altitude_msl);

/**
 * 驗證 GNSS 速度數據合理性
 */
bool isGNSSVelocityValid(float vel_north, float vel_east, float vel_down);

/**
 * 驗證 GNSS 精度數據
 */
bool isGNSSAccuracyAcceptable(
    float horizontal_accuracy, 
    float vertical_accuracy,
    float min_horizontal_threshold = 10.0f,
    float min_vertical_threshold = 20.0f
);

// =============================================================================
// 🧭 導航狀態驗證
// =============================================================================

/**
 * 驗證導航狀態的有效性
 */
struct NavigationValidationResult {
    bool is_valid;
    uint32_t error_flags;
    
    // 錯誤標誌定義
    static constexpr uint32_t ERROR_TIMESTAMP_ZERO     = 0x00000001;
    static constexpr uint32_t ERROR_QUATERNION_INVALID = 0x00000002;
    static constexpr uint32_t ERROR_POSITION_EXCESSIVE = 0x00000004;
    static constexpr uint32_t ERROR_VELOCITY_EXCESSIVE = 0x00000008;
    static constexpr uint32_t ERROR_UNCERTAINTY_HIGH   = 0x00000010;
    static constexpr uint32_t ERROR_NAN_VALUES         = 0x00000020;
    static constexpr uint32_t ERROR_FLAGS_INCONSISTENT = 0x00000040;
};

NavigationValidationResult validateNavigationState(const NavigationState& nav_state);

/**
 * 驗證不確定性數據是否合理
 */
bool isUncertaintyReasonable(const NavigationState& nav_state);

// =============================================================================
// 🔧 四元數驗證工具
// =============================================================================

/**
 * 驗證四元數並嘗試正規化
 * @param qw, qx, qy, qz 四元數分量（可能會被修改）
 * @return 驗證和正規化結果
 */
struct QuaternionValidationResult {
    bool was_valid;          // 原始四元數是否有效
    bool normalized_success; // 正規化是否成功
    float original_norm;     // 原始範數
    float final_norm;        // 最終範數
};

QuaternionValidationResult validateAndNormalizeQuaternion(
    float& qw, float& qx, float& qy, float& qz
);

// =============================================================================
// 📊 統計驗證工具
// =============================================================================

/**
 * 時間序列數據一致性檢查
 */
struct TimeSeriesValidationResult {
    bool is_consistent;
    uint64_t time_gap_us;           // 與上次數據的時間間隔
    bool time_gap_excessive;        // 時間間隔是否過大
    bool timestamp_reversed;        // 時間戳是否倒退
};

TimeSeriesValidationResult validateTimestamp(
    uint64_t current_timestamp_us,
    uint64_t previous_timestamp_us,
    uint64_t max_gap_us = 1000000  // 默認最大間隔 1 秒
);

/**
 * 數據跳躍檢測
 */
template<typename T>
struct JumpDetectionResult {
    bool jump_detected;
    T current_value;
    T previous_value;
    T difference;
    T threshold_used;
};

template<typename T>
JumpDetectionResult<T> detectJump(
    T current_value, 
    T previous_value, 
    T threshold
) {
    JumpDetectionResult<T> result;
    result.current_value = current_value;
    result.previous_value = previous_value;
    result.difference = current_value - previous_value;
    result.threshold_used = threshold;
    result.jump_detected = (abs(result.difference) > threshold);
    return result;
}

// =============================================================================
// 🔍 組合驗證函數
// =============================================================================

/**
 * 綜合系統健康檢查
 */
struct SystemHealthResult {
    bool overall_health;
    IMUValidationResult imu_result;
    GNSSValidationResult gnss_result;
    NavigationValidationResult nav_result;
    uint32_t warning_flags;
    
    // 警告標誌
    static constexpr uint32_t WARNING_IMU_DEGRADED     = 0x00000001;
    static constexpr uint32_t WARNING_GNSS_POOR        = 0x00000002;
    static constexpr uint32_t WARNING_NAV_UNCERTAIN    = 0x00000004;
    static constexpr uint32_t WARNING_TIME_SYNC_ISSUE  = 0x00000008;
};

SystemHealthResult validateSystemHealth(
    const IMUData& imu_data,
    const GNSSData& gnss_data,
    const NavigationState& nav_state
);

/**
 * 驗證感測器數據時間同步性
 */
bool validateSensorTimestamp(
    uint64_t imu_timestamp_us,
    uint64_t gnss_timestamp_us,
    uint64_t max_desync_us = 100000  // 默認最大不同步時間 100ms
);

// =============================================================================
// 📈 性能監控工具
// =============================================================================

/**
 * 數據更新率監控
 */
class UpdateRateMonitor {
private:
    uint64_t last_timestamp_us;
    float target_rate_hz;
    float current_rate_hz;
    uint32_t update_count;
    uint64_t monitor_start_time_us;
    
public:
    explicit UpdateRateMonitor(float expected_rate_hz);
    
    void reset();
    void update(uint64_t timestamp_us);
    
    float getCurrentRate() const { return current_rate_hz; }
    bool isRateAcceptable(float tolerance = 0.1f) const;  // 10% 容差
    uint32_t getUpdateCount() const { return update_count; }
};

} // namespace DataValidation