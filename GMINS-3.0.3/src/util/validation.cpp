#include "validation.h"
#include <cmath>
#include <algorithm>

namespace DataValidation {

// =============================================================================
// 🧪 IMU 數據驗證實現
// =============================================================================

IMUValidationResult validateIMUData(const IMUData& data) {
    IMUValidationResult result = {};
    result.is_valid = true;
    result.error_flags = 0;
    
    // 檢查時間戳
    if (data.timestamp_us == 0) {
        result.error_flags |= IMUValidationResult::ERROR_TIMESTAMP_ZERO;
        result.is_valid = false;
    }
    
    // 檢查 NaN 或 Inf 值
    if (hasInvalidValues(data)) {
        result.error_flags |= IMUValidationResult::ERROR_NAN_VALUES;
        result.is_valid = false;
    }
    
    // 檢查加速度合理性
    if (!isAccelMagnitudeValid(data.accel_x, data.accel_y, data.accel_z)) {
        result.error_flags |= IMUValidationResult::ERROR_ACCEL_MAGNITUDE;
        result.is_valid = false;
    }
    
    // 檢查陀螺儀合理性
    if (!isGyroMagnitudeValid(data.gyro_x, data.gyro_y, data.gyro_z)) {
        result.error_flags |= IMUValidationResult::ERROR_GYRO_MAGNITUDE;
        result.is_valid = false;
    }
    
    // 檢查四元數有效性
    if (data.flags & IMU_QUATERNION_VALID) {
        if (!mu::is_valid_quaternion(data.quat_w, data.quat_x, data.quat_y, data.quat_z)) {
            result.error_flags |= IMUValidationResult::ERROR_QUATERNION_INVALID;
            result.is_valid = false;
        }
    }
    
    // 檢查溫度範圍
    if (data.temperature < -40.0f || data.temperature > 85.0f) {
        result.error_flags |= IMUValidationResult::ERROR_TEMPERATURE_RANGE;
        // 溫度異常不影響整體有效性，只記錄錯誤
    }
    
    // 檢查標誌一致性
    bool has_valid_data = (data.flags & IMU_ACCEL_VALID) || 
                         (data.flags & IMU_GYRO_VALID) ||
                         (data.flags & IMU_QUATERNION_VALID);
    if (!has_valid_data && result.is_valid) {
        result.error_flags |= IMUValidationResult::ERROR_FLAGS_INCONSISTENT;
        result.is_valid = false;
    }
    
    return result;
}

bool hasInvalidValues(const IMUData& data) {
    return !std::isfinite(data.accel_x) || !std::isfinite(data.accel_y) || !std::isfinite(data.accel_z) ||
           !std::isfinite(data.gyro_x) || !std::isfinite(data.gyro_y) || !std::isfinite(data.gyro_z) ||
           !std::isfinite(data.mag_x) || !std::isfinite(data.mag_y) || !std::isfinite(data.mag_z) ||
           !std::isfinite(data.quat_w) || !std::isfinite(data.quat_x) || 
           !std::isfinite(data.quat_y) || !std::isfinite(data.quat_z) ||
           !std::isfinite(data.temperature);
}

bool isAccelMagnitudeValid(float accel_x, float accel_y, float accel_z) {
    float magnitude = std::sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    
    // 重力加速度應該在 8-12 m/s² 範圍內（考慮運動加速度）
    return (magnitude >= 8.0f && magnitude <= 12.0f);
}

bool isGyroMagnitudeValid(float gyro_x, float gyro_y, float gyro_z) {
    float magnitude = std::sqrt(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
    
    // 角速度不應超過 10 rad/s (約 573 度/秒)
    return (magnitude <= 10.0f);
}

// =============================================================================
// 📡 GNSS 數據驗證實現
// =============================================================================

GNSSValidationResult validateGNSSData(const GNSSData& data) {
    GNSSValidationResult result = {};
    result.is_valid = true;
    result.error_flags = 0;
    
    // 檢查時間戳
    if (data.timestamp_us == 0) {
        result.error_flags |= GNSSValidationResult::ERROR_TIMESTAMP_ZERO;
        result.is_valid = false;
    }
    
    // 檢查 NaN 或 Inf 值
    if (!std::isfinite(data.latitude) || !std::isfinite(data.longitude) || 
        !std::isfinite(data.altitude_msl) || !std::isfinite(data.velocity_north) ||
        !std::isfinite(data.velocity_east) || !std::isfinite(data.velocity_down)) {
        result.error_flags |= GNSSValidationResult::ERROR_NAN_VALUES;
        result.is_valid = false;
    }
    
    // 檢查位置合理性
    if (!isWGS84PositionValid(data.latitude, data.longitude, data.altitude_msl)) {
        result.error_flags |= (GNSSValidationResult::ERROR_LATITUDE_RANGE | 
                              GNSSValidationResult::ERROR_LONGITUDE_RANGE |
                              GNSSValidationResult::ERROR_ALTITUDE_RANGE);
        result.is_valid = false;
    }
    
    // 檢查速度合理性
    if (!isGNSSVelocityValid(data.velocity_north, data.velocity_east, data.velocity_down)) {
        result.error_flags |= GNSSValidationResult::ERROR_VELOCITY_EXCESSIVE;
        result.is_valid = false;
    }
    
    // 檢查定位精度
    if (!isGNSSAccuracyAcceptable(data.horizontal_accuracy, data.vertical_accuracy)) {
        result.error_flags |= GNSSValidationResult::ERROR_ACCURACY_POOR;
        // 精度不佳不直接影響數據有效性，只記錄警告
    }
    
    // 檢查衛星數量
    if (data.satellites_used > data.satellites_visible || data.satellites_visible > 50) {
        result.error_flags |= GNSSValidationResult::ERROR_SATELLITE_COUNT;
        result.is_valid = false;
    }
    
    // 檢查定位類型
    if (data.fix_type > GNSS_FIX_RTK_FIXED) {
        result.error_flags |= GNSSValidationResult::ERROR_FIX_TYPE_INVALID;
        result.is_valid = false;
    }
    
    return result;
}

bool isWGS84PositionValid(double latitude, double longitude, float altitude_msl) {
    // 檢查緯度範圍 [-90, 90]
    if (latitude < -90.0 || latitude > 90.0) return false;
    
    // 檢查經度範圍 [-180, 180]
    if (longitude < -180.0 || longitude > 180.0) return false;
    
    // 檢查海拔合理性 [-1000m, 50000m]
    if (altitude_msl < -1000.0f || altitude_msl > 50000.0f) return false;
    
    return true;
}

bool isGNSSVelocityValid(float vel_north, float vel_east, float vel_down) {
    // 計算總速度
    float total_velocity = std::sqrt(vel_north*vel_north + vel_east*vel_east + vel_down*vel_down);
    
    // 檢查速度是否過大（考慮飛行器應用，限制在 200 m/s）
    return (total_velocity <= 200.0f);
}

bool isGNSSAccuracyAcceptable(float horizontal_accuracy, float vertical_accuracy,
                             float min_horizontal_threshold, float min_vertical_threshold) {
    // 檢查精度數值有效性
    if (horizontal_accuracy < 0.0f || vertical_accuracy < 0.0f) return false;
    
    // 檢查精度是否在可接受範圍內
    return (horizontal_accuracy <= min_horizontal_threshold && 
            vertical_accuracy <= min_vertical_threshold);
}

// =============================================================================
// 🧭 導航狀態驗證實現
// =============================================================================

NavigationValidationResult validateNavigationState(const NavigationState& nav_state) {
    NavigationValidationResult result = {};
    result.is_valid = true;
    result.error_flags = 0;
    
    // 檢查時間戳
    if (nav_state.timestamp_us == 0) {
        result.error_flags |= NavigationValidationResult::ERROR_TIMESTAMP_ZERO;
        result.is_valid = false;
    }
    
    // 檢查 NaN 或 Inf 值
    if (!std::isfinite(nav_state.position_north) || !std::isfinite(nav_state.position_east) ||
        !std::isfinite(nav_state.position_down) || !std::isfinite(nav_state.velocity_north) ||
        !std::isfinite(nav_state.velocity_east) || !std::isfinite(nav_state.velocity_down) ||
        !std::isfinite(nav_state.euler_roll) || !std::isfinite(nav_state.euler_pitch) ||
        !std::isfinite(nav_state.euler_yaw)) {
        result.error_flags |= NavigationValidationResult::ERROR_NAN_VALUES;
        result.is_valid = false;
    }
    
    // 檢查歐拉角有效性 (範圍檢查)
    if (nav_state.euler_roll < -180.0f || nav_state.euler_roll > 180.0f ||
        nav_state.euler_pitch < -90.0f || nav_state.euler_pitch > 90.0f ||
        nav_state.euler_yaw < -180.0f || nav_state.euler_yaw > 360.0f) {  // YAW 允許 0-360° 範圍
        result.error_flags |= NavigationValidationResult::ERROR_QUATERNION_INVALID; // 重用錯誤代碼
        result.is_valid = false;
    }
    
    // 檢查位置合理性（相對於基準點）
    float position_magnitude = std::sqrt(nav_state.position_north * nav_state.position_north +
                                        nav_state.position_east * nav_state.position_east);
    if (position_magnitude > 100000.0f) {  // 100 km 半徑
        result.error_flags |= NavigationValidationResult::ERROR_POSITION_EXCESSIVE;
        result.is_valid = false;
    }
    
    // 檢查速度合理性
    float velocity_magnitude = std::sqrt(nav_state.velocity_north * nav_state.velocity_north +
                                        nav_state.velocity_east * nav_state.velocity_east +
                                        nav_state.velocity_down * nav_state.velocity_down);
    if (velocity_magnitude > 200.0f) {  // 200 m/s 最大速度
        result.error_flags |= NavigationValidationResult::ERROR_VELOCITY_EXCESSIVE;
        result.is_valid = false;
    }
    
    // 檢查不確定性
    if (!isUncertaintyReasonable(nav_state)) {
        result.error_flags |= NavigationValidationResult::ERROR_UNCERTAINTY_HIGH;
        // 不確定性高不影響數據有效性，只記錄警告
    }
    
    // 檢查標誌一致性
    bool has_valid_nav = (nav_state.flags & NAV_POSITION_VALID) ||
                        (nav_state.flags & NAV_VELOCITY_VALID) ||
                        (nav_state.flags & NAV_ATTITUDE_VALID);
    if (!has_valid_nav && result.is_valid) {
        result.error_flags |= NavigationValidationResult::ERROR_FLAGS_INCONSISTENT;
        result.is_valid = false;
    }
    
    return result;
}

bool isUncertaintyReasonable(const NavigationState& nav_state) {
    // 檢查位置不確定性（應小於 50 米）
    float max_position_std = std::max({nav_state.position_std_north, 
                                      nav_state.position_std_east, 
                                      nav_state.position_std_down});
    if (max_position_std > 50.0f) return false;
    
    // 檢查速度不確定性（應小於 5 m/s）
    float max_velocity_std = std::max({nav_state.velocity_std_north, 
                                      nav_state.velocity_std_east, 
                                      nav_state.velocity_std_down});
    if (max_velocity_std > 5.0f) return false;
    
    // 檢查姿態不確定性（應小於 10 度 ≈ 0.175 弧度）
    float max_attitude_std = std::max({nav_state.attitude_std_roll, 
                                      nav_state.attitude_std_pitch, 
                                      nav_state.attitude_std_yaw});
    if (max_attitude_std > 0.175f) return false;
    
    return true;
}

// =============================================================================
// 🔧 四元數驗證工具實現
// =============================================================================

QuaternionValidationResult validateAndNormalizeQuaternion(
    float& qw, float& qx, float& qy, float& qz) {
    
    QuaternionValidationResult result = {};
    
    // 計算原始範數
    result.original_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    result.was_valid = mu::is_valid_quaternion(qw, qx, qy, qz);
    
    // 嘗試正規化
    result.normalized_success = mu::quaternion_normalize(qw, qx, qy, qz);
    
    // 計算最終範數
    result.final_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    
    return result;
}

// =============================================================================
// 📊 統計驗證工具實現
// =============================================================================

TimeSeriesValidationResult validateTimestamp(
    uint64_t current_timestamp_us,
    uint64_t previous_timestamp_us,
    uint64_t max_gap_us) {
    
    TimeSeriesValidationResult result = {};
    
    if (previous_timestamp_us == 0) {
        // 第一次接收數據
        result.is_consistent = true;
        result.time_gap_us = 0;
        result.time_gap_excessive = false;
        result.timestamp_reversed = false;
        return result;
    }
    
    // 檢查時間戳是否倒退
    if (current_timestamp_us < previous_timestamp_us) {
        result.timestamp_reversed = true;
        result.is_consistent = false;
    }
    
    // 計算時間間隔
    result.time_gap_us = current_timestamp_us - previous_timestamp_us;
    
    // 檢查時間間隔是否過大
    result.time_gap_excessive = (result.time_gap_us > max_gap_us);
    
    result.is_consistent = !result.timestamp_reversed && !result.time_gap_excessive;
    
    return result;
}

// =============================================================================
// 🔍 組合驗證函數實現
// =============================================================================

SystemHealthResult validateSystemHealth(
    const IMUData& imu_data,
    const GNSSData& gnss_data,
    const NavigationState& nav_state) {
    
    SystemHealthResult result = {};
    result.warning_flags = 0;
    
    // 驗證各子系統
    result.imu_result = validateIMUData(imu_data);
    result.gnss_result = validateGNSSData(gnss_data);
    result.nav_result = validateNavigationState(nav_state);
    
    // 判斷整體健康狀況
    result.overall_health = result.imu_result.is_valid && 
                           result.gnss_result.is_valid && 
                           result.nav_result.is_valid;
    
    // 設定警告標誌
    if (!result.imu_result.is_valid || imu_data.data_quality < 128) {
        result.warning_flags |= SystemHealthResult::WARNING_IMU_DEGRADED;
    }
    
    if (gnss_data.horizontal_accuracy > 5.0f || gnss_data.satellites_used < 6) {
        result.warning_flags |= SystemHealthResult::WARNING_GNSS_POOR;
    }
    
    if (!isUncertaintyReasonable(nav_state) || nav_state.overall_quality < 128) {
        result.warning_flags |= SystemHealthResult::WARNING_NAV_UNCERTAIN;
    }
    
    // 檢查時間同步性
    if (!validateSensorTimestamp(imu_data.timestamp_us, gnss_data.timestamp_us)) {
        result.warning_flags |= SystemHealthResult::WARNING_TIME_SYNC_ISSUE;
    }
    
    return result;
}

bool validateSensorTimestamp(
    uint64_t imu_timestamp_us,
    uint64_t gnss_timestamp_us,
    uint64_t max_desync_us) {
    
    if (imu_timestamp_us == 0 || gnss_timestamp_us == 0) {
        return false;  // 無效時間戳
    }
    
    uint64_t time_diff = (imu_timestamp_us > gnss_timestamp_us) ? 
                        (imu_timestamp_us - gnss_timestamp_us) : 
                        (gnss_timestamp_us - imu_timestamp_us);
    
    return (time_diff <= max_desync_us);
}

// =============================================================================
// 📈 性能監控工具實現
// =============================================================================

UpdateRateMonitor::UpdateRateMonitor(float expected_rate_hz) 
    : target_rate_hz(expected_rate_hz), 
      current_rate_hz(0.0f),
      update_count(0),
      last_timestamp_us(0),
      monitor_start_time_us(0) {
}

void UpdateRateMonitor::reset() {
    last_timestamp_us = 0;
    current_rate_hz = 0.0f;
    update_count = 0;
    monitor_start_time_us = 0;
}

void UpdateRateMonitor::update(uint64_t timestamp_us) {
    if (monitor_start_time_us == 0) {
        monitor_start_time_us = timestamp_us;
        last_timestamp_us = timestamp_us;
        update_count = 1;
        return;
    }
    
    update_count++;
    
    // 計算當前平均更新率
    uint64_t total_time_us = timestamp_us - monitor_start_time_us;
    if (total_time_us > 0) {
        current_rate_hz = (float)(update_count - 1) * 1000000.0f / (float)total_time_us;
    }
    
    last_timestamp_us = timestamp_us;
}

bool UpdateRateMonitor::isRateAcceptable(float tolerance) const {
    if (current_rate_hz == 0.0f) return false;
    
    float rate_error = std::abs(current_rate_hz - target_rate_hz) / target_rate_hz;
    return (rate_error <= tolerance);
}

} // namespace DataValidation