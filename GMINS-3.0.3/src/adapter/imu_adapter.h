#pragma once

/**
 * IMU Data Adapter
 * 
 * 提供 IMUData 結構與 mu 之間的適配介面
 * 職責：
 * - 從 IMUData 提取數學計算所需的數據
 * - 確保數據有效性檢查
 * - 格式轉換和單位統一
 * 
 * 設計原則：
 * - Header-only, inline 函數
 * - 無狀態設計
 * - 回傳 bool 表示操作成功/失敗
 * - 只依賴 data_types.h 和 math_utils.h
 */

#include "../data/data_types.h"
#include "../util/math_utils.h"
#include <cstdio>

namespace IMUAdapter {
    
    // ============================================================================
    // 姿態數據提取
    // ============================================================================
    
    /**
     * 從 IMU 四元數提取歐拉角 (NED 座標系)
     * @param imu_data IMU 數據結構
     * @param roll, pitch, yaw 輸出歐拉角（弧度）
     * @return true 如果提取成功
     */
    inline bool extractEulerAngles(const IMUData& imu_data, 
                                   float& roll, float& pitch, float& yaw) {
        // 檢查四元數數據是否有效
        if (!(imu_data.flags & IMU_QUATERNION_VALID)) {
            return false;
        }
        
        // 檢查四元數數值有效性
        if (!mu::is_valid_quaternion(imu_data.quat_w, imu_data.quat_x, 
                                           imu_data.quat_y, imu_data.quat_z)) {
            return false;
        }
        
        // 轉換為歐拉角
        mu::quaternion_to_euler_ned(
            imu_data.quat_w, imu_data.quat_x, imu_data.quat_y, imu_data.quat_z,
            roll, pitch, yaw
        );
        
        return true;
    }
    
    /**
     * 提取四元數（已驗證）
     * @param imu_data IMU 數據結構
     * @param qw, qx, qy, qz 輸出四元數分量
     * @return true 如果四元數有效
     */
    inline bool extractQuaternion(const IMUData& imu_data,
                                  float& qw, float& qx, float& qy, float& qz) {
        if (!(imu_data.flags & IMU_QUATERNION_VALID)) {
            return false;
        }
        
        if (!mu::is_valid_quaternion(imu_data.quat_w, imu_data.quat_x,
                                           imu_data.quat_y, imu_data.quat_z)) {
            return false;
        }
        
        qw = imu_data.quat_w;
        qx = imu_data.quat_x;
        qy = imu_data.quat_y;
        qz = imu_data.quat_z;
        return true;
    }
    
    /**
     * 從歐拉角轉換為四元數並提取
     * @param imu_data IMU 數據結構
     * @param qw, qx, qy, qz 輸出四元數分量
     * @return true 如果歐拉角有效並成功轉換
     */
    inline bool extractQuaternionFromEuler(const IMUData& imu_data,
                                          float& qw, float& qx, float& qy, float& qz) {
        if (!(imu_data.flags & IMU_EULER_VALID)) {
            return false;
        }
        
        // 檢查歐拉角數值有效性
        if (!isfinite(imu_data.euler_roll) || !isfinite(imu_data.euler_pitch) || !isfinite(imu_data.euler_yaw)) {
            return false;
        }
        
        // 歐拉角轉四元數 (ZYX 旋轉順序: Roll-Pitch-Yaw)
        float roll = imu_data.euler_roll;
        float pitch = imu_data.euler_pitch;
        float yaw = imu_data.euler_yaw;
        
        float cr = cos(roll * 0.5f);
        float sr = sin(roll * 0.5f);
        float cp = cos(pitch * 0.5f);
        float sp = sin(pitch * 0.5f);
        float cy = cos(yaw * 0.5f);
        float sy = sin(yaw * 0.5f);
        
        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
        
        // 檢查轉換後的四元數有效性
        if (!mu::is_valid_quaternion(qw, qx, qy, qz)) {
            return false;
        }
        
        return true;
    }
    
    /**
     * 智能提取四元數 - 優先使用四元數，退回到歐拉角
     * @param imu_data IMU 數據結構
     * @param qw, qx, qy, qz 輸出四元數分量
     * @return true 如果成功提取四元數
     */
    inline bool extractQuaternionSmart(const IMUData& imu_data,
                                      float& qw, float& qx, float& qy, float& qz) {
        // 優先嘗試直接的四元數
        if (extractQuaternion(imu_data, qw, qx, qy, qz)) {
            return true;
        }
        
        // 退回到歐拉角轉換
        return extractQuaternionFromEuler(imu_data, qw, qx, qy, qz);
    }
    
    // ============================================================================
    // 向量數據提取
    // ============================================================================
    
    /**
     * 提取加速度向量（Body frame, m/s²）
     * @param imu_data IMU 數據結構
     * @return 加速度向量，如果數據無效回傳零向量
     */
    inline mu::Vector3f getAccelerationVector(const IMUData& imu_data) {
        if (!(imu_data.flags & IMU_ACCEL_VALID)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        // 檢查數值有效性
        if (!isfinite(imu_data.accel_x) || !isfinite(imu_data.accel_y) || !isfinite(imu_data.accel_z)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        return mu::Vector3f(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
    }
    
    /**
     * 提取角速度向量（Body frame, rad/s）
     * @param imu_data IMU 數據結構
     * @return 角速度向量，如果數據無效回傳零向量
     */
    inline mu::Vector3f getGyroVector(const IMUData& imu_data) {
        if (!(imu_data.flags & IMU_GYRO_VALID)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        // 檢查數值有效性
        if (!isfinite(imu_data.gyro_x) || !isfinite(imu_data.gyro_y) || !isfinite(imu_data.gyro_z)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        return mu::Vector3f(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
    }
    
    /**
     * 提取磁力計向量（Body frame, μT）
     * @param imu_data IMU 數據結構
     * @return 磁力計向量，如果數據無效回傳零向量
     */
    inline mu::Vector3f getMagnetometerVector(const IMUData& imu_data) {
        if (!(imu_data.flags & IMU_MAG_VALID)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        // 檢查數值有效性
        if (!isfinite(imu_data.mag_x) || !isfinite(imu_data.mag_y) || !isfinite(imu_data.mag_z)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        return mu::Vector3f(imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
    }
    
    // ============================================================================
    // 數據驗證與品質檢查
    // ============================================================================
    
    /**
     * 檢查 IMU 數據是否適合用於融合計算
     * @param imu_data IMU 數據結構
     * @return true 如果數據品質良好
     */
    inline bool isDataValid(const IMUData& imu_data) {
        // 檢查基本標誌
        if (!(imu_data.flags & (IMU_ACCEL_VALID | IMU_GYRO_VALID))) {
            return false;
        }
        
        // 檢查加速度合理性（避免極端值，但允許零值）
        mu::Vector3f accel = getAccelerationVector(imu_data);
        float accel_mag = accel.magnitude();
        if (accel_mag > 50.0f) { // 只檢查上限，允許零加速度（靜止狀態）
            return false;
        }
        
        // 檢查角速度合理性
        mu::Vector3f gyro = getGyroVector(imu_data);
        if (gyro.magnitude() > 20.0f) { // 超出合理角速度範圍
            return false;
        }
        
        // 檢查時間戳合理性
        return imu_data.timestamp_us > 0;
    }
    
    /**
     * 計算數據品質分數 (0-100)
     * @param imu_data IMU 數據結構
     * @return 品質分數，100 為最佳
     */
    inline uint8_t calculateDataQuality(const IMUData& imu_data) {
        uint8_t score = 0;
        
        // 基本有效性（50分）
        if (imu_data.flags & IMU_ACCEL_VALID) score += 15;
        if (imu_data.flags & IMU_GYRO_VALID) score += 15;
        if (imu_data.flags & IMU_QUATERNION_VALID) score += 20;
        
        // 校准狀態（25分）
        if (imu_data.flags & IMU_CALIBRATED) score += 25;
        
        // 數據新鮮度（15分）
        if (imu_data.flags & IMU_DATA_FRESH) score += 15;
        
        // 溫度合理性（10分）
        if (isfinite(imu_data.temperature) && 
            imu_data.temperature > -40.0f && imu_data.temperature < 85.0f) {
            score += 10;
        }
        
        return mu::min(score, static_cast<uint8_t>(100));
    }
    
    // ============================================================================
    // 時間戳和同步工具
    // ============================================================================
    
    /**
     * 檢查數據時間戳是否過時
     * @param imu_data IMU 數據結構
     * @param current_time_us 當前時間戳（微秒）
     * @param max_age_us 最大允許年齡（微秒）
     * @return true 如果數據過時
     */
    inline bool isDataStale(const IMUData& imu_data, 
                           timestamp_us_t current_time_us,
                           timestamp_us_t max_age_us = 100000) { // 預設100ms
        return (current_time_us - imu_data.timestamp_us) > max_age_us;
    }
    
    /**
     * 計算兩個 IMU 數據間的時間差（秒）
     * @param imu1, imu2 IMU 數據結構
     * @return 時間差（秒）
     */
    inline float calculateTimeDelta(const IMUData& imu1, const IMUData& imu2) {
        return static_cast<float>(imu2.timestamp_us - imu1.timestamp_us) * 1e-6f;
    }
    
    // ============================================================================
    // 單位轉換工具
    // ============================================================================
    
    /**
     * 提取偏航角（弧度）
     * @param imu_data IMU 數據結構
     * @param yaw 輸出偏航角（弧度）
     * @return true 如果提取成功
     */
    inline bool getYawAngle(const IMUData& imu_data, float& yaw) {
        // IMU 數據中的 euler_yaw 已經是度數，直接使用
        if (!isfinite(imu_data.euler_yaw)) {
            return false;
        }
        
        // 直接返回度數值，不需要轉換
        yaw = imu_data.euler_yaw;
        return true;
    }
    
    /**
     * 提取偏航角（度數）
     * @param imu_data IMU 數據結構
     * @param yaw_deg 輸出偏航角（度數）
     * @return true 如果提取成功
     */
    inline bool getYawAngleDegrees(const IMUData& imu_data, float& yaw_deg) {
        float yaw_rad;
        if (getYawAngle(imu_data, yaw_rad)) {
            yaw_deg = yaw_rad * mu::kRadToDeg;
            return true;
        }
        return false;
    }
    
    // ============================================================================
    // 調試和診斷工具
    // ============================================================================
    
    /**
     * 生成 IMU 數據的調試字符串
     * @param imu_data IMU 數據結構
     * @param buffer 輸出緩衝區
     * @param buffer_size 緩衝區大小
     */
    inline void debugString(const IMUData& imu_data, char* buffer, size_t buffer_size) {
        float roll, pitch, yaw;
        bool euler_ok = extractEulerAngles(imu_data, roll, pitch, yaw);
        
        snprintf(buffer, buffer_size, 
                "IMU[%llu] A:(%.2f,%.2f,%.2f) G:(%.2f,%.2f,%.2f) RPY:%s T:%.1f Q:%d",
                imu_data.timestamp_us,
                imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                euler_ok ? "OK" : "FAIL",
                imu_data.temperature,
                calculateDataQuality(imu_data));
    }
    
} // namespace IMUAdapter