#pragma once

/**
 * GNSS Data Adapter
 * 
 * 提供 GNSSData 結構與 mu 之間的適配介面
 * 職責：
 * - 從 GNSSData 提取數學計算所需的數據
 * - 處理 GNSS 航向數據和精度評估
 * - Shift 校正計算支援
 * - 座標和速度數據格式轉換
 * 
 * 設計原則：
 * - Header-only, inline 函數
 * - 無狀態設計
 * - 回傳 bool 表示操作成功/失敗
 * - 處理 GNSS 數據特有的品質指標
 */

#include "../data/data_types.h"
#include "../util/math_utils.h"
#include <cstdio>

namespace GNSSAdapter {
    
    // ============================================================================
    // 航向數據提取與處理
    // ============================================================================
    
    /**
     * 提取 GNSS 航向角（弧度）
     * @param gnss_data GNSS 數據結構
     * @param heading 輸出航向角（弧度，從北向順時針）
     * @param accuracy 輸出航向精度（弧度，可選）
     * @return true 如果航向數據有效且精度足夠
     */
    inline bool getHeading(const GNSSData& gnss_data, float& heading, 
                          float* accuracy = nullptr) {
        // 檢查航向數據有效性
        if (!(gnss_data.flags & GNSS_HEADING_VALID)) {
            return false;
        }
        
        // 檢查數值有效性
        if (!isfinite(gnss_data.gnss_heading)) {
            return false;
        }
        
        // 檢查航向精度（如果可用）
        if (isfinite(gnss_data.heading_accuracy)) {
            // 精度閾值：5 度 = ~0.087 弧度
            if (gnss_data.heading_accuracy > 0.087f) {
                return false; // 精度不足
            }
            if (accuracy) {
                *accuracy = gnss_data.heading_accuracy;
            }
        }
        
        // 正規化航向角到 [-π, π] 範圍
        heading = mu::wrapAngle2Pi(gnss_data.gnss_heading);
        return true;
    }
    
    /**
     * 提取 GNSS 航向角（度數）
     * @param gnss_data GNSS 數據結構
     * @param heading_deg 輸出航向角（度數，從北向順時針）
     * @param accuracy_deg 輸出航向精度（度數，可選）
     * @return true 如果航向數據有效
     */
    inline bool getHeadingDegrees(const GNSSData& gnss_data, float& heading_deg,
                                 float* accuracy_deg = nullptr) {
        float heading_rad, accuracy_rad;
        float* acc_ptr = accuracy_deg ? &accuracy_rad : nullptr;
        
        if (getHeading(gnss_data, heading_rad, acc_ptr)) {
            heading_deg = heading_rad * mu::kRadToDeg;
            if (accuracy_deg) {
                *accuracy_deg = accuracy_rad * mu::kRadToDeg;
            }
            return true;
        }
        return false;
    }
    
    /**
     * 計算 GNSS 航向與 IMU 偏航角的 Shift 偏移量
     * @param gnss_data GNSS 數據結構
     * @param imu_yaw_rad IMU 偏航角（弧度）
     * @param shift_offset 輸出偏移量（弧度）
     * @return true 如果計算成功
     */
    inline bool calculateShiftOffset(const GNSSData& gnss_data, 
                                    float imu_yaw_rad, float& shift_offset) {
        float gnss_heading;
        if (!getHeading(gnss_data, gnss_heading)) {
            return false;
        }
        
        // 使用 mu 計算角度差異
        shift_offset = mu::calculate_shift_offset(gnss_heading, imu_yaw_rad);
        return true;
    }
    
    // ============================================================================
    // 位置和速度數據提取
    // ============================================================================
    
    /**
     * 提取位置數據（WGS84）
     * @param gnss_data GNSS 數據結構
     * @param lat_deg, lon_deg 輸出緯度經度（度數）
     * @param alt_m 輸出海拔高度（米）
     * @return true 如果位置數據有效
     */
    inline bool getPosition(const GNSSData& gnss_data, 
                           double& lat_deg, double& lon_deg, float& alt_m) {
        if (!(gnss_data.flags & GNSS_POSITION_VALID)) {
            return false;
        }
        
        // 檢查緯度經度合理性
        if (gnss_data.latitude < -90.0 || gnss_data.latitude > 90.0 ||
            gnss_data.longitude < -180.0 || gnss_data.longitude > 180.0) {
            return false;
        }
        
        lat_deg = gnss_data.latitude;
        lon_deg = gnss_data.longitude;
        alt_m = gnss_data.altitude_msl;
        return true;
    }
    
    /**
     * 提取速度向量（NED 座標系, m/s）
     * @param gnss_data GNSS 數據結構
     * @return 速度向量，如果數據無效回傳零向量
     */
    inline mu::Vector3f getVelocityVector(const GNSSData& gnss_data) {
        if (!(gnss_data.flags & GNSS_VELOCITY_VALID)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        // 檢查數值有效性
        if (!isfinite(gnss_data.velocity_north) || 
            !isfinite(gnss_data.velocity_east) || 
            !isfinite(gnss_data.velocity_down)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        return mu::Vector3f(gnss_data.velocity_north, 
                                  gnss_data.velocity_east, 
                                  gnss_data.velocity_down);
    }
    
    /**
     * 提取地面速度和航跡角
     * @param gnss_data GNSS 數據結構
     * @param ground_speed 輸出地面速度（m/s）
     * @param course_rad 輸出航跡角（弧度）
     * @return true 如果數據有效
     */
    inline bool getGroundTrack(const GNSSData& gnss_data, 
                              float& ground_speed, float& course_rad) {
        if (!(gnss_data.flags & GNSS_VELOCITY_VALID)) {
            return false;
        }
        
        if (!isfinite(gnss_data.ground_speed) || !isfinite(gnss_data.course_over_ground)) {
            return false;
        }
        
        ground_speed = gnss_data.ground_speed;
        course_rad = gnss_data.course_over_ground;
        return true;
    }
    
    // ============================================================================
    // 精度和品質評估
    // ============================================================================
    
    /**
     * 獲取位置精度指標
     * @param gnss_data GNSS 數據結構
     * @param h_acc 輸出水平精度（米）
     * @param v_acc 輸出垂直精度（米）
     * @return true 如果精度數據可用
     */
    inline bool getPositionAccuracy(const GNSSData& gnss_data, 
                                   float& h_acc, float& v_acc) {
        if (!(gnss_data.flags & GNSS_ACCURACY_VALID)) {
            return false;
        }
        
        if (!isfinite(gnss_data.horizontal_accuracy) || 
            !isfinite(gnss_data.vertical_accuracy)) {
            return false;
        }
        
        h_acc = gnss_data.horizontal_accuracy;
        v_acc = gnss_data.vertical_accuracy;
        return true;
    }
    
    /**
     * 計算 GNSS 數據品質分數 (0-100)
     * @param gnss_data GNSS 數據結構
     * @return 品質分數，100 為最佳
     */
    inline uint8_t calculateDataQuality(const GNSSData& gnss_data) {
        uint8_t score = 0;
        
        // 基本定位品質（40分）
        switch (gnss_data.fix_type) {
            case GNSS_FIX_RTK_FIXED:  score += 40; break;
            case GNSS_FIX_RTK_FLOAT:  score += 35; break;
            case GNSS_FIX_DGPS:       score += 25; break;
            case GNSS_FIX_3D:         score += 15; break;
            case GNSS_FIX_2D:         score += 5;  break;
            default:                  score += 0;  break;
        }
        
        // 衛星數量（20分）
        if (gnss_data.satellites_used >= 8) score += 20;
        else if (gnss_data.satellites_used >= 6) score += 15;
        else if (gnss_data.satellites_used >= 4) score += 10;
        
        // 精度稀釋因子 HDOP（20分）
        if (gnss_data.hdop < 1.0f) score += 20;
        else if (gnss_data.hdop < 2.0f) score += 15;
        else if (gnss_data.hdop < 5.0f) score += 10;
        else if (gnss_data.hdop < 10.0f) score += 5;
        
        // 數據完整性（20分）
        if (gnss_data.flags & GNSS_POSITION_VALID) score += 5;
        if (gnss_data.flags & GNSS_VELOCITY_VALID) score += 5;
        if (gnss_data.flags & GNSS_HEADING_VALID) score += 5;
        if (gnss_data.flags & GNSS_ACCURACY_VALID) score += 5;
        
        return mu::min(score, static_cast<uint8_t>(100));
    }
    
    /**
     * 檢查 GNSS 數據是否適合用於 Shift 校正
     * @param gnss_data GNSS 數據結構
     * @return true 如果適合用於校正
     */
    inline bool isValidForShiftCalibration(const GNSSData& gnss_data) {
        // 需要有效的航向數據
        if (!(gnss_data.flags & GNSS_HEADING_VALID)) {
            return false;
        }
        
        // 需要足夠的定位精度
        if (gnss_data.fix_type < GNSS_FIX_3D) {
            return false;
        }
        
        // 需要足夠的衛星數量
        if (gnss_data.satellites_used < 4) {
            return false;
        }
        
        // 需要合理的航向精度
        if (isfinite(gnss_data.heading_accuracy) && gnss_data.heading_accuracy > 0.175f) { // 10度
            return false;
        }
        
        // 需要合理的地面速度（避免靜止時的航向不穩定）
        if (gnss_data.ground_speed < 1.0f) { // 1 m/s 最小速度
            return false;
        }
        
        return true;
    }
    
    // ============================================================================
    // 時間和同步
    // ============================================================================
    
    /**
     * 檢查 GNSS 數據是否過時
     * @param gnss_data GNSS 數據結構
     * @param current_time_us 當前時間戳（微秒）
     * @param max_age_us 最大允許年齡（微秒）
     * @return true 如果數據過時
     */
    inline bool isDataStale(const GNSSData& gnss_data,
                           timestamp_us_t current_time_us,
                           timestamp_us_t max_age_us = 1000000) { // 預設1秒
        return (current_time_us - gnss_data.timestamp_us) > max_age_us;
    }
    
    /**
     * 獲取 GPS 時間信息
     * @param gnss_data GNSS 數據結構
     * @param week GPS 週數
     * @param tow_ms 週內時間（毫秒）
     * @return true 如果 GPS 時間可用
     */
    inline bool getGPSTime(const GNSSData& gnss_data, uint16_t& week, uint32_t& tow_ms) {
        if (gnss_data.gps_week == 0) {
            return false;
        }
        
        week = gnss_data.gps_week;
        tow_ms = gnss_data.tow_ms;
        return true;
    }
    
    // ============================================================================
    // 座標轉換支援
    // ============================================================================
    
    /**
     * 計算兩個 GNSS 位置間的距離（Haversine 公式）
     * @param gnss1, gnss2 兩個 GNSS 數據點
     * @param distance 輸出距離（米）
     * @return true 如果計算成功
     */
    inline bool calculateDistance(const GNSSData& gnss1, const GNSSData& gnss2, 
                                 float& distance) {
        double lat1, lon1, lat2, lon2;
        float alt1 = 0.0f, alt2 = 0.0f;  // 本地變數接住高度參數
        
        if (!getPosition(gnss1, lat1, lon1, alt1) || 
            !getPosition(gnss2, lat2, lon2, alt2)) {
            return false;
        }
        
        // Haversine 公式
        constexpr double EARTH_RADIUS = 6378137.0; // WGS84 地球半徑（米）
        
        double dlat = (lat2 - lat1) * mu::kDegToRad;
        double dlon = (lon2 - lon1) * mu::kDegToRad;
        double lat1_rad = lat1 * mu::kDegToRad;
        double lat2_rad = lat2 * mu::kDegToRad;
        
        double a = sin(dlat/2) * sin(dlat/2) + 
                   cos(lat1_rad) * cos(lat2_rad) * sin(dlon/2) * sin(dlon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        distance = static_cast<float>(EARTH_RADIUS * c);
        return true;
    }
    
    // ============================================================================
    // 調試和診斷工具
    // ============================================================================
    
    /**
     * 生成 GNSS 數據的調試字符串
     * @param gnss_data GNSS 數據結構
     * @param buffer 輸出緩衝區
     * @param buffer_size 緩衝區大小
     */
    inline void debugString(const GNSSData& gnss_data, char* buffer, size_t buffer_size) {
        float heading_deg = 0.0f;
        bool heading_ok = getHeadingDegrees(gnss_data, heading_deg);
        
        snprintf(buffer, buffer_size,
                "GNSS[%llu] Pos:(%.6f,%.6f,%.1f) Fix:%d Sats:%d HDOP:%.1f Hdg:%s GS:%.1f Q:%d",
                gnss_data.timestamp_us,
                gnss_data.latitude, gnss_data.longitude, gnss_data.altitude_msl,
                static_cast<int>(gnss_data.fix_type),
                gnss_data.satellites_used,
                gnss_data.hdop,
                heading_ok ? "OK" : "FAIL",
                gnss_data.ground_speed,
                calculateDataQuality(gnss_data));
    }
    
} // namespace GNSSAdapter