#pragma once

#include "../data/data_types.h"
#include <Arduino.h>

/**
 * @brief 基於數據源的發佈器 - 跟隨實際數據流頻率
 * 
 * 概念：
 * - 不設固定頻率，跟隨數據源的自然更新頻率
 * - IMU 更新時 → 發佈 IMU 相關協議
 * - GNSS 更新時 → 發佈 GNSS 相關協議  
 * - ODO 更新時 → 發佈 ODO 相關協議
 * - Raw 更新時 → 發佈 Raw 相關協議
 */
class SourceBasedPublisher {
public:
    // 各種數據源的回調類型
    using IMUCallback = void(*)(const IMUData&, const NavigationState&);
    using GNSSCallback = void(*)(const GNSSData&, const NavigationState&);
    using ODOCallback = void(*)(const NavigationState&);  // ODO 可能只需要 Nav
    using RawCallback = void(*)(const IMUData&, const GNSSData&);  // Raw 需要原始數據
    
    struct CallbackSet {
        IMUCallback imu_callback = nullptr;
        GNSSCallback gnss_callback = nullptr; 
        ODOCallback odo_callback = nullptr;
        RawCallback raw_callback = nullptr;
        
        // 統計各源的調用次數
        uint32_t imu_calls = 0;
        uint32_t gnss_calls = 0;
        uint32_t odo_calls = 0;
        uint32_t raw_calls = 0;
    } callbacks_;
    
    /**
     * @brief 註冊各協議的回調函數
     */
    void registerIMUCallback(IMUCallback callback) { callbacks_.imu_callback = callback; }
    void registerGNSSCallback(GNSSCallback callback) { callbacks_.gnss_callback = callback; }
    void registerODOCallback(ODOCallback callback) { callbacks_.odo_callback = callback; }
    void registerRawCallback(RawCallback callback) { callbacks_.raw_callback = callback; }
    
    /**
     * @brief IMU 數據更新時調用 (跟隨 IMU 自然頻率)
     */
    void onIMUUpdate(const IMUData& imu_data, const NavigationState& nav_state) {
        if (callbacks_.imu_callback) {
            callbacks_.imu_callback(imu_data, nav_state);
            callbacks_.imu_calls++;
        }
        
        // Raw 協議可能也需要 IMU 數據
        if (callbacks_.raw_callback) {
            // 需要同時提供 IMU 和最新的 GNSS (暫存)
            callbacks_.raw_callback(imu_data, last_gnss_data_);
        }
    }
    
    /**
     * @brief GNSS 數據更新時調用 (跟隨 GNSS 自然頻率)
     */
    void onGNSSUpdate(const GNSSData& gnss_data, const NavigationState& nav_state) {
        // 暫存最新的 GNSS 數據
        last_gnss_data_ = gnss_data;
        
        if (callbacks_.gnss_callback) {
            callbacks_.gnss_callback(gnss_data, nav_state);
            callbacks_.gnss_calls++;
        }
    }
    
    /**
     * @brief ODO 相關更新時調用 (可能從 NavigationState 的特定更新觸發)
     */
    void onODOUpdate(const NavigationState& nav_state) {
        if (callbacks_.odo_callback) {
            callbacks_.odo_callback(nav_state);
            callbacks_.odo_calls++;
        }
    }
    
    /**
     * @brief 統計輸出
     */
    void printStats() {
        static uint32_t last_stats_time = 0;
        uint32_t now = millis();
        
        if (now - last_stats_time >= 5000) {  // 每5秒
            LOGI("📊 源頻率統計 (5秒):");
            LOGI("  IMU協議: %.1fHz (%lu次)", callbacks_.imu_calls / 5.0f, callbacks_.imu_calls);
            LOGI("  GNSS協議: %.1fHz (%lu次)", callbacks_.gnss_calls / 5.0f, callbacks_.gnss_calls);
            LOGI("  ODO協議: %.1fHz (%lu次)", callbacks_.odo_calls / 5.0f, callbacks_.odo_calls);
            LOGI("  Raw協議: %.1fHz (%lu次)", callbacks_.raw_calls / 5.0f, callbacks_.raw_calls);
            
            // 重置計數
            callbacks_.imu_calls = callbacks_.gnss_calls = callbacks_.odo_calls = callbacks_.raw_calls = 0;
            last_stats_time = now;
        }
    }
    
private:
    GNSSData last_gnss_data_;  // 暫存最新 GNSS 數據供 Raw 使用
};