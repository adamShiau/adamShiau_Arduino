#ifndef SYSTEM_CONTROLLER_POLLING_H
#define SYSTEM_CONTROLLER_POLLING_H

#include <Arduino.h>

// 固定頻率輪詢控制器
class SystemControllerPolling {
private:
    static const uint32_t DEFAULT_FREQ_HZ = 100;    // 預設 100Hz
    static const uint32_t POLL_INTERVAL_US = 1000000 / DEFAULT_FREQ_HZ; // 10000us = 10ms
    
    uint32_t last_poll_time_us_ = 0;
    uint32_t poll_frequency_hz_ = DEFAULT_FREQ_HZ;
    
    // 最後一次有效數據快照（無動態分配）
    struct {
        GNSSData gnss;
        IMUData imu;  
        NavState nav;
        bool gnss_valid = false;
        bool imu_valid = false;
        bool nav_valid = false;
    } last_valid_data_;

public:
    // 設定輪詢頻率
    void setPollingRate(uint32_t frequency_hz) {
        poll_frequency_hz_ = frequency_hz;
    }
    
    // 主要輪詢函數 - 固定頻率調用
    bool pollAndTransmit() {
        uint32_t current_time = micros();
        
        // 檢查是否到達輪詢時間
        if (current_time - last_poll_time_us_ < (1000000 / poll_frequency_hz_)) {
            return false; // 尚未到時間
        }
        
        last_poll_time_us_ = current_time;
        
        // 嘗試獲取最新數據，失敗則使用上次數據
        updateDataSnapshots();
        
        // 固定頻率傳輸（無論數據是否更新）
        return transmitCurrentData();
    }

private:
    void updateDataSnapshots() {
        // 嘗試更新，失敗就保持原值
        GNSSData temp_gnss;
        if (getLatestGNSSData(temp_gnss)) {
            last_valid_data_.gnss = temp_gnss;
            last_valid_data_.gnss_valid = true;
        }
        
        IMUData temp_imu;
        if (getLatestIMUData(temp_imu)) {
            last_valid_data_.imu = temp_imu;
            last_valid_data_.imu_valid = true;
        }
        
        NavState temp_nav;
        if (getLatestNavData(temp_nav)) {
            last_valid_data_.nav = temp_nav;
            last_valid_data_.nav_valid = true;
        }
    }
    
    bool transmitCurrentData() {
        // 使用固定緩衝區，無動態分配
        static uint8_t tx_buffer[300];
        
        // 根據當前模式傳輸對應數據
        switch(current_protocol_mode) {
            case PROTOCOL_MAVLINK_GPS:
                return transmitGPS(last_valid_data_.gnss, tx_buffer);
            case PROTOCOL_MAVLINK_ODOM:
                return transmitOdometry(last_valid_data_.nav, tx_buffer);
            default:
                return false;
        }
    }
    
    // 簡化的數據獲取函數（無阻塞）
    bool getLatestGNSSData(GNSSData& data) {
        // 非阻塞式獲取最新 GNSS 數據
        return DataBuffer::tryGetGNSS(data);
    }
    
    bool getLatestIMUData(IMUData& data) {
        // 非阻塞式獲取最新 IMU 數據  
        return DataBuffer::tryGetIMU(data);
    }
    
    bool getLatestNavData(NavState& data) {
        // 非阻塞式獲取最新導航數據
        return DataBuffer::tryGetNav(data);
    }
};

// 全域輪詢控制器
extern SystemControllerPolling g_system_controller;

// Arduino loop() 中的使用方式
#define SYSTEM_POLL() g_system_controller.pollAndTransmit()

#endif