#ifndef SIMPLIFIED_POLLING_MODEL_H
#define SIMPLIFIED_POLLING_MODEL_H

#include <Arduino.h>

// 極簡輪詢模型 - 只需要兩個持續更新函數
class SimplifiedPollingModel {
private:
    // 固定100Hz輪詢計時
    uint32_t last_poll_us_ = 0;
    static const uint32_t POLL_INTERVAL_US = 10000; // 100Hz = 10ms = 10000us
    
    // 數據快照（靜態分配，無堆疊壓力）
    struct {
        // IMU 數據快照
        float gyro[3] = {0};
        float accel[3] = {0};
        float quat[4] = {1, 0, 0, 0};
        bool imu_valid = false;
        
        // GNSS 數據快照
        double lat = 0.0;
        double lon = 0.0;
        float alt = 0.0f;
        float vel[3] = {0};
        bool gnss_valid = false;
    } data_snapshot_;

public:
    // 主輪詢循環 - 替代所有複雜協議管理
    void continuousUpdate() {
        uint32_t now = micros();
        
        // 固定100Hz檢查
        if (now - last_poll_us_ >= POLL_INTERVAL_US) {
            last_poll_us_ = now;
            
            // 持續更新數據快照
            updateDataSnapshots();
            
            // 根據當前模式傳輸數據
            switch(getCurrentMode()) {
                case MODE_IMU_FOCUS:
                    sendIMUBasedData();
                    break;
                case MODE_GNSS_FOCUS:
                    sendGNSSBasedData();
                    break;
                case MODE_COMBINED:
                    sendIMUBasedData();
                    sendGNSSBasedData();
                    break;
            }
        }
    }

private:
    void updateDataSnapshots() {
        // 嘗試更新 IMU 數據（非阻塞）
        if (tryGetLatestIMU()) {
            data_snapshot_.imu_valid = true;
        }
        
        // 嘗試更新 GNSS 數據（非阻塞）
        if (tryGetLatestGNSS()) {
            data_snapshot_.gnss_valid = true;
        }
    }
    
    // 持續發送 IMU 基礎數據
    void sendIMUBasedData() {
        if (!data_snapshot_.imu_valid) return;
        
        // 使用固定緩衝區，零堆疊消耗
        static uint8_t imu_buffer[300];
        
        // 填充 MAVLink ATTITUDE 或 ODOMETRY
        fillMAVLinkIMU(imu_buffer);
        transmitPacket(imu_buffer);
    }
    
    // 持續發送 GNSS 基礎數據  
    void sendGNSSBasedData() {
        if (!data_snapshot_.gnss_valid) return;
        
        // 使用固定緩衝區，零堆疊消耗
        static uint8_t gnss_buffer[300];
        
        // 填充 MAVLink GPS_INPUT 或 GPS_RAW_INT
        fillMAVLinkGNSS(gnss_buffer);
        transmitPacket(gnss_buffer);
    }
    
    void fillMAVLinkIMU(uint8_t* buffer) {
        // 直接使用快照數據，無結構拷貝
        mavlink_attitude_t attitude;
        attitude.roll = calculateRoll(data_snapshot_.quat);
        attitude.pitch = calculatePitch(data_snapshot_.quat);
        attitude.yaw = calculateYaw(data_snapshot_.quat);
        attitude.time_boot_ms = millis(); // 簡單時間戳
        
        // 採用舊版本方式：明確指定 System ID = 199
        mavlink_message_t msg;
        mavlink_msg_attitude_encode(199, 1, &msg, &attitude);
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    }
    
    void fillMAVLinkGNSS(uint8_t* buffer) {
        // 直接使用快照數據，無結構拷貝
        mavlink_gps_input_t gps_input;
        gps_input.lat = (int32_t)(data_snapshot_.lat * 1e7);
        gps_input.lon = (int32_t)(data_snapshot_.lon * 1e7);
        gps_input.alt = data_snapshot_.alt * 1000; // m -> mm
        gps_input.time_usec = (uint64_t)millis() * 1000; // 簡單時間戳
        
        // 採用舊版本方式：明確指定 System ID = 199, Component ID = 1
        mavlink_message_t msg;
        mavlink_msg_gps_input_encode(199, 1, &msg, &gps_input);
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    }
};

// 全域簡化模型
extern SimplifiedPollingModel g_polling_model;

// Arduino loop() 中的使用
#define CONTINUOUS_UPDATE() g_polling_model.continuousUpdate()

#endif