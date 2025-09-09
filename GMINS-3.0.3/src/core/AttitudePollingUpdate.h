#ifndef ATTITUDE_POLLING_UPDATE_H
#define ATTITUDE_POLLING_UPDATE_H

#include <Arduino.h>

// 姿態角輪詢更新系統
class AttitudePollingUpdate {
private:
    // 姿態數據快照 - 靜態分配，無堆疊壓力
    struct AttitudeSnapshot {
        float roll = 0.0f;
        float pitch = 0.0f; 
        float yaw = 0.0f;
        float quat_w = 1.0f;
        float quat_x = 0.0f;
        float quat_y = 0.0f;
        float quat_z = 0.0f;
        bool is_valid = false;
        uint32_t last_update_ms = 0;
    } current_attitude_;
    
public:
    // 固定頻率輪詢 - 無論數據何時到達
    void updateIfAvailable() {
        // 嘗試獲取新姿態數據（非阻塞）
        AttitudeSnapshot new_data;
        if (tryGetLatestAttitude(new_data)) {
            // 有新數據就更新，沒有就保持原值
            current_attitude_ = new_data;
            current_attitude_.last_update_ms = millis();
        }
        // 關鍵：無論是否有新數據，當前姿態都是有效的
    }
    
    // 獲取當前姿態（100Hz固定可用）
    const AttitudeSnapshot& getCurrentAttitude() const {
        return current_attitude_;
    }
    
    // 檢查數據新鮮度（可選）
    bool isDataFresh(uint32_t max_age_ms = 100) const {
        return (millis() - current_attitude_.last_update_ms) <= max_age_ms;
    }
    
    // 直接用於MAVLink傳輸（零拷貝）
    void fillMAVLinkAttitude(mavlink_attitude_t& msg) {
        // 直接填充，無時間戳計算複雜度
        msg.roll = current_attitude_.roll;
        msg.pitch = current_attitude_.pitch;
        msg.yaw = current_attitude_.yaw;
        // 時間戳變成簡單的計數器
        msg.time_boot_ms = millis();  // 或使用固定遞增
    }
    
    void fillMAVLinkQuaternion(mavlink_attitude_quaternion_t& msg) {
        msg.q1 = current_attitude_.quat_w;
        msg.q2 = current_attitude_.quat_x; 
        msg.q3 = current_attitude_.quat_y;
        msg.q4 = current_attitude_.quat_z;
        msg.time_boot_ms = millis();
    }

private:
    bool tryGetLatestAttitude(AttitudeSnapshot& data) {
        // 非阻塞式獲取最新姿態數據
        // 如果 Xsens 有新數據就取，沒有就返回 false
        return XsensDataBuffer::tryGetAttitude(data);
    }
};

// 全域姿態管理器
extern AttitudePollingUpdate g_attitude_manager;

// 使用宏簡化
#define UPDATE_ATTITUDE() g_attitude_manager.updateIfAvailable()
#define GET_CURRENT_ATTITUDE() g_attitude_manager.getCurrentAttitude()

#endif