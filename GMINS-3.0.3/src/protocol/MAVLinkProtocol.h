#pragma once

#include "IProtocol.h"
#include "../core/system_health.h"
#include "../util/math_utils.h"
#include <functional>
#include <vector>

// 使用舊版本統一引入方式 - 採用 ArduinoPixhawk 相同方法
#include <MAVLink.h>

/**
 * @brief MAVLink v2.0 協議實現 - 使用官方 MAVLink library
 * 
 * 專為 Pixhawk 飛控設計，確保 QGC 完全兼容
 * 使用官方 MAVLink C library 進行消息編碼和發送
 */
class MAVLinkProtocol : public IProtocol {
public:
    // MAVLink v1 規範 - 採用舊版本方式，系統ID=1，組件ID=199
    static constexpr uint8_t SYSTEM_ID = 1;    // 標準系統ID
    static constexpr uint8_t COMPONENT_ID = 199; // 自定義組件ID
    static constexpr uint8_t MAVLINK_CHANNEL = MAVLINK_COMM_0;
    
    // MAVLink 消息緩衝區
    static constexpr size_t MAX_MAVLINK_BUFFER_SIZE = MAVLINK_MAX_PACKET_LEN;
    static constexpr size_t MAX_PAYLOAD_SIZE = MAVLINK_MAX_PAYLOAD_LEN;

public:
    MAVLinkProtocol(); // 構造函數實現移到 .cpp 檔案
    
    bool initialize(ITransport* transport) override {
        if (!transport) {
            logProtocolError("initialize", "傳輸層指標為空");
            return false;
        }
        
        transport_ = transport;
        sequence_number_ = 0;
        
        logProtocolInfo("initialize", "MAVLink v1 協議初始化完成 - 舊版統一引入方式");
        return true;
    }
    
    void cleanup() override {
        transport_ = nullptr;
        logProtocolInfo("cleanup", "MAVLink 協議已清理");
    }
    
    bool isReady() const override {
        return transport_ && transport_->isReady();
    }
    
    ProtocolType getType() const override {
        return ProtocolType::MAVLINK_V1;
    }
    
    const char* getName() const override {
        return "MAVLink_v1_Legacy";
    }
    
    uint32_t getVersion() const override {
        return 100;  // v1.0
    }
    
    size_t getMaxPayloadSize() const override {
        return MAX_PAYLOAD_SIZE;
    }
    
    bool sendIMUBasedData(const NavigationState& nav_data) {
        // 🚫 簡化版本：不發送IMU相關訊息，只保留GPS_INPUT
        // 直接返回成功，避免發送不必要的封包
        return true;
    }
    
    // sendIMUData() 死雞測試版本 - 不發送 ATTITUDE
    bool sendIMUData(const IMUData& imu_data) override {
        // 🔴 死雞測試：不發送 ATTITUDE，直接返回成功
        return true;
    }
    
    bool sendGNSSBasedData(const GNSSData& gnss_data);
    
    // 實現缺失的虛函數 - 實現移到 .cpp 檔案
    bool sendNavigationData(const NavigationState& nav_data) override;
    bool sendGNSSData(const GNSSData& gnss_data) override;
    
    // 實現HEARTBEAT消息 - QGC連接必需
    bool sendSystemStatus(const SystemHealth& health) override {
        return sendHeartbeat();
    }
    
    // QGC連接核心：HEARTBEAT消息
    bool sendHeartbeat() {
        if (!transport_ || !transport_->isReady()) {
            return false;
        }
        
        mavlink_message_t msg;
        mavlink_heartbeat_t heartbeat = {};
        
        // 設定載具類型和飛行模式
        heartbeat.type = MAV_TYPE_GENERIC;           // 通用載具
        heartbeat.autopilot = MAV_AUTOPILOT_GENERIC; // 通用自駕儀
        heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        heartbeat.custom_mode = 0;
        heartbeat.system_status = MAV_STATE_ACTIVE;  // 系統活躍
        heartbeat.mavlink_version = 3;               // MAVLink版本
        
        mavlink_msg_heartbeat_encode(SYSTEM_ID, COMPONENT_ID, &msg, &heartbeat);
        
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        
        if (len > 0) {
            size_t sent = transport_->write(buffer, len);
            return (sent == len);
        }
        
        return false;
    }
    
    bool sendModeChangeHeader() override {
        ModeHeader header = createModeHeader(0);  // 模式標頭沒有額外 payload
        return sendHeader(header);
    }
    
    bool validatePacketSize(size_t payload_size) const override {
        if (payload_size > MAX_PAYLOAD_SIZE) {
            logProtocolError("validatePacketSize", 
                           "封包大小超過 MAVLink 限制");
            return false;
        }
        return true;
    }
    
    // Push→Pull 解耦：實現新介面 - 實現移到 .cpp 檔案
    void onIMUUpdate(const IMUData& imu_data, const NavigationState& nav_state) override;
    
    void onGNSSUpdate(const GNSSData& gnss_data, const NavigationState& nav_state) override;
    
    void tick(uint32_t now_us) override {
        // MAVLink 不再需要 tick 排程器 - 直接在 onIMUUpdate/onGNSSUpdate 中發送
        // 留空以符合介面要求，暫時移除TDD調用
    }

private:
    uint8_t sequence_number_;
    
    // 自適應時間戳變量 - 死雞測試精簡版本
    uint32_t odometry_first_time_ms_;
    uint32_t gps_first_time_ms_;        // 保留但暫時不用
    
    // Push→Pull 解耦：數據快照
    NavigationState snap_nav_;
    IMUData snap_imu_;
    GNSSData snap_gnss_;
    uint32_t imu_update_time_;
    uint32_t gnss_update_time_;
    
    // TDD 輸出頻率監控變量
    uint32_t tdd_mavlink_output_count_;
    uint32_t tdd_odometry_output_count_;
    uint32_t tdd_gps_output_count_;
    uint32_t tdd_last_output_log_time_;
    
    // TDD 輸出頻率監控函數
    void tddTrackMAVLinkOutput(const char* packet_type);
    
    // 測試數據發送函數
    bool sendTestGPSInput();
    
    // MAVLink封包解析函數
    static void parseMAVLinkPacket(const uint8_t* buffer, uint16_t len);
};

// 編譯期大小檢查
VALIDATE_PACKET_SIZE(MAVLinkProtocol, 255);