#pragma once

#include "IProtocol.h"
#include "../util/crc_calculator.h"
#include "../util/minspixhawk_crc.h"
#include "../core/system_health.h"
#include "../util/math_utils.h"
#include "../util/log.h"  // 需要TDD_FREQ宏
// #include "../util/tdd_flow_checker.h"  // 🔒 臨時移除避免死機

/**
 * @brief AR-1A-FC 自家協議實現
 * 
 * 固定52字節封包格式，專為自家GUI設計
 * 包含IMU、溫度、時間、姿態等完整數據
 */
class AR1AFCProtocol : public IProtocol {
public:
    // AR-1A-FC 協議規範
    static constexpr size_t PACKET_SIZE = 52;           // 固定封包大小
    static constexpr size_t MAX_PAYLOAD_SIZE = 48;      // 52 - 4(CRC) = 48
    static constexpr uint32_t PROTOCOL_VERSION = 100;   // v1.0
    static constexpr size_t CRC_SIZE = 4;               // CRC-32
    
    // AR-1A-FC 封包標頭 (4 bytes) - 符合您的原始實現
    static inline const uint8_t PACKET_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
    
    // AR-1A-FC 完整封包結構 (52字節)
    struct AR1AFCPacket {
        uint8_t header[4];          // 0xFE 0x81 0xFF 0x55
        float gyro_x, gyro_y, gyro_z;       // 角速度 (DPS) - 12 bytes
        float accel_x, accel_y, accel_z;    // 加速度 (g) - 12 bytes  
        float temperature;                  // 溫度 (°C) - 4 bytes
        uint32_t time_counter;              // 時間計數器 (ms) - 4 bytes
        float pitch, roll, yaw;             // 姿態角 (deg) - 12 bytes (已校正)
        uint32_t crc32;                     // CRC-32 校驗 - 4 bytes
    } __attribute__((packed));
    
    // 編譯期大小檢查
    static_assert(sizeof(AR1AFCPacket) == PACKET_SIZE, 
                  "AR-1A-FC 封包大小必須為52字節");

public:
    AR1AFCProtocol() : packet_counter_(0), last_packet_time_ms_(0), first_packet_time_ms_(0), base_timestamp_ms_(0),
                        imu_update_time_(0) {
                        // 🔒 TDD checkers 臨時移除避免死機
                        // ar1afc_imu_update_checker_("AR1AFC:onIMUUpdate", "onIMUUpdate", 5000, true),
                        // ar1afc_gnss_update_checker_("AR1AFC:onGNSSUpdate", "onGNSSUpdate", 5000, true),
                        // ar1afc_tick_checker_("AR1AFC:tick", "tick", 5000, true),
                        // ar1afc_send_checker_("AR1AFC:sendNavigationData", "sendNavigationData", 5000, true),
                        // ar1afc_transport_checker_("AR1AFC:transport", "write", 5000, true) {
        // 初始化數據快照
        memset(&snap_nav_, 0, sizeof(snap_nav_));
        memset(&snap_imu_, 0, sizeof(snap_imu_));
    }
    
    bool initialize(ITransport* transport) override {
        if (!transport) {
            logProtocolError("initialize", "傳輸層指標為空");
            return false;
        }
        
        transport_ = transport;
        packet_counter_ = 0;
        last_packet_time_ms_ = 0;
        first_packet_time_ms_ = 0;
        base_timestamp_ms_ = millis();  // 基準時間戳
        
        logProtocolInfo("initialize", "AR-1A-FC 協議初始化完成，自適應時間軸已啟用");
        return true;
    }
    
    void cleanup() override {
        transport_ = nullptr;
        logProtocolInfo("cleanup", "AR-1A-FC 協議已清理");
    }
    
    bool isReady() const override {
        return transport_ && transport_->isReady();
    }
    
    ProtocolType getType() const override {
        return ProtocolType::GUI_JSON;  // 復用此枚舉代表自家格式
    }
    
    const char* getName() const override {
        return "AR-1A-FC";
    }
    
    uint32_t getVersion() const override {
        return PROTOCOL_VERSION;
    }
    
    size_t getMaxPayloadSize() const override {
        return MAX_PAYLOAD_SIZE;
    }
    
    bool sendNavigationData(const NavigationState& nav_data) override {
        // TDD頻率監控：AR1AFC sendNavigationData調用頻率
        // 移除：過度細節的頻率監控
        
        // 調試輸出已移到.cpp實現中
        logNavigationDebugInfo(nav_data);
        
        // AR-1A-FC 格式主要從 NavigationState 提取 IMU 部分
        AR1AFCPacket packet{};
        
        // 設置標頭
        memcpy(packet.header, PACKET_HEADER, 4);
        
        // 角速度數據 (DPS) - 直接使用原始數據 (與 MINSPixhawk 一致，假設 MTi 輸出已經是 DPS)
        packet.gyro_x = nav_data.angular_velocity_x;
        packet.gyro_y = nav_data.angular_velocity_y;
        packet.gyro_z = nav_data.angular_velocity_z;
        
        // 加速度數據 (g) - NavigationState 中的加速度欄位
        packet.accel_x = nav_data.acceleration_east / 9.80665f; // m/s² -> g
        packet.accel_y = nav_data.acceleration_north / 9.80665f;
        packet.accel_z = nav_data.acceleration_down / 9.80665f;
        
        // 溫度 (從NavigationState獲取，如果沒有則使用默認值)
        // TODO: NavigationState應該包含溫度信息，暫時使用測試值
        packet.temperature = 26.5f;  // 使用SystemController設定的測試溫度
        
        // 時間計數器：基於實際發送時間，反映真實輸出頻率
        uint32_t current_time = millis();
        if (first_packet_time_ms_ == 0) {
            first_packet_time_ms_ = current_time;
            packet.time_counter = 0;  // 第一個包從0開始
        } else {
            // 時間軸 = 從第一個包開始的累積時間，反映實際輸出間隔
            packet.time_counter = current_time - first_packet_time_ms_;
        }
        last_packet_time_ms_ = current_time;
        
        // 姿態角 (度) - 直接使用 NavigationState 中的歐拉角 (已經是度數)
        packet.pitch = nav_data.euler_pitch;  // 度數，直接使用
        packet.roll = nav_data.euler_roll;    // 度數，直接使用  
        packet.yaw = nav_data.euler_yaw;      // 度數，直接使用 (包含 90° 校正)
        
        return sendAR1AFCPacket(packet);
    }
    
    bool sendIMUData(const IMUData& imu_data) override {
        AR1AFCPacket packet{};
        
        // 設置標頭
        memcpy(packet.header, PACKET_HEADER, 4);
        
        // 角速度數據 (DPS) - 直接使用原始數據 (與 MINSPixhawk 一致，假設 MTi 輸出已經是 DPS)
        packet.gyro_x = imu_data.gyro_x;
        packet.gyro_y = imu_data.gyro_y;
        packet.gyro_z = imu_data.gyro_z;
        
        // 加速度數據 (g)
        packet.accel_x = imu_data.accel_x / 9.80665f; // m/s² -> g
        packet.accel_y = imu_data.accel_y / 9.80665f;
        packet.accel_z = imu_data.accel_z / 9.80665f;
        
        // 溫度 (°C)
        packet.temperature = imu_data.temperature;
        
        // 時間計數器：基於實際發送時間，反映真實輸出頻率
        uint32_t current_time = millis();
        if (first_packet_time_ms_ == 0) {
            first_packet_time_ms_ = current_time;
            packet.time_counter = 0;  // 第一個包從0開始
        } else {
            // 時間軸 = 從第一個包開始的累積時間，反映實際輸出間隔
            packet.time_counter = current_time - first_packet_time_ms_;
        }
        last_packet_time_ms_ = current_time;
        
        // 姿態角 (度) - 直接使用 IMUData 中的歐拉角 (已經是度數)
        packet.pitch = imu_data.euler_pitch;  // 度數，直接使用
        packet.roll = imu_data.euler_roll;    // 度數，直接使用
        packet.yaw = imu_data.euler_yaw;      // 度數，直接使用
        
        return sendAR1AFCPacket(packet);
    }
    
    bool sendGNSSData(const GNSSData& gnss_data) override {
        // AR-1A-FC 格式不直接支援 GNSS，但可以將 heading 數據融入姿態
        AR1AFCPacket packet{};
        
        // 設置標頭
        memcpy(packet.header, PACKET_HEADER, 4);
        
        // 角速度和加速度設為0（GNSS不提供）
        packet.gyro_x = packet.gyro_y = packet.gyro_z = 0.0f;
        packet.accel_x = packet.accel_y = packet.accel_z = 0.0f;
        
        // 溫度
        packet.temperature = 25.0f;
        
        // 時間計數器
        packet.time_counter = static_cast<uint32_t>(gnss_data.timestamp_us / 1000);
        
        // 姿態角 - 僅 yaw 有效（來自 GNSS heading）
        packet.pitch = 0.0f;
        packet.roll = 0.0f;
        packet.yaw = gnss_data.course_over_ground;  // GNSS heading (度)
        
        return sendAR1AFCPacket(packet);
    }
    
    bool sendSystemStatus(const SystemHealth& health) override {
        // AR-1A-FC 可用特殊標記表示系統狀態
        AR1AFCPacket packet{};
        
        // 設置標頭
        memcpy(packet.header, PACKET_HEADER, 4);
        
        // 用特殊值表示系統狀態
        if (health.all_systems_ok) {
            packet.gyro_x = 999.0f;  // 特殊標記：系統正常
            packet.gyro_y = 888.0f;
            packet.gyro_z = 777.0f;
        } else {
            packet.gyro_x = -999.0f;  // 特殊標記：系統異常
            packet.gyro_y = -888.0f;
            packet.gyro_z = -777.0f;
        }
        
        packet.accel_x = packet.accel_y = packet.accel_z = 0.0f;
        packet.temperature = health.cpu_temperature;
        packet.time_counter = static_cast<uint32_t>(micros() / 1000);
        packet.pitch = packet.roll = packet.yaw = 0.0f;
        
        return sendAR1AFCPacket(packet);
    }
    
    bool sendModeChangeHeader() override {
        ModeHeader header = createModeHeader(PACKET_SIZE);
        return sendHeader(header);
    }
    
    bool validatePacketSize(size_t payload_size) const override {
        if (payload_size != MAX_PAYLOAD_SIZE) {
            logProtocolError("validatePacketSize", 
                           "AR-1A-FC 封包大小必須為48字節");
            return false;
        }
        return true;
    }
    
    // AR-1A-FC 特有功能：發送原始緩衝區數據
    bool sendRawPacket(const uint8_t* buffer, size_t size) {
        if (!isReady()) {
            logProtocolError("sendRawPacket", "協議未準備就緒");
            return false;
        }
        
        if (size != PACKET_SIZE) {
            logProtocolError("sendRawPacket", "封包大小不正確");
            return false;
        }
        
        size_t sent = transport_->write(buffer, size);
        if (sent != size) {
            logProtocolError("sendRawPacket", "封包發送不完整");
            return false;
        }
        
        packet_counter_++;
        return true;
    }
    
    // Push→Pull 解耦：實現新介面
    void onIMUUpdate(const IMUData& imu_data, const NavigationState& nav_state) override {
        // TDD頻率監控：AR1AFC IMU更新頻率
        // 移除：過度細節的頻率監控
        
        // AR1AFC 直接跟隨 IMU 觸發頻率 - 無限制發送
        snap_imu_ = imu_data;
        snap_nav_ = nav_state;
        imu_update_time_ = micros();
        
        // 立即發送，不等待 tick
        if (snap_nav_.flags & NAV_ATTITUDE_VALID) {
            if (transport_ && transport_->isReady()) {
                // TDD頻率監控：AR1AFC 封包發送頻率
                // 移除：重複的封包發送監控
                bool success = sendNavigationData(snap_nav_);
            }
        }
    }
    
    void onGNSSUpdate(const GNSSData& gnss_data, const NavigationState& nav_state) override {
        // ar1afc_gnss_update_checker_.recordIn();  // 🔒 臨時移除避免死機
        
        // AR1AFC 不直接使用 GNSS，但可能更新導航狀態
        snap_nav_ = nav_state;
        
        // ar1afc_gnss_update_checker_.recordOut();  // 🔒 臨時移除避免死機
        // ar1afc_gnss_update_checker_.update();  // 🔒 臨時移除避免死機
    }
    
    void tick(uint32_t now_us) override {
        // ar1afc_tick_checker_.recordIn();  // 🔒 臨時移除避免死機
        
        // AR1AFC 不再需要 tick 排程器 - 直接在 onIMUUpdate 中發送
        // 留空以符合介面要求
        
        // ar1afc_tick_checker_.recordOut();  // 🔒 臨時移除避免死機
        // ar1afc_tick_checker_.update();  // 🔒 臨時移除避免死機
    }

private:
    uint32_t packet_counter_;
    uint32_t last_packet_time_ms_;     // 上一個包的發送時間
    uint32_t first_packet_time_ms_;    // 第一個包的發送時間
    uint32_t base_timestamp_ms_;       // 基準時間戳
    
    // Push→Pull 解耦：數據快照
    NavigationState snap_nav_;
    IMUData snap_imu_;
    uint32_t imu_update_time_;
    
    // 🔒 TDD 頻率監控 - 臨時移除避免死機
    // TDDFlowChecker ar1afc_imu_update_checker_;
    // TDDFlowChecker ar1afc_gnss_update_checker_;
    // TDDFlowChecker ar1afc_tick_checker_;
    // TDDFlowChecker ar1afc_send_checker_;
    // TDDFlowChecker ar1afc_transport_checker_;
    
    // AR-1A-FC 官方規範字節序工具函數
    
    // Little Endian (LSB first) - 用於大部分字段
    static inline void put_u32_le(uint8_t* dst, uint32_t v) {
        dst[0] = (v      ) & 0xFF;  // LSB first
        dst[1] = (v >> 8 ) & 0xFF;
        dst[2] = (v >> 16) & 0xFF;
        dst[3] = (v >> 24) & 0xFF;  // MSB last
    }
    
    static inline void put_f32_le(uint8_t* dst, float f) {
        static_assert(sizeof(float) == 4, "float must be 32-bit");
        uint32_t u;
        memcpy(&u, &f, 4);
        put_u32_le(dst, u);
    }
    
    // Big Endian (MSB first) - 僅用於溫度和CRC
    static inline void put_u32_be(uint8_t* dst, uint32_t v) {
        dst[0] = (v >> 24) & 0xFF;  // MSB first
        dst[1] = (v >> 16) & 0xFF;
        dst[2] = (v >> 8 ) & 0xFF;
        dst[3] = (v      ) & 0xFF;  // LSB last
    }
    
    static inline void put_f32_be(uint8_t* dst, float f) {
        static_assert(sizeof(float) == 4, "float must be 32-bit");
        uint32_t u;
        memcpy(&u, &f, 4);
        put_u32_be(dst, u);
    }
    
    // AR1AFC封包發送實現 - 在.cpp文件中實現
    bool sendAR1AFCPacket(AR1AFCPacket& packet);
    
    // 調試輸出函數 - 在.cpp文件中實現
    void logNavigationDebugInfo(const NavigationState& nav_data);
};

// 編譯期大小檢查
VALIDATE_PACKET_SIZE(AR1AFCProtocol, 48);