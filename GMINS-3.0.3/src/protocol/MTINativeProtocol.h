#pragma once

#include "IProtocol.h"
#include "../core/system_health.h"
#include "../util/tdd_flow_checker.h"

/**
 * @brief MTI 原生協議實現
 * 
 * 直接透傳 XSENS MTI 原始數據
 * 保持原始 XBUS 格式不變
 */
class MTINativeProtocol : public IProtocol {
public:
    // MTI 協議規範
    static constexpr size_t MAX_PAYLOAD_SIZE = 512;     // MTI 最大封包大小
    static constexpr uint32_t PROTOCOL_VERSION = 100;   // v1.0
    
    // XBUS 訊息類型
    enum class XBUSMessageType : uint8_t {
        MTDATA2 = 0x36,         // MTData2 主要數據
        ERROR = 0x42,           // 錯誤訊息
        WARNING = 0x43,         // 警告訊息
        WAKEUP = 0x3E,          // 喚醒訊息
        GOTOCONFIGACK = 0x3D    // 配置確認
    };
    
    // XBUS 封包結構
    struct XBUSPacket {
        uint8_t preamble = 0xFA;        // XBUS 前導字
        uint8_t bus_id = 0xFF;          // 總線 ID
        uint8_t message_id;             // 訊息 ID
        uint8_t length;                 // 數據長度
        uint8_t data[MAX_PAYLOAD_SIZE]; // 原始數據
        uint8_t checksum;               // 校驗和
    } __attribute__((packed));
    
    // 編譯期大小檢查
    static_assert(sizeof(XBUSPacket) - MAX_PAYLOAD_SIZE == 5, 
                  "XBUS 標頭大小必須為5字節");

public:
    MTINativeProtocol() : packet_counter_(0),
                          mti_transport_checker_("MTI:transport", "write", 5000, true) {}
    
    bool initialize(ITransport* transport) override {
        if (!transport) {
            logProtocolError("initialize", "傳輸層指標為空");
            return false;
        }
        
        transport_ = transport;
        packet_counter_ = 0;
        
        logProtocolInfo("initialize", "MTI Native 協議初始化完成");
        return true;
    }
    
    void cleanup() override {
        transport_ = nullptr;
        logProtocolInfo("cleanup", "MTI Native 協議已清理");
    }
    
    bool isReady() const override {
        return transport_ && transport_->isReady();
    }
    
    ProtocolType getType() const override {
        return ProtocolType::MTI_NATIVE;
    }
    
    const char* getName() const override {
        return "MTI_Native";
    }
    
    uint32_t getVersion() const override {
        return PROTOCOL_VERSION;
    }
    
    size_t getMaxPayloadSize() const override {
        return MAX_PAYLOAD_SIZE;
    }
    
    bool sendNavigationData(const NavigationState& nav_data) override {
        // 將 NavigationState 轉換為 MTDATA2 格式
        return sendMTData2Packet(nav_data);
    }
    
    bool sendIMUData(const IMUData& imu_data) override {
        // 將 IMUData 轉換為 MTDATA2 格式
        return sendMTData2FromIMU(imu_data);
    }
    
    bool sendGNSSData(const GNSSData& gnss_data) override {
        // 將 GNSSData 轉換為 MTDATA2 格式
        return sendMTData2FromGNSS(gnss_data);
    }
    
    bool sendSystemStatus(const SystemHealth& health) override {
        // 發送系統狀態為 WARNING 或 ERROR 訊息
        if (health.all_systems_ok) {
            return sendStatusMessage(XBUSMessageType::WAKEUP, "System OK");
        } else {
            return sendStatusMessage(XBUSMessageType::WARNING, "System Warning");
        }
    }
    
    bool sendModeChangeHeader() override {
        ModeHeader header = createModeHeader(0);
        return sendHeader(header);
    }
    
    bool validatePacketSize(size_t payload_size) const override {
        if (payload_size > MAX_PAYLOAD_SIZE) {
            logProtocolError("validatePacketSize", 
                           "MTI 封包大小超過限制");
            return false;
        }
        return true;
    }
    
    // Push→Pull 解耦：實現新介面
    void onIMUUpdate(const IMUData& imu_data, const NavigationState& nav_state) override {
        // MTI Native 透傳原始 XBUS 數據
        // 暫時不做額外處理
    }
    
    void onGNSSUpdate(const GNSSData& gnss_data, const NavigationState& nav_state) override {
        // MTI Native 不處理 GNSS
    }
    
    void tick(uint32_t now_us) override {
        // MTI Native 沒有主動排程需求
        // 主要是被動透傳
    }
    
    // MTI 特有功能：透傳原始 XBUS 數據
    bool sendRawXBUSData(const uint8_t* xbus_data, size_t data_size) {
        if (!isReady()) {
            logProtocolError("sendRawXBUSData", "協議未準備就緒");
            return false;
        }
        
        if (!validatePacketSize(data_size)) {
            return false;
        }
        
        // 直接透傳原始 XBUS 數據
        mti_transport_checker_.recordIn();
        size_t sent = transport_->write(xbus_data, data_size);
        if (sent != data_size) {
            logProtocolError("sendRawXBUSData", "數據透傳不完整");
            // 發送失敗，不記錄 OUT
            mti_transport_checker_.update();
            return false;
        }
        mti_transport_checker_.recordOut();
        mti_transport_checker_.update();
        
        packet_counter_++;
        logProtocolInfo("sendRawXBUSData", "MTI 原始數據透傳完成");
        return true;
    }

private:
    uint32_t packet_counter_;
    TDDFlowChecker mti_transport_checker_;
    
    bool sendMTData2Packet(const NavigationState& nav_data) {
        XBUSPacket packet{};
        packet.message_id = static_cast<uint8_t>(XBUSMessageType::MTDATA2);
        
        // 構建 MTDATA2 數據結構
        size_t offset = 0;
        
        // 時間戳 (XDI_TimestampGroup | XDI_SampleTime64)
        uint16_t data_id = 0x1060;  // Timestamp
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        
        uint64_t timestamp = nav_data.timestamp_us;
        memcpy(packet.data + offset, &timestamp, 8);
        offset += 8;
        
        // 歐拉角 (XDI_OrientationGroup | XDI_EulerAngles)  
        data_id = 0x2030;  // EulerAngles 
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        
        // 轉換度數為弧度 (MTI 原生格式使用弧度)
        float euler_rad[3] = {
            nav_data.euler_roll * mu::kDegToRad,
            nav_data.euler_pitch * mu::kDegToRad, 
            nav_data.euler_yaw * mu::kDegToRad
        };
        memcpy(packet.data + offset, euler_rad, 12);
        offset += 12;
        
        // 加速度 (XDI_AccelerationGroup | XDI_Acceleration)
        data_id = 0x4020;  // Acceleration
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        
        float accel[3] = {
            nav_data.acceleration_north,
            nav_data.acceleration_east,
            nav_data.acceleration_down
        };
        memcpy(packet.data + offset, accel, 12);
        offset += 12;
        
        // 角速度 (XDI_AngularVelocityGroup | XDI_RateOfTurn)
        data_id = 0x8020;  // Rate of Turn
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        
        float gyro[3] = {
            nav_data.angular_velocity_x,
            nav_data.angular_velocity_y,
            nav_data.angular_velocity_z
        };
        memcpy(packet.data + offset, gyro, 12);
        offset += 12;
        
        // 位置 (XDI_PositionGroup | XDI_LatLon)
        if (nav_data.flags & NAV_POSITION_VALID) {
            data_id = 0x5040;  // Position (Lat/Lon)
            memcpy(packet.data + offset, &data_id, 2);
            offset += 2;
            
            // 使用相對位置（NED座標系）
            double lat_lon[2] = {
                nav_data.position_north,   // 北向位置
                nav_data.position_east     // 東向位置
            };
            memcpy(packet.data + offset, lat_lon, 16);
            offset += 16;
        }
        
        packet.length = static_cast<uint8_t>(offset);
        packet.checksum = calculateXBUSChecksum(packet);
        
        return sendXBUSPacket(packet);
    }
    
    bool sendMTData2FromIMU(const IMUData& imu_data) {
        XBUSPacket packet{};
        packet.message_id = static_cast<uint8_t>(XBUSMessageType::MTDATA2);
        
        size_t offset = 0;
        
        // 時間戳
        uint16_t data_id = 0x1060;
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        memcpy(packet.data + offset, &imu_data.timestamp_us, 8);
        offset += 8;
        
        // 歐拉角
        data_id = 0x2030;  // Euler Angles
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        
        // 直接使用 IMUData 中的歐拉角，轉換度數為弧度
        float euler[3] = { 
            imu_data.euler_roll * mu::kDegToRad,
            imu_data.euler_pitch * mu::kDegToRad, 
            imu_data.euler_yaw * mu::kDegToRad 
        };
        memcpy(packet.data + offset, euler, 12);
        offset += 12;
        
        // 加速度
        data_id = 0x4020;
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        float accel[3] = { imu_data.accel_x, imu_data.accel_y, imu_data.accel_z };
        memcpy(packet.data + offset, accel, 12);
        offset += 12;
        
        // 角速度
        data_id = 0x8020;
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        float gyro[3] = { imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z };
        memcpy(packet.data + offset, gyro, 12);
        offset += 12;
        
        // 溫度
        data_id = 0x0810;  // Temperature
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        memcpy(packet.data + offset, &imu_data.temperature, 4);
        offset += 4;
        
        packet.length = static_cast<uint8_t>(offset);
        packet.checksum = calculateXBUSChecksum(packet);
        
        return sendXBUSPacket(packet);
    }
    
    bool sendMTData2FromGNSS(const GNSSData& gnss_data) {
        XBUSPacket packet{};
        packet.message_id = static_cast<uint8_t>(XBUSMessageType::MTDATA2);
        
        size_t offset = 0;
        
        // 時間戳
        uint16_t data_id = 0x1060;
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        memcpy(packet.data + offset, &gnss_data.timestamp_us, 8);
        offset += 8;
        
        // GPS 位置
        data_id = 0x5040;  // Position
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        
        double lat_lon[2] = {gnss_data.latitude, gnss_data.longitude};
        memcpy(packet.data + offset, lat_lon, 16);
        offset += 16;
        
        // GPS 高度
        data_id = 0x5020;  // Altitude
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        memcpy(packet.data + offset, &gnss_data.altitude_msl, 4);
        offset += 4;
        
        // GPS 速度
        data_id = 0x5030;  // Velocity
        memcpy(packet.data + offset, &data_id, 2);
        offset += 2;
        memcpy(packet.data + offset, &gnss_data.ground_speed, 4);
        offset += 4;
        
        packet.length = static_cast<uint8_t>(offset);
        packet.checksum = calculateXBUSChecksum(packet);
        
        return sendXBUSPacket(packet);
    }
    
    bool sendStatusMessage(XBUSMessageType msg_type, const char* message) {
        XBUSPacket packet{};
        packet.message_id = static_cast<uint8_t>(msg_type);
        
        size_t msg_len = strlen(message);
        if (msg_len > MAX_PAYLOAD_SIZE) {
            msg_len = MAX_PAYLOAD_SIZE;
        }
        
        memcpy(packet.data, message, msg_len);
        packet.length = static_cast<uint8_t>(msg_len);
        packet.checksum = calculateXBUSChecksum(packet);
        
        return sendXBUSPacket(packet);
    }
    
    bool sendXBUSPacket(const XBUSPacket& packet) {
        if (!isReady()) {
            logProtocolError("sendXBUSPacket", "協議未準備就緒");
            return false;
        }
        
        size_t packet_size = 5 + packet.length;  // 標頭(4) + length + data + checksum(1)
        
        mti_transport_checker_.recordIn();
        size_t sent = transport_->write(reinterpret_cast<const uint8_t*>(&packet), packet_size);
        if (sent != packet_size) {
            logProtocolError("sendXBUSPacket", "XBUS 封包發送不完整");
            // 發送失敗，不記錄 OUT
            mti_transport_checker_.update();
            return false;
        }
        mti_transport_checker_.recordOut();
        mti_transport_checker_.update();
        
        packet_counter_++;
        return true;
    }
    
    uint8_t calculateXBUSChecksum(const XBUSPacket& packet) const {
        // XBUS 校驗和：所有字節(除校驗和本身)的簡單累加取反
        uint8_t sum = 0;
        sum += packet.bus_id;
        sum += packet.message_id;
        sum += packet.length;
        
        for (int i = 0; i < packet.length; i++) {
            sum += packet.data[i];
        }
        
        return 0x100 - sum;  // 取補數
    }
};

// 編譯期大小檢查
VALIDATE_PACKET_SIZE(MTINativeProtocol, 512);