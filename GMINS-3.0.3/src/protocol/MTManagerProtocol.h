#pragma once

#include "IProtocol.h"
#include "../core/system_health.h"
#include "../util/tdd_flow_checker.h"
#include "../util/math_utils.h"

/**
 * @brief MT Manager 協議實現
 * 
 * 專門為 XSENS MT Manager 軟體提供數據輸出
 * 使用 MT Manager 相容的數據格式
 * 支援高頻率數據輸出(100Hz)
 */
class MTManagerProtocol : public IProtocol {
public:
    // MT Manager 協議規範
    static constexpr size_t MAX_PAYLOAD_SIZE = 1024;    // MT Manager 最大封包大小
    static constexpr uint32_t PROTOCOL_VERSION = 100;   // v1.0
    static constexpr uint32_t DEFAULT_OUTPUT_RATE_HZ = 100; // 預設輸出頻率
    
    // MT Manager 數據封包類型
    enum class MTManagerPacketType : uint16_t {
        IMU_DATA = 0x1001,          // IMU 數據封包
        GNSS_DATA = 0x1002,         // GNSS 數據封包  
        NAVIGATION_DATA = 0x1003,   // 導航數據封包
        STATUS_DATA = 0x1004,       // 系統狀態封包
        CALIBRATION_DATA = 0x1005   // 校準數據封包
    };
    
    // MT Manager 相容 XBUS 封包格式 (基於實際 MTI 數據)
    struct MTManagerPacket {
        uint8_t preamble = 0xFA;                // XBUS 前導字
        uint8_t bus_id = 0xFF;                  // 總線 ID  
        uint8_t message_id = 0x36;              // MTDATA2 訊息 ID
        uint8_t payload_size;                   // 數據大小
        uint8_t payload[MAX_PAYLOAD_SIZE];      // 數據內容
        uint8_t checksum;                       // XBUS 校驗和
    } __attribute__((packed));
    
    // MT Manager 擴展封包格式 (包含時間戳和類型)
    struct MTManagerExtendedPacket {
        MTManagerPacket xbus_packet;            // 標準 XBUS 封包
        uint32_t timestamp_us;                  // 擴展時間戳
        uint16_t data_type;                     // 數據類型標識
    } __attribute__((packed));
    
    // IMU 數據結構 (MT Manager 格式)
    struct MTManagerIMUData {
        float accel_x, accel_y, accel_z;        // 加速度 (m/s²)
        float gyro_x, gyro_y, gyro_z;           // 角速度 (rad/s)  
        float euler_roll, euler_pitch, euler_yaw; // 歐拉角 (度)
        float temperature;                       // 溫度 (°C)
        uint32_t status_flags;                   // 狀態標誌
    } __attribute__((packed));
    
    // GNSS 數據結構 (MT Manager 格式)
    struct MTManagerGNSSData {
        double latitude, longitude;              // 經緯度 (度)
        float altitude_msl;                      // 海拔高度 (m)
        float ground_speed;                      // 地面速度 (m/s)
        float course_angle;                      // 航向角 (度)
        uint8_t satellites_used;                 // 使用衛星數
        uint8_t fix_type;                        // 定位類型
        float hdop, vdop;                        // 精度因子
        uint32_t status_flags;                   // 狀態標誌
    } __attribute__((packed));
    
    // 導航數據結構 (MT Manager 格式)
    struct MTManagerNavigationData {
        // 位置 (NED 座標系)
        float position_north, position_east, position_down; 
        // 速度 (NED 座標系)
        float velocity_north, velocity_east, velocity_down;
        // 姿態 (歐拉角)
        float euler_roll, euler_pitch, euler_yaw;
        // 角速度
        float angular_velocity_x, angular_velocity_y, angular_velocity_z;
        // 加速度 (NED 座標系)  
        float acceleration_north, acceleration_east, acceleration_down;
        // 導航狀態標誌
        uint32_t navigation_flags;
        // 精度估計
        float position_accuracy, velocity_accuracy, attitude_accuracy;
    } __attribute__((packed));

public:
    MTManagerProtocol() : packet_counter_(0), output_rate_hz_(DEFAULT_OUTPUT_RATE_HZ),
                          last_output_time_us_(0),
                          mt_transport_checker_("MTManager:transport", "write", 5000, true) {}
    
    bool initialize(ITransport* transport) override {
        if (!transport) {
            logProtocolError("initialize", "傳輸層指標為空");
            return false;
        }
        
        transport_ = transport;
        packet_counter_ = 0;
        last_output_time_us_ = 0;
        
        logProtocolInfo("initialize", "MT Manager 協議初始化完成");
        return true;
    }
    
    void cleanup() override {
        transport_ = nullptr;
        logProtocolInfo("cleanup", "MT Manager 協議已清理");
    }
    
    bool isReady() const override {
        return transport_ && transport_->isReady();
    }
    
    ProtocolType getType() const override {
        return ProtocolType::MTI_NATIVE; // 暫時使用 MTI_NATIVE，之後可擴充枚舉
    }
    
    const char* getName() const override {
        return "MT_Manager";
    }
    
    uint32_t getVersion() const override {
        return PROTOCOL_VERSION;
    }
    
    size_t getMaxPayloadSize() const override {
        return MAX_PAYLOAD_SIZE;
    }
    
    bool sendNavigationData(const NavigationState& nav_data) override {
        MTManagerNavigationData mt_nav{};
        
        // 轉換 NavigationState 到 MT Manager 格式
        mt_nav.position_north = nav_data.position_north;
        mt_nav.position_east = nav_data.position_east;
        mt_nav.position_down = nav_data.position_down;
        
        mt_nav.velocity_north = nav_data.velocity_north;
        mt_nav.velocity_east = nav_data.velocity_east;
        mt_nav.velocity_down = nav_data.velocity_down;
        
        mt_nav.euler_roll = nav_data.euler_roll;
        mt_nav.euler_pitch = nav_data.euler_pitch;
        mt_nav.euler_yaw = nav_data.euler_yaw;
        
        mt_nav.angular_velocity_x = nav_data.angular_velocity_x;
        mt_nav.angular_velocity_y = nav_data.angular_velocity_y;
        mt_nav.angular_velocity_z = nav_data.angular_velocity_z;
        
        mt_nav.acceleration_north = nav_data.acceleration_north;
        mt_nav.acceleration_east = nav_data.acceleration_east;
        mt_nav.acceleration_down = nav_data.acceleration_down;
        
        mt_nav.navigation_flags = nav_data.flags;
        
        // 設定精度估計 (使用預設值，可根據需要調整)
        mt_nav.position_accuracy = 1.0f;
        mt_nav.velocity_accuracy = 0.1f;
        mt_nav.attitude_accuracy = 0.5f;
        
        return sendMTManagerPacket(MTManagerPacketType::NAVIGATION_DATA, 
                                  &mt_nav, sizeof(mt_nav), nav_data.timestamp_us);
    }
    
    bool sendIMUData(const IMUData& imu_data) override {
        MTManagerIMUData mt_imu{};
        
        // 轉換 IMUData 到 MT Manager 格式
        mt_imu.accel_x = imu_data.accel_x;
        mt_imu.accel_y = imu_data.accel_y; 
        mt_imu.accel_z = imu_data.accel_z;
        
        mt_imu.gyro_x = imu_data.gyro_x;
        mt_imu.gyro_y = imu_data.gyro_y;
        mt_imu.gyro_z = imu_data.gyro_z;
        
        mt_imu.euler_roll = imu_data.euler_roll;
        mt_imu.euler_pitch = imu_data.euler_pitch;
        mt_imu.euler_yaw = imu_data.euler_yaw;
        
        mt_imu.temperature = imu_data.temperature;
        mt_imu.status_flags = (imu_data.flags & 0xFFFFFFFF);
        
        return sendMTManagerPacket(MTManagerPacketType::IMU_DATA,
                                  &mt_imu, sizeof(mt_imu), imu_data.timestamp_us);
    }
    
    bool sendGNSSData(const GNSSData& gnss_data) override {
        MTManagerGNSSData mt_gnss{};
        
        // 轉換 GNSSData 到 MT Manager 格式
        mt_gnss.latitude = gnss_data.latitude;
        mt_gnss.longitude = gnss_data.longitude;
        mt_gnss.altitude_msl = gnss_data.altitude_msl;
        mt_gnss.ground_speed = gnss_data.ground_speed;
        mt_gnss.course_angle = gnss_data.course_over_ground;
        mt_gnss.satellites_used = gnss_data.satellites_used;
        mt_gnss.fix_type = static_cast<uint8_t>(gnss_data.fix_type);
        mt_gnss.hdop = gnss_data.hdop;
        mt_gnss.vdop = gnss_data.vdop;
        mt_gnss.status_flags = (gnss_data.flags & 0xFFFFFFFF);
        
        return sendMTManagerPacket(MTManagerPacketType::GNSS_DATA,
                                  &mt_gnss, sizeof(mt_gnss), gnss_data.timestamp_us);
    }
    
    bool sendSystemStatus(const SystemHealth& health) override {
        // 創建狀態數據結構
        struct StatusData {
            uint32_t system_flags;
            float cpu_usage;
            float memory_usage;
            uint32_t error_count;
            char status_message[64];
        } __attribute__((packed));
        
        StatusData status{};
        status.system_flags = health.all_systems_ok ? 0x01 : 0x00;
        status.cpu_usage = 0.0f;    // 可根據需要實現
        status.memory_usage = 0.0f; // 可根據需要實現
        status.error_count = 0;     // 可根據需要實現
        
        const char* msg = health.all_systems_ok ? "System OK" : "System Warning";
        strncpy(status.status_message, msg, sizeof(status.status_message) - 1);
        status.status_message[sizeof(status.status_message) - 1] = '\0';
        
        return sendMTManagerPacket(MTManagerPacketType::STATUS_DATA,
                                  &status, sizeof(status), micros());
    }
    
    bool sendModeChangeHeader() override {
        ModeHeader header = createModeHeader(0);
        return sendHeader(header);
    }
    
    bool validatePacketSize(size_t payload_size) const override {
        if (payload_size > MAX_PAYLOAD_SIZE) {
            logProtocolError("validatePacketSize", "MT Manager 封包大小超過限制");
            return false;
        }
        return true;
    }
    
    // Push→Pull 解耦：實現新介面
    void onIMUUpdate(const IMUData& imu_data, const NavigationState& nav_state) override {
        latest_imu_data_ = imu_data;
        imu_data_ready_ = true;
    }
    
    void onGNSSUpdate(const GNSSData& gnss_data, const NavigationState& nav_state) override {
        latest_gnss_data_ = gnss_data;
        gnss_data_ready_ = true;
    }
    
    void tick(uint32_t now_us) override {
        // 檢查輸出時機 (支援可調頻率)
        uint32_t interval_us = 1000000 / output_rate_hz_;
        if (now_us - last_output_time_us_ >= interval_us) {
            
            // 依序發送可用的數據
            if (imu_data_ready_) {
                sendIMUData(latest_imu_data_);
                imu_data_ready_ = false;
            }
            
            if (gnss_data_ready_) {
                sendGNSSData(latest_gnss_data_);
                gnss_data_ready_ = false;
            }
            
            last_output_time_us_ = now_us;
        }
    }
    
    // MT Manager 特有功能
    void setOutputRate(uint32_t rate_hz) {
        if (rate_hz > 0 && rate_hz <= 1000) {
            output_rate_hz_ = rate_hz;
            logProtocolInfo("setOutputRate", "MT Manager 輸出頻率已更新");
        }
    }
    
    uint32_t getOutputRate() const {
        return output_rate_hz_;
    }

private:
    uint32_t packet_counter_;
    uint32_t output_rate_hz_;
    uint32_t last_output_time_us_;
    TDDFlowChecker mt_transport_checker_;
    
    // 數據快照
    IMUData latest_imu_data_;
    GNSSData latest_gnss_data_;
    bool imu_data_ready_ = false;
    bool gnss_data_ready_ = false;
    
    bool sendMTManagerPacket(MTManagerPacketType packet_type, 
                            const void* payload, size_t payload_size,
                            uint32_t timestamp_us) {
        if (!isReady()) {
            logProtocolError("sendMTManagerPacket", "協議未準備就緒");
            return false;
        }
        
        if (!validatePacketSize(payload_size)) {
            return false;
        }
        
        MTManagerPacket packet{};
        packet.payload_size = static_cast<uint8_t>(payload_size);
        
        // 複製數據到封包
        memcpy(packet.payload, payload, payload_size);
        
        // 計算 XBUS 校驗和
        packet.checksum = calculateXBUSChecksum(packet);
        
        // 發送標準 XBUS 格式封包
        size_t total_size = 5 + payload_size;  // 標頭(4) + payload + checksum(1)
        
        mt_transport_checker_.recordIn();
        size_t sent = transport_->write(reinterpret_cast<const uint8_t*>(&packet), total_size);
        if (sent != total_size) {
            logProtocolError("sendMTManagerPacket", "MT Manager 封包發送不完整");
            mt_transport_checker_.update();
            return false;
        }
        mt_transport_checker_.recordOut();
        mt_transport_checker_.update();
        
        packet_counter_++;
        return true;
    }
    
    uint8_t calculateXBUSChecksum(const MTManagerPacket& packet) const {
        // XBUS 校驗和：所有字節(除校驗和本身)的簡單累加取反
        uint8_t sum = 0;
        sum += packet.bus_id;
        sum += packet.message_id;
        sum += packet.payload_size;
        
        for (int i = 0; i < packet.payload_size; i++) {
            sum += packet.payload[i];
        }
        
        return 0x100 - sum;  // 取補數
    }
    
    // MT Manager 特有：解析實際 MTI XBUS 數據
    bool parseAndForwardMTIData(const uint8_t* raw_xbus_data, size_t data_size) {
        if (!isReady() || data_size < 5) {
            return false;
        }
        
        // 驗證 XBUS 格式
        if (raw_xbus_data[0] != 0xFA || raw_xbus_data[1] != 0xFF || raw_xbus_data[2] != 0x36) {
            logProtocolError("parseAndForwardMTIData", "無效的 XBUS 格式");
            return false;
        }
        
        uint8_t payload_size = raw_xbus_data[3];
        if (data_size != 5 + payload_size) {
            logProtocolError("parseAndForwardMTIData", "XBUS 封包大小不符");
            return false;
        }
        
        // 直接轉發原始 MTI 數據到 MT Manager
        mt_transport_checker_.recordIn();
        size_t sent = transport_->write(raw_xbus_data, data_size);
        if (sent != data_size) {
            logProtocolError("parseAndForwardMTIData", "MTI 數據轉發失敗");
            mt_transport_checker_.update();
            return false;
        }
        mt_transport_checker_.recordOut();
        mt_transport_checker_.update();
        
        packet_counter_++;
        logProtocolInfo("parseAndForwardMTIData", "MTI 原始數據已轉發至 MT Manager");
        return true;
    }
};

// 編譯期大小檢查
VALIDATE_PACKET_SIZE(MTManagerProtocol, 1024);