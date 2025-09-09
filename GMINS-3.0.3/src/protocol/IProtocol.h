#pragma once

#include <cstdint>
#include <cstddef>
#include "../transport/ITransport.h"
#include "../data/data_types.h"
#include "../core/system_health.h"
#include "../util/log.h"

/**
 * @brief 協議抽象介面
 * 
 * 定義統一的封包格式化和傳輸介面
 * 支援 MAVLink、GUI JSON、MTI 等協議
 */
class IProtocol {
public:
    // 協議類型枚舉
    enum class ProtocolType {
        MAVLINK_V1,     // Pixhawk MAVLink v1.0 (舊版本)
        MAVLINK_V2,     // Pixhawk MAVLink v2.0
        GUI_JSON,       // 自家 GUI JSON 格式
        MTI_NATIVE,     // MTI 原生封包格式
        UNKNOWN
    };
    
    // 模式識別標頭結構（防止接收端解錯）
    struct ModeHeader {
        uint32_t magic_number;      // 0xDEADBEEF
        ProtocolType protocol_type;
        uint32_t protocol_version;
        uint32_t payload_size;
        uint32_t checksum;
    };
    
    static constexpr uint32_t MAGIC_NUMBER = 0xDEADBEEF;
    static constexpr size_t HEADER_SIZE = sizeof(ModeHeader);
    
public:
    virtual ~IProtocol() = default;
    
    // 協議初始化
    virtual bool initialize(ITransport* transport) = 0;
    virtual void cleanup() = 0;
    virtual bool isReady() const = 0;
    
    // 協議資訊
    virtual ProtocolType getType() const = 0;
    virtual const char* getName() const = 0;
    virtual uint32_t getVersion() const = 0;
    virtual size_t getMaxPayloadSize() const = 0;
    
    // Push→Pull 解耦：數據快照更新（不直接發送）
    virtual void onIMUUpdate(const IMUData& imu_data, const NavigationState& nav_state) {}
    virtual void onGNSSUpdate(const GNSSData& gnss_data, const NavigationState& nav_state) {}
    
    // 協議內排程器：決定實際發送時機
    virtual void tick(uint32_t now_us) = 0;
    
    // 傳統數據封包化和傳輸（保留向後相容）
    virtual bool sendNavigationData(const NavigationState& nav_data) = 0;
    virtual bool sendIMUData(const IMUData& imu_data) = 0;
    virtual bool sendGNSSData(const GNSSData& gnss_data) = 0;
    virtual bool sendSystemStatus(const SystemHealth& health) = 0;
    
    // 模式切換支援（關鍵功能）
    virtual bool sendModeChangeHeader() = 0;
    virtual bool validatePacketSize(size_t payload_size) const = 0;
    
protected:
    ITransport* transport_;
    
    // 統一錯誤處理 (簡化版本 - 移除詳細log以提升性能)
    void logProtocolError(const char* operation, const char* details) const {
        // 只記錄關鍵錯誤，避免高頻log影響性能
        LOG_ERR("PROTOCOL", "[%s] %s failed", getName(), operation);
    }
    
    void logProtocolInfo(const char* operation, const char* details) const {
        // INFO級別log已禁用以提升性能 
        // LOG_INFO("PROTOCOL", "[%s] %s: %s", getName(), operation, details);
    }
    
    // 模式識別標頭生成
    ModeHeader createModeHeader(size_t payload_size) const {
        ModeHeader header{};
        header.magic_number = MAGIC_NUMBER;
        header.protocol_type = getType();
        header.protocol_version = getVersion();
        header.payload_size = static_cast<uint32_t>(payload_size);
        header.checksum = calculateHeaderChecksum(header);
        return header;
    }
    
    // 簡單校驗和計算
    uint32_t calculateHeaderChecksum(const ModeHeader& header) const {
        uint32_t sum = header.magic_number;
        sum ^= static_cast<uint32_t>(header.protocol_type);
        sum ^= header.protocol_version;
        sum ^= header.payload_size;
        return sum;
    }
    
    // 發送模式標頭 (移除高頻log以提升性能)
    bool sendHeader(const ModeHeader& header) {
        if (!transport_ || !transport_->isReady()) {
            logProtocolError("sendHeader", "transport_not_ready");
            return false;
        }
        
        size_t sent = transport_->write(reinterpret_cast<const uint8_t*>(&header), HEADER_SIZE);
        if (sent != HEADER_SIZE) {
            logProtocolError("sendHeader", "incomplete_send");
            return false;
        }
        
        return true;
    }
};

/**
 * @brief 編譯期封包大小檢查工具
 */
template<typename ProtocolClass, size_t MaxSize>
class PacketSizeValidator {
public:
    static constexpr bool validate() {
        static_assert(ProtocolClass::MAX_PAYLOAD_SIZE <= MaxSize, 
                     "協議封包大小超過限制");
        return true;
    }
};

// 編譯期檢查宏
#define VALIDATE_PACKET_SIZE(ProtocolClass, MaxSize) \
    static_assert(PacketSizeValidator<ProtocolClass, MaxSize>::validate(), \
                  "封包大小驗證失敗")