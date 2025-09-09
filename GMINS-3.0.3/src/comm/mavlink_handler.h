#pragma once

#include "../data/data_types.h"
#include <stdint.h>

class MAVLinkHandler {
public:
    MAVLinkHandler();
    ~MAVLinkHandler();
    
    bool initialize(uint8_t system_id, uint8_t component_id);
    
    size_t processIncomingData(const uint8_t* buffer, size_t length);
    
    size_t createHeartbeatMessage(uint8_t* buffer, size_t buffer_size);
    size_t createOdometryMessage(const NavigationState& nav_state, uint8_t* buffer, size_t buffer_size);
    size_t createGPSInputMessage(const GNSSData& gnss_data, uint8_t* buffer, size_t buffer_size);
    size_t createIMUMessage(const IMUData& imu_data, uint8_t* buffer, size_t buffer_size);
    
    void setHeartbeatInterval(uint32_t interval_ms) { heartbeat_interval_ = interval_ms; }
    bool shouldSendHeartbeat() const;
    
    struct MAVLinkStats {
        uint32_t messages_sent;
        uint32_t messages_received;
        uint32_t parse_errors;
        uint32_t crc_errors;
        uint32_t last_heartbeat;
    };
    
    MAVLinkStats getStats() const { return stats_; }
    
private:
    uint8_t system_id_;
    uint8_t component_id_;
    uint8_t message_sequence_;
    
    uint32_t heartbeat_interval_;
    uint32_t last_heartbeat_time_;
    
    MAVLinkStats stats_;
    
    uint16_t calculateCRC(const uint8_t* data, size_t length, uint16_t crc_extra);
    void updateStats(bool sent, bool error = false);
};