#pragma once

#include "../data/data_types.h"
#include <stdint.h>

enum class XBusMessageType {
    CONFIG = 0x30,
    OUTPUT_CONFIG = 0x32,
    DATA_PACKET = 0x36,
    ERROR = 0x42,
    WARNING = 0x43
};

struct XBusPacket {
    uint8_t preamble;
    uint8_t bus_id;
    uint8_t message_id;
    uint8_t length;
    uint8_t* data;
    uint8_t checksum;
};

class XBusHandler {
public:
    XBusHandler();
    ~XBusHandler();
    
    bool initialize();
    
    size_t processIncomingData(const uint8_t* buffer, size_t length);
    bool parseIMUData(const XBusPacket& packet, IMUData& imu_data);
    
    size_t createConfigMessage(uint8_t* buffer, size_t buffer_size);
    
    struct XBusStats {
        uint32_t packets_processed;
        uint32_t parse_errors;
        uint32_t checksum_errors;
        uint32_t data_packets;
        uint32_t config_packets;
        uint32_t error_packets;
    };
    
    XBusStats getStats() const { return stats_; }
    
    void setIMUCallback(void (*callback)(const IMUData&));
    
private:
    uint8_t receive_buffer_[256];
    size_t buffer_index_;
    bool packet_in_progress_;
    
    XBusStats stats_;
    void (*imu_callback_)(const IMUData&);
    
    bool validatePacket(const XBusPacket& packet);
    uint8_t calculateChecksum(const uint8_t* data, size_t length);
    bool extractQuaternion(const uint8_t* data, float& w, float& x, float& y, float& z);
    bool extractAcceleration(const uint8_t* data, float& ax, float& ay, float& az);
    bool extractAngularVelocity(const uint8_t* data, float& gx, float& gy, float& gz);
};