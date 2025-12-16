#pragma once
#include <Arduino.h>
#include "nmea_parser.h"

// 封包類型定義
enum PacketType {
    PKT_HEADING = 0x01,         // 航向數據
    PKT_POSITION = 0x02,        // 位置數據 (lat, lon, alt)
    PKT_POS_HEADING = 0x03,     // 位置+高度+航向 (lat, lon, alt, heading)
    PKT_MOTION = 0x04,          // 運動數據 (speed, heading)
    PKT_SATELLITES = 0x05,      // 衛星數據 (count, hdop, fix)
    PKT_CNO_GPS = 0x06,         // GPS CNO 數據
    PKT_CNO_GLONASS = 0x07,     // GLONASS CNO 數據
    PKT_CNO_GALILEO = 0x08,     // Galileo CNO 數據
    PKT_TIME = 0x09,            // 時間數據
    PKT_ALL_GNSS = 0x10         // 完整 GNSS 數據包
};

// GPS 狀態碼定義
enum DataStatus {
    DATA_ALL_VALID = 0x00,      // 位置和航向都有效
    DATA_POS_ONLY = 0x01,       // 僅位置有效，航向無效
    DATA_NO_FIX = 0x02,         // 無定位信號
    DATA_UNSTABLE = 0x03,       // 數據不穩定/驗證失敗
    DATA_INVALID = 0xFF         // 數據無效
};

// 封包協議類
class PacketProtocol {
public:
    // 創建各種類型的封包
    static bool createHeadingPacket(float heading, uint8_t* buffer, uint16_t* packet_size, uint8_t status = DATA_ALL_VALID);
    static bool createPositionPacket(double lat, double lon, float alt, uint8_t* buffer, uint16_t* packet_size, uint8_t status = DATA_ALL_VALID);
    static bool createPosHeadingPacket(double lat, double lon, float alt, float heading, uint8_t* buffer, uint16_t* packet_size, uint8_t status = DATA_ALL_VALID);
    static bool createMotionPacket(float speed, float heading, uint8_t* buffer, uint16_t* packet_size, uint8_t status = DATA_ALL_VALID);
    static bool createSatellitePacket(uint8_t used, uint8_t visible, float hdop, uint8_t fix_type, uint8_t* buffer, uint16_t* packet_size, uint8_t status = DATA_ALL_VALID);
    static bool createCnoPacket(uint8_t packet_type, SatelliteSystem* sat_sys, uint8_t* buffer, uint16_t* packet_size, uint8_t status = DATA_ALL_VALID);
    static bool createTimePacket(uint8_t hour, uint8_t min, uint8_t sec, uint8_t day, uint8_t month, uint16_t year, uint8_t* buffer, uint16_t* packet_size, uint8_t status = DATA_ALL_VALID);
    static bool createAllGnssPacket(GpsData* gps, uint8_t* buffer, uint16_t* packet_size);

    // 封包格式驗證
    static bool validatePacket(uint8_t* buffer, uint16_t buffer_size);
    static PacketType getPacketType(uint8_t* buffer);
    static uint16_t getPacketDataSize(uint8_t* buffer);

    // GPS 狀態評估
    static uint8_t determineGpsStatus(GpsData* gps);

    // 常數定義
    static const uint8_t HEADER_BYTE1 = 0xFA;
    static const uint8_t HEADER_BYTE2 = 0xFF;
    static const uint16_t MAX_PACKET_SIZE = 512;  // 最大封包大小
    static const uint8_t HEADER_SIZE = 6;         // FA FF + CMD + Status + Size(2)

private:
    // 內部輔助函數
    static void writePacketHeader(uint8_t* buffer, uint16_t data_size, uint8_t cmd, uint8_t status = DATA_ALL_VALID);
    static uint8_t calculateChecksum(uint8_t* data, uint16_t length);
    static void writeFloat(uint8_t* buffer, uint16_t offset, float value);
    static void writeDouble(uint8_t* buffer, uint16_t offset, double value);
    static void writeUint16(uint8_t* buffer, uint16_t offset, uint16_t value);

    // 統一的封包建構器
    static bool createPacket(uint8_t packet_type, uint8_t* buffer, uint16_t* packet_size,
                           const uint8_t* payload_data, uint16_t payload_size, uint8_t status = DATA_ALL_VALID);
};