#pragma once
#include <Arduino.h>

// 封包類型定義
enum PacketType {
    PKT_HEADING = 0x01,         // 航向數據
    PKT_POSITION = 0x02,        // 位置數據
    PKT_POS_HEADING = 0x03,     // 位置+航向
    PKT_MOTION = 0x04,          // 運動數據
    PKT_SATELLITES = 0x05,      // 衛星數據
    PKT_TIME = 0x09,            // 時間數據
    PKT_ALL_GNSS = 0x10         // 完整GNSS數據
};

// GPS 數據狀態定義
enum DataStatus {
    DATA_ALL_VALID = 0x00,      // 位置和航向都有效
    DATA_POS_ONLY = 0x01,       // 僅位置有效，航向無效
    DATA_NO_FIX = 0x02,         // 無定位信號
    DATA_UNSTABLE = 0x03,       // 數據不穩定/驗證失敗
    DATA_INVALID = 0xFF         // 數據無效
};

// GNSS 數據結構
struct GnssData {
    bool valid;                 // 數據是否有效
    uint32_t timestamp;         // 接收時間戳
    uint8_t packet_type;        // 原始封包類型
    DataStatus status;          // GPS數據狀態

    // 位置資訊
    double latitude;            // 緯度 (degrees)
    double longitude;           // 經度 (degrees)
    float altitude;             // 高度 (meters)

    // 運動資訊
    float heading;              // 航向角 (degrees)
    float speed;                // 速度 (m/s)

    // 衛星資訊
    uint8_t satellites_used;    // 使用衛星數
    uint8_t satellites_visible; // 可見衛星數
    uint8_t fix_type;           // 定位類型
    float hdop;                 // 水平精度稀釋

    // 時間資訊 (高精度)
    uint8_t hour, minute, second;
    uint16_t millisecond;           // 毫秒 (0-999)
    uint8_t day, month;
    uint16_t year;
    bool time_valid;
};

// 封包解析器
class PacketParser {
public:
    // 解析封包並提取GNSS數據
    static bool parsePacket(uint8_t* buffer, uint16_t buffer_size, GnssData* result);

    // 驗證封包格式
    static bool validatePacket(uint8_t* buffer, uint16_t buffer_size);

private:
    // 內部解析函數
    static bool parseHeadingPacket(uint8_t* data, uint16_t size, GnssData* result);
    static bool parsePositionPacket(uint8_t* data, uint16_t size, GnssData* result);
    static bool parsePosHeadingPacket(uint8_t* data, uint16_t size, GnssData* result);
    static bool parseMotionPacket(uint8_t* data, uint16_t size, GnssData* result);
    static bool parseSatellitePacket(uint8_t* data, uint16_t size, GnssData* result);
    static bool parseTimePacket(uint8_t* data, uint16_t size, GnssData* result);

    // 輔助函數
    static float readFloat(uint8_t* buffer, uint16_t offset);
    static double readDouble(uint8_t* buffer, uint16_t offset);
    static uint16_t readUint16(uint8_t* buffer, uint16_t offset);
    static uint8_t calculateChecksum(uint8_t* data, uint16_t length);

    // 常數
    static const uint8_t HEADER_BYTE1 = 0xFA;
    static const uint8_t HEADER_BYTE2 = 0xFF;
    static const uint8_t HEADER_SIZE = 6;         // FA FF + CMD + STATUS + Size(2)
};