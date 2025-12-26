#include "packet_protocol.h"

// 寫入封包頭部 (新格式: FA FF [CMD][Status][Size][Payload][Checksum])
void PacketProtocol::writePacketHeader(uint8_t* buffer, uint16_t data_size, uint8_t cmd, uint8_t status) {
    buffer[0] = HEADER_BYTE1;           // 0xFA
    buffer[1] = HEADER_BYTE2;           // 0xFF
    buffer[2] = cmd;                    // Command
    buffer[3] = status;                 // Status (新增)
    writeUint16(buffer, 4, data_size);  // Size (2 bytes, little endian)
}

// 計算 XOR 校驗和
uint8_t PacketProtocol::calculateChecksum(uint8_t* data, uint16_t length) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// 寫入 float (little endian)
void PacketProtocol::writeFloat(uint8_t* buffer, uint16_t offset, float value) {
    union {
        float f;
        uint8_t b[4];
    } float_union;
    float_union.f = value;

    buffer[offset + 0] = float_union.b[0];
    buffer[offset + 1] = float_union.b[1];
    buffer[offset + 2] = float_union.b[2];
    buffer[offset + 3] = float_union.b[3];
}

// 寫入 double (little endian)
void PacketProtocol::writeDouble(uint8_t* buffer, uint16_t offset, double value) {
    union {
        double d;
        uint8_t b[8];
    } double_union;
    double_union.d = value;

    for (int i = 0; i < 8; i++) {
        buffer[offset + i] = double_union.b[i];
    }
}

// 寫入 uint16 (little endian)
void PacketProtocol::writeUint16(uint8_t* buffer, uint16_t offset, uint16_t value) {
    buffer[offset + 0] = (uint8_t)(value & 0xFF);
    buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

// 統一的封包建構器
bool PacketProtocol::createPacket(uint8_t packet_type, uint8_t* buffer, uint16_t* packet_size,
                                const uint8_t* payload_data, uint16_t payload_size, uint8_t status) {
    uint16_t data_size = payload_size + 1;  // payload + checksum

    writePacketHeader(buffer, data_size, packet_type, status);

    // 複製 payload 到 buffer
    for (uint16_t i = 0; i < payload_size; i++) {
        buffer[HEADER_SIZE + i] = payload_data[i];
    }

    // 計算校驗和 (CMD + Status + payload)
    uint8_t checksum = calculateChecksum(&buffer[2], 2 + payload_size);
    buffer[HEADER_SIZE + payload_size] = checksum;

    *packet_size = HEADER_SIZE + data_size;
    return true;
}

// 創建航向封包
bool PacketProtocol::createHeadingPacket(float heading, uint8_t* buffer, uint16_t* packet_size, uint8_t status) {
    uint8_t payload[4];
    writeFloat(payload, 0, heading);
    return createPacket(PKT_HEADING, buffer, packet_size, payload, 4, status);
}

// 創建位置封包 (包含UTC時間)
bool PacketProtocol::createPositionPacket(double lat, double lon, float alt, uint8_t* buffer, uint16_t* packet_size, uint8_t status) {
    // 取得當前GPS數據以獲得UTC時間
    extern NmeaParser gps_parser;
    GpsData* gps = gps_parser.getData();

    uint8_t payload[29];  // 8+8+4+9 bytes (lat+lon+alt+time+ms)
    writeDouble(payload, 0, lat);        // 緯度 8 bytes
    writeDouble(payload, 8, lon);        // 經度 8 bytes
    writeFloat(payload, 16, alt);        // 海拔 4 bytes

    // UTC 時間 9 bytes (高精度)
    payload[20] = gps->hour;             // 小時
    payload[21] = gps->minute;           // 分鐘
    payload[22] = gps->second;           // 秒
    writeUint16(payload, 23, gps->millisecond); // 毫秒 2 bytes
    payload[25] = gps->day;              // 日
    payload[26] = gps->month;            // 月
    writeUint16(payload, 27, gps->year); // 年 2 bytes

    return createPacket(PKT_POSITION, buffer, packet_size, payload, 29, status);
}

// 創建位置+航向封包
bool PacketProtocol::createPosHeadingPacket(double lat, double lon, float alt, float heading, uint8_t* buffer, uint16_t* packet_size, uint8_t status) {
    uint8_t payload[24];  // 8+8+4+4 bytes
    writeDouble(payload, 0, lat);
    writeDouble(payload, 8, lon);
    writeFloat(payload, 16, alt);
    writeFloat(payload, 20, heading);
    return createPacket(PKT_POS_HEADING, buffer, packet_size, payload, 24, status);
}

// 創建運動封包
bool PacketProtocol::createMotionPacket(float speed, float heading, uint8_t* buffer, uint16_t* packet_size, uint8_t status) {
    uint8_t payload[8];  // 4+4 bytes
    writeFloat(payload, 0, speed);
    writeFloat(payload, 4, heading);
    return createPacket(PKT_MOTION, buffer, packet_size, payload, 8, status);
}

// 創建衛星封包
bool PacketProtocol::createSatellitePacket(uint8_t used, uint8_t visible, float hdop, uint8_t fix_type, uint8_t* buffer, uint16_t* packet_size, uint8_t status) {
    uint8_t payload[7];  // 1+1+4+1 bytes
    payload[0] = used;
    payload[1] = visible;
    writeFloat(payload, 2, hdop);
    payload[6] = fix_type;
    return createPacket(PKT_SATELLITES, buffer, packet_size, payload, 7, status);
}

// 創建 CNO 封包
bool PacketProtocol::createCnoPacket(uint8_t packet_type, SatelliteSystem* sat_sys, uint8_t* buffer, uint16_t* packet_size, uint8_t status) {
    uint8_t payload[26];  // 1+1+12+12 bytes
    payload[0] = sat_sys->sat_count;
    payload[1] = sat_sys->valid_sats;

    for (int i = 0; i < 12; i++) {
        payload[2 + i] = sat_sys->cno_values[i];
        payload[14 + i] = sat_sys->prn_numbers[i];
    }
    return createPacket(packet_type, buffer, packet_size, payload, 26, status);
}

// 創建時間封包
bool PacketProtocol::createTimePacket(uint8_t hour, uint8_t min, uint8_t sec, uint8_t day, uint8_t month, uint16_t year, uint8_t* buffer, uint16_t* packet_size, uint8_t status) {
    uint8_t payload[7];  // 3+2+2 bytes
    payload[0] = hour;
    payload[1] = min;
    payload[2] = sec;
    payload[3] = day;
    payload[4] = month;
    writeUint16(payload, 5, year);
    return createPacket(PKT_TIME, buffer, packet_size, payload, 7, status);
}

// 創建完整 GNSS 封包
bool PacketProtocol::createAllGnssPacket(GpsData* gps, uint8_t* buffer, uint16_t* packet_size) {
    uint8_t payload[122];  // 預計最大 payload 大小
    uint16_t offset = 0;

    // 位置資料 (8+8+4=20 bytes)
    writeDouble(payload, offset, gps->latitude);
    offset += 8;
    writeDouble(payload, offset, gps->longitude);
    offset += 8;
    writeFloat(payload, offset, gps->altitude);
    offset += 4;

    // 定位品質 (1+1+1+4=7 bytes)
    payload[offset++] = gps->fix_type;
    payload[offset++] = gps->satellites_used;
    payload[offset++] = gps->satellites_visible;
    writeFloat(payload, offset, gps->hdop);
    offset += 4;

    // 運動資料 (4+4=8 bytes)
    writeFloat(payload, offset, gps->speed);
    offset += 4;
    writeFloat(payload, offset, gps->heading);
    offset += 4;

    // 衛星系統資料 (3 * 26 = 78 bytes)
    for (int sys = 0; sys < SAT_SYSTEM_COUNT; sys++) {
        payload[offset++] = gps->sat_systems[sys].sat_count;
        payload[offset++] = gps->sat_systems[sys].valid_sats;

        for (int i = 0; i < 12; i++) {
            payload[offset++] = gps->sat_systems[sys].cno_values[i];
        }
        for (int i = 0; i < 12; i++) {
            payload[offset++] = gps->sat_systems[sys].prn_numbers[i];
        }
    }

    // 時間資料 (3+2+2=7 bytes)
    payload[offset++] = gps->hour;
    payload[offset++] = gps->minute;
    payload[offset++] = gps->second;
    payload[offset++] = gps->day;
    payload[offset++] = gps->month;
    writeUint16(payload, offset, gps->year);
    offset += 2;

    // 狀態標記 (1+1=2 bytes)
    payload[offset++] = gps->valid_position ? 1 : 0;
    payload[offset++] = gps->valid_time ? 1 : 0;

    uint8_t gps_status = determineGpsStatus(gps);
    return createPacket(PKT_ALL_GNSS, buffer, packet_size, payload, offset, gps_status);
}

// 封包驗證
bool PacketProtocol::validatePacket(uint8_t* buffer, uint16_t buffer_size) {
    if (buffer_size < HEADER_SIZE + 1) return false;  // 至少要有頭部 + checksum
    if (buffer[0] != HEADER_BYTE1 || buffer[1] != HEADER_BYTE2) return false;

    uint16_t data_size = (buffer[5] << 8) | buffer[4];  // little endian
    if (buffer_size != HEADER_SIZE + data_size) return false;

    uint8_t expected_checksum = calculateChecksum(&buffer[2], 2 + data_size - 1);
    uint8_t actual_checksum = buffer[buffer_size - 1];

    return expected_checksum == actual_checksum;
}

// 取得封包類型
PacketType PacketProtocol::getPacketType(uint8_t* buffer) {
    return (PacketType)buffer[2];
}

// 取得封包數據大小
uint16_t PacketProtocol::getPacketDataSize(uint8_t* buffer) {
    return (buffer[5] << 8) | buffer[4];  // little endian
}

// GPS 資料驗證函數
bool isGpsDataValid(GpsData* gps) {
    // 檢查GPS數據指標有效性
    if (gps == nullptr) {
        Serial.println("❌ GPS驗證失敗: 資料指標無效");
        return false;
    }

    // 檢查座標範圍
    if (gps->latitude < -90.0 || gps->latitude > 90.0) {
        Serial.print("❌ GPS驗證失敗: 緯度超出範圍 (");
        Serial.print(gps->latitude);
        Serial.println("°)");
        return false;
    }
    if (gps->longitude < -180.0 || gps->longitude > 180.0) {
        Serial.print("❌ GPS驗證失敗: 經度超出範圍 (");
        Serial.print(gps->longitude);
        Serial.println("°)");
        return false;
    }

    // 檢查座標是否為零點 (避免無效數據)
    if (gps->latitude == 0.0 && gps->longitude == 0.0) {
        Serial.println("❌ GPS驗證失敗: 無效的零點座標 (0.0, 0.0)");
        return false;
    }

    // 檢查衛星數量 (至少8顆)
    if (gps->satellites_used < 8) {
        Serial.print("❌ GPS驗證失敗: 衛星數量不足 (");
        Serial.print(gps->satellites_used);
        Serial.println("/8)");
        return false;
    }

    // 檢查海拔合理性 (-500m ~ 10000m)
    if (gps->altitude < -500.0 || gps->altitude > 10000.0) {
        Serial.print("❌ GPS驗證失敗: 海拔超出合理範圍 (");
        Serial.print(gps->altitude);
        Serial.println("m)");
        return false;
    }

    // 檢查時間格式有效性
    if (gps->hour > 23 || gps->minute > 59 || gps->second > 59) {
        Serial.print("❌ GPS驗證失敗: 時間格式異常 (");
        Serial.print(gps->hour);
        Serial.print(":");
        Serial.print(gps->minute);
        Serial.print(":");
        Serial.print(gps->second);
        Serial.println(")");
        return false;
    }

    // 檢查日期格式有效性
    if (gps->month > 12 || gps->month == 0 || gps->day > 31 || gps->day == 0) {
        Serial.print("❌ GPS驗證失敗: 日期格式異常 (");
        Serial.print(gps->day);
        Serial.print("/");
        Serial.print(gps->month);
        Serial.println(")");
        return false;
    }

    return true;  // 資料通過驗證
}

// 根據 GPS 數據狀態決定封包狀態
uint8_t PacketProtocol::determineGpsStatus(GpsData* gps) {
    // 🛡️ 首先進行資料驗證
    if (!isGpsDataValid(gps)) {
        Serial.println("❌ GPS資料驗證失敗，狀態設為DATA_UNSTABLE");
        return DATA_UNSTABLE;  // 0x03 - 資料異常，數據不穩定
    }

    // 🔍 檢查海拔濾波狀態
    if (gps->fix_type >= GPS_3D_FIX && !gps->altitude_stable) {
        Serial.println("❌ 海拔濾波失敗，狀態設為DATA_UNSTABLE");
        return DATA_UNSTABLE;  // 0x03 - 海拔異常，數據不穩定
    }

    // 檢查是否有定位信號
    if (gps->fix_type == GPS_NO_FIX) {
        return DATA_NO_FIX;
    }

    // 檢查位置和航向有效性
    bool has_position = gps->valid_position && (gps->fix_type >= GPS_2D_FIX);
    bool has_heading = (gps->heading > 0.0 && gps->heading <= 360.0);

    if (has_position && has_heading) {
        return DATA_ALL_VALID;
    } else if (has_position && !has_heading) {
        return DATA_POS_ONLY;
    } else {
        return DATA_INVALID;
    }
}