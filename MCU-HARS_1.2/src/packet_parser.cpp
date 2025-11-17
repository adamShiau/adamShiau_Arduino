#include "packet_parser.h"

// 驗證封包格式 (新格式: [Header] [CMD][status][Size] [Payload] [Checksum])
bool PacketParser::validatePacket(uint8_t* buffer, uint16_t buffer_size) {
    if (buffer_size < HEADER_SIZE + 1) return false;
    if (buffer[0] != HEADER_BYTE1 || buffer[1] != HEADER_BYTE2) return false;

    uint16_t data_size = readUint16(buffer, 4);  // Size 位於位置4-5
    if (buffer_size != HEADER_SIZE + data_size) return false;

    uint16_t payload_size = data_size - 1;
    uint8_t expected_checksum = calculateChecksum(&buffer[2], 2 + payload_size);  // CMD + status + payload
    uint8_t actual_checksum = buffer[buffer_size - 1];

    return expected_checksum == actual_checksum;
}

// 解析封包並提取GNSS數據
bool PacketParser::parsePacket(uint8_t* buffer, uint16_t buffer_size, GnssData* result) {
    // 初始化結果
    result->valid = false;
    result->timestamp = millis();
    result->time_valid = false;

    // 驗證封包
    if (!validatePacket(buffer, buffer_size)) {
        return false;
    }

    // 提取基本資訊 (新格式: [Header] [CMD][status][Size] [Payload] [Checksum])
    result->packet_type = buffer[2];        // CMD 位於位置2
    result->status = (DataStatus)buffer[3]; // STATUS 位於位置3
    uint16_t data_size = readUint16(buffer, 4);  // Size 位於位置4-5
    uint8_t* payload = &buffer[6];          // Payload 從位置6開始
    uint16_t payload_size = data_size - 1;  // data_size = payload + checksum

    // 根據封包類型進行解析
    bool parse_success = false;
    switch (result->packet_type) {
        case PKT_HEADING:
            parse_success = parseHeadingPacket(payload, payload_size, result);
            break;
        case PKT_POSITION:
            parse_success = parsePositionPacket(payload, payload_size, result);
            break;
        case PKT_POS_HEADING:
            parse_success = parsePosHeadingPacket(payload, payload_size, result);
            break;
        case PKT_MOTION:
            parse_success = parseMotionPacket(payload, payload_size, result);
            break;
        case PKT_SATELLITES:
            parse_success = parseSatellitePacket(payload, payload_size, result);
            break;
        case PKT_TIME:
            parse_success = parseTimePacket(payload, payload_size, result);
            break;
        default:
            return false;
    }

    if (parse_success) {
        result->valid = true;

        // 顯示解析出的數據
        Serial.print("Parsed packet type: 0x");
        Serial.print(result->packet_type, HEX);
        Serial.print(" | Status: ");

        switch (result->status) {
            case DATA_ALL_VALID:
                Serial.print("ALL_VALID");
                break;
            case DATA_POS_ONLY:
                Serial.print("POS_ONLY");
                break;
            case DATA_NO_FIX:
                Serial.print("NO_FIX");
                break;
            case DATA_NO_SIGNAL:
                Serial.print("NO_SIGNAL");
                break;
            case DATA_INVALID:
                Serial.print("INVALID");
                break;
            default:
                Serial.print("UNKNOWN");
                break;
        }

        switch (result->packet_type) {
            case PKT_HEADING:
                Serial.print(" | Heading: ");
                Serial.print(result->heading);
                Serial.println("°");
                break;

            case PKT_POSITION:
                Serial.print(" | Lat: ");
                Serial.print(result->latitude, 6);
                Serial.print(" Lon: ");
                Serial.print(result->longitude, 6);
                Serial.print(" Alt: ");
                Serial.print(result->altitude);
                Serial.println("m");
                break;

            case PKT_POS_HEADING:
                Serial.print(" | Lat: ");
                Serial.print(result->latitude, 6);
                Serial.print(" Lon: ");
                Serial.print(result->longitude, 6);
                Serial.print(" Alt: ");
                Serial.print(result->altitude);
                Serial.print("m Heading: ");
                Serial.print(result->heading);
                Serial.println("°");
                break;

            case PKT_MOTION:
                Serial.print(" | Speed: ");
                Serial.print(result->speed);
                Serial.print("m/s Heading: ");
                Serial.print(result->heading);
                Serial.println("°");
                break;

            case PKT_SATELLITES:
                Serial.print(" | Sats Used: ");
                Serial.print(result->satellites_used);
                Serial.print(" Visible: ");
                Serial.print(result->satellites_visible);
                Serial.print(" HDOP: ");
                Serial.print(result->hdop);
                Serial.print(" Fix: ");
                Serial.println(result->fix_type);
                break;

            case PKT_TIME:
                Serial.print(" | Time: ");
                if (result->hour < 10) Serial.print("0");
                Serial.print(result->hour);
                Serial.print(":");
                if (result->minute < 10) Serial.print("0");
                Serial.print(result->minute);
                Serial.print(":");
                if (result->second < 10) Serial.print("0");
                Serial.print(result->second);
                Serial.print(" Date: ");
                if (result->day < 10) Serial.print("0");
                Serial.print(result->day);
                Serial.print("/");
                if (result->month < 10) Serial.print("0");
                Serial.print(result->month);
                Serial.print("/");
                Serial.println(result->year);
                break;

            default:
                Serial.println();
                break;
        }
    }

    return parse_success;
}

// 解析 HEADING 封包
bool PacketParser::parseHeadingPacket(uint8_t* data, uint16_t size, GnssData* result) {
    if (size != 4) return false;

    result->heading = readFloat(data, 0);
    return true;
}

// 解析 POSITION 封包
bool PacketParser::parsePositionPacket(uint8_t* data, uint16_t size, GnssData* result) {
    if (size != 20) return false;

    result->latitude = readDouble(data, 0);
    result->longitude = readDouble(data, 8);
    result->altitude = readFloat(data, 16);
    return true;
}

// 解析 POS_HEADING 封包
bool PacketParser::parsePosHeadingPacket(uint8_t* data, uint16_t size, GnssData* result) {
    if (size != 24) return false;

    result->latitude = readDouble(data, 0);
    result->longitude = readDouble(data, 8);
    result->altitude = readFloat(data, 16);
    result->heading = readFloat(data, 20);

    return true;
}

// 解析 MOTION 封包
bool PacketParser::parseMotionPacket(uint8_t* data, uint16_t size, GnssData* result) {
    if (size != 8) return false;

    result->speed = readFloat(data, 0);
    result->heading = readFloat(data, 4);
    return true;
}

// 解析 SATELLITES 封包
bool PacketParser::parseSatellitePacket(uint8_t* data, uint16_t size, GnssData* result) {
    if (size != 7) return false;

    result->satellites_used = data[0];
    result->satellites_visible = data[1];
    result->hdop = readFloat(data, 2);
    result->fix_type = data[6];
    return true;
}

// 解析 TIME 封包
bool PacketParser::parseTimePacket(uint8_t* data, uint16_t size, GnssData* result) {
    if (size != 7) return false;

    result->hour = data[0];
    result->minute = data[1];
    result->second = data[2];
    result->day = data[3];
    result->month = data[4];
    result->year = readUint16(data, 5);
    result->time_valid = true;
    return true;
}

// 輔助函數實現
float PacketParser::readFloat(uint8_t* buffer, uint16_t offset) {
    union {
        float f;
        uint8_t b[4];
    } float_union;

    for (int i = 0; i < 4; i++) {
        float_union.b[i] = buffer[offset + i];
    }
    return float_union.f;
}

double PacketParser::readDouble(uint8_t* buffer, uint16_t offset) {
    union {
        double d;
        uint8_t b[8];
    } double_union;

    for (int i = 0; i < 8; i++) {
        double_union.b[i] = buffer[offset + i];
    }
    return double_union.d;
}

uint16_t PacketParser::readUint16(uint8_t* buffer, uint16_t offset) {
    return buffer[offset] | (buffer[offset + 1] << 8);
}

uint8_t PacketParser::calculateChecksum(uint8_t* data, uint16_t length) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}