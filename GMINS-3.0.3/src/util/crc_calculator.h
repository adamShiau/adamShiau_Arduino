#pragma once
#include <stdint.h>
#include <stddef.h>

class CRCCalculator {
private:
    static const uint32_t CRC32_POLYNOMIAL = 0xEDB88320;
    static uint32_t crc32_table[256];
    static bool table_initialized;

    void initTable() {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (size_t j = 0; j < 8; j++) {
                if (c & 1) c = CRC32_POLYNOMIAL ^ (c >> 1);
                else       c >>= 1;
            }
            crc32_table[i] = c;
        }
        table_initialized = true;
    }

public:
    CRCCalculator() {
        if (!table_initialized) initTable();
    }

    uint32_t calculateCRC32(const uint8_t* data, size_t length) {
        return updateCRC32(0xFFFFFFFF, data, length) ^ 0xFFFFFFFF;
    }

    uint32_t updateCRC32(uint32_t crc, const uint8_t* data, size_t length) {
        uint32_t c = crc;
        for (size_t i = 0; i < length; i++) {
            c = crc32_table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
        }
        return c;
    }
};