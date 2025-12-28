#include "crc32.h"

// This polynomial matches the one used in your original common.h
// (standard CRC-32 / CRC-32/MPEG-2 style polynomial value: 0x04C11DB7)
#define POLYNOMIAL_32 0x04C11DB7u

// Keep CRC table private to this module (no global symbol leakage)
static uint32_t crc_table[256];
static bool crc_inited = false;

void crc32_init_table(void)
{
    if (crc_inited) return;

    for (int i = 0; i < 256; ++i) {
        uint32_t remainder = (uint32_t)i << 24;
        for (int bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80000000u) {
                remainder = (remainder << 1) ^ POLYNOMIAL_32;
            } else {
                remainder = (remainder << 1);
            }
        }
        crc_table[i] = remainder;
    }
    crc_inited = true;
}

void gen_crc32(const uint8_t* header,
               const uint8_t* payload,
               size_t payload_len,
               uint8_t* crc_out)
{
    // Ensure table exists even if caller forgot to init
    if (!crc_inited) crc32_init_table();

    uint32_t remainder = 0xFFFFFFFFu;

    // header (固定 4B)
    for (int i = 0; i < 4; i++) {
        uint8_t index = (uint8_t)((remainder >> 24) ^ header[i]);
        remainder = (remainder << 8) ^ crc_table[index];
    }

    // payload
    for (size_t i = 0; i < payload_len; i++) {
        uint8_t index = (uint8_t)((remainder >> 24) ^ payload[i]);
        remainder = (remainder << 8) ^ crc_table[index];
    }

    // 輸出 big-endian
    crc_out[0] = (uint8_t)((remainder >> 24) & 0xFF);
    crc_out[1] = (uint8_t)((remainder >> 16) & 0xFF);
    crc_out[2] = (uint8_t)((remainder >>  8) & 0xFF);
    crc_out[3] = (uint8_t)((remainder      ) & 0xFF);
}
