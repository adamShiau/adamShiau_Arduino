#pragma once
#include <stdint.h>
#include <stddef.h>
#warning "crc32.h included"

/**
 * @brief Initialize the CRC32 lookup table.
 *
 * Call once at startup (e.g., in setup()) before gen_crc32().
 * Safe to call multiple times.
 */
void crc32_init_table(void);

/**
 * @brief Generate CRC32 for KVH header + payload.
 *
 * CRC is computed over:
 *   - header: 4 bytes (KVH header, excluding the first 0xFA byte in your packet framing if applicable)
 *   - payload: payload_len bytes
 *
 * @param header      Pointer to 4-byte header array
 * @param payload     Pointer to payload bytes
 * @param payload_len Payload length in bytes
 * @param crc_out     Pointer to 4-byte array for big-endian CRC output
 */
void gen_crc32(const uint8_t* header,
               const uint8_t* payload,
               size_t payload_len,
               uint8_t* crc_out);

