#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Send a command frame to Nios2/UART device.
 *
 * Packet layout:
 *   [header(2)] [cmd(1)] [value(4, big-endian)] [ch(1)] [trailer(2)]
 *
 * @return number of bytes written (expected 10)
 */
size_t sendCmd(Stream& port,
               const uint8_t header[2],
               const uint8_t trailer[2],
               uint8_t cmd,
               int32_t value,
               uint8_t ch);


bool sendSN(Stream& port,
            const uint8_t header[2],
            const uint8_t trailer[2],
            uint8_t cmd,
            const uint8_t sn_ascii[12]); // 12 bytes

