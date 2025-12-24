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
size_t sendCmd(Print& port,
               const uint8_t header[2],
               const uint8_t trailer[2],
               uint8_t cmd,
               int32_t value,
               uint8_t ch);
