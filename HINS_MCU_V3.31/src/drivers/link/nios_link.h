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

/**
 * @brief Convert datarate index (0~4) to FPGA sync counter value and send CMD_SYNC_CNT.
 *
 * Mapping:
 *   0 -> 10Hz  -> 5,000,000
 *   1 -> 50Hz  -> 1,000,000
 *   2 -> 100Hz ->   500,000
 *   3 -> 200Hz ->   250,000
 *   4 -> 400Hz ->   125,000
 *
 * @param port UART stream to Nios/FPGA
 * @param cmd_sync_cnt command id for "sync counter" (e.g., CMD_SYNC_CNT)
 * @param dr_index datarate index from PC
 * @return true if index valid and command sent; false if index out of range
 */
bool nios_send_cfg_datarate(Stream& port, uint8_t cmd_sync_cnt, uint8_t dr_index);

