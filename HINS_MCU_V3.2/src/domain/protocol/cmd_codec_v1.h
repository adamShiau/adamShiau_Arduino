#pragma once
#include <stdint.h>
#include "../../common.h"  // for cmd_ctrl_t, rx_condition_t, be_bytes_to_i32, etc.

/**
 * @brief Decode command payload (V1) from raw bytes into cmd_ctrl_t.
 *
 * Input buffer layout (as used today):
 *   data[0] : rx_condition_t (e.g., RX_CONDITION_ABBA_5556 / RX_CONDITION_EFFE_5354)
 *   data[1] : cmd
 *   data[2..5] : value (big-endian int32)
 *   data[6] : ch
 *
 * This function does NOT perform UART I/O. It only decodes an already-received buffer.
 */
void decode_cmd_v1(uint8_t* data, cmd_ctrl_t* rx);
