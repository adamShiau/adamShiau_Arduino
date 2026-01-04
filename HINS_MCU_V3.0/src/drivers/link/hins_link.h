#pragma once
#include <Arduino.h>
#include "../../usecase/usecase_types.h"

// 讀到 ACK(0x00) -> OK
// 讀到 NACK(!=0) -> FAIL（你可視需求改成 BAD_PARAM / FAIL）
// timeout -> TIMEOUT
UsecaseResult hins_send_and_wait_ack_base(Stream& port_hins,
                                          const uint8_t* tx, uint16_t tx_len,
                                          uint32_t timeout_ms,
                                          uint8_t* out_ack_code = nullptr);
