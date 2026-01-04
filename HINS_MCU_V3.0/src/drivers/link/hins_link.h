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

// 通用 MIP transact：送一包 MIP command，抓 ACK(0xF1)；若是 Read，額外抓一個 response field。
// out_resp_data 存的是「field data」（不含 field_len/field_desc）。
Status hins_mip_transact(Stream& port_hins,
                         const uint8_t* tx, uint16_t tx_len,
                         uint32_t timeout_ms,
                         uint8_t* out_desc_set,
                         uint8_t* out_cmd_desc,
                         uint8_t* out_ack_code,
                         uint8_t* out_ack_echo,
                         uint8_t* out_resp_desc,
                         uint8_t* out_resp_data, uint16_t resp_cap,
                         uint16_t* out_resp_len);


