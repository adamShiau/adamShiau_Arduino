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

// mip frame: [75 65][desc_set][payload_len][payload...][ck1 ck2]
bool hins_send_mip_raw(Stream& port_hins, const uint8_t* mip);


/*** usage: 
static const uint8_t HINS_HDR[] = {0x75,0x65,0x82,0x13,0x13,0x49};
uint8_t hins_payload[32];
bool ok = hins_read_stream_payload(
    g_cmd_port_hins,
    HINS_HDR,
    sizeof(HINS_HDR),
    17,                  // 協議定義的 payload_len
    hins_payload,
    sizeof(hins_payload),
    2                    // 2ms best-effort timeout
);
*/
bool hins_read_stream_payload(
    Stream& port_hins,
    const uint8_t* header,
    uint16_t header_len,
    uint16_t payload_len,
    uint8_t* out_payload,
    uint16_t out_cap,
    uint32_t timeout_ms
);



