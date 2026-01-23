#pragma once
#include <Arduino.h>
#include "../../usecase/usecase_types.h"
#include "../../common.h"

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


Status hins_true_heading_transact_u32ns(
    Stream& port_hins,
    uint32_t timeout_ms,
    const true_heading_t* th,
    uint8_t* out_ack_code,
    uint8_t* out_ack_echo
);

Status hins_true_heading_transact_u64ns(
    Stream& port_hins,
    uint32_t timeout_ms,
    const true_heading_t* th,
    uint8_t* out_ack_code,
    uint8_t* out_ack_echo
);


/**
 * @brief 從串列埠抓取一個完整的 MIP 原始封包 (含 Header 與 Checksum)
 * @return Status::OK 成功, Status::TIMEOUT 超時, Status::BAD_PARAM 格式錯誤
 */
Status hins_capture_raw_mip(Stream& port_hins, 
                            uint8_t* out_buf, uint16_t buf_cap, 
                            uint16_t* out_len, uint32_t timeout_ms);

/**
 * @brief 抓取 MIP 數據包（自動跳過 ACK 包）
 * @param ignore_set 指定要跳過的 Descriptor Set (例如 0x0C 或 0x01)
 * @param timeout_ms 總等待超時時間
 */
Status hins_capture_mip_data(Stream& port_hins, 
                             uint8_t* out_buf, uint16_t buf_cap, 
                             uint16_t* out_len, 
                             uint8_t ignore_set,
                             uint32_t timeout_ms);



