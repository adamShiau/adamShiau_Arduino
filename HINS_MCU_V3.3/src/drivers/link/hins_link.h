#pragma once
#include <Arduino.h>
#include "../../usecase/usecase_types.h"
#include "../../common.h"

#define HINS_MAX_PAYLOAD_SIZE 256

typedef enum {
    HINS_RD_FIND_HEADER = 0,
    HINS_RD_READ_PAYLOAD,
    HINS_RD_CHECK_CHECKSUM
} HINS_RD_State;

typedef struct {
    HINS_RD_State state;
    uint16_t hdr_idx;
    uint16_t pay_idx;
    uint16_t chk_idx;
    uint16_t datalen;
    uint8_t  payload[HINS_MAX_PAYLOAD_SIZE]; // 這裡改用 local 定義
    uint8_t  checksum[2];
} HINS_RD_Ctx;

// #pragma pack(push, 1)
typedef struct __attribute__((packed)) {
    uint8_t     header[2];      // 固定 0xEB, 0x90
    uint8_t     fix_type;       // 1 byte
    uint16_t    status_flag;    // 2 bytes bit 0: rcv_1, bit 1: rcv_2, bit 2: ant_offset
    uint16_t    valid_flag_da;  // 2 bytes
    float       heading_da;     // 4 bytes
    float       imu_heading;    // 4 bytes
    float       g_heading_offset; // 4 bytes
    double      gps_tow;        // 8 bytes
    uint16_t    filter_state;  // 2 bytes
    uint16_t    dynamic_mode;   // 2 bytes
    uint16_t    status_flag_82; // 2 bytes
    uint8_t     case_flag;       // 1 byte
    uint8_t     ck[2];          // 2 bytes Fletcher-16
} gui_monitor_t;
// #pragma pack(pop)

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




uint8_t* hins_parse_stream_bytewise(Stream& port, const uint8_t* header, uint8_t header_len, uint16_t payload_len);


void hins_true_heading_standard(
    Stream& port_hins,
    const true_heading_t* th
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


void hins_send_gui_monitor(Stream& port_gui, const gui_monitor_t* mon);
