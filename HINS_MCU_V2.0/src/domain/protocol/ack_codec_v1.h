#pragma once
#include <Arduino.h>
#include <stdint.h>

// 你已經有 Status enum 的話就 include 你那個檔
// #include "../../usecase/usecase_types.h"

enum class AckStatus : uint8_t {
  OK = 0,
  ERR = 1,
  BAD_PARAM = 2,
  TIMEOUT = 3,
  NOT_SUPPORTED = 4,
  BUSY = 5,
};

// 只提供送出 API：router 不需要知道封包細節
bool send_ack_v1(Print& pc_port, uint8_t cmd_id, AckStatus st);
bool send_result_v1(Print& pc_port, uint8_t cmd_id, AckStatus st);

// 之後 payload B 要加資料時，再擴充這個，不用動 cmd_dispatch
bool send_result_v1(Print& pc_port,
                    uint8_t cmd_id,
                    AckStatus st,
                    const uint8_t* payload,
                    uint16_t payload_len);
