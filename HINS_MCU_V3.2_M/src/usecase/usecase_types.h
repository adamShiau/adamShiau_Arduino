#pragma once
#include <stdint.h>

enum class Status : uint8_t {
  OK = 0,
  ERR = 1,
  BAD_PARAM = 2,
  TIMEOUT = 3,
  NOT_SUPPORTED = 4,
  BUSY = 5,
};

enum class IoRoute : uint8_t {
  NONE = 0,
  FPGA = 1,
  HINS  = 2,
};

struct TransactionSpec {
  bool expect_response = false;   // false: write-only / true: dump/query
  IoRoute route = IoRoute::NONE;
  uint16_t timeout_ms = 0;
  uint8_t max_retry = 0;
};

struct UsecaseResult {
  uint8_t cmd_id = 0;
  Status  status = Status::OK;
  uint16_t error_code = 0;

  // payload A：先不用
  const uint8_t* payload = nullptr;
  uint16_t payload_len = 0;
};
