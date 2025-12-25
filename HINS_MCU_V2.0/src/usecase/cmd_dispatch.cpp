#include "cmd_dispatch.h"
#include <Arduino.h>
#include "../domain/protocol/ack_codec_v1.h"   // send_ack_v1 / send_result_v1
#include "parameter_service.h"      // parameter_service_handle_ex
#include "../output_mode_setting.h"         // output_mode_setting
#include "../common.h"                      // cmd_mux(), fog_parameter() etc.

static constexpr uint8_t CMD_TEST_ACK_ONLY   = 0xF0;
static constexpr uint8_t CMD_TEST_ACK_RESULT = 0xF1;
static constexpr uint8_t CMD_TEST_TIMEOUT    = 0xF2; // 回 ACK + RESULT(TIMEOUT)



// -----------------------------
// 1) CommandSpec lookup (TransactionSpec)
// -----------------------------
static inline TransactionSpec get_command_spec(uint8_t cmd_id)
{
  TransactionSpec spec; // default: expect_response=false, route=NONE...

  switch (cmd_id) {
    case CMD_DUMP_FOG:
    case CMD_DUMP_MIS:
    case CMD_DUMP_SN:
      spec.expect_response = true;
      spec.route = IoRoute::FPGA;
      spec.timeout_ms = 300;
      spec.max_retry = 0;
      break;

    case CMD_TEST_ACK_RESULT:
      // 測試 gating：需要 RESULT，但不做任何 I/O
      spec.expect_response = true;
      spec.route = IoRoute::NONE;
      spec.timeout_ms = 0;
      spec.max_retry = 0;
      break;

    default:
      break;
  }
  return spec;
}

// -----------------------------
// 2) Port selection
// -----------------------------
// PC 回覆 port：建議用 Serial（若你已用 app_state 封裝 g_cmd_port，可以換成 g_cmd_port）
static inline Print& pc_port()
{
  return Serial; // <-- 若你要走 g_cmd_port，改成：return g_cmd_port;
}

// FPGA link port：你目前與 FPGA 溝通看起來是 Serial1（sendCmd(Serial1, ...)）
static inline Print& fpga_port()
{
  return Serial1; // <-- 若你 FPGA port 不同，改這裡即可
}

static inline Print& output_port()
{
  return Serial2; // <-- 若你 FPGA port 不同，改這裡即可
}

// -----------------------------
// 3) Status mapping helper
// -----------------------------
static inline AckStatus to_ack_status(Status st)
{
  switch (st) {
    case Status::OK:            return AckStatus::OK;
    case Status::ERR:           return AckStatus::ERR;
    case Status::BAD_PARAM:     return AckStatus::BAD_PARAM;
    case Status::TIMEOUT:       return AckStatus::TIMEOUT;
    case Status::NOT_SUPPORTED: return AckStatus::NOT_SUPPORTED;
    case Status::BUSY:          return AckStatus::BUSY;
    default:                    return AckStatus::ERR;
  }
}

// -----------------------------
// 4) Main dispatch
// -----------------------------
void cmd_dispatch(cmd_ctrl_t* cmd,
                  fog_parameter_t* params,
                  output_fn_t* output_fn,
                  auto_rst_t* auto_rst)
{
  if (!cmd || !params || !output_fn || !auto_rst) {
    // 若 cmd 為 nullptr，無法知道 cmd_id；這裡先不送 ACK
    return;
  }

  // Decide mux (你的既有邏輯：cmd > 7 => MUX_OUTPUT / else MUX_PARAMETER)
  cmd_mux(cmd);

  // 先送 ACK：目前先一律 OK（之後可以加入更嚴格的條件檢查再回 BAD_PARAM/NOT_SUPPORTED）
  (void)send_ack_v1(output_port(), cmd->cmd, AckStatus::OK);

    // ---- Route C test commands: no side effects ----
  const TransactionSpec spec = get_command_spec(cmd->cmd);

    // 0xF0: ACK only (no RESULT, no behavior)
  if (cmd->cmd == CMD_TEST_ACK_ONLY) {
    return;
  }

 if (cmd->cmd == CMD_TEST_ACK_RESULT) {
  (void)send_result_v1(output_port(), cmd->cmd, AckStatus::OK);
  return;
  }

  if (cmd->cmd == CMD_TEST_TIMEOUT) {
    (void)send_result_v1(output_port(), cmd->cmd, AckStatus::TIMEOUT);
    return;
  }

  // -------------------------
  // Parameter path (FPGA/INS related)
  // -------------------------
  if (cmd->mux == MUX_PARAMETER) {
    // spec 決定這個命令需不需要 RESULT
    const TransactionSpec spec = get_command_spec(cmd->cmd);

    // 新版 parameter_service：回 UsecaseResult（payload A：先不用 payload）
    UsecaseResult r = parameter_service_handle_ex(fpga_port(), cmd, params, spec);

    // dump/query 才回 RESULT（payload A：只回 status）
    if (spec.expect_response) {
      (void)send_result_v1(pc_port(), cmd->cmd, to_ack_status(r.status));
    }
    return;
  }

  // -------------------------
  // Output path (streaming / mode / output_fn)
  // -------------------------
  // 先保守：沿用你原本流程（也就是你前面同意的 cmd_dispatch 集中 orchestration）
  // 目前 output 不回 RESULT（你若想對 output 命令也回 RESULT，可在 spec 表加）
  fog_parameter(cmd, params);
  output_mode_setting(cmd, output_fn, auto_rst);

  if (*output_fn) {
    (*output_fn)(cmd, params);
  }
}

