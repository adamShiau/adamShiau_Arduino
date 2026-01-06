#include "cmd_dispatch.h"
#include <Arduino.h>
#include "../domain/protocol/ack_codec_v1.h"   // send_ack_v1 / send_result_v1
#include "parameter_service.h"      // parameter_service_handle_ex
#include "../usecase/output_service/output_mode_setting.h"         // output_mode_setting
#include "../common.h"                      // cmd_mux(), fog_parameter() etc.
#include "../myUART.h" 


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
      spec.timeout_ms = 500;
      spec.max_retry = 0;
      break;
    case CMD_HINS_PING:
    case CMD_HINS_MIP:
      spec.expect_response = true;
      spec.route = IoRoute::HINS;
      spec.timeout_ms = 1500;
      spec.max_retry = 0;
      break;

    default:
      break;
  }
  return spec;
}

static inline bool is_dump_cmd(uint8_t cmd_id)
{
  return (cmd_id == CMD_DUMP_FOG) ||
         (cmd_id == CMD_DUMP_MIS) ||
         (cmd_id == CMD_DUMP_SN);
}


// -----------------------------
// 2) Port selection
// -----------------------------
// PC 回覆 port：建議用 Serial（若你已用 app_state 封裝 g_cmd_port，可以換成 g_cmd_port）
static inline Stream& pc_port()
{
  return Serial; 
}

static inline Stream& hins_port()
{
  return Serial3; 
}

static inline Stream& fpga_port()
{
  return Serial1; // <-- 若你 FPGA port 不同，改這裡即可
}

static inline Stream& output_port()
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

  const TransactionSpec spec = get_command_spec(cmd->cmd); // spec（決定是否要 RESULT、timeout 等）

  cmd_mux(cmd); // Decide mux (你的既有邏輯：cmd > 7 => MUX_OUTPUT / else MUX_PARAMETER)

  (void)send_ack_v1(output_port(), cmd->cmd, AckStatus::OK); // ACK 一律回（使用 output_port，避免跟 debug 文本混在一起）

  // -------------------------
  // Parameter path (FPGA/INS related)
  // -------------------------
  if (cmd->mux == MUX_PARAMETER) {

    // 新版 parameter_service：回 UsecaseResult（payload A：先不用 payload）
    UsecaseResult r = parameter_service_handle_ex2(fpga_port(), hins_port(), cmd, params, spec);
    // DEBUG_PRINT("[DISPATCH] cmd=0x%02X st=%d payload_len=%u payload_ptr=%p\r\n",
    //         cmd->cmd, (int)r.status, (unsigned)r.payload_len, r.payload);


    // dump/query 才回 RESULT（payload A：只回 status）
    if (spec.expect_response) {
      if (is_dump_cmd(cmd->cmd)) {
        // dump 類：成功時不要在這裡送 RESULT，避免和 common::recv_and_store() 的 payload RESULT 重複
        // 若 dump 流程一開始就失敗（例如 BAD_PARAM/BUSY），才在這裡回一包「空 RESULT」錯誤碼
        if (r.status != Status::OK) {
          (void)send_result_v1(output_port(), cmd->cmd, to_ack_status(r.status));
        }
      } else {
        // 非 dump 類：維持原本行為（status-only RESULT）
        // (void)send_result_v1(output_port(), cmd->cmd, to_ack_status(r.status));
        if (r.payload && r.payload_len > 0) {
          (void)send_result_v1(output_port(), cmd->cmd, to_ack_status(r.status),
                              r.payload, r.payload_len);
        } else {
          (void)send_result_v1(output_port(), cmd->cmd, to_ack_status(r.status));
        }
      }
    }
    return;
  }

  // -------------------------
  // Output path (streaming / mode / output_fn)
  // -------------------------
    UsecaseResult r = parameter_service_handle_ex2(fpga_port(), hins_port(), cmd, params, spec);
    output_mode_setting(cmd, output_fn, auto_rst);

  // if (*output_fn) {
  //   (*output_fn)(cmd, params);
  // }
}

