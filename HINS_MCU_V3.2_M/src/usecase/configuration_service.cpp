#include "configuration_service.h"

#include "../app/app_state.h"        // g_cmd_port_fpga, output_port_begin, ahrs_attitude
#include "../drivers/link/nios_link.h"
#include "../domain/model/command_id.h"           // CMD_SYNC_CNT
#include "../common.h"               // set_data_rate, DEBUG_PRINT (if available)

// MCU side sync counters (must match FPGA expectations)
#define DR_10Hz_CNT   5000000UL
#define DR_50Hz_CNT   1000000UL
#define DR_100Hz_CNT  500000UL
#define DR_200Hz_CNT  250000UL
#define DR_400Hz_CNT  125000UL

static bool map_datarate_index(uint8_t idx, uint32_t* out_cnt, float* out_hz)
{
  if (!out_cnt || !out_hz) return false;

  switch (idx) {
    case 0: *out_cnt = DR_10Hz_CNT;  *out_hz = 10.0f;  return true;
    case 1: *out_cnt = DR_50Hz_CNT;  *out_hz = 50.0f;  return true;
    case 2: *out_cnt = DR_100Hz_CNT; *out_hz = 100.0f; return true;
    case 3: *out_cnt = DR_200Hz_CNT; *out_hz = 200.0f; return true;
    case 4: *out_cnt = DR_400Hz_CNT; *out_hz = 400.0f; return true;
    default: return false;
  }
}

static bool map_baudrate_index(uint8_t idx, uint32_t* out_baud)
{
  if (!out_baud) return false;

  switch (idx) {
    case 0: *out_baud = 9600UL;   return true;
    case 1: *out_baud = 115200UL; return true;
    case 2: *out_baud = 230400UL; return true;
    case 3: *out_baud = 460800UL; return true;
    case 4: *out_baud = 921600UL; return true;
    default: return false;
  }
}

static uint8_t cfg_get_u8(const fog_parameter_t* p, uint8_t cfg_idx, uint8_t fallback)
{
  if (!p) return fallback;

  // config[] element is mem_unit_t; prefer int_val for indexes
  // NOTE: We avoid hard-coding enum values; if type isn't int, still cast float.
  const mem_unit_t* u = &p->config[cfg_idx];
  // If your mem_unit_t has a 'type' enum, keep this simple and robust:
  // - int indexes are stored in int_val
  // - otherwise fallback to float_val
  uint8_t v = 0;
  if (u->type == TYPE_INT) {
    v = (uint8_t)u->data.int_val;
  } else {
    v = (uint8_t)u->data.float_val;
  }
  return v;
}

bool apply_datarate_index(uint8_t dr_index)
{
  uint32_t cnt = 0;
  float hz = 0.0f;
  if (!map_datarate_index(dr_index, &cnt, &hz)) {
    return false;
  }

  // 1) Update FPGA (CMD_SYNC_CNT, ch=6)
  DEBUG_PRINT("Applying datarate index %d (cnt=%d, hz=%f)\n", dr_index, cnt, hz);
  if (!nios_send_cfg_datarate(g_cmd_port_fpga, CMD_SYNC_CNT, dr_index)) {
    return false;
  }

  // 2) Update MCU local rate (used by ahrs)
  // set_data_rate(cnt);
  ahrs_attitude.init(hz);

  return true;
}

bool apply_baudrate_index(uint8_t br_index)
{
  uint32_t baud = 1;
  
  if (!map_baudrate_index(br_index, &baud)) {
    return false;
  }

  // Only MCU-side UART, FPGA not involved
  DEBUG_PRINT("Applying baudrate index %d (baud=%d)\n", br_index, baud);
  output_port_begin(baud);
  return true;
}

void apply_configuration_from_container(const fog_parameter_t* params)
{
  // config[0] = datarate index, config[1] = baudrate index
  uint8_t dr_idx = cfg_get_u8(params, 0, 2); // default to 100Hz
  uint8_t br_idx = cfg_get_u8(params, 1, 1); // default to 115200

  // Apply in order: datarate then baudrate
  (void)apply_datarate_index(dr_idx);
  (void)apply_baudrate_index(br_idx);
}
