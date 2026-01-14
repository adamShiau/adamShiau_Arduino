#include "app_state.h"
#include "../output_mode_config.h"   // 確保 fn_ptr/輸出函式宣告一致
#include <Arduino.h>
#include "../myUART.h"

cmd_ctrl_t g_cmd = {
  .condition = RX_CONDITION_INIT,
  .SN = {0},
  .complete = 0,
  .mux = MUX_ESCAPE,
  .select_fn = SEL_IDLE,
  .ch = 0,
  .cmd = 0,
  .run = 0,
  .value = 0
};

auto_rst_t g_auto_rst = { .status = 0, .fn_mode = MODE_RST };

fog_parameter_t g_fog_params;     // 原本就是容器
fn_ptr g_output_fn = acq_rst;     // 原本預設指向 rst

Madgwick ahrs_attitude;           // ★集中定義點（取代分散在 .ino 或其他地方）

Stream& g_cmd_port_fpga    = Serial1;      //使用在common.cpp 的 switch case
Stream& g_cmd_port_hins    = Serial3; 
Stream* g_p_output_stream  = &Serial2;


// void output_port_begin(uint32_t baud) // used in configiration_service to change baudrate
// {
//   Serial2.begin(baud);
// }

void output_port_begin(uint32_t baud) 
{
  // 透過指標存取並開啟 baudrate
  if (g_p_output_stream) {
      if (g_p_output_stream == &Serial2) Serial2.begin(baud);
      else if (g_p_output_stream == &Serial) Serial.begin(baud);
  }
}
