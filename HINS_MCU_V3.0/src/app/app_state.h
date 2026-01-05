#pragma once
#include "../common.h"
#include "../domain/model/memory_manage.h"
#include "../output_mode_config.h"
#include "../MadgwickAHRS_IMU.h"
#include <Arduino.h>


// 全域狀態集中在這裡（只宣告）
extern cmd_ctrl_t      g_cmd;
extern auto_rst_t      g_auto_rst;
extern fog_parameter_t g_fog_params;
extern fn_ptr          g_output_fn;


extern Stream& g_cmd_port_fpga;      // fpga
extern Stream& g_cmd_port_output;    // output port
extern Stream& g_cmd_port_hins;       // ins

// Reconfigure MCU output UART baudrate (g_cmd_port_output is Stream&).
void output_port_begin(uint32_t baud);

// acq_ahrs.cpp 會用到
extern Madgwick        ahrs_attitude;