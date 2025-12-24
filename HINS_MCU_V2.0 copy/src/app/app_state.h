#pragma once
#include "../common.h"
#include "../memory_manage.h"
#include "../output_mode_config.h"
#include "../MadgwickAHRS_IMU.h"
#include <Arduino.h>


// 全域狀態集中在這裡（只宣告）
extern cmd_ctrl_t      g_cmd;
extern auto_rst_t      g_auto_rst;
extern fog_parameter_t g_fog_params;
extern fn_ptr          g_output_fn;

//使用在common.cpp 的 switch case
extern Print& g_cmd_port;


// acq_ahrs.cpp 會用到
extern Madgwick        ahrs_attitude;