#pragma once
#include <Arduino.h>
#include "../common.h"  // cmd_ctrl_t, sys_ctrl_t
#include "../domain/model/output_fn_t.h" // output_fn_t
#include "../drivers/link/nios_link.h" // sendCmd


// #include "../domain/model/memory_manage.h" // fog_parameter_t


void system_recovery_init(cmd_ctrl_t* rx, fn_ptr* output_fn, const sys_ctrl_t* sys_ctrl);

bool set_cfg_auto_run(auto_run_mode_t status);

bool set_cfg_fn_mode(output_mode_t mode);

void FPGA_Config_init(void);

void FPGA_Config_rst(void);