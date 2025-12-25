#pragma once

#include <Arduino.h>
#include "memory_manage.h"
#include "output_mode_config.h"        // legacy fn_ptr
#include "domain/model/output_fn_t.h"  // new output_fn_t

int output_mode_setting(cmd_ctrl_t* rx, fn_ptr* output_fn, auto_rst_t* auto_rst);

// void clear_SEL_EN(cmd_ctrl_t*);

