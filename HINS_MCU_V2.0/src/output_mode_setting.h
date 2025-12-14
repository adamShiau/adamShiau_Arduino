#ifndef OUTPUT_MODE_SETTING_H
#define OUTPUT_MODE_SETTING_H

#include <Arduino.h> 
#include "common.h"
#include "memory_manage.h"
#include "output_mode_config.h"

// 回傳 0=成功, <0=錯誤（例如未知模式）
int output_mode_setting(cmd_ctrl_t*, fn_ptr*, auto_rst_t*);

// void clear_SEL_EN(cmd_ctrl_t*);

#endif /* OUTPUT_MODE_SETTING_H */
