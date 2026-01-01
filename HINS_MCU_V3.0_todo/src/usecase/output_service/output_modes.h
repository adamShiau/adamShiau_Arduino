#pragma once

#include "../../domain/model/output_fn_t.h"     // output_fn_t / fn_ptr type
#include "../../common.h"               // cmd_ctrl_t（如果你之後願意也可再拆更乾淨）
#include "../../domain/model/memory_manage.h"        // fog_parameter_t（目前還在 src/ 先沿用）

void acq_rst (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter);
void acq_imu (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter);
void acq_ahrs(cmd_ctrl_t* rx, fog_parameter_t* fog_parameter);
// void acq_fog(...); // 之後補
