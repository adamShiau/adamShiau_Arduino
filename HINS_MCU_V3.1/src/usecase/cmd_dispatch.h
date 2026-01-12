#pragma once

#include "../domain/model/output_fn_t.h"
#include "usecase_types.h"

// using output_fn_t = void (*)(cmd_ctrl_t*, fog_parameter_t*);

// Orchestration entry: decode -> cmd_dispatch -> (ACK/RESULT + usecase calls)
void cmd_dispatch(cmd_ctrl_t* cmd,
                  fog_parameter_t* params,
                  output_fn_t* output_fn,
                  auto_rst_t* auto_rst);
