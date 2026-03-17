#pragma once
#include "../../output_mode_config.h"  // 這裡面已經 typedef 了 fn_ptr

using output_fn_t = fn_ptr;

// using output_fn_t = void (*)(cmd_ctrl_t*, fog_parameter_t*);
