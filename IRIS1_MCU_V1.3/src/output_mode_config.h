#ifndef OUTPUT_MODE_CONFIG_H
#define OUTPUT_MODE_CONFIG_H

// #include <Arduino.h> 
#include "common.h"
#include "memory_manage.h"


typedef void (*fn_ptr) (cmd_ctrl_t*, fog_parameter_t*);

void acq_rst(cmd_ctrl_t*, fog_parameter_t*);
void acq_imu(cmd_ctrl_t*, fog_parameter_t*);
void acq_ahrs(cmd_ctrl_t*, fog_parameter_t*);


#endif /* OUTPUT_MODE_CONFIG_H */
