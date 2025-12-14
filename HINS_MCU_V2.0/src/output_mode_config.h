#ifndef OUTPUT_MODE_CONFIG_H
#define OUTPUT_MODE_CONFIG_H

// #include <Arduino.h> 
#include "common.h"
#include "memory_manage.h"
#include "MadgwickAHRS_IMU.h"

extern Madgwick ahrs_attitude; //define in IRIS1_MCU.ino, declare here for use in acq_ahrs.cpp

typedef void (*fn_ptr) (cmd_ctrl_t*, fog_parameter_t*);

void acq_rst(cmd_ctrl_t*, fog_parameter_t*);
void acq_imu(cmd_ctrl_t*, fog_parameter_t*);
void acq_ahrs(cmd_ctrl_t*, fog_parameter_t*);


#endif /* OUTPUT_MODE_CONFIG_H */
