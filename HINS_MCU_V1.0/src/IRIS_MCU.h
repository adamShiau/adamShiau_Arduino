#ifndef IRIS_MCU_H
#define IRIS_MCU_H

#define MCU_VERSION "IRIS1_MCU_V1.5-3"

#include "myUART.h"
#include "common.h"
#include "memory_manage.h"
#include "output_mode_config.h"
#include "output_mode_setting.h"
#include "MadgwickAHRS_IMU.h"


// Definition and initialization of my_cmd, structure type is defined in common.h
cmd_ctrl_t my_cmd = {
	.condition = RX_CONDITION_INIT,
	.SN = {0},
  .complete = 0,
  .mux = MUX_ESCAPE,
  .select_fn = SEL_IDLE,
  .ch = 0,  
  .cmd = 0,
  .run = 0,
  .value = 0
};

// Definition and initialization of auto_rst, structure type is defined in common.h
auto_rst_t auto_rst = {
	.status = 0,
	.fn_mode = MODE_RST
};

fog_parameter_t fog_params;	 //parameter container

// my_sensor_t sensor_raw = {}, sensor_cali = {};

fn_ptr output_fn = acq_rst;

#endif /* IRIS_MCU_H */
