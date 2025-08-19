#ifndef IRIS_MCU_H
#define IRIS_MCU_H

#include "myUART.h"
#include "common.h"
#include "memory_manage.h"


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

fog_parameter_t fog_params;	 //parameter container

#endif /* IRIS_MCU_H */
