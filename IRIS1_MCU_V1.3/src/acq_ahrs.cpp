#include "common.h"
#include "output_mode_config.h"

#define INT_SYNC 1
#define EXT_SYNC 2
#define STOP_RUN 4

static unsigned long t_start = 0;

void acq_ahrs (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    if(rx->select_fn == SEL_AHRS) {
        rx->select_fn = SEL_IDLE; //clear select_fn
        DEBUG_PRINT("-> select acq_ahrs mode\n");
        if(rx->value == INT_SYNC) { //internal sync mode
            // DEBUG_PRINT("acq_fog select internal mode\n");
            DEBUG_PRINT("acq_ahrs select internal mode\n");
            rx->run = 1;
        }
        else if(rx->value == EXT_SYNC) { //external sync mode
            DEBUG_PRINT("acq_ahrs select external mode\n");
            rx->run = 1;
        }
        else if(rx->value == STOP_RUN) { //stop
            DEBUG_PRINT("acq_ahrs select stop\n");
            rx->run = 0;
        }
    }
}
