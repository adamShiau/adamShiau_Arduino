#include "../../common.h"
#include "../../domain/model/memory_manage.h"
// #include "output_mode_config.h"

static unsigned int t_start = 0;

void acq_rst (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    rx -> select_fn = SEL_IDLE; // clear
    if((millis() - t_start)>=5000) 
    {
        t_start = millis();
        DEBUG_PRINT("IDLE\n");
        Serial2.println("IDLE");
    }
}
