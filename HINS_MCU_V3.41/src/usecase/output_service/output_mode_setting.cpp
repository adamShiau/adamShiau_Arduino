#include "output_mode_setting.h"
#include "output_modes.h"
#include "../recovery_service.h"


int output_mode_setting(cmd_ctrl_t* rx, fn_ptr* output_fn, sys_ctrl_t* sys_ctrl)
{
	if(rx->mux == MUX_OUTPUT)
	{
		DEBUG_PRINT("\noutput_mode_setting\n");
		DEBUG_PRINT("cmd= %u\n", rx->cmd);
		rx->mux = MUX_ESCAPE;

		switch(rx->cmd) {
			case MODE_RST: {
				*output_fn = acq_rst;
				rx->select_fn = SEL_RST;
				set_cfg_fn_mode(MODE_RST);
				// sys_ctrl->fn_mode = MODE_RST;
				break;
			}
			case MODE_IMU: {
				*output_fn = acq_imu;
				DEBUG_PRINT("output_fn select to acq_imu\n");
				rx->select_fn = SEL_IMU;
				set_cfg_fn_mode(MODE_IMU);
				// sys_ctrl->fn_mode = MODE_IMU;
				break;
			}
            case MODE_AHRS: {
				*output_fn = acq_ahrs;
				DEBUG_PRINT("output_fn select to acq_ahrs\n");
				rx->select_fn = SEL_AHRS;
				// sys_ctrl->fn_mode = MODE_AHRS;
				break;
			}
			case MODE_HINS: {
				*output_fn = acq_hins;
				DEBUG_PRINT("output_fn select to acq_hins\n");
				rx->select_fn = SEL_HINS;
				set_cfg_fn_mode(MODE_HINS);
				// sys_ctrl->fn_mode = MODE_HINS;
				break;
			}
			case MODE_CV7: {
				*output_fn = acq_cv7;
				DEBUG_PRINT("output_fn select to acq_cv7\n");
				rx->select_fn = SEL_CV7;
				set_cfg_fn_mode(MODE_CV7);
				// sys_ctrl->fn_mode = MODE_CV7;
				break;
			}

      		default: DEBUG_PRINT("function mode out of range\n");break;
      }
	}
	else if(rx->mux == MUX_DEFAULT)
	{
		rx->mux = MUX_ESCAPE;
		*output_fn = acq_rst;
		DEBUG_PRINT("just start, enter acq_rst ");
	}
}

