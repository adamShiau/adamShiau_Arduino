#include "output_mode_setting.h"
#include "output_modes.h"


int output_mode_setting(cmd_ctrl_t* rx, fn_ptr* output_fn, auto_rst_t* auto_rst)
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
				auto_rst->fn_mode = MODE_RST;
				break;
			}
			case MODE_IMU: {
				*output_fn = acq_imu;
				DEBUG_PRINT("output_fn select to acq_imu\n");
				rx->select_fn = SEL_IMU;
				auto_rst->fn_mode = MODE_IMU;
				break;
			}
            case MODE_AHRS: {
				*output_fn = acq_ahrs;
				DEBUG_PRINT("output_fn select to acq_ahrs\n");
				rx->select_fn = SEL_AHRS;
				auto_rst->fn_mode = MODE_AHRS;
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

//   if(fog_op_status==1) // for auto reset
//   {
    // fog_op_status=0;
    // Serial.println("AUTO RST select function");
	// 	switch(rst_fn_flag) {
	// 		case MODE_RST: {
	// 			output_fn = fn_rst;
	// 			break;
	// 		}
	// 		case MODE_FOG: {
	// 			output_fn = acq_fog;
    //     Serial.println("MODE_FOG");
	// 			break;
	// 		}
	// 		case MODE_IMU: {
	// 			output_fn = acq_imu; 
	// 			break;
	// 		}
	// 		case MODE_FOG_HP_TEST: {
	// 			output_fn = acq_HP_test; 
	// 			break;
	// 		}
	// 		case MODE_NMEA: {
	// 			output_fn = acq_nmea;
	// 			break;
    //         }
    //   case MODE_ATT_NMEA: {
	// 			output_fn = acq_att_nmea;
	// 			break;
    //         }
    //   case MODE_FOG_PARAMETER: {
    //       output_fn = acq_fog_parameter;
    //       break;
    //   }
    //   default: break;
    //   }
    //   eeprom.Parameter_Write(EEPROM_ADDR_SELECT_FN, select_fn);
    //   eeprom.Parameter_Write(EEPROM_ADDR_OUTPUT_FN, rst_fn_flag);
    //   eeprom.Parameter_Write(EEPROM_ADDR_REG_VALUE, uart_value);
	// }
}

