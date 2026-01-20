#include "parameter_service.h"
#include "configuration_service.h"
#include "../drivers/link/hins_link.h"


extern Uart Serial3;

void parameter_service_handle(Stream& port, cmd_ctrl_t* rx, fog_parameter_t* fog_inst)
{
	TransactionSpec spec; // default: no response
  	(void)parameter_service_handle_ex(port, rx, fog_inst, spec);
}

UsecaseResult parameter_service_handle_ex(Stream& port, cmd_ctrl_t* rx, fog_parameter_t* fog_inst, const TransactionSpec& spec)
{

	UsecaseResult result;
	result.cmd_id = rx ? rx->cmd : 0; // make sure rx is not null, oherwise cmd id is 0

	if (!rx || !fog_inst) {
		result.status = Status::BAD_PARAM;
		return result;
	}

	if(rx->mux == MUX_PARAMETER){
        DEBUG_PRINT("fog_parameter mode, condition: %d:\n", rx->condition);
        rx->mux = MUX_ESCAPE;

			if(rx->condition == RX_CONDITION_ABBA_5556 || rx->condition == RX_CONDITION_EFFE_5354) {
		
        switch(rx->cmd ){
					case CMD_MOD_FREQ: {
						DEBUG_PRINT("CMD_MOD_FREQ:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MOD_FREQ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MOD_FREQ - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_MOD_AMP_H: {
						DEBUG_PRINT("CMD_MOD_AMP_H:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MOD_AMP_H, rx->value, rx->ch);
							update_parameter_container(rx, fog_inst, CMD_MOD_AMP_H - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_MOD_AMP_L: {
						DEBUG_PRINT("CMD_MOD_AMP_L:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MOD_AMP_L, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_MOD_AMP_L - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_POLARITY: {
						DEBUG_PRINT("CMD_POLARITY:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_POLARITY, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_POLARITY - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_WAIT_CNT: {
						DEBUG_PRINT("CMD_WAIT_CNT:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_WAIT_CNT, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_WAIT_CNT - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_ERR_AVG: {
						DEBUG_PRINT("CMD_ERR_AVG:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_ERR_AVG, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_ERR_AVG - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_GAIN1: {
						DEBUG_PRINT("CMD_GAIN1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_GAIN1, rx->value, rx->ch);
             				 update_parameter_container(rx, fog_inst, CMD_GAIN1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_CONST_STEP: {
						DEBUG_PRINT("CMD_CONST_STEP:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CONST_STEP, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_CONST_STEP - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_FB_ON: {
						DEBUG_PRINT("CMD_FB_ON:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_FB_ON, rx->value, rx->ch);
             			 	update_parameter_container(rx, fog_inst, CMD_FB_ON - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_GAIN2: {
						DEBUG_PRINT("CMD_GAIN2:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_GAIN2, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_GAIN2 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_ERR_OFFSET: {
						DEBUG_PRINT("CMD_ERR_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_ERR_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_ERR_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_DAC_GAIN: {
						DEBUG_PRINT("CMD_DAC_GAIN:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_DAC_GAIN, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_DAC_GAIN - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_CUT_OFF: {
						DEBUG_PRINT("CMD_CUT_OFF:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CUT_OFF, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_CUT_OFF - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_OUT_TH: {
						DEBUG_PRINT("CMD_OUT_TH:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_OUT_TH, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_OUT_TH - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_OUT_TH_EN: {
						DEBUG_PRINT("CMD_OUT_TH_EN:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_OUT_TH_EN, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_OUT_TH_EN - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_COMP_T1: {
						DEBUG_PRINT("CMD_SF_COMP_T1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_COMP_T1, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_SF_COMP_T1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_COMP_T2: {
						DEBUG_PRINT("CMD_SF_COMP_T2:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_COMP_T2, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_COMP_T2 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_1_SLOPE: {
						DEBUG_PRINT("CMD_SF_1_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_1_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_1_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_1_OFFSET: {
						DEBUG_PRINT("CMD_SF_1_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_1_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_1_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_2_SLOPE: {
						DEBUG_PRINT("CMD_SF_2_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_2_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_2_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_2_OFFSET: {
						DEBUG_PRINT("CMD_SF_2_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_2_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_2_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_3_SLOPE: {
						DEBUG_PRINT("CMD_SF_3_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_3_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_3_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_3_OFFSET: {
						DEBUG_PRINT("CMD_SF_3_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_3_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_3_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_BIAS_COMP_T1: {
						DEBUG_PRINT("CMD_BIAS_COMP_T1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_COMP_T1, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_COMP_T1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_BIAS_COMP_T2: {
						DEBUG_PRINT("CMD_BIAS_COMP_T2:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_COMP_T2, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_COMP_T2 - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_1_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_1_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_1_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_1_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_1_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_1_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_1_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_1_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_2_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_2_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_2_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_2_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_2_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_2_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_2_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_2_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_3_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_3_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_3_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_3_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_3_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_3_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_3_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_3_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_SF_SLOPE_XLM: {
						DEBUG_PRINT("CMD_SF_SLOPE_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_SLOPE_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_SLOPE_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_SF_OFFSET_XLM: {
						DEBUG_PRINT("CMD_SF_OFFSET_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_OFFSET_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_OFFSET_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_SLOPE_XLM: {
						DEBUG_PRINT("CMD_BIAS_SLOPE_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_SLOPE_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_SLOPE_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_OFFSET_XLM: {
						DEBUG_PRINT("CMD_BIAS_OFFSET_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_OFFSET_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_OFFSET_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					/***------------- mis-alignment command, accl */
					case CMD_MIS_AX: {
						DEBUG_PRINT("CMD_MIS_AX:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_AX, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AX - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_AY: {
						DEBUG_PRINT("CMD_MIS_AY:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_AY, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AY - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_AZ: {
						DEBUG_PRINT("CMD_MIS_AZ:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_AZ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AZ - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A11: {
						DEBUG_PRINT("CMD_MIS_A11:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A11, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A11 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A12: {
						DEBUG_PRINT("CMD_MIS_A12:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A12, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A12 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A13: {
						DEBUG_PRINT("CMD_MIS_A13:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A13, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A13 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A21: {
						DEBUG_PRINT("CMD_MIS_A21:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A21, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A21 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A22: {
						DEBUG_PRINT("CMD_MIS_A22:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A22, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A22 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A23: {
						DEBUG_PRINT("CMD_MIS_A23:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A23, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A23 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A31: {
						DEBUG_PRINT("CMD_MIS_A31:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A31, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A31 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A32: {
						DEBUG_PRINT("CMD_MIS_A32:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A32, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A32 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A33: {
						DEBUG_PRINT("CMD_MIS_A33:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A33, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A33 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					/***------------- mis-alignment command, gyro */
					case CMD_MIS_GX: {
						DEBUG_PRINT("CMD_MIS_GX:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_GX, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GX - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_GY: {
						DEBUG_PRINT("CMD_MIS_GY:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_GY, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GY - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_GZ: {
						DEBUG_PRINT("CMD_MIS_GZ:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_GZ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GZ - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G11: {
						DEBUG_PRINT("CMD_MIS_G11:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G11, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G11 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G12: {
						DEBUG_PRINT("CMD_MIS_G12:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G12, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G12 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G13: {
						DEBUG_PRINT("CMD_MIS_G13:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G13, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G13 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G21: {
						DEBUG_PRINT("CMD_MIS_G21:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G21, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G21 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G22: {
						DEBUG_PRINT("CMD_MIS_G22:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G22, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G22 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G23: {
						DEBUG_PRINT("CMD_MIS_G23:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G23, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G23 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G31: {
						DEBUG_PRINT("CMD_MIS_G31:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G31, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G31 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G32: {
						DEBUG_PRINT("CMD_MIS_G32:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G32, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G32 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G33: {
						DEBUG_PRINT("CMD_MIS_G33:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G33, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G33 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_DUMP_FOG: {
						DEBUG_PRINT("CMD_DUMP_FOG:\n");
						bool ok = dump_fog_param(fog_inst, rx->ch);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_MIS: {
						DEBUG_PRINT("CMD_DUMP_MIS:\n");
						bool ok = dump_misalignment_param(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_SN: {
						DEBUG_PRINT("CMD_DUMP_SN:\n");
						bool ok = dump_SN(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_CONFIG: {
						DEBUG_PRINT("CMD_DUMP_CONFIG:\n");
						bool ok = dump_config(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_VERSION: {
						DEBUG_PRINT("CMD_DUMP_VERSION:\n");
						bool ok = dump_version(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DATA_OUT_START: { // not use now
						DEBUG_PRINT("CMD_DATA_OUT_START:\n");
						// start_flag = rx->value;
						break;
					}
					case CMD_SYNC_CNT: {
						DEBUG_PRINT("CMD_SYNC_CNT:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SYNC_CNT, rx->value, rx->ch);
						}			
						break;
					} 
					case CMD_HW_TIMER_RST: {
						DEBUG_PRINT("CMD_HW_TIMER_RST:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_HW_TIMER_RST, rx->value, rx->ch);
						}			
						break;
					}
					case CMD_CFG_DR: {
						DEBUG_PRINT("CMD_CFG_DR:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							// PC sends datarate index (0~4). Convert to FPGA sync counter and forward to FPGA (ch fixed to 6).
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CFG_DR, rx->value, rx->ch); delay(100);
             				update_parameter_container(rx, fog_inst, CMD_CFG_DR - CFG_CONTAINER_TO_CMD_OFFSET);
							// nios_send_cfg_datarate(port, CMD_SYNC_CNT, (uint8_t)rx->value);
							(void)apply_datarate_index((uint8_t)rx->value);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_CFG_BR: {
						DEBUG_PRINT("CMD_CFG_BR:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CFG_BR, rx->value, rx->ch); delay(100);
             				update_parameter_container(rx, fog_inst, CMD_CFG_BR - CFG_CONTAINER_TO_CMD_OFFSET);
							(void)apply_baudrate_index((uint8_t)rx->value);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					/*** Hins communication */
					case CMD_HINS_PING: {
									DEBUG_PRINT("CMD_HINS_PING:\n");	
									uint8_t cmd[] = { 
						0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6 
						};

						// 送出指令
						Serial3.write(cmd, sizeof(cmd));
						// Serial3.flush();   // 確保送完
						Serial.println("Command sent");

						// 設定 timeout = 1.5 秒
						unsigned long startTime = millis();
						const unsigned long timeoutMs = 1500;

						// 等待回傳並印出
						while (millis() - startTime < timeoutMs) {
						// Serial.println(Serial3.available());
						if (Serial3.available() > 0) {
							uint8_t b = Serial3.read();
							
							// 以 HEX 格式印出
							if (b < 0x10) Serial.print("0");
							Serial.print(b, HEX);
							Serial.print(" ");
						}
						}

						Serial.println("\nTimeout reached\n");


									break;
								}

								default:{
									DEBUG_PRINT("condition 1 default case\n");
								} 
							}
					
						}
			else if(rx->condition == RX_CONDITION_CDDC_5758) {
				switch(rx->cmd ){
					case CMD_WRITE_SN: {
						DEBUG_PRINT("CMD_WRITE_SN:\n");
						sendSN(port, HDR_CDDC, TRL_5758, CMD_WRITE_SN, rx->SN);
						// for (int i=0; i<13; i++) {
						// 	DEBUG_PRINT("i: %d, SN: %x\n", i, rx->SN[i]);
						// }
						for (int i = 0; i < 13; i++) {
							fog_inst->sn[i] = rx->SN[i];
						}
						DEBUG_PRINT("SN: %s\n", fog_inst->sn);
						break;
					}
					default:{
						DEBUG_PRINT("condition 2 default case\n");
					} 
				}
			}
			
		}

}

UsecaseResult parameter_service_handle_ex2(Stream& port, Stream& port_hins, cmd_ctrl_t* rx, fog_parameter_t* fog_inst, const TransactionSpec& spec)
{

	UsecaseResult result;
	result.cmd_id = rx ? rx->cmd : 0; // make sure rx is not null, oherwise cmd id is 0

	if (!rx || !fog_inst) {
		result.status = Status::BAD_PARAM;
		return result;
	}

	if(rx->mux == MUX_PARAMETER){
        DEBUG_PRINT("fog_parameter mode, condition: %d:\n", rx->condition);
        rx->mux = MUX_ESCAPE;

			if(rx->condition == RX_CONDITION_ABBA_5556 || rx->condition == RX_CONDITION_EFFE_5354) {
		
        switch(rx->cmd ){
					case CMD_MOD_FREQ: {
						DEBUG_PRINT("CMD_MOD_FREQ:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MOD_FREQ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MOD_FREQ - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_MOD_AMP_H: {
						DEBUG_PRINT("CMD_MOD_AMP_H:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MOD_AMP_H, rx->value, rx->ch);
							update_parameter_container(rx, fog_inst, CMD_MOD_AMP_H - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_MOD_AMP_L: {
						DEBUG_PRINT("CMD_MOD_AMP_L:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MOD_AMP_L, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_MOD_AMP_L - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_POLARITY: {
						DEBUG_PRINT("CMD_POLARITY:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_POLARITY, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_POLARITY - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_WAIT_CNT: {
						DEBUG_PRINT("CMD_WAIT_CNT:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_WAIT_CNT, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_WAIT_CNT - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_ERR_AVG: {
						DEBUG_PRINT("CMD_ERR_AVG:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_ERR_AVG, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_ERR_AVG - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_GAIN1: {
						DEBUG_PRINT("CMD_GAIN1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_GAIN1, rx->value, rx->ch);
             				 update_parameter_container(rx, fog_inst, CMD_GAIN1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_CONST_STEP: {
						DEBUG_PRINT("CMD_CONST_STEP:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CONST_STEP, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_CONST_STEP - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_FB_ON: {
						DEBUG_PRINT("CMD_FB_ON:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_FB_ON, rx->value, rx->ch);
             			 	update_parameter_container(rx, fog_inst, CMD_FB_ON - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_GAIN2: {
						DEBUG_PRINT("CMD_GAIN2:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_GAIN2, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_GAIN2 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_ERR_OFFSET: {
						DEBUG_PRINT("CMD_ERR_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_ERR_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_ERR_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_DAC_GAIN: {
						DEBUG_PRINT("CMD_DAC_GAIN:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_DAC_GAIN, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_DAC_GAIN - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_CUT_OFF: {
						DEBUG_PRINT("CMD_CUT_OFF:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CUT_OFF, rx->value, rx->ch);
             				 update_parameter_container(rx, fog_inst, CMD_CUT_OFF - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_OUT_TH: {
						DEBUG_PRINT("CMD_OUT_TH:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_OUT_TH, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_OUT_TH - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_OUT_TH_EN: {
						DEBUG_PRINT("CMD_OUT_TH_EN:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_OUT_TH_EN, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_OUT_TH_EN - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_COMP_T1: {
						DEBUG_PRINT("CMD_SF_COMP_T1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_COMP_T1, rx->value, rx->ch);
              				update_parameter_container(rx, fog_inst, CMD_SF_COMP_T1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_COMP_T2: {
						DEBUG_PRINT("CMD_SF_COMP_T2:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_COMP_T2, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_COMP_T2 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_1_SLOPE: {
						DEBUG_PRINT("CMD_SF_1_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_1_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_1_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_1_OFFSET: {
						DEBUG_PRINT("CMD_SF_1_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_1_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_1_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_2_SLOPE: {
						DEBUG_PRINT("CMD_SF_2_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_2_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_2_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_2_OFFSET: {
						DEBUG_PRINT("CMD_SF_2_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_2_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_2_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_3_SLOPE: {
						DEBUG_PRINT("CMD_SF_3_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_3_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_3_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_3_OFFSET: {
						DEBUG_PRINT("CMD_SF_3_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_3_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_3_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_BIAS_COMP_T1: {
						DEBUG_PRINT("CMD_BIAS_COMP_T1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_COMP_T1, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_COMP_T1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_BIAS_COMP_T2: {
						DEBUG_PRINT("CMD_BIAS_COMP_T2:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_COMP_T2, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_COMP_T2 - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_1_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_1_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_1_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_1_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_1_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_1_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_1_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_1_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_2_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_2_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_2_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_2_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_2_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_2_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_2_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_2_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_3_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_3_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_3_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_3_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_3_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_3_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_3_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_3_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_SF_SLOPE_XLM: {
						DEBUG_PRINT("CMD_SF_SLOPE_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_SLOPE_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_SLOPE_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_SF_OFFSET_XLM: {
						DEBUG_PRINT("CMD_SF_OFFSET_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_SF_OFFSET_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_OFFSET_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_SLOPE_XLM: {
						DEBUG_PRINT("CMD_BIAS_SLOPE_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_SLOPE_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_SLOPE_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_OFFSET_XLM: {
						DEBUG_PRINT("CMD_BIAS_OFFSET_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(port, HDR_ABBA, TRL_5556, CMD_BIAS_OFFSET_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_OFFSET_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					/***------------- mis-alignment command, accl */
					case CMD_MIS_AX: {
						DEBUG_PRINT("CMD_MIS_AX:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_AX, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AX - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_AY: {
						DEBUG_PRINT("CMD_MIS_AY:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_AY, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AY - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_AZ: {
						DEBUG_PRINT("CMD_MIS_AZ:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_AZ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AZ - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A11: {
						DEBUG_PRINT("CMD_MIS_A11:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A11, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A11 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A12: {
						DEBUG_PRINT("CMD_MIS_A12:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A12, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A12 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A13: {
						DEBUG_PRINT("CMD_MIS_A13:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A13, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A13 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A21: {
						DEBUG_PRINT("CMD_MIS_A21:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A21, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A21 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A22: {
						DEBUG_PRINT("CMD_MIS_A22:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A22, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A22 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A23: {
						DEBUG_PRINT("CMD_MIS_A23:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A23, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A23 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A31: {
						DEBUG_PRINT("CMD_MIS_A31:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A31, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A31 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A32: {
						DEBUG_PRINT("CMD_MIS_A32:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A32, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A32 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A33: {
						DEBUG_PRINT("CMD_MIS_A33:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_A33, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A33 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					/***------------- mis-alignment command, gyro */
					case CMD_MIS_GX: {
						DEBUG_PRINT("CMD_MIS_GX:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_GX, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GX - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_GY: {
						DEBUG_PRINT("CMD_MIS_GY:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_GY, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GY - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_GZ: {
						DEBUG_PRINT("CMD_MIS_GZ:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_GZ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GZ - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G11: {
						DEBUG_PRINT("CMD_MIS_G11:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G11, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G11 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G12: {
						DEBUG_PRINT("CMD_MIS_G12:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G12, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G12 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G13: {
						DEBUG_PRINT("CMD_MIS_G13:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G13, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G13 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G21: {
						DEBUG_PRINT("CMD_MIS_G21:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G21, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G21 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G22: {
						DEBUG_PRINT("CMD_MIS_G22:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G22, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G22 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G23: {
						DEBUG_PRINT("CMD_MIS_G23:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G23, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G23 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G31: {
						DEBUG_PRINT("CMD_MIS_G31:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G31, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G31 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G32: {
						DEBUG_PRINT("CMD_MIS_G32:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G32, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G32 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G33: {
						DEBUG_PRINT("CMD_MIS_G33:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_MIS_G33, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G33 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_DUMP_FOG: {
						DEBUG_PRINT("CMD_DUMP_FOG:\n");
						bool ok = dump_fog_param(fog_inst, rx->ch);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_MIS: {
						DEBUG_PRINT("CMD_DUMP_MIS:\n");
						bool ok = dump_misalignment_param(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_SN: {
						DEBUG_PRINT("CMD_DUMP_SN:\n");
						bool ok = dump_SN(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_CONFIG: {
						DEBUG_PRINT("CMD_DUMP_CONFIG:\n");
						bool ok = dump_config(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DUMP_VERSION: {
						DEBUG_PRINT("CMD_DUMP_VERSION:\n");
						bool ok = dump_version(fog_inst);
						result.status = ok ? Status::OK : Status::TIMEOUT;
						break;
					} 
					case CMD_DATA_OUT_START: { // not use now
						DEBUG_PRINT("CMD_DATA_OUT_START:\n");
						// start_flag = rx->value;
						break;
					}
					case CMD_SYNC_CNT: {
						DEBUG_PRINT("CMD_SYNC_CNT:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_SYNC_CNT, rx->value, rx->ch);
						}			
						break;
					} 
					case CMD_HW_TIMER_RST: {
						DEBUG_PRINT("CMD_HW_TIMER_RST:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_HW_TIMER_RST, rx->value, rx->ch);
						}			
						break;
					}
					case CMD_CFG_DR: {
						DEBUG_PRINT("CMD_CFG_DR:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							// PC sends datarate index (0~4). Convert to FPGA sync counter and forward to FPGA (ch fixed to 6).
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CFG_DR, rx->value, rx->ch); delay(100);
             				update_parameter_container(rx, fog_inst, CMD_CFG_DR - CFG_CONTAINER_TO_CMD_OFFSET);
							// nios_send_cfg_datarate(port, CMD_SYNC_CNT, (uint8_t)rx->value);
							(void)apply_datarate_index((uint8_t)rx->value);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_CFG_BR: {
						DEBUG_PRINT("CMD_CFG_BR:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(port, HDR_ABBA, TRL_5556, CMD_CFG_BR, rx->value, rx->ch); delay(100);
             				update_parameter_container(rx, fog_inst, CMD_CFG_BR - CFG_CONTAINER_TO_CMD_OFFSET);
							(void)apply_baudrate_index((uint8_t)rx->value);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} //  CMD_CFG_BR
					default:{
						DEBUG_PRINT("condition 1 default case\n");
					} 
				}
		
			} 
			else if(rx->condition == RX_CONDITION_CDDC_5758) {
				switch(rx->cmd ){
					case CMD_WRITE_SN: {
						DEBUG_PRINT("CMD_WRITE_SN:\n");
						sendSN(port, HDR_CDDC, TRL_5758, CMD_WRITE_SN, rx->SN);
						// for (int i=0; i<13; i++) {
						// 	DEBUG_PRINT("i: %d, SN: %x\n", i, rx->SN[i]);
						// }
						for (int i = 0; i < 13; i++) {
							fog_inst->sn[i] = rx->SN[i];
						}
						DEBUG_PRINT("SN: %s\n", fog_inst->sn);
						break;
					}
					default:{
						DEBUG_PRINT("condition 2 default case\n");
					} 
				}
			}
			else if(rx->condition == RX_CONDITION_BCCB_5152) {
				switch(rx->cmd ){
				/*** Hins communication */
					case CMD_HINS_PING: {
						if (!rx->hins_payload || rx->hins_payload_len == 0) {
							result.status = Status::BAD_PARAM;
							break;
						}

						uint8_t ack_code = 0xFF;
						UsecaseResult r = hins_send_and_wait_ack_base(
							port_hins,
							rx->hins_payload, rx->hins_payload_len,
							1500,
							&ack_code
						);

						result = r;  // 或 result.status = r.status; 視你 UsecaseResult 結構
						// 可選：log
						DEBUG_PRINT("[HINS_ACK] code=0x%02X status=%d\r\n", ack_code, (int)result.status);
						break;
					} // CMD_HINS_PING

					case CMD_HINS_MIP: {
						if (!rx->hins_payload || rx->hins_payload_len == 0) {
							result.status = Status::BAD_PARAM;
							break;
						}

						// 1. 發送指令到 HINS
						port_hins.write(rx->hins_payload, rx->hins_payload_len);

						// 2. 準備靜態 Buffer 存放回傳，供 RESULT 封裝使用
						static uint8_t mip_raw_packet[512]; 
						uint16_t actual_len = 0;

						// 3. 調用 driver 層的 capture 函數
						Status st = hins_capture_raw_mip(port_hins, 
														mip_raw_packet, sizeof(mip_raw_packet), 
														&actual_len, 1500);

						// 4. 設定回傳結果
						result.status = st;
						if (st == Status::OK) {
							result.payload = mip_raw_packet;
							result.payload_len = actual_len;
						} else {
							result.payload_len = 0;
						}

						DEBUG_PRINT("[HINS_MIP_RAW] len=%u status=%d\r\n", (unsigned)actual_len, (int)st);
						break;
					}

					// case CMD_HINS_MIP: {
					// 	if (!rx->hins_payload || rx->hins_payload_len == 0) {
					// 		result.status = Status::BAD_PARAM;
					// 		break;
					// 	}

					// 	// RESULT payload buffer（static 確保回傳時還活著）
					// 	// 格式：version(1) flags(1) ack_code(1) ack_echo(1) resp_desc(1) resp_len_L(1) resp_len_H(1) resp_data...
					// 	static uint8_t out_buf[1 + 1 + 1 + 1 + 1 + 2 + 512];

					// 	uint8_t desc_set = 0, cmd_desc = 0;
					// 	uint8_t ack_code = 0xFF, ack_echo = 0x00;
					// 	uint8_t resp_desc = 0;
					// 	uint16_t resp_len = 0;
					// 	uint8_t resp_data[512];

					// 	Status st = hins_mip_transact(
					// 		port_hins,
					// 		rx->hins_payload, rx->hins_payload_len,
					// 		1500,
					// 		&desc_set, &cmd_desc,
					// 		&ack_code, &ack_echo,
					// 		&resp_desc,
					// 		resp_data, sizeof(resp_data),
					// 		&resp_len
					// 	);

					// 	// 組回傳 payload（永遠回 header，方便 debug）
					// 	uint8_t flags = 0;
					// 	// bit0=ack_present（只有成功讀到 ack_echo==cmd_desc 才算 present；這裡簡化：ack_code!=0xFF 就當 present）
					// 	if (ack_code != 0xFF) flags |= 0x01;
					// 	if (resp_len > 0)     flags |= 0x02;

					// 	uint16_t idx = 0;
					// 	out_buf[idx++] = 0x01;        // version
					// 	out_buf[idx++] = flags;
					// 	out_buf[idx++] = ack_code;
					// 	out_buf[idx++] = ack_echo;
					// 	out_buf[idx++] = resp_desc;
					// 	out_buf[idx++] = (uint8_t)(resp_len & 0xFF);
					// 	out_buf[idx++] = (uint8_t)((resp_len >> 8) & 0xFF);

					// 	// copy resp
					// 	if (resp_len > 0) {
					// 		uint16_t copy_len = min<uint16_t>(resp_len, (uint16_t)(sizeof(out_buf) - idx));
					// 		memcpy(&out_buf[idx], resp_data, copy_len);
					// 		idx += copy_len;
					// 	}

					// 	result.status = st;
					// 	result.payload = out_buf;
					// 	result.payload_len = idx;

					// 	DEBUG_PRINT("[HINS_MIP] ds=0x%02X cmd=0x%02X ack=0x%02X echo=0x%02X resp=0x%02X len=%u st=%d\r\n",
					// 				desc_set, cmd_desc, ack_code, ack_echo, resp_desc, (unsigned)resp_len, (int)st);
					// 	break;
					
					// } // CMD_HINS_MIP
					
					default:{
						DEBUG_PRINT("condition 4 default case\n");
					} 
				}
			}
			
		}
		return result;
}
