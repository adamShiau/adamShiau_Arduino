#include "recovery_service.h"
#include "./output_service/output_modes.h" // 定義各個輸出函數
#include "../app/app_state.h"        // g_cmd_port_fpga

#ifndef EXT_SYNC
  #define EXT_SYNC 2
#endif

void system_recovery_init(cmd_ctrl_t* rx, fn_ptr* output_fn, const sys_ctrl_t* sys_ctrl) {
    /***
     * sys_ctrl->auto_run 與 sys_ctrl->fn_mode 會在 
     * apply_configuration_from_container 時將記憶體數值
     * 讀出並放到 sys_ctrl 裡面，此處只要使用即可不需修改值
     */

    // 檢查是否需要自動執行
    if (sys_ctrl->auto_run == ENABLE && sys_ctrl->fn_mode != MODE_RST) {

        // 根據模式直接配置指標，不經過 cmd_dispatch
        switch(sys_ctrl->fn_mode) {
            case MODE_IMU:  *output_fn = acq_imu;   rx->select_fn = SEL_IMU;  break;
            case MODE_AHRS: *output_fn = acq_ahrs;  rx->select_fn = SEL_AHRS; break;
            case MODE_HINS: *output_fn = acq_hins;  rx->select_fn = SEL_HINS; break;
            case MODE_CV7:  *output_fn = acq_cv7;   rx->select_fn = SEL_CV7;  break;
        }
        
        // 強制進入執行狀態
        rx->run = 1;
        
        // 執行第一次觸發 ahrs_handle_setup 內的啟動流程
        if (*output_fn) {
             rx->value = EXT_SYNC; 
        }
        
        DEBUG_PRINT("Recovery: Auto-run Mode 0x%02X enabled.\n", sys_ctrl->fn_mode);
    } 
    else {
        DEBUG_PRINT("Recovery: Auto-run is disabled or no valid config found.\n");
    }
}

bool set_cfg_auto_run(auto_run_mode_t status) {
    uint8_t val = 0;
    val = (uint8_t)status;

    if (!nios_send_cfg_cmd(g_cmd_port_fpga, CMD_CFG_AUTO_RUN, val)) {
        return false;
    }
    DEBUG_PRINT("\nSetting_cfg_auto_run to : %d\n", val);

    return true;
}

bool set_cfg_fn_mode(output_mode_t mode) {
    uint8_t val = 0;
    val = (uint8_t)mode;

    if (!nios_send_cfg_cmd(g_cmd_port_fpga, CMD_CFG_FN_MODE, val)) {
        return false;
    }
    DEBUG_PRINT("\nSetting_cfg_fn_mode to : %d\n", val);

    return true;
}