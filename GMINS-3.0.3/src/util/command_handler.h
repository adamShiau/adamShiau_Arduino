#pragma once

#include <Arduino.h>
#include "../core/system_controller.h"

/**
 * 命令處理器 - 處理Serial命令用於協議切換
 * 
 * 支持的命令:
 * - AR1AFC: 切換到AR1AFC協議 (自家封包格式)
 * - MAVLINK: 切換到MAVLink協議 (QGroundControl)
 * - XBUS/MTI: 切換到XBUS協議 (原始MTI格式)
 * - STATUS: 顯示系統狀態
 * - FREQ: 顯示實際封包輸出頻率
 * - HELP: 顯示命令說明
 */
class CommandHandler {
private:
    SystemController* system_controller_;
    uint32_t* protocol_switches_counter_;
    uint32_t* nav_callbacks_counter_;
    uint32_t* total_cycles_counter_;

public:
    /**
     * 構造函數
     * @param sys_ctrl 系統控制器指針
     * @param protocol_switches 協議切換次數計數器
     * @param nav_callbacks 導航回調次數計數器
     * @param total_cycles 總循環次數計數器
     */
    CommandHandler(SystemController* sys_ctrl, 
                  uint32_t* protocol_switches,
                  uint32_t* nav_callbacks, 
                  uint32_t* total_cycles);

    /**
     * 檢查並處理Serial命令
     * 在主循環中調用此函數
     */
    void checkAndProcessCommands();

private:
    /**
     * 處理協議切換命令
     * @param protocol_name 協議名稱
     */
    void handleProtocolSwitch(const String& protocol_name);

    /**
     * 處理STATUS命令 - 顯示系統狀態
     */
    void handleStatusCommand();

    /**
     * 處理DIAG命令 - 診斷協議問題
     */
    void handleDiagCommand();

    /**
     * 處理HELP命令 - 顯示命令說明
     */
    void handleHelpCommand();

    /**
     * 處理FREQ命令 - 顯示實際封包輸出頻率
     */
    void handleFrequencyCommand();

    /**
     * 顯示未知命令錯誤
     * @param cmd 未知的命令
     */
    void handleUnknownCommand(const String& cmd);
};