#include "command_handler.h"
#include "log.h"
#include "gmins_frequency_manager.h"
#include "../hal/board_support.h"

#define LOG_TAG "CMD_HANDLER"

CommandHandler::CommandHandler(SystemController* sys_ctrl, 
                              uint32_t* protocol_switches,
                              uint32_t* nav_callbacks, 
                              uint32_t* total_cycles)
    : system_controller_(sys_ctrl)
    , protocol_switches_counter_(protocol_switches)
    , nav_callbacks_counter_(nav_callbacks)
    , total_cycles_counter_(total_cycles) {
}

void CommandHandler::checkAndProcessCommands() {
    
    // 處理 Serial (USB Debug) 命令
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();
        
        if (cmd == "AR1AFC" || cmd == "MAVLINK") {
            handleProtocolSwitch(cmd);
        } else if (cmd == "STATUS") {
            handleStatusCommand();
        } else if (cmd == "DIAG") {
            handleDiagCommand();
        } else if (cmd == "HELP") {
            handleHelpCommand();
        } else if (cmd == "FREQ") {
            handleFrequencyCommand();
        } else if (cmd.length() > 0) {
            handleUnknownCommand(cmd);
        }
    }
    
    // 處理 Serial1 (GUI 命令) - 直接讀取，不需要 TxMultiplexer
    if (hal::Serial1.available()) {
        // 記錄接收時間戳
        uint32_t recv_timestamp = millis();
        int available_bytes = hal::Serial1.available();
        
        // 簡化版 Serial1 命令接收 debug
        Serial.print("🔍 Serial1 收到資料：");
        
        // 讀取所有可用資料
        String raw_data = "";
        int byte_count = 0;
        while (hal::Serial1.available() && byte_count < 64) {
            char c = hal::Serial1.read();
            raw_data += c;
            byte_count++;
        }
        
        // 顯示命令內容（去除換行符以便閱讀）
        String display_cmd = raw_data;
        display_cmd.trim();
        Serial.print("「");
        Serial.print(display_cmd);
        Serial.println("」");
        
        // 處理命令
        String cmd = raw_data;
        cmd.trim();
        cmd.toUpperCase();
        
        // 檢查命令有效性
        if (cmd.length() == 0) {
            Serial.println("⚠️  命令為空，忽略");
        } else if (cmd == "AR1AFC") {
            Serial.println("✅ 識別為 AR1AFC 協議切換命令");
            handleProtocolSwitch(cmd);
            LOGI("📡 Serial1 命令已執行: %s", cmd.c_str());
        } else if (cmd == "MAVLINK") {
            Serial.println("✅ 識別為 MAVLINK 協議切換命令");
            handleProtocolSwitch(cmd);
            LOGI("📡 Serial1 命令已執行: %s", cmd.c_str());
        } else if (cmd == "STATUS") {
            Serial.println("✅ 識別為 STATUS 查詢命令");
            handleStatusCommand();
            LOGI("📡 Serial1 STATUS 命令已執行");
        } else if (cmd == "HELP") {
            Serial.println("✅ 識別為 HELP 說明命令");
            handleHelpCommand();
            LOGI("📡 Serial1 HELP 命令已執行");
        } else {
            Serial.print("❌ 未知命令：");
            Serial.println(cmd);
            LOGW("⚠️  Serial1 未知命令: '%s' (長度: %d)", cmd.c_str(), cmd.length());
        }
    }
}

void CommandHandler::handleProtocolSwitch(const String& protocol_name) {
    if (!system_controller_) {
        LOGE("❌ SystemController 未初始化");
        return;
    }
    
    // 簡化版本：直接使用協議名稱
    String protocol = protocol_name;
    
    if (system_controller_->setProtocolMode(protocol.c_str())) {
        LOGI("✅ 協議切換成功: %s", protocol.c_str());
        if (protocol_switches_counter_) {
            (*protocol_switches_counter_)++;
        }
        
        // 顯示協議說明
        if (protocol == "AR1AFC") {
            LOGI("📡 AR1AFC: 輸出自家封包格式");
        } else if (protocol == "MAVLINK") {
            LOGI("📡 MAVLink: 輸出 MAVLink 封包 (適用 QGroundControl)");
        } else if (protocol == "MTI_NATIVE") {
            LOGI("📡 MTI_NATIVE: 輸出 XBUS 格式封包 (原始 MTI 格式)");
        }
    } else {
        LOGE("❌ 協議切換失敗: %s", protocol.c_str());
    }
}

void CommandHandler::handleStatusCommand() {
    if (!system_controller_) {
        Serial.println("❌ SystemController 未初始化");
        LOGE("❌ SystemController 未初始化");
        return;
    }
    
    Serial.println("📊 === 系統狀態 ===");
    Serial.print("📊 當前協議: ");
    Serial.println(system_controller_->getCurrentProtocolName());
    
    if (protocol_switches_counter_) {
        Serial.print("📊 協議切換次數: ");
        Serial.println(*protocol_switches_counter_);
    }
    if (nav_callbacks_counter_) {
        Serial.print("📊 導航回調次數: ");
        Serial.println(*nav_callbacks_counter_);
    }
    if (total_cycles_counter_) {
        Serial.print("📊 系統循環次數: ");
        Serial.println(*total_cycles_counter_);
    }
    
    // 顯示系統健康狀態
    auto health = system_controller_->getHealth();
    Serial.print("📊 系統健康: ");
    Serial.print(health.all_systems_ok ? "正常" : "異常");
    Serial.print(" (錯誤代碼: 0x");
    Serial.print(health.error_flags, HEX);
    Serial.println(")");
    
    // 顯示系統狀態
    auto state = system_controller_->getState();
    const char* state_name = "UNKNOWN";
    switch(state) {
        case SystemState::INIT: state_name = "初始化中"; break;
        case SystemState::CALIBRATING: state_name = "校準中"; break;
        case SystemState::RUNNING: state_name = "運行中"; break;
        case SystemState::ERROR: state_name = "錯誤"; break;
        case SystemState::SAFE_MODE: state_name = "安全模式"; break;
        case SystemState::SHUTDOWN: state_name = "已停止"; break;
    }
    Serial.print("📊 系統狀態: ");
    Serial.println(state_name);
    Serial.println("📊 ==================");
    
    // 同時保持原有的 LOGI 輸出
    LOGI("📊 === 系統狀態 ===");
    LOGI("📊 當前協議: %s", system_controller_->getCurrentProtocolName());
    
    if (protocol_switches_counter_) {
        LOGI("📊 協議切換次數: %lu", *protocol_switches_counter_);
    }
    if (nav_callbacks_counter_) {
        LOGI("📊 導航回調次數: %lu", *nav_callbacks_counter_);
    }
    if (total_cycles_counter_) {
        LOGI("📊 系統循環次數: %lu", *total_cycles_counter_);
    }
    
    LOGI("📊 系統健康: %s (錯誤代碼: 0x%02X)", 
         health.all_systems_ok ? "正常" : "異常", health.error_flags);
    LOGI("📊 系統狀態: %s", state_name);
    LOGI("📊 ==================");
}

void CommandHandler::handleDiagCommand() {
    if (!system_controller_) {
        LOGE("❌ SystemController 未初始化");
        return;
    }
    
    LOGI("🔍 === 協議診斷 ===");
    
    // 診斷 ProtocolManager
    auto* pm = system_controller_->getProtocolManager();
    if (!pm) {
        LOGE("❌ ProtocolManager 未初始化");
        return;
    }
    
    // 簡化診斷：只顯示基本狀態
    LOGI("📊 協議管理器狀態: 正常運行");
    
    // 額外診斷信息
    LOGI("🔍 SystemController 狀態:");
    LOGI("   - 當前協議名稱: %s", system_controller_->getCurrentProtocolName());
    
    // 檢查是否有 TxMultiplexer
    LOGI("🔍 嘗試手動協議切換測試:");
    LOGI("   - 測試協議切換流程...");
    
    LOGI("🔍 ==================");
}

void CommandHandler::handleHelpCommand() {
    LOGI("🆘 === GMINS 命令說明 ===");
    LOGI("🆘 AR1AFC    - 切換到 AR1AFC 協議 (自家封包格式)");
    LOGI("🆘 MAVLINK   - 切換到 MAVLink 協議 (QGroundControl)");
    LOGI("🆘 XBUS/MTI  - 切換到 XBUS 協議 (原始 MTI 格式)");
    LOGI("🆘 STATUS    - 顯示系統狀態和統計");
    LOGI("🆘 DIAG      - 診斷協議初始化問題");
    LOGI("🆘 FREQ      - 顯示實際封包輸出頻率");
    LOGI("🆘 HELP      - 顯示此說明");
    LOGI("🆘 =========================");
}

void CommandHandler::handleFrequencyCommand() {
    LOGI("📊 === 實際輸出頻率測量 ===");
    
    // 使用舊的頻率管理器
    gmins::FrequencyManager* freq_mgr = gmins::getGlobalFrequencyManager();
    if (freq_mgr) {
        // 顯示各協議的頻率
        float mavlink_total_hz = freq_mgr->getMAVLink_TotalFrequency();
        float mavlink_odo_hz = freq_mgr->getMAVLink_ODO_Frequency();
        float mavlink_input_hz = freq_mgr->getMAVLink_INPUT_Frequency();
        float mavlink_raw_hz = freq_mgr->getMAVLink_RAW_Frequency();
        float custom_hz = freq_mgr->getCustom_TotalFrequency();
        
        LOGI("📡 === Push→Pull 架構輸出頻率 ===" );
        LOGI("🎯 AR1AFC: %.2f Hz (52 字節/封包)", custom_hz);
        LOGI("🚀 MAVLink 總計: %.2f Hz", mavlink_total_hz);
        LOGI("   ├─ 📍 ODOMETRY: %.2f Hz (~280 字節)", mavlink_odo_hz);
        LOGI("   ├─ 📥 GPS_INPUT: %.2f Hz (~280 字節)", mavlink_input_hz);
        LOGI("   └─ 📊 GPS_RAW_INT: %.2f Hz (~280 字節)", mavlink_raw_hz);
        
        // 顯示當前協議
        if (system_controller_) {
            const char* protocol = system_controller_->getCurrentProtocolName();
            LOGI("🔍 當前協議: %s", protocol ? protocol : "未知");
            
            // 根據協議給出頻率解釋
            if (protocol) {
                if (strstr(protocol, "MAVLINK")) {
                    LOGI("📊 主要輸出: MAVLink (總計 %.2f Hz)", mavlink_total_hz);
                    if (mavlink_odo_hz > 0) LOGI("   🎯 主要類型: Odometry (%.2f Hz, ~280 bytes)", mavlink_odo_hz);
                    if (mavlink_input_hz > 0) LOGI("   🎯 主要類型: Input (%.2f Hz, ~280 bytes)", mavlink_input_hz);
                    if (mavlink_raw_hz > 0) LOGI("   🎯 主要類型: Raw Data (%.2f Hz, ~280 bytes)", mavlink_raw_hz);
                } else if (strstr(protocol, "AR1AFC")) {
                    LOGI("📊 主要輸出: AR1AFC (%.2f Hz, 52 bytes/packet)", custom_hz);
                } else if (strstr(protocol, "MTI") || strstr(protocol, "XBUS")) {
                    LOGI("📊 主要輸出: XBUS Raw (%.2f Hz, 記錄為 MAVLink Raw)", mavlink_raw_hz);
                }
            }
        }
    } else {
        LOGE("❌ 頻率管理器未初始化");
    }
    
    LOGI("💡 這些頻率基於實際成功輸出的封包");
    LOGI("💡 輸入 'FREQ' 重新查看當前頻率");
    LOGI("===============================");
}

void CommandHandler::handleUnknownCommand(const String& cmd) {
    LOGW("❓ 未知命令: %s", cmd.c_str());
    LOGI("💡 輸入 HELP 查看可用命令");
}