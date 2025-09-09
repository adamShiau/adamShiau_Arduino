// 暫時啟用LOG_INFO用於協議切換診斷
#define LOG_LEVEL_LOCAL LOG_INFO  // 顯示INFO及以上等級
// #define DISABLE_PROTOCOL_LOGS 1   // 暫時啟用協議LOG

#pragma once

/**
 * 雙模式協議管理器 - AR1AFC + MAVLink 簡化版本
 * 
 * 特點:
 * - 支持 AR1AFC (自家GUI) 和 MAVLink (Pixhawk) 模式切換
 * - 固定100Hz輪詢，無複雜TDD檢查器
 * - 極簡切換邏輯，無6步驟複雜序列
 * - 預分配協議實例，無動態創建
 * - 保持系統穩定性
 */

#include "../protocol/AR1AFCProtocol.h"
#include "../protocol/MAVLinkProtocol.h"
#include "../transport/TxMultiplexer.h"
#include "../util/log.h"

class ProtocolManagerDualMode {
public:
    enum class Protocol {
        AR1AFC,     // 自家GUI模式
        MAVLINK,    // Pixhawk模式
        NONE        // 無協議狀態
    };

    explicit ProtocolManagerDualMode(TxMultiplexer* tx_mux);
    ~ProtocolManagerDualMode();
    
    /**
     * 核心功能：固定100Hz持續更新
     */
    void continuousUpdate(const GNSSData& gnss_data, const NavigationState& nav_state, const IMUData& imu_data);
    
    /**
     * 簡化的協議切換 - 取代複雜的6步驟序列
     */
    bool switchToAR1AFC();
    bool switchToMAVLink();
    
    // 高性能版本：移除複雜的頻率調整方法
    
    /**
     * 通用協議切換（供串口命令使用）
     */
    bool setProtocolMode(const char* protocol_name);
    
    /**
     * 狀態查詢
     */
    const char* getCurrentProtocolName() const;
    bool isReady() const;
    
    /**
     * 數據快照更新方法
     */
    void updateGNSSSnapshot(const GNSSData& gnss_data, const NavigationState& nav_state);
    void updateIMUSnapshot(const IMUData& imu_data, const NavigationState& nav_state);

private:
    TxMultiplexer* tx_mux_;
    Protocol current_protocol_;
    
    // 預分配的協議實例
    AR1AFCProtocol* ar1afc_protocol_;
    MAVLinkProtocol* mavlink_protocol_;
    IProtocol* active_protocol_;  // 指向當前活躍協議
    
    // 數據快照：保存最新的感測器數據
    GNSSData latest_gnss_data_;
    IMUData latest_imu_data_;
    NavigationState latest_nav_state_;
    
    // 高性能版本：移除複雜的頻率控制和統計變數
    
    /**
     * 內部協議切換實現 - 極簡版本
     */
    bool switchToProtocol(Protocol target_protocol);
};

/**
 * 使用方式：
 * 
 * // 創建雙模式管理器
 * ProtocolManagerDualMode protocolManager(&txMultiplexer);
 * 
 * void loop() {
 *     // 固定100Hz更新，支持兩種模式
 *     protocolManager.continuousUpdate(gnss_data, nav_state, imu_data);
 * }
 * 
 * // 串口命令切換
 * if (Serial.available()) {
 *     String cmd = Serial.readStringUntil('\n');
 *     if (cmd == "AR1AFC") {
 *         protocolManager.switchToAR1AFC();
 *     } else if (cmd == "MAVLINK") {
 *         protocolManager.switchToMAVLink();
 *     }
 * }
 */