#pragma once

/**
 * @file xsens_mti_driver.h
 * @brief 簡化版 Xsens MTI 驅動程序
 * 
 * 純粹的命令發送器，不參與數據讀取或複雜檢查
 */

#include <Arduino.h>
#include "../comm/uart_interface.h"
#include "../util/log.h"

/**
 * @brief 簡化版 Xsens MTI 驅動程序
 * 
 * 只提供基本的命令發送功能，無錯誤檢查，無ACK等待
 */
class XsensMTIDriver {
public:
    /**
     * @brief 建構函數
     * @param uart_interface 用於與 MTI 通信的 UART 介面
     */
    explicit XsensMTIDriver(UARTInterface* uart_interface);
    ~XsensMTIDriver() = default;
    
    // 禁止拷貝
    XsensMTIDriver(const XsensMTIDriver&) = delete;
    XsensMTIDriver& operator=(const XsensMTIDriver&) = delete;
    
    /**
     * @brief 簡單發送命令 (不等待回應，不檢查錯誤)
     * @param cmd_id 命令 ID
     * @param data 命令數據 (可選)
     * @param data_len 數據長度
     */
    void sendCommandSimple(uint8_t cmd_id, const uint8_t* data = nullptr, uint8_t data_len = 0);
    
    /**
     * @brief 執行 MTI 設備初始化配置
     * 包含：配置模式 → 初始化 → 採樣頻率 → 輸出配置 → 測量模式 → 校準
     * 注意：不包含 Heading Reset，需要單獨調用 resetHeading()
     */
    void initialize();
    
    /**
     * @brief 重置 heading (YAW) 為 0
     * 使用 SETNOROTATION 命令進行 heading reset
     * @param current_yaw_deg 當前的 YAW 角度 (度數)，用於記錄重置前後的變化
     */
    void resetHeading(float current_yaw_deg = 0.0f);

private:
    UARTInterface* uart_interface_;   // UART 介面
    
    // XBUS 協議常數
    static constexpr uint8_t XBUS_PREAMBLE = 0xFA;
    static constexpr uint8_t XBUS_BUS_ID = 0xFF;
    
    /**
     * @brief 計算 XBUS 校驗和
     * @param data 數據緩衝區
     * @param length 數據長度
     * @return 校驗和值
     */
    uint8_t calculateChecksum(const uint8_t* data, size_t length);
};