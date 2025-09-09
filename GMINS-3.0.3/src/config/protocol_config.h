#pragma once

#include <stdint.h>

/**
 * @file protocol_config.h
 * @brief 協議專用配置參數
 * 
 * 定義每個協議的優化參數，包括：
 * - 推薦波特率
 * - 緩衝區大小
 * - 超時設定
 * - 特殊配置標誌
 */

namespace protocol_config {

/**
 * @brief 協議配置結構
 */
struct ProtocolConfig {
    uint32_t recommended_baud_rate;     // 推薦波特率
    uint16_t tx_buffer_size;            // 發送緩衝區大小
    uint16_t rx_buffer_size;            // 接收緩衝區大小
    uint32_t flush_timeout_ms;          // 緩衝區清空超時
    bool requires_mode_header;          // 是否需要模式切換標頭
    const char* description;            // 配置描述
};

// AR-1A-FC 協議配置 - 中速優化平衡
constexpr ProtocolConfig AR1AFC_CONFIG = {
    .recommended_baud_rate = 230400,    // 適中速率，52字節@122Hz僅佔2.8%
    .tx_buffer_size = 512,              // 適中緩衝區
    .rx_buffer_size = 256,              // 單向輸出，小接收緩衝
    .flush_timeout_ms = 100,            // 快速切換
    .requires_mode_header = true,
    .description = "AR-1A-FC 高頻封包格式 (122Hz@230400)"
};

// MAVLink 協議配置 - QGroundControl高速優化
constexpr ProtocolConfig MAVLINK_CONFIG = {
    .recommended_baud_rate = 460800,    // 高速率，支援複雜MAVLink封包
    .tx_buffer_size = 2048,             // MAVLink封包較大且變長
    .rx_buffer_size = 1024,             // 雙向通訊需要較大接收緩衝
    .flush_timeout_ms = 200,            // 較寬鬆的超時
    .requires_mode_header = true,
    .description = "MAVLink v2.0 雙向通訊 (460800 baud)"
};

// MTI Native 協議配置 - 原始數據輸出
constexpr ProtocolConfig MTI_NATIVE_CONFIG = {
    .recommended_baud_rate = 115200,    // MTI標準速率
    .tx_buffer_size = 512,              // 較小封包
    .rx_buffer_size = 128,              // 主要輸出
    .flush_timeout_ms = 50,             // 快速切換
    .requires_mode_header = false,      // 原始格式不需要
    .description = "MTI-680 原始 XBUS 格式"
};

// 默認配置 - 保守設定
constexpr ProtocolConfig DEFAULT_CONFIG = {
    .recommended_baud_rate = 115200,
    .tx_buffer_size = 256,
    .rx_buffer_size = 256,
    .flush_timeout_ms = 500,
    .requires_mode_header = false,
    .description = "默認保守配置"
};

/**
 * @brief 根據協議類型獲取配置
 * @param protocol_type 協議類型枚舉值
 * @return 對應的協議配置
 */
inline const ProtocolConfig& getProtocolConfig(int protocol_type) {
    switch (protocol_type) {
        case 0: return AR1AFC_CONFIG;       // ProtocolType::AR1AFC
        case 1: return MAVLINK_CONFIG;      // ProtocolType::MAVLINK  
        case 2: return MTI_NATIVE_CONFIG;   // ProtocolType::MTI_NATIVE
        default: return DEFAULT_CONFIG;
    }
}

/**
 * @brief 波特率相容性檢查
 * @param current_baud 當前波特率
 * @param target_baud 目標波特率
 * @return true 如果切換是安全的
 */
inline bool isBaudRateCompatible(uint32_t current_baud, uint32_t target_baud) {
    // 允許的波特率範圍
    const uint32_t MIN_BAUD = 9600;
    const uint32_t MAX_BAUD = 2000000;
    
    return (target_baud >= MIN_BAUD && target_baud <= MAX_BAUD);
}

/**
 * @brief 預定義的安全波特率列表
 */
constexpr uint32_t SAFE_BAUD_RATES[] = {
    9600, 19200, 38400, 57600, 115200, 
    230400, 460800, 921600, 1000000, 2000000
};

constexpr size_t SAFE_BAUD_RATES_COUNT = sizeof(SAFE_BAUD_RATES) / sizeof(SAFE_BAUD_RATES[0]);

} // namespace protocol_config