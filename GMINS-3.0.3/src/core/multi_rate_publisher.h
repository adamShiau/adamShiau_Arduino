#pragma once

#include "../data/data_types.h"
#include <Arduino.h>
#include <vector>

/**
 * @brief 多頻率發佈器 - 讓不同協議有各自的調用頻率
 * 
 * 用法：
 * - 每個協議註冊自己的目標頻率
 * - NavigationState 更新時，只有到時間的協議才會被調用
 * - 各協議可以有不同的輸出頻率，互不干擾
 */
class MultiRatePublisher {
public:
    // 協議回調函數類型
    using ProtocolCallback = void(*)(const NavigationState&);
    
    struct ProtocolChannel {
        const char* name;
        ProtocolCallback callback;
        uint32_t target_interval_us;  // 目標間隔（微秒）
        uint32_t last_call_time_us;   // 上次調用時間
        bool enabled;
        
        // 統計
        uint32_t call_count;
        uint32_t skip_count;
    };
    
    MultiRatePublisher() : channels_() {}
    
    /**
     * @brief 註冊協議通道
     * @param name 協議名稱
     * @param callback 協議回調函數
     * @param target_hz 目標頻率（Hz）
     */
    void registerChannel(const char* name, ProtocolCallback callback, float target_hz) {
        uint32_t interval_us = (uint32_t)(1000000.0f / target_hz);
        
        channels_.push_back({
            .name = name,
            .callback = callback,
            .target_interval_us = interval_us,
            .last_call_time_us = 0,
            .enabled = true,
            .call_count = 0,
            .skip_count = 0
        });
        
        LOGI("📡 註冊協議通道: %s @ %.1fHz (間隔 %luμs)", name, target_hz, interval_us);
    }
    
    /**
     * @brief 發佈 NavigationState 到所有到時間的協議
     * @param nav_state 導航狀態
     */
    void publish(const NavigationState& nav_state) {
        uint32_t now_us = micros();
        
        for (auto& channel : channels_) {
            if (!channel.enabled) continue;
            
            // 檢查是否到了調用時間
            if (now_us - channel.last_call_time_us >= channel.target_interval_us) {
                // 調用協議回調
                channel.callback(nav_state);
                channel.last_call_time_us = now_us;
                channel.call_count++;
            } else {
                channel.skip_count++;
            }
        }
        
        // 定期輸出統計
        static uint32_t last_stats_time = 0;
        if (now_us - last_stats_time >= 5000000) {  // 每5秒
            printStats();
            last_stats_time = now_us;
        }
    }
    
    /**
     * @brief 啟用/禁用協議通道
     */
    void setChannelEnabled(const char* name, bool enabled) {
        for (auto& channel : channels_) {
            if (strcmp(channel.name, name) == 0) {
                channel.enabled = enabled;
                LOGI("📡 協議通道 %s: %s", name, enabled ? "啟用" : "禁用");
                return;
            }
        }
        LOGW("❌ 找不到協議通道: %s", name);
    }
    
private:
    std::vector<ProtocolChannel> channels_;
    
    void printStats() {
        LOGI("📊 多頻率發佈器統計:");
        for (const auto& channel : channels_) {
            if (channel.enabled) {
                float actual_hz = channel.call_count / 5.0f;  // 5秒統計
                LOGI("  %s: %.1fHz (調用:%lu 跳過:%lu)", 
                     channel.name, actual_hz, channel.call_count, channel.skip_count);
            }
        }
        
        // 重置計數器
        for (auto& channel : channels_) {
            channel.call_count = 0;
            channel.skip_count = 0;
        }
    }
};