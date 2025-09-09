#define LOG_TAG "MONITOR"

// ===== 監控模組日誌控制選項 (保護避免被上層覆蓋) =====
#ifndef LOG_LEVEL_LOCAL
// 🔧 開發調試：全部顯示
// #define LOG_LEVEL_LOCAL LOG_DEBUG
// ⚙️ 正常運行：標準訊息  
#define LOG_LEVEL_LOCAL LOG_INFO
// ⚠️ 生產環境：僅警告以上
// #define LOG_LEVEL_LOCAL LOG_WARN
#endif

#ifndef LOG_LEVEL_MASK_LOCAL
// 開發版本 (包含DEBUG)
// #define LOG_LEVEL_MASK_LOCAL (LOG_BIT_DEBUG | LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)
// ⚙️ 正常運行版本 (過濾DEBUG)
// 完全靜音（本檔）
#define LOG_LEVEL_MASK_LOCAL 0
// ⚠️ 生產環境版本
// #define LOG_LEVEL_MASK_LOCAL (LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)
#endif

#include "data_flow_monitor.h"
#include "log.h"
#include <Arduino.h>
#include <string.h>

namespace monitor {

    // ==================== DataFlowMonitor 實作 ====================
    
    DataFlowMonitor::DataFlowMonitor(const char* channel_name, uint32_t report_interval_ms) 
        : report_interval_ms_(report_interval_ms), last_report_time_(0),
          last_bytes_snapshot_(0), last_packets_snapshot_(0), last_operations_snapshot_(0) {
        
        // 初始化統計數據
        memset(&stats_, 0, sizeof(stats_));
        
        // 安全地複製通道名稱，避免指針失效問題
        if (channel_name) {
            strncpy(stats_.channel_name, channel_name, sizeof(stats_.channel_name) - 1);
            stats_.channel_name[sizeof(stats_.channel_name) - 1] = '\0'; // 確保字串結尾
        } else {
            strcpy(stats_.channel_name, "Unknown");
        }
        
        stats_.start_time = millis();
        stats_.last_update_time = stats_.start_time;
        stats_.is_active = true;
        
        last_report_time_ = stats_.start_time;
        
        LOGD("DataFlowMonitor created for '%s' (interval: %lu ms)", 
             stats_.channel_name, report_interval_ms_);
    }
    
    DataFlowMonitor::~DataFlowMonitor() {
        LOGD("DataFlowMonitor destroyed for '%s'", stats_.channel_name);
    }
    
    void DataFlowMonitor::recordBytes(uint32_t bytes) {
        if (!stats_.is_active) return;
        
        // 優化：保留累積數據但移除高頻日誌
        stats_.total_bytes += bytes;
        updateTimestamp();
        
        // 移除高頻日誌輸出減少CPU負載
        // LOGD("[%s] Recorded %lu bytes (total: %lu)", 
        //      stats_.channel_name, bytes, (uint32_t)stats_.total_bytes);
    }
    
    void DataFlowMonitor::recordPackets(uint32_t packets) {
        if (!stats_.is_active) return;
        
        // 優化：保留累積數據但移除高頻日誌
        stats_.total_packets += packets;
        updateTimestamp();
        
        // 移除高頻日誌輸出減少CPU負載
        // LOGD("[%s] Recorded %lu packets (total: %lu)", 
        //      stats_.channel_name, packets, (uint32_t)stats_.total_packets);
    }
    
    void DataFlowMonitor::recordOperations(uint32_t operations) {
        if (!stats_.is_active) return;
        
        // 優化：保留累積數據但移除高頻日誌
        stats_.total_operations += operations;
        updateTimestamp();
        
        // 移除高頻日誌輸出減少CPU負轉
        // LOGD("[%s] Recorded %lu operations (total: %lu)", 
        //      stats_.channel_name, operations, (uint32_t)stats_.total_operations);
    }
    
    bool DataFlowMonitor::shouldReport() {
        if (!stats_.is_active) return false;
        
        uint32_t current_time = millis();
        
        // 檢查是否到達報告間隔，更新統計和計算頻率
        if (current_time - last_report_time_ >= report_interval_ms_) {
            calculateIntervalStats();
            
            // 計算頻率 (使用實際經過時間)
            float actual_interval_seconds = (current_time - last_report_time_) / 1000.0f;
            if (actual_interval_seconds > 0) {
                stats_.byte_rate_Bps     = stats_.interval_bytes / actual_interval_seconds;
                stats_.packet_rate_hz    = stats_.interval_packets / actual_interval_seconds;
                stats_.operation_rate_hz = stats_.interval_operations / actual_interval_seconds;
            } else {
                stats_.byte_rate_Bps = 0.0f;
                stats_.packet_rate_hz = 0.0f;
                stats_.operation_rate_hz = 0.0f;
            }
            
            // 更新報告時間 (在計算頻率之後)
            last_report_time_ = current_time;
            return true;
        }
        
        return false;
    }
    
    void DataFlowMonitor::generateReport() {
        if (!stats_.is_active) return;
        
        uint32_t current_time = millis();
        float actual_interval_seconds = (current_time - last_report_time_) / 1000.0f;
        
        // 頻率已經在 shouldReport() 中計算過了，直接使用
        LOGI("=== %s 數據流量報告 (%.1f秒實際區間) ===", stats_.channel_name, actual_interval_seconds);
        
        // 區間統計 (加最小門檻避免雜訊)
        if (stats_.interval_bytes >= 1) {
            LOGI("📥 區間字節: %lu bytes | 頻率: %.2f Bytes/sec", 
                 stats_.interval_bytes, stats_.byte_rate_Bps);
        }
        
        if (stats_.interval_packets > 0) {
            LOGI("📦 區間封包: %lu 個 | 頻率: %.2f Hz", 
                 stats_.interval_packets, stats_.packet_rate_hz);
        }
        
        if (stats_.interval_operations > 0) {
            LOGI("⚙️ 區間操作: %lu 次 | 頻率: %.2f Hz", 
                 stats_.interval_operations, stats_.operation_rate_hz);
        }
        
        // 累積統計 (Arduino 兼容的 64 位顯示)
        uint32_t bytes_low = (uint32_t)(stats_.total_bytes & 0xFFFFFFFF);
        uint32_t packets_low = (uint32_t)(stats_.total_packets & 0xFFFFFFFF);
        uint32_t ops_low = (uint32_t)(stats_.total_operations & 0xFFFFFFFF);
        
        LOGI("📊 累積總計: %lu bytes, %lu packets, %lu ops", 
             bytes_low, packets_low, ops_low);
        
        // 執行時間
        uint32_t uptime_seconds = (current_time - stats_.start_time) / 1000;
        LOGI("⏱️ 監控時間: %lu 秒", uptime_seconds);
        
        LOG_RAW(""); // 空行分隔
        
        // 更新報告時間
        last_report_time_ = current_time;
    }
    
    DataFlowStats DataFlowMonitor::getStats() const {
        return stats_;
    }
    
    void DataFlowMonitor::resetStats() {
        LOGI("重置 %s 監控統計數據...", stats_.channel_name);
        
        // 保留配置參數
        char name_backup[64];
        strncpy(name_backup, stats_.channel_name, sizeof(name_backup) - 1);
        name_backup[sizeof(name_backup) - 1] = '\0';
        bool active = stats_.is_active;
        
        // 重置統計數據
        memset(&stats_, 0, sizeof(stats_));
        strncpy(stats_.channel_name, name_backup, sizeof(stats_.channel_name) - 1);
        stats_.channel_name[sizeof(stats_.channel_name) - 1] = '\0';
        stats_.is_active = active;
        stats_.start_time = millis();
        stats_.last_update_time = stats_.start_time;
        
        // 重置快照
        last_bytes_snapshot_ = 0;
        last_packets_snapshot_ = 0;
        last_operations_snapshot_ = 0;
        last_report_time_ = stats_.start_time;
    }
    
    void DataFlowMonitor::setReportInterval(uint32_t interval_ms) {
        report_interval_ms_ = interval_ms;
        LOGD("[%s] Report interval set to %lu ms", stats_.channel_name, interval_ms);
    }
    
    void DataFlowMonitor::setEnabled(bool enable) {
        stats_.is_active = enable;
        LOGI("[%s] Monitor %s", stats_.channel_name, enable ? "enabled" : "disabled");
    }
    
    bool DataFlowMonitor::isEnabled() const {
        return stats_.is_active;
    }
    
    void DataFlowMonitor::calculateIntervalStats() {
        // 計算區間差異 (64位減法安全)
        stats_.interval_bytes = (uint32_t)(stats_.total_bytes - last_bytes_snapshot_);
        stats_.interval_packets = (uint32_t)(stats_.total_packets - last_packets_snapshot_);
        stats_.interval_operations = (uint32_t)(stats_.total_operations - last_operations_snapshot_);
        
        // 更新快照
        last_bytes_snapshot_ = stats_.total_bytes;
        last_packets_snapshot_ = stats_.total_packets;
        last_operations_snapshot_ = stats_.total_operations;
    }
    
    void DataFlowMonitor::updateTimestamp() {
        stats_.last_update_time = millis();
    }
    
    // ==================== MultiChannelMonitor 實作 ====================
    
    MultiChannelMonitor::MultiChannelMonitor() : monitor_count_(0) {
        // 初始化監控器陣列
        for (size_t i = 0; i < MAX_CHANNELS; i++) {
            monitors_[i] = nullptr;
        }
        
        LOGI("MultiChannelMonitor created (max channels: %d)", MAX_CHANNELS);
    }
    
    MultiChannelMonitor::~MultiChannelMonitor() {
        // 注意：不刪除監控器，因為它們可能在其他地方管理
        LOGI("MultiChannelMonitor destroyed (%d channels)", monitor_count_);
    }
    
    bool MultiChannelMonitor::registerMonitor(DataFlowMonitor* monitor) {
        if (!monitor || monitor_count_ >= MAX_CHANNELS) {
            LOGE("Failed to register monitor (count: %d/%d)", monitor_count_, MAX_CHANNELS);
            return false;
        }
        
        monitors_[monitor_count_++] = monitor;
        LOGI("Registered monitor '%s' (total: %d)", 
             monitor->getStats().channel_name, monitor_count_);
        return true;
    }
    
    void MultiChannelMonitor::unregisterMonitor(DataFlowMonitor* monitor) {
        for (size_t i = 0; i < monitor_count_; i++) {
            if (monitors_[i] == monitor) {
                // 移除並壓縮陣列
                for (size_t j = i; j < monitor_count_ - 1; j++) {
                    monitors_[j] = monitors_[j + 1];
                }
                monitor_count_--;
                monitors_[monitor_count_] = nullptr;
                
                LOGI("Unregistered monitor (remaining: %d)", monitor_count_);
                return;
            }
        }
        LOGW("Monitor not found for unregistration");
    }
    
    void MultiChannelMonitor::updateAll() {
        // 只更新所有監控器的統計，不自動觸發報告
        for (size_t i = 0; i < monitor_count_; i++) {
            if (monitors_[i]) {
                monitors_[i]->shouldReport(); // 僅更新統計
            }
        }
    }
    
    void MultiChannelMonitor::resetAll() {
        LOGI("Resetting all %d monitors...", monitor_count_);
        for (size_t i = 0; i < monitor_count_; i++) {
            if (monitors_[i]) {
                monitors_[i]->resetStats();
            }
        }
    }
    
    void MultiChannelMonitor::generateSummaryReport() {
        LOGI("=== 多通道監控摘要 ===");
        LOGI("註冊通道數: %d", monitor_count_);
        
        uint32_t total_bytes = 0;
        uint32_t total_packets = 0;
        uint32_t active_channels = 0;
        
        for (size_t i = 0; i < monitor_count_; i++) {
            if (monitors_[i]) {
                DataFlowStats stats = monitors_[i]->getStats();
                total_bytes += stats.total_bytes;
                total_packets += stats.total_packets;
                
                if (stats.is_active) {
                    active_channels++;
                }
                
                LOGI("  [%d] %s: %lu bytes, %lu packets, %s", 
                     i, stats.channel_name, stats.total_bytes, stats.total_packets,
                     stats.is_active ? "active" : "inactive");
            }
        }
        
        LOGI("活躍通道: %lu/%d", active_channels, monitor_count_);
        LOGI("系統總計: %lu bytes, %lu packets", total_bytes, total_packets);
        LOGI("========================");
    }
    
    // ==================== 工廠函數實作 ====================
    
    DataFlowMonitor* createUARTMonitor(const char* channel_name) {
        char full_name[64];
        snprintf(full_name, sizeof(full_name), "UART-%s", channel_name);
        return new DataFlowMonitor(full_name, 5000); // 5秒間隔
    }
    
    DataFlowMonitor* createSensorMonitor(const char* sensor_name) {
        char full_name[64];
        snprintf(full_name, sizeof(full_name), "SENSOR-%s", sensor_name);
        return new DataFlowMonitor(full_name, 5000); // 5秒間隔
    }
    
    DataFlowMonitor* createProtocolMonitor(const char* protocol_name) {
        char full_name[64];
        snprintf(full_name, sizeof(full_name), "PROTO-%s", protocol_name);
        return new DataFlowMonitor(full_name, 5000); // 5秒間隔
    }

} // namespace monitor