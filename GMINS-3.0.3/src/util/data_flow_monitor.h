#ifndef DATA_FLOW_MONITOR_H
#define DATA_FLOW_MONITOR_H

#include <stdint.h>
#include <stddef.h>

/**
 * @file data_flow_monitor.h
 * @brief 通用數據流量與頻率監控模組
 * @author GMINS Team
 * @version 1.0
 * 
 * 用途：
 * - 監控UART、感測器、通訊介面的數據流量
 * - 統計頻率、字節數、封包數
 * - 提供區間統計與累積統計
 * - 可重複使用於不同項目
 */

namespace monitor {

    /**
     * @brief 數據流量統計結構
     */
    struct DataFlowStats {
        // 累積統計 (64位避免長時間運行溢位)
        uint64_t total_bytes;           // 總字節數
        uint64_t total_packets;         // 總封包數
        uint64_t total_operations;      // 總操作次數
        
        // 區間統計 (5秒間隔)
        uint32_t interval_bytes;        // 區間字節數
        uint32_t interval_packets;      // 區間封包數  
        uint32_t interval_operations;   // 區間操作次數
        
        // 頻率計算
        float byte_rate_Bps;            // 字節頻率 (Bytes/sec) - 大寫B避免與bps混淆
        float packet_rate_hz;           // 封包頻率 (Hz)
        float operation_rate_hz;        // 操作頻率 (Hz)
        
        // 時間戳記
        uint32_t last_update_time;      // 最後更新時間
        uint32_t start_time;            // 開始監控時間
        
        // 狀態
        bool is_active;                 // 監控是否啟用
        char channel_name[64];          // 通道名稱 (複製儲存，避免指針問題)
    };

    /**
     * @brief 數據流量監控器類別
     */
    class DataFlowMonitor {
    private:
        DataFlowStats stats_;
        uint32_t last_report_time_;
        uint32_t report_interval_ms_;
        
        // 區間統計快照 (64位配合累積計數)
        uint64_t last_bytes_snapshot_;
        uint64_t last_packets_snapshot_;
        uint64_t last_operations_snapshot_;
        
    public:
        /**
         * @brief 建構子
         * @param channel_name 通道名稱
         * @param report_interval_ms 報告間隔 (毫秒，預設5000ms)
         */
        DataFlowMonitor(const char* channel_name, uint32_t report_interval_ms = 5000);
        
        /**
         * @brief 解構子
         */
        ~DataFlowMonitor();
        
        /**
         * @brief 記錄字節數據
         * @param bytes 字節數
         * @warning 不要在ISR中調用！包含LOGD輸出不適合中斷環境
         */
        void recordBytes(uint32_t bytes);
        
        /**
         * @brief 記錄封包數據
         * @param packets 封包數
         * @warning 不要在ISR中調用！包含LOGD輸出不適合中斷環境
         */
        void recordPackets(uint32_t packets);
        
        /**
         * @brief 記錄操作數據
         * @param operations 操作次數
         * @warning 不要在ISR中調用！包含LOGD輸出不適合中斷環境
         */
        void recordOperations(uint32_t operations);
        
        /**
         * @brief 更新統計並檢查是否需要報告
         * @return true 如果需要報告
         */
        bool shouldReport();
        
        /**
         * @brief 生成並顯示統計報告
         */
        void generateReport();
        
        /**
         * @brief 取得當前統計數據
         * @return 統計數據結構
         */
        DataFlowStats getStats() const;
        
        /**
         * @brief 重置所有統計數據
         */
        void resetStats();
        
        /**
         * @brief 設定報告間隔
         * @param interval_ms 間隔時間 (毫秒)
         */
        void setReportInterval(uint32_t interval_ms);
        
        /**
         * @brief 啟用/停用監控
         * @param enable true為啟用
         */
        void setEnabled(bool enable);
        
        /**
         * @brief 檢查監控是否啟用
         * @return true 如果啟用
         */
        bool isEnabled() const;

    private:
        /**
         * @brief 計算區間統計
         */
        void calculateIntervalStats();
        
        /**
         * @brief 更新時間戳記
         */
        void updateTimestamp();
    };

    /**
     * @brief 多通道監控管理器
     */
    class MultiChannelMonitor {
    private:
        static const size_t MAX_CHANNELS = 8;
        DataFlowMonitor* monitors_[MAX_CHANNELS];
        size_t monitor_count_;
        
    public:
        /**
         * @brief 建構子
         */
        MultiChannelMonitor();
        
        /**
         * @brief 解構子
         */
        ~MultiChannelMonitor();
        
        /**
         * @brief 註冊新的監控通道
         * @param monitor 監控器指標
         * @return true 如果成功註冊
         */
        bool registerMonitor(DataFlowMonitor* monitor);
        
        /**
         * @brief 移除監控通道
         * @param monitor 監控器指標
         */
        void unregisterMonitor(DataFlowMonitor* monitor);
        
        /**
         * @brief 更新所有通道並生成報告
         */
        void updateAll();
        
        /**
         * @brief 重置所有通道統計
         */
        void resetAll();
        
        /**
         * @brief 生成全通道統計摘要
         */
        void generateSummaryReport();
    };

    // 全域工廠函數
    
    /**
     * @brief 創建UART監控器
     * @param channel_name 通道名稱
     * @return 監控器指標
     */
    DataFlowMonitor* createUARTMonitor(const char* channel_name);
    
    /**
     * @brief 創建感測器監控器
     * @param sensor_name 感測器名稱  
     * @return 監控器指標
     */
    DataFlowMonitor* createSensorMonitor(const char* sensor_name);
    
    /**
     * @brief 創建通訊協議監控器
     * @param protocol_name 協議名稱
     * @return 監控器指標
     */
    DataFlowMonitor* createProtocolMonitor(const char* protocol_name);

} // namespace monitor

#endif // DATA_FLOW_MONITOR_H