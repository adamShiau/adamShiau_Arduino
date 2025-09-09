#ifndef GMINS_FREQUENCY_MANAGER_H
#define GMINS_FREQUENCY_MANAGER_H

#include "data_flow_monitor.h"
#include <stdint.h>

/**
 * @file gmins_frequency_manager.h
 * @brief GMINS專案專用的頻率監控管理器
 * @author GMINS Team
 * @version 1.0
 * 
 * 專門針對GMINS項目的所有數據流進行頻率監控：
 * - MTI/XSENS IMU 頻率
 * - GNSS 三種封包頻率 (GNGGA, GNRMC, PLSHD)
 * - MAVLink 輸出頻率 (ODO, INPUT, RAW)
 * - 自定義封包頻率
 */

namespace gmins {

    /**
     * @brief GMINS專用的頻率管理器
     * 整合所有感測器和輸出的頻率監控
     */
    class FrequencyManager {
    public:
        // 感測器輸入監控器
        enum InputSensor {
            MTI_IMU = 0,        // XSENS MTI IMU
            GNSS_GNGGA,         // GNSS GNGGA封包
            GNSS_GNRMC,         // GNSS GNRMC封包  
            GNSS_PLSHD,         // GNSS PLSHD封包
            INPUT_SENSOR_COUNT
        };
        
        // MAVLink輸出監控器
        enum OutputMAVLink {
            MAVLINK_ODO = 0,    // MAVLink Odometry
            MAVLINK_INPUT,      // MAVLink Input
            MAVLINK_RAW,        // MAVLink Raw Data
            OUTPUT_MAVLINK_COUNT
        };
        
        // 自定義封包監控器
        enum CustomPacket {
            CUSTOM_FUSION = 0,  // 融合數據封包
            CUSTOM_STATUS,      // 狀態封包
            CUSTOM_DEBUG,       // 調試封包
            CUSTOM_PACKET_COUNT
        };

    private:
        // 監控器陣列
        monitor::DataFlowMonitor* input_monitors_[INPUT_SENSOR_COUNT];
        monitor::DataFlowMonitor* output_monitors_[OUTPUT_MAVLINK_COUNT];
        monitor::DataFlowMonitor* custom_monitors_[CUSTOM_PACKET_COUNT];
        
        // 多通道管理器
        monitor::MultiChannelMonitor mcm_;
        
        // 頻率目標值 (用於性能檢查)
        struct FrequencyTargets {
            float mti_target_hz = 100.0f;      // MTI目標100Hz
            float gnss_target_hz = 10.0f;      // GNSS目標10Hz
            float mavlink_target_hz = 50.0f;   // MAVLink目標50Hz
            float custom_target_hz = 20.0f;    // 自定義目標20Hz
        } targets_;
        
        // 監控設定
        bool initialized_ = false;
        uint32_t report_interval_ms_ = 5000;  // 5秒報告間隔

    public:
        /**
         * @brief 建構子
         * @param report_interval_ms 報告間隔(毫秒)
         */
        FrequencyManager(uint32_t report_interval_ms = 5000);
        
        /**
         * @brief 解構子
         */
        ~FrequencyManager();
        
        /**
         * @brief 初始化所有監控器
         * @return true 如果初始化成功
         */
        bool initialize();
        
        /**
         * @brief 關閉所有監控器
         */
        void shutdown();
        
        // ==================== 感測器輸入記錄 ====================
        
        /**
         * @brief 記錄MTI/XSENS IMU數據
         * @param bytes 字節數
         */
        void recordMTI(uint32_t bytes);
        
        /**
         * @brief 記錄GNSS GNGGA封包
         * @param bytes 字節數
         */
        void recordGNSS_GNGGA(uint32_t bytes);
        
        /**
         * @brief 記錄GNSS GNRMC封包
         * @param bytes 字節數
         */
        void recordGNSS_GNRMC(uint32_t bytes);
        
        /**
         * @brief 記錄GNSS PLSHD封包
         * @param bytes 字節數
         */
        void recordGNSS_PLSHD(uint32_t bytes);
        
        // ==================== MAVLink輸出記錄 ====================
        
        /**
         * @brief 記錄MAVLink Odometry輸出
         * @param bytes 字節數
         */
        void recordMAVLink_ODO(uint32_t bytes);
        
        /**
         * @brief 記錄MAVLink Input輸出
         * @param bytes 字節數
         */
        void recordMAVLink_INPUT(uint32_t bytes);
        
        /**
         * @brief 記錄MAVLink Raw輸出
         * @param bytes 字節數
         */
        void recordMAVLink_RAW(uint32_t bytes);
        
        // ==================== 自定義封包記錄 ====================
        
        /**
         * @brief 記錄融合數據封包
         * @param bytes 字節數
         */
        void recordCustom_Fusion(uint32_t bytes);
        
        /**
         * @brief 記錄狀態封包
         * @param bytes 字節數
         */
        void recordCustom_Status(uint32_t bytes);
        
        /**
         * @brief 記錄調試封包
         * @param bytes 字節數
         */
        void recordCustom_Debug(uint32_t bytes);
        
        // ==================== 頻率查詢與報告 ====================
        
        /**
         * @brief 取得MTI頻率
         * @return 頻率(Hz)
         */
        float getMTI_Frequency() const;
        
        /**
         * @brief 取得GNSS總頻率
         * @return 頻率(Hz)
         */
        float getGNSS_TotalFrequency() const;
        
        /**
         * @brief 取得MAVLink總頻率
         * @return 頻率(Hz)
         */
        float getMAVLink_TotalFrequency() const;
        
        /**
         * @brief 取得MAVLink Odometry頻率
         * @return 頻率(Hz)
         */
        float getMAVLink_ODO_Frequency() const;
        
        /**
         * @brief 取得MAVLink Input頻率
         * @return 頻率(Hz)
         */
        float getMAVLink_INPUT_Frequency() const;
        
        /**
         * @brief 取得MAVLink Raw頻率
         * @return 頻率(Hz)
         */
        float getMAVLink_RAW_Frequency() const;
        
        /**
         * @brief 取得自定義封包總頻率
         * @return 頻率(Hz)
         */
        float getCustom_TotalFrequency() const;
        
        /**
         * @brief 生成完整的頻率報告
         */
        void generateFrequencyReport();
        
        /**
         * @brief 生成簡要的頻率摘要
         */
        void generateQuickSummary();
        
        /**
         * @brief 更新所有監控器並檢查報告
         */
        void updateAll();
        
        /**
         * @brief 檢查頻率是否達到目標
         * @return true 如果所有頻率都在目標範圍內
         */
        bool checkFrequencyTargets();
        
        /**
         * @brief 設定頻率目標
         * @param mti_hz MTI目標頻率
         * @param gnss_hz GNSS目標頻率
         * @param mavlink_hz MAVLink目標頻率
         * @param custom_hz 自定義目標頻率
         */
        void setFrequencyTargets(float mti_hz, float gnss_hz, float mavlink_hz, float custom_hz);
        
        /**
         * @brief 重置所有統計
         */
        void resetAllStatistics();
        
        /**
         * @brief 設定報告間隔
         * @param interval_ms 間隔(毫秒)
         */
        void setReportInterval(uint32_t interval_ms);
        
    private:
        /**
         * @brief 創建並註冊監控器
         */
        void createMonitors();
        
        /**
         * @brief 檢查單一頻率目標
         * @param actual 實際頻率
         * @param target 目標頻率
         * @param tolerance 容忍度(百分比)
         * @return true 如果在容忍範圍內
         */
        bool checkSingleTarget(float actual, float target, float tolerance = 0.1f) const;
    };

    // ==================== 全域便利函數 ====================
    
    /**
     * @brief 取得全域頻率管理器實例
     * @return FrequencyManager指標
     */
    FrequencyManager* getGlobalFrequencyManager();
    
    /**
     * @brief 初始化全域頻率管理器
     * @param report_interval_ms 報告間隔
     * @return true 如果初始化成功
     */
    bool initializeGlobalFrequencyManager(uint32_t report_interval_ms = 5000);
    
    /**
     * @brief 關閉全域頻率管理器
     */
    void shutdownGlobalFrequencyManager();

} // namespace gmins

// ==================== 便利宏定義 ====================

// 快速記錄宏 (使用全域管理器，包含空指針檢查)
#define GMINS_RECORD_MTI(bytes)           do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordMTI(bytes); } while(0)
#define GMINS_RECORD_GNSS_GNGGA(bytes)    do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordGNSS_GNGGA(bytes); } while(0)
#define GMINS_RECORD_GNSS_GNRMC(bytes)    do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordGNSS_GNRMC(bytes); } while(0)
#define GMINS_RECORD_GNSS_PLSHD(bytes)    do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordGNSS_PLSHD(bytes); } while(0)
#define GMINS_RECORD_MAVLINK_ODO(bytes)   do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordMAVLink_ODO(bytes); } while(0)
#define GMINS_RECORD_MAVLINK_INPUT(bytes) do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordMAVLink_INPUT(bytes); } while(0)
#define GMINS_RECORD_MAVLINK_RAW(bytes)   do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordMAVLink_RAW(bytes); } while(0)
#define GMINS_RECORD_CUSTOM_FUSION(bytes) do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->recordCustom_Fusion(bytes); } while(0)

// 快速查詢宏 (包含空指針檢查)
#define GMINS_GET_MTI_HZ()        (gmins::getGlobalFrequencyManager() ? gmins::getGlobalFrequencyManager()->getMTI_Frequency() : 0.0f)
#define GMINS_GET_GNSS_HZ()       (gmins::getGlobalFrequencyManager() ? gmins::getGlobalFrequencyManager()->getGNSS_TotalFrequency() : 0.0f)
#define GMINS_GET_MAVLINK_HZ()    (gmins::getGlobalFrequencyManager() ? gmins::getGlobalFrequencyManager()->getMAVLink_TotalFrequency() : 0.0f)

// 快速報告宏 (包含空指針檢查)
#define GMINS_UPDATE_FREQUENCIES() do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->updateAll(); } while(0)
#define GMINS_REPORT_FREQUENCIES() do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->generateFrequencyReport(); } while(0)
#define GMINS_QUICK_SUMMARY()      do { auto* mgr = gmins::getGlobalFrequencyManager(); if (mgr) mgr->generateQuickSummary(); } while(0)

#endif // GMINS_FREQUENCY_MANAGER_H