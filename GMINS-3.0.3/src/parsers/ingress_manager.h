/**
 * @file ingress_manager.h
 * @brief Ingress 數據管理器
 * 
 * 統一管理所有 Ingress Framer，提供簡化的介面供主程序使用。
 * 負責協調 HAL 數據源和各種協議 Framer 之間的數據流。
 */

#pragma once
#include "../comm/IByteSource.h"
#include "IFramer.h"
#include "xbus_framer.h"
#include "nmea_framer.h"

/**
 * @brief Ingress 數據管理器
 * 
 * 統一管理和協調各種協議的 Framer，簡化使用者介面。
 */
class IngressManager {
public:
    IngressManager();
    ~IngressManager() = default;
    
    // 禁止拷貝
    IngressManager(const IngressManager&) = delete;
    IngressManager& operator=(const IngressManager&) = delete;
    
    /**
     * @brief 初始化 Ingress 管理器
     * @return true 如果初始化成功
     */
    bool initialize();
    
    /**
     * @brief 關閉 Ingress 管理器
     */
    void shutdown();
    
    /**
     * @brief 處理一輪數據
     * 
     * 從各個數據源讀取數據並餵給對應的 Framer。
     * 應在主循環中定期調用。
     */
    void process();
    
    /**
     * @brief 設定 XBUS 封包回調
     * @param callback 回調函數
     */
    void setXbusFrameCallback(std::function<void(const uint8_t*, size_t)> callback);
    
    /**
     * @brief 設定 NMEA 句子回調
     * @param callback 回調函數
     */
    void setNmeaFrameCallback(std::function<void(const uint8_t*, size_t)> callback);
    
    /**
     * @brief 獲取 XBUS Framer 參考
     * @return XBUS Framer 參考
     */
    XbusFramer& getXbusFramer() { return xbus_framer_; }
    
    /**
     * @brief 獲取 NMEA Framer 參考
     * @return NMEA Framer 參考
     */
    NmeaFramer& getNmeaFramer() { return nmea_framer_; }
    
    /**
     * @brief 重置所有 Framer
     */
    void resetFramers();
    
    /**
     * @brief 報告統計信息
     */
    void reportStats() const;
    
    /**
     * @brief 重置統計信息
     */
    void resetStats();
    
    /**
     * @brief 檢查是否已初始化
     * @return true 如果已初始化
     */
    bool isInitialized() const { return initialized_; }

private:
    bool initialized_;                   // 初始化狀態
    XbusFramer xbus_framer_;            // XBUS 協議 Framer
    NmeaFramer nmea_framer_;            // NMEA 協議 Framer
    
    // 數據源（從 HAL 取得）
    IByteSource* imu_source_;           // IMU 數據源
    IByteSource* gnss_source_;          // GNSS 數據源
    
    // 內部統計
    struct {
        uint32_t process_calls = 0;      // process() 調用次數
        uint32_t total_imu_bytes = 0;    // 處理的 IMU 字節總數
        uint32_t total_gnss_bytes = 0;   // 處理的 GNSS 字節總數
        uint32_t last_report_time = 0;   // 上次報告時間
    } stats_;
    
    /**
     * @brief 更新數據源指針
     */
    void updateDataSources();
};