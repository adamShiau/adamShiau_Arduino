/**
 * @file xbus_framer.h
 * @brief XSENS XBUS 協議封包組合器
 * 
 * 實現 XSENS MTI 設備的 XBUS 協議封包識別和組合功能。
 * 支援標準格式：0xFA 0xFF MID LEN [DATA...] CHK
 */

#pragma once
#include "IFramer.h"

/**
 * @brief XSENS XBUS 協議封包組合器
 * 
 * 處理 XSENS MTI 設備的 XBUS 協議數據流，將原始字節組合成完整的 XBUS 封包。
 * 
 * XBUS 封包格式：
 * - 0xFA: 同步字節 1
 * - 0xFF: 同步字節 2  
 * - MID: 訊息 ID
 * - LEN: 數據長度（0-254，255為擴展長度標記）
 * - [DATA...]: 可選數據段
 * - CHK: 校驗和（使總和為0的值）
 */
class XbusFramer : public IFramer {
public:
    XbusFramer();
    ~XbusFramer() = default;
    
    // IFramer 實現
    void feed(const uint8_t* data, size_t len) override;
    void reset() override;
    const char* getTypeName() const override { return "XbusFramer"; }
    const char* getStateDescription() const override;
    
    /**
     * @brief 獲取統計信息
     */
    struct Stats {
        uint32_t total_bytes_processed = 0;    // 處理的總字節數
        uint32_t frames_completed = 0;         // 成功組合的封包數
        uint32_t checksum_errors = 0;          // 校驗和錯誤數
        uint32_t oversized_frames = 0;         // 超大封包錯誤數
        uint32_t sync_errors = 0;              // 同步錯誤數
    };
    
    const Stats& getStats() const { return stats_; }
    void resetStats() { stats_ = {}; }

private:
    /**
     * @brief XBUS 協議狀態機
     */
    enum State {
        WAIT_FA,        // 等待第一個同步字節 0xFA
        WAIT_FF,        // 等待第二個同步字節 0xFF  
        READ_MID,       // 讀取訊息 ID
        READ_LEN,       // 讀取數據長度
        READ_DATA,      // 讀取數據段（如果有）
        READ_CHK        // 讀取校驗和
    };
    
    State state_;                    // 當前狀態
    static constexpr size_t MAX_FRAME_SIZE = 1024; // 最大封包大小 (支援擴展長度)
    uint8_t buffer_[MAX_FRAME_SIZE]; // 封包緩衝區
    size_t pos_;                     // 當前位置
    uint16_t data_length_;           // 當前封包的數據長度 (支援擴展長度)
    Stats stats_;                    // 統計信息
    
    /**
     * @brief 驗證 XBUS 封包校驗和
     * @param frame 完整封包（從 0xFA 開始）
     * @param len 封包總長度
     * @return true 如果校驗和正確（總和為0）
     */
    bool verifyChecksum(const uint8_t* frame, size_t len) const;
    
    /**
     * @brief 處理完整封包
     * 驗證校驗和並觸發回調
     */
    void processCompleteFrame();
    
    /**
     * @brief 獲取狀態名稱（調試用）
     */
    const char* getStateName(State state) const;
};