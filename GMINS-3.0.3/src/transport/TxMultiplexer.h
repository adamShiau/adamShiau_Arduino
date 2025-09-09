#pragma once

#include <Arduino.h>
#include "ITransport.h"
#include "MonitoredTransport.h"
#include "../util/log.h"
#include <map>
#include <string>
#include <cstring>
#include <functional>

#define LOG_TAG "TX_MUX"

/**
 * @brief 傳輸互斥器
 * 
 * 確保同一時間只有一個協議可以持有特定的傳輸資源（如 Serial1）
 * 用於實現 AR1AFC/MAVLink/MTI 協議的互斥輸出
 */
class TxMultiplexer {
public:
    explicit TxMultiplexer(ITransport* transport);
    
    ~TxMultiplexer();
    
    // 禁止拷貝
    TxMultiplexer(const TxMultiplexer&) = delete;
    TxMultiplexer& operator=(const TxMultiplexer&) = delete;
    
    /**
     * @brief 獲取傳輸控制權
     * @param owner_name 請求者標識（用於日誌）
     * @return ITransport* 成功時返回監控傳輸指針，失敗返回 nullptr
     */
    ITransport* acquire(const char* owner_name);
    
    /**
     * @brief 釋放傳輸控制權
     * @param owner_name 當前持有者標識
     * @return true 成功釋放
     */
    bool release(const char* owner_name);
    
    /**
     * @brief 強制釋放（系統級操作）
     * 用於錯誤恢復或系統關閉
     */
    void forceRelease();
    
    /**
     * @brief 獲取當前持有者
     * @return const char* 當前持有者名稱，無人持有時返回 nullptr
     */
    const char* getCurrentOwner() const {
        return current_owner_[0] != '\0' ? current_owner_ : nullptr;
    }
    
    /**
     * @brief 檢查是否被特定持有者占用
     * @param owner_name 要檢查的持有者名稱
     * @return true 如果被該持有者占用
     */
    bool isOwnedBy(const char* owner_name) const {
        return owner_name && current_owner_[0] != '\0' && strcmp(current_owner_, owner_name) == 0;
    }
    
    /**
     * @brief 檢查傳輸是否空閒
     * @return true 如果無人持有
     */
    bool isFree() const {
        return current_owner_[0] == '\0';
    }
    
    /**
     * @brief 獲取底層傳輸（僅供狀態查詢）
     * 不提供控制權，僅用於狀態檢查
     */
    const ITransport* getTransport() const {
        return transport_;
    }
    
    /**
     * @brief 協議切換專用：先釋放，再獲取
     * @param old_owner 當前持有者
     * @param new_owner 新持有者
     * @return ITransport* 成功時返回 transport 指針
     */
    ITransport* switchOwner(const char* old_owner, const char* new_owner);
    
    /**
     * @brief 獲取協議輸出監控器
     * @return DataFlowMonitor* 監控器指針，用於集成到 MultiChannelMonitor
     */
    monitor::DataFlowMonitor* getOutputMonitor() const {
        return monitored_transport_ ? monitored_transport_->getMonitor() : nullptr;
    }

    /**
     * @brief 包裝的 write 方法，用於統計協議頻率
     * @param data 數據指針
     * @param length 數據長度
     * @param protocol_name 協議名稱（用於統計）
     * @return size_t 實際寫入的字節數
     */
    size_t writeWithStats(const uint8_t* data, size_t length, const char* protocol_name);
    
    /**
     * @brief 處理 FREQ 命令，顯示各協議的實際輸出頻率
     */
    // 🗑️ 已移除 handleFrequencyCommand() - 原本用於顯示統計
    
    /**
     * @brief 更新所有協議的頻率統計
     * 應該在主循環中定期調用
     */
    // 🗑️ 已移除 updateFrequencyStats() - 原本用於更新統計

private:
    ITransport* transport_;               // 底層傳輸
    MonitoredTransport* monitored_transport_;  // 監控傳輸裝飾器
    char current_owner_[16];              // 當前持有者標識（固定緩衝，避免懸掛指標）
    
    // 🗑️ 已移除危險的協議統計功能
    // 原本的 ProtocolStats 和 protocol_stats_ map 會導致記憶體累積
    
    /**
     * @brief 記錄協議活動統計
     * @param protocol_name 協議名稱
     * @param bytes 字節數
     */
    void recordProtocolActivity(const char* protocol_name, size_t bytes) const;
};

/**
 * @brief 傳輸 RAII 守衛
 * 
 * 自動管理傳輸控制權的獲取和釋放
 * 適用於單個函數內的短期傳輸使用
 */
class TxGuard {
public:
    TxGuard(TxMultiplexer* mux, const char* owner_name) 
        : mux_(mux), owner_name_(owner_name), transport_(nullptr) {
        transport_ = mux_->acquire(owner_name_);
    }
    
    ~TxGuard() {
        if (transport_ != nullptr) {
            mux_->release(owner_name_);
        }
    }
    
    // 禁止拷貝
    TxGuard(const TxGuard&) = delete;
    TxGuard& operator=(const TxGuard&) = delete;
    
    /**
     * @brief 檢查是否成功獲取傳輸
     */
    bool isValid() const {
        return transport_ != nullptr;
    }
    
    /**
     * @brief 獲取傳輸指針
     */
    ITransport* get() const {
        return transport_;
    }
    
    /**
     * @brief 重載 -> 操作符
     */
    ITransport* operator->() const {
        return transport_;
    }
    
    /**
     * @brief 重載 bool 轉換
     */
    explicit operator bool() const {
        return transport_ != nullptr;
    }

private:
    TxMultiplexer* mux_;
    const char* owner_name_;
    ITransport* transport_;
};