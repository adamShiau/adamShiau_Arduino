#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include "IByteSource.h"

#ifndef UART_IF_RING_SIZE
#define UART_IF_RING_SIZE 8192  // IMU 建議 >= 8KB；GNSS 4~8KB
#endif

/**
 * @brief Feed-based UART Interface with Ring Buffer
 * 
 * feed-based ring buffer，不碰硬體
 * 被 HAL 餵食的 ring buffer 設計
 * 
 * 用法：
 * - feed() 由 HAL sink 呼叫餵入數據
 * - HAL 層唯一讀者  
 * - Parser/Router 透過 IByteSource 介面讀取
 */
class UARTInterface final : public IByteSource {
public:
    UARTInterface() = default;
    ~UARTInterface() = default;

    // 禁止拷貝
    UARTInterface(const UARTInterface&) = delete;
    UARTInterface& operator=(const UARTInterface&) = delete;
    UARTInterface(UARTInterface&&) = delete;
    UARTInterface& operator=(UARTInterface&&) = delete;

    /**
     * @brief 初始化 - 不再控制硬體，保留為 no-op
     * @param baud 波特率（已由 HAL 處理）
     * @return 總是回傳 true
     */
    bool initialize(uint32_t baud = 0) { 
        (void)baud; // 避免未使用警告
        return true; 
    }

    /**
     * @brief HAL sink 餵食入口點
     * @param data 數據緩衝區
     * @param len 數據長度
     * 
     * 這是唯一讓 HAL 層餵入數據的地方
     * 使用 ring buffer 快速路徑與溢位處理
     */
    void feed(const uint8_t* data, size_t len);

    // IByteSource 實作
    /**
     * @brief 查詢 ring buffer 中可讀數據量
     * @return 可讀字節數
     */
    size_t available() const override;

    /**
     * @brief 從 ring buffer 讀取數據
     * @param dst 目標緩衝區
     * @param n 要讀取的最大字節數
     * @return 實際讀取字節數
     */
    size_t read(uint8_t* dst, size_t n) override;

    /**
     * @brief 設定 TX 回調函數（可選）
     * @param tx 傳送函數（例如連接到 HAL 的 Serial1）
     */
    void setTx(std::function<size_t(const uint8_t*, size_t)> tx) { 
        tx_ = std::move(tx); 
    }

    /**
     * @brief 透過 TX 回調傳送數據
     * @param src 數據來源
     * @param n 數據長度
     * @return 實際傳送字節數
     */
    size_t write(const uint8_t* src, size_t n) { 
        return tx_ ? tx_(src, n) : 0; 
    }

    /**
     * @brief 統計資料結構
     */
    struct Stats { 
        uint64_t total_in = 0;   // 總輸入字節數
        uint64_t total_out = 0;  // 總輸出字節數  
        uint64_t drops = 0;      // 丟棄字節數（溢位）
    };

    /**
     * @brief 取得統計資料
     * @return 統計數據結構
     */
    Stats getStats() const { return stats_; }

    /**
     * @brief 設定溢位策略
     * @param v true=丟棄最舊數據並插入新數據；false=丟棄新數據
     */
    void setDropOldestOnOverflow(bool v) { drop_oldest_ = v; }

private:
    // 簡易 ring buffer（單生產者/單消費者）
    uint8_t buf_[UART_IF_RING_SIZE]{};
    size_t head_ = 0;  // 寫入位置
    size_t tail_ = 0;  // 讀取位置
    bool drop_oldest_ = true;  // 溢位策略
    
    // TX 回調函數（可選）
    std::function<size_t(const uint8_t*, size_t)> tx_;
    
    // 統計數據
    mutable Stats stats_{};

    /**
     * @brief 取得 ring buffer 剩餘空間
     * @return 可寫入字節數
     */
    size_t freeSpace() const;

    /**
     * @brief 取得 ring buffer 已用空間
     * @return 已使用字節數
     */
    size_t usedSpace() const;

    /**
     * @brief 推入單一字節（處理溢位策略）
     * @param b 要推入的字節
     */
    void pushByte(uint8_t b);
};