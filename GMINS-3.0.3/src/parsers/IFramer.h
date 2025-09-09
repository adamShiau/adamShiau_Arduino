/**
 * @file IFramer.h
 * @brief 抽象封包組合器介面
 * 
 * 定義統一的 framing 介面，用於將原始字節流組合成完整的協議封包。
 * 支援多種協議：XBUS (XSENS)、NMEA (GNSS) 等。
 */

#pragma once
#include <cstdint>
#include <cstddef>
#include <functional>

/**
 * @brief 抽象封包組合器介面
 * 
 * 負責從原始字節流中識別並組合完整的協議封包，
 * 提供統一的介面供不同協議的實現類使用。
 */
struct IFramer {
    virtual ~IFramer() = default;
    
    /**
     * @brief 餵入原始字節數據
     * @param data 原始數據緩衝區
     * @param len 數據長度
     * 
     * 處理傳入的字節流，內部維護狀態機來識別協議邊界。
     * 當組成完整封包時，會觸發 on_frame 回調。
     */
    virtual void feed(const uint8_t* data, size_t len) = 0;
    
    /**
     * @brief 重置內部狀態
     * 
     * 清除內部緩衝區和狀態機，用於錯誤恢復或重新開始。
     */
    virtual void reset() {}
    
    /**
     * @brief 完整封包回調函數
     * 
     * 當組成完整封包時被調用。回調參數：
     * - frame: 完整封包數據（包含協議頭尾）
     * - len: 封包長度
     * 
     * 注意：frame 指針只在回調期間有效，需要的話請立即複製數據。
     */
    std::function<void(const uint8_t* frame, size_t len)> on_frame;
    
    /**
     * @brief 獲取 Framer 類型名稱（調試用）
     * @return 類型名稱字串
     */
    virtual const char* getTypeName() const { return "IFramer"; }
    
    /**
     * @brief 獲取當前內部狀態（調試用）
     * @return 狀態描述字串
     */
    virtual const char* getStateDescription() const { return "Unknown"; }
};