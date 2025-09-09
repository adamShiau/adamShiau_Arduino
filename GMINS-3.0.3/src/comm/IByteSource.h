#pragma once
#include <cstddef>
#include <cstdint>

/**
 * @brief 抽象字節數據源介面
 * 
 * 為 Parser/Router 提供統一的數據讀取介面，
 * 支援從 UART、文件、網絡等不同來源讀取數據。
 */
struct IByteSource {
    virtual ~IByteSource() = default;
    
    /**
     * @brief 查詢可用的字節數
     * @return 當前緩衝區中可讀取的字節數
     */
    virtual size_t available() const = 0;
    
    /**
     * @brief 讀取數據到緩衝區
     * @param dst 目標緩衝區指針
     * @param n 要讀取的最大字節數
     * @return 實際讀取的字節數 (0 <= 返回值 <= n)
     */
    virtual size_t read(uint8_t* dst, size_t n) = 0;
};