#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

/**
 * @brief 解析器共用工具函數
 * 
 * 提供 XBUS 和 NMEA 解析器共用的基礎功能
 */

// ============================================================================
// 二進位數據解析
// ============================================================================

/**
 * @brief 從緩衝區解析 32 位浮點數（大端序）
 */
inline float parseFloat32(const uint8_t* buffer) {
    if (!buffer) return 0.0f;
    
    union {
        uint32_t i;
        float f;
    } converter;
    
    // 大端序轉換
    converter.i = (static_cast<uint32_t>(buffer[0]) << 24) |
                  (static_cast<uint32_t>(buffer[1]) << 16) |
                  (static_cast<uint32_t>(buffer[2]) << 8) |
                   static_cast<uint32_t>(buffer[3]);
    
    return converter.f;
}

/**
 * @brief 從緩衝區解析 64 位雙精度浮點數（大端序）
 */
inline double parseFloat64(const uint8_t* buffer) {
    if (!buffer) return 0.0;
    
    union {
        uint64_t i;
        double d;
    } converter;
    
    // 大端序轉換（8字節）
    converter.i = (static_cast<uint64_t>(buffer[0]) << 56) |
                  (static_cast<uint64_t>(buffer[1]) << 48) |
                  (static_cast<uint64_t>(buffer[2]) << 40) |
                  (static_cast<uint64_t>(buffer[3]) << 32) |
                  (static_cast<uint64_t>(buffer[4]) << 24) |
                  (static_cast<uint64_t>(buffer[5]) << 16) |
                  (static_cast<uint64_t>(buffer[6]) << 8) |
                   static_cast<uint64_t>(buffer[7]);
    
    return converter.d;
}

/**
 * @brief 從緩衝區解析 16 位整數（大端序）
 */
inline uint16_t parseUint16(const uint8_t* buffer) {
    if (!buffer) return 0;
    return (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
}

/**
 * @brief 從緩衝區解析 32 位整數（大端序）
 */
inline uint32_t parseUint32(const uint8_t* buffer) {
    if (!buffer) return 0;
    return (static_cast<uint32_t>(buffer[0]) << 24) |
           (static_cast<uint32_t>(buffer[1]) << 16) |
           (static_cast<uint32_t>(buffer[2]) << 8) |
            static_cast<uint32_t>(buffer[3]);
}

// ============================================================================
// NMEA 字串解析
// ============================================================================

/**
 * @brief 分割 NMEA 句子為 token
 * @param sentence NMEA 句子
 * @param delimiter 分隔符
 * @param tokens 輸出 token 陣列
 * @param max_tokens 最大 token 數量
 * @return 實際解析的 token 數量
 */
inline int tokenize(const char* sentence, char delimiter, const char** tokens, int max_tokens) {
    if (!sentence || !tokens || max_tokens <= 0) return 0;
    
    static char sentence_copy[256];  // 靜態緩衝區避免動態分配
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    int count = 0;
    char* token = sentence_copy;
    char* next_delimiter;
    
    while (count < max_tokens && token) {
        next_delimiter = strchr(token, delimiter);
        if (next_delimiter) {
            *next_delimiter = '\0';
        }
        
        tokens[count++] = token;
        
        if (next_delimiter) {
            token = next_delimiter + 1;
        } else {
            break;
        }
    }
    
    return count;
}

/**
 * @brief 解析緯度字串（NMEA 格式：DDMM.MMMM）
 * @param coord_str 座標字串
 * @param hemisphere 半球標記（N/S）
 * @param result 輸出結果（十進位度）
 * @return 解析是否成功
 */
inline bool parseLatitude(const char* coord_str, char hemisphere, double& result) {
    if (!coord_str || strlen(coord_str) < 4) return false;
    
    double coord = atof(coord_str);
    if (coord == 0.0) return false;
    
    // 分離度和分
    int degrees = static_cast<int>(coord / 100);
    double minutes = coord - (degrees * 100);
    
    // 轉換為十進位度
    result = degrees + (minutes / 60.0);
    
    // 南半球為負值
    if (hemisphere == 'S' || hemisphere == 's') {
        result = -result;
    }
    
    return true;
}

/**
 * @brief 解析經度字串（NMEA 格式：DDDMM.MMMM）
 * @param coord_str 座標字串
 * @param hemisphere 半球標記（E/W）
 * @param result 輸出結果（十進位度）
 * @return 解析是否成功
 */
inline bool parseLongitude(const char* coord_str, char hemisphere, double& result) {
    if (!coord_str || strlen(coord_str) < 4) return false;
    
    double coord = atof(coord_str);
    if (coord == 0.0) return false;
    
    // 分離度和分
    int degrees = static_cast<int>(coord / 100);
    double minutes = coord - (degrees * 100);
    
    // 轉換為十進位度
    result = degrees + (minutes / 60.0);
    
    // 西半球為負值
    if (hemisphere == 'W' || hemisphere == 'w') {
        result = -result;
    }
    
    return true;
}

/**
 * @brief 計算 NMEA checksum
 * @param sentence NMEA 句子（不含 $ 和 *）
 * @return checksum 值
 */
inline uint8_t calculateNmeaChecksum(const char* sentence) {
    if (!sentence) return 0;
    
    uint8_t checksum = 0;
    for (const char* p = sentence; *p && *p != '*'; p++) {
        checksum ^= static_cast<uint8_t>(*p);
    }
    return checksum;
}

/**
 * @brief 驗證 NMEA checksum
 * @param sentence 完整 NMEA 句子
 * @return 校驗是否正確
 */
inline bool verifyNmeaChecksum(const char* sentence) {
    if (!sentence) return false;
    
    const char* checksum_start = strrchr(sentence, '*');
    if (!checksum_start) return false;
    
    // 解析 checksum
    uint8_t expected_checksum = static_cast<uint8_t>(strtol(checksum_start + 1, nullptr, 16));
    
    // 計算實際 checksum（從 $ 後開始，到 * 前結束）
    const char* start = strchr(sentence, '$');
    if (!start) return false;
    start++; // 跳過 $
    
    uint8_t actual_checksum = 0;
    for (const char* p = start; p < checksum_start; p++) {
        actual_checksum ^= static_cast<uint8_t>(*p);
    }
    
    return actual_checksum == expected_checksum;
}

// ============================================================================
// 數據品質計算
// ============================================================================

/**
 * @brief 計算通用數據品質
 * @param data_age_ms 數據年齡（毫秒）
 * @param max_age_ms 最大允許年齡
 * @return 品質值 (0-255)
 */
inline uint8_t calculateDataQuality(uint32_t data_age_ms, uint32_t max_age_ms = 1000) {
    if (data_age_ms >= max_age_ms) return 0;
    
    // 線性衰減：新數據品質高，舊數據品質低
    uint32_t quality_percent = ((max_age_ms - data_age_ms) * 100) / max_age_ms;
    return static_cast<uint8_t>((quality_percent * 255) / 100);
}