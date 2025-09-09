#pragma once

/**
 * 編譯時功能開關
 * 
 * 用於控制可選功能的編譯，減少代碼體積和運行時開銷
 * 主要針對診斷、調試和統計功能
 * 
 * 使用方式：
 * - 在編譯時定義宏來覆蓋默認設置
 * - 例如：-DFEATURE_DIAGNOSTICS=0 來禁用診斷功能
 */

// =============================================================================
// 診斷和調試功能
// =============================================================================

/**
 * 診斷功能總開關
 * 控制所有診斷相關的代碼編譯
 * 包括：數據品質計算、調試字串、詳細統計
 */
#ifndef FEATURE_DIAGNOSTICS
    #define FEATURE_DIAGNOSTICS 1
#endif

/**
 * 詳細統計功能
 * 控制詳細的性能統計和監控代碼
 */
#ifndef FEATURE_DETAILED_STATS
    #define FEATURE_DETAILED_STATS 1
#endif

/**
 * 調試字串生成
 * 控制各種 debugString() 函數的編譯
 */
#ifndef FEATURE_DEBUG_STRINGS
    #define FEATURE_DEBUG_STRINGS 1
#endif

/**
 * 數據品質評分
 * 控制 calculateDataQuality() 函數的編譯
 */
#ifndef FEATURE_QUALITY_SCORING
    #define FEATURE_QUALITY_SCORING 1
#endif

// =============================================================================
// 協議相關功能
// =============================================================================

/**
 * 模式切換標頭
 * 控制協議切換時的模式標頭發送
 */
#ifndef FEATURE_MODE_HEADERS
    #define FEATURE_MODE_HEADERS 1
#endif

/**
 * 協議統計
 * 控制協議層的詳細統計功能
 */
#ifndef FEATURE_PROTOCOL_STATS
    #define FEATURE_PROTOCOL_STATS 1
#endif

// =============================================================================
// 系統健康監控
// =============================================================================

/**
 * 組件健康檢查
 * 控制詳細的組件健康監控
 */
#ifndef FEATURE_HEALTH_MONITORING
    #define FEATURE_HEALTH_MONITORING 1
#endif

/**
 * 錯誤恢復機制
 * 控制自動錯誤恢復功能
 */
#ifndef FEATURE_ERROR_RECOVERY
    #define FEATURE_ERROR_RECOVERY 1
#endif

// =============================================================================
// 日誌和輸出控制
// =============================================================================

/**
 * 詳細日誌輸出
 * 控制詳細的日誌訊息輸出
 */
#ifndef FEATURE_VERBOSE_LOGGING
    #define FEATURE_VERBOSE_LOGGING 1
#endif

/**
 * 性能測量
 * 控制運行時性能測量代碼
 */
#ifndef FEATURE_PERFORMANCE_MEASUREMENT
    #define FEATURE_PERFORMANCE_MEASUREMENT 0  // 默認關閉，因為可能影響性能
#endif

// =============================================================================
// 便利宏定義
// =============================================================================

/**
 * 條件編譯宏 - 診斷功能
 */
#if FEATURE_DIAGNOSTICS
    #define IF_DIAGNOSTICS(code) code
    #define IF_NOT_DIAGNOSTICS(code)
#else
    #define IF_DIAGNOSTICS(code)
    #define IF_NOT_DIAGNOSTICS(code) code
#endif

/**
 * 條件編譯宏 - 詳細統計
 */
#if FEATURE_DETAILED_STATS
    #define IF_DETAILED_STATS(code) code
#else
    #define IF_DETAILED_STATS(code)
#endif

/**
 * 條件編譯宏 - 調試字串
 */
#if FEATURE_DEBUG_STRINGS
    #define IF_DEBUG_STRINGS(code) code
#else
    #define IF_DEBUG_STRINGS(code)
#endif

/**
 * 條件編譯宏 - 品質評分
 */
#if FEATURE_QUALITY_SCORING
    #define IF_QUALITY_SCORING(code) code
#else
    #define IF_QUALITY_SCORING(code)
#endif

/**
 * 條件編譯宏 - 詳細日誌
 */
#if FEATURE_VERBOSE_LOGGING
    #define IF_VERBOSE_LOGGING(code) code
#else
    #define IF_VERBOSE_LOGGING(code)
#endif

/**
 * 條件編譯宏 - 性能測量
 */
#if FEATURE_PERFORMANCE_MEASUREMENT
    #define IF_PERFORMANCE_MEASUREMENT(code) code
#else
    #define IF_PERFORMANCE_MEASUREMENT(code)
#endif

// =============================================================================
// 精簡模式預設配置
// =============================================================================

/**
 * 精簡模式開關
 * 當啟用時，自動關閉所有非必要功能
 */
#ifdef GMINS_MINIMAL_MODE
    #undef FEATURE_DIAGNOSTICS
    #define FEATURE_DIAGNOSTICS 0
    
    #undef FEATURE_DETAILED_STATS
    #define FEATURE_DETAILED_STATS 0
    
    #undef FEATURE_DEBUG_STRINGS
    #define FEATURE_DEBUG_STRINGS 0
    
    #undef FEATURE_QUALITY_SCORING
    #define FEATURE_QUALITY_SCORING 0
    
    #undef FEATURE_VERBOSE_LOGGING
    #define FEATURE_VERBOSE_LOGGING 0
    
    #undef FEATURE_PROTOCOL_STATS
    #define FEATURE_PROTOCOL_STATS 0
    
    // 保留核心功能
    #undef FEATURE_MODE_HEADERS
    #define FEATURE_MODE_HEADERS 1
    
    #undef FEATURE_ERROR_RECOVERY
    #define FEATURE_ERROR_RECOVERY 1
    
    #undef FEATURE_HEALTH_MONITORING
    #define FEATURE_HEALTH_MONITORING 1
#endif

// =============================================================================
// 編譯時檢查
// =============================================================================

// 確保基本功能始終可用
#if !FEATURE_MODE_HEADERS && !FEATURE_ERROR_RECOVERY
    #error "至少需要啟用 MODE_HEADERS 或 ERROR_RECOVERY 功能"
#endif

// 提供編譯時配置摘要（在 LOG_INFO 中使用）
#define GMINS_FEATURE_SUMMARY \
    "DIAG:" #FEATURE_DIAGNOSTICS \
    " STATS:" #FEATURE_DETAILED_STATS \
    " DEBUG:" #FEATURE_DEBUG_STRINGS \
    " QUALITY:" #FEATURE_QUALITY_SCORING \
    " VERBOSE:" #FEATURE_VERBOSE_LOGGING