#ifndef TDD_FLOW_CHECKER_H
#define TDD_FLOW_CHECKER_H

#include <stdint.h>

/*
 * TDD Flow Checker 使用標準 log 系統控制輸出
 * 可在各模組中透過 LOG_LEVEL_MASK_LOCAL 控制顯示
 */

/**
 * @file tdd_flow_checker.h
 * @brief 輕量級流入/流出檢測器 - TDD 最小化設計
 * @version 1.0
 * 
 * 設計原則：
 * - 最小化實現，只做 0/1 判斷（有/無數據）
 * - 5秒週期顯示：模組名::函數名 IN=0or1 OUT=0or1
 * - 獨立使用，無依賴外部大型監控系統
 * - 可在任何地方加入：小到一個引用，大到整個模組
 */

/**
 * @brief TDD 流程檢測器類別
 * 
 * 用途：
 * - 追蹤函數/模組的流入和流出狀態
 * - 每5秒顯示一次狀態報告
 * - 簡單的二進制狀態（0=沒有活動, 1=有活動）
 */
class TDDFlowChecker {
private:
    // 基本識別
    const char* module_name;      // 模組名稱（不拷貝，使用指針節省記憶體）
    const char* function_name;    // 函數名稱
    
    // 狀態追蹤（最小化設計）
    bool has_in_activity;         // 本週期是否有流入活動
    bool has_out_activity;        // 本週期是否有流出活動
    
    // 頻率計算（新增功能）
    uint32_t in_count;            // 流入計數
    uint32_t out_count;           // 流出計數
    bool enable_frequency_calc;   // 是否啟用頻率計算
    
    // 時間控制
    uint32_t last_report_time;    // 上次報告時間
    uint32_t report_interval;     // 報告間隔（預設5000ms）

public:
    /**
     * @brief 建構子
     * @param module_name 模組名稱
     * @param function_name 函數名稱  
     * @param report_interval_ms 報告間隔（預設5000ms）
     */
    TDDFlowChecker(const char* module_name, 
                   const char* function_name,
                   uint32_t report_interval_ms = 5000,
                   bool enable_frequency = false);

    /**
     * @brief 記錄流入活動
     * @note 設置 has_in_activity = true
     */
    void recordIn();

    /**
     * @brief 記錄流出活動  
     * @note 設置 has_out_activity = true
     */
    void recordOut();

    /**
     * @brief 更新檢查器並在需要時生成報告
     * @note 檢查是否到達報告時間，如果是則顯示狀態並重置
     */
    void update();
    
    /**
     * @brief 取得流入頻率 (Hz)
     * @return 流入頻率，如果未啟用頻率計算返回 0
     */
    float getInFrequency() const;
    
    /**
     * @brief 取得流出頻率 (Hz)
     * @return 流出頻率，如果未啟用頻率計算返回 0
     */
    float getOutFrequency() const;

private:
    /**
     * @brief 生成並顯示狀態報告
     * @note 格式: [模組名::函數名] IN=0or1 OUT=0or1 或加入頻率資訊
     */
    void generateReport();

    /**
     * @brief 重置週期狀態
     */
    void resetPeriodState();
};

// 全域便利宏定義（可選使用）

/**
 * @brief 快速宏：記錄流入
 * @param checker TDDFlowChecker 實例
 */
#define TDD_RECORD_IN(checker) (checker).recordIn()

/**
 * @brief 快速宏：記錄流出  
 * @param checker TDDFlowChecker 實例
 */
#define TDD_RECORD_OUT(checker) (checker).recordOut()

/**
 * @brief 快速宏：更新檢查器
 * @param checker TDDFlowChecker 實例
 */
#define TDD_UPDATE(checker) (checker).update()

#endif // TDD_FLOW_CHECKER_H