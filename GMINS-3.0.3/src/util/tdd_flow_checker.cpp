#include "tdd_flow_checker.h"

// 統一使用項目的 log 系統
#define LOG_TAG "TDD_FLOW"
#include "log.h"

// Arduino環境支援
#ifdef ARDUINO
    #include <Arduino.h>
#else
    // 非Arduino環境需要外部提供millis()
    extern uint32_t millis();
#endif

TDDFlowChecker::TDDFlowChecker(const char* module_name, 
                               const char* function_name,
                               uint32_t report_interval_ms,
                               bool enable_frequency)
    : module_name(module_name)
    , function_name(function_name)
    , has_in_activity(false)
    , has_out_activity(false)
    , in_count(0)
    , out_count(0)
    , enable_frequency_calc(enable_frequency)
    , last_report_time(0)
    , report_interval(report_interval_ms)
{
    // 初始化時間戳
    last_report_time = millis();
}

void TDDFlowChecker::recordIn() {
    // 系統啟動40秒內不記錄，避免死機
    if (millis() < 40000) {
        return;
    }
    
    has_in_activity = true;
    // 優化：只在需要頻率計算時才累加
    if (enable_frequency_calc) {
        in_count++;
    }
}

void TDDFlowChecker::recordOut() {
    // 系統啟動40秒內不記錄，避免死機
    if (millis() < 40000) {
        return;
    }
    
    has_out_activity = true;
    // 優化：只在需要頻率計算時才累加
    if (enable_frequency_calc) {
        out_count++;
    }
}

void TDDFlowChecker::update() {
    uint32_t current_time = millis();
    
    // 系統啟動40秒內不執行TDD檢查，避免死機
    if (current_time < 40000) {
        return;
    }
    
    // 檢查是否到達報告時間
    if (current_time - last_report_time >= report_interval) {
        generateReport();
        resetPeriodState();
        last_report_time = current_time;
    }
}

void TDDFlowChecker::generateReport() {
    if (enable_frequency_calc && report_interval > 0) {
        // 優化：使用整數計算避免浮點數運算，提高精度到0.1Hz
        uint32_t elapsed_ms = report_interval;
        uint32_t in_freq_x10 = (in_count * 10000UL) / elapsed_ms;
        uint32_t out_freq_x10 = (out_count * 10000UL) / elapsed_ms;
        
        // 顯示狀態 + 頻率（整數格式避免sprintf浮點運算）
        LOGI("%s::%s IN=%d OUT=%d [%u.%uHz/%u.%uHz]", 
             module_name, 
             function_name,
             has_in_activity ? 1 : 0,
             has_out_activity ? 1 : 0,
             in_freq_x10/10, in_freq_x10%10,
             out_freq_x10/10, out_freq_x10%10);
    } else {
        // 簡化模式：只顯示活動狀態
        LOGI("%s::%s IN=%d OUT=%d", 
             module_name, 
             function_name,
             has_in_activity ? 1 : 0,
             has_out_activity ? 1 : 0);
    }
}

void TDDFlowChecker::resetPeriodState() {
    has_in_activity = false;
    has_out_activity = false;
    if (enable_frequency_calc) {
        in_count = 0;
        out_count = 0;
    }
}

float TDDFlowChecker::getInFrequency() const {
    if (!enable_frequency_calc) {
        return 0.0f;
    }
    float elapsed_sec = report_interval / 1000.0f;
    return in_count / elapsed_sec;
}

float TDDFlowChecker::getOutFrequency() const {
    if (!enable_frequency_calc) {
        return 0.0f;
    }
    float elapsed_sec = report_interval / 1000.0f;
    return out_count / elapsed_sec;
}