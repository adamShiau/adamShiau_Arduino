#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

// 🛡️ 堆疊水位監控工具
struct StackMonitor {
    uint32_t last_report_ms = 0;
    uint32_t min_free_stack = UINT32_MAX;
    
    void checkStackUsage() {
        uint32_t free_stack = getFreeStackBytes();
        if (free_stack < min_free_stack) {
            min_free_stack = free_stack;
        }
        
        // 5秒報告一次
        uint32_t now = millis();
        if (now - last_report_ms >= 5000u) {
            last_report_ms = now;
            
            char buffer[128];
            sprintf(buffer, "STACK 5s: current_free=%luB min_free=%luB", 
                    (unsigned long)free_stack, (unsigned long)min_free_stack);
            LOGI("%s", buffer);
            
            // 重置最小值計數
            min_free_stack = free_stack;
        }
    }
    
private:
    uint32_t getFreeStackBytes() {
        #ifdef ESP32
            // ESP32 FreeRTOS
            #include "freertos/FreeRTOS.h"
            #include "freertos/task.h"
            UBaseType_t hw = uxTaskGetStackHighWaterMark(nullptr);
            return hw * sizeof(StackType_t);  // 轉換為字節
        #elif defined(__AVR__)
            // Arduino AVR - 估算堆疊使用
            extern int __heap_start, *__brkval;
            int v;
            return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
        #else
            // 通用方法 - 使用局部變量地址估算
            static uint8_t* stack_bottom = nullptr;
            if (stack_bottom == nullptr) {
                uint8_t dummy;
                stack_bottom = &dummy;
            }
            uint8_t current_stack;
            ptrdiff_t used = stack_bottom - &current_stack;
            // 假設總堆疊大小為 8KB (可根據實際調整)
            const uint32_t TOTAL_STACK_SIZE = 8192;
            return (used > 0 && used < TOTAL_STACK_SIZE) ? (TOTAL_STACK_SIZE - used) : 1024;
        #endif
    }
};