/**
 * @file uart_interface.cpp
 * @brief Feed-based UART Interface Implementation
 * 
 * 實現被HAL餵食的ring buffer UART介面，完全不接觸硬體。
 * 採用高效的ring buffer設計，支援快速路徑和慢路徑處理。
 */

#include "uart_interface.h"
#include <cstring>
#include <algorithm>

size_t UARTInterface::freeSpace() const {
    // 空一格避開head==tail歧義（區分空和滿的狀態）
    return UART_IF_RING_SIZE - usedSpace() - 1;
}

size_t UARTInterface::usedSpace() const {
    if (head_ >= tail_) {
        return head_ - tail_;
    }
    return UART_IF_RING_SIZE - (tail_ - head_);
}

void UARTInterface::pushByte(uint8_t b) {
    size_t next = (head_ + 1) % UART_IF_RING_SIZE;
    
    if (next == tail_) {
        // Ring buffer滿了，根據策略處理
        if (drop_oldest_) {
            // 丟最舊：前進tail指針，覆蓋舊數據
            tail_ = (tail_ + 1) % UART_IF_RING_SIZE;
            stats_.drops++;
        } else {
            // 丟最新：直接放棄新數據
            stats_.drops++;
            return;
        }
    }
    
    buf_[head_] = b;
    head_ = next;
}

void UARTInterface::feed(const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        return;
    }
    
    // 快速路徑：如果空間足夠且沒有環繞，使用memcpy優化
    size_t space = freeSpace();
    
    if (space >= len) {
        // 有足夠空間，檢查是否需要處理環繞
        if (head_ >= tail_) {
            // 線性情況：head在tail後面
            size_t tail_free = UART_IF_RING_SIZE - head_;
            
            if (tail_free >= len) {
                // 一次memcpy就能搞定
                std::memcpy(&buf_[head_], data, len);
                head_ = (head_ + len) % UART_IF_RING_SIZE;
            } else {
                // 需要兩次memcpy：尾部 + 頭部
                std::memcpy(&buf_[head_], data, tail_free);
                std::memcpy(&buf_[0], data + tail_free, len - tail_free);
                head_ = len - tail_free;
            }
            
            stats_.total_in += len;
            return;
        } else {
            // head < tail 的情況，中間有連續空間
            std::memcpy(&buf_[head_], data, len);
            head_ += len;
            stats_.total_in += len;
            return;
        }
    }
    
    // 慢路徑：空間不足或需要特殊處理，逐字節推入
    // 這裡會觸發溢位策略
    for (size_t i = 0; i < len; ++i) {
        pushByte(data[i]);
    }
    
    stats_.total_in += len;
}

size_t UARTInterface::available() const {
    return usedSpace();
}

size_t UARTInterface::read(uint8_t* dst, size_t n) {
    if (!dst || n == 0) {
        return 0;
    }
    
    size_t cnt = 0;
    size_t available_bytes = usedSpace();
    
    // 限制讀取數量為實際可用數量
    n = std::min(n, available_bytes);
    
    // 優化：如果數據是連續的，使用memcpy
    if (n > 0) {
        if (tail_ < head_) {
            // 線性情況：數據連續
            std::memcpy(dst, &buf_[tail_], n);
            tail_ += n;
            cnt = n;
        } else {
            // 環繞情況：可能需要兩次拷貝
            size_t tail_to_end = UART_IF_RING_SIZE - tail_;
            
            if (n <= tail_to_end) {
                // 一次拷貝就夠了
                std::memcpy(dst, &buf_[tail_], n);
                tail_ = (tail_ + n) % UART_IF_RING_SIZE;
                cnt = n;
            } else {
                // 需要兩次拷貝
                std::memcpy(dst, &buf_[tail_], tail_to_end);
                std::memcpy(dst + tail_to_end, &buf_[0], n - tail_to_end);
                tail_ = n - tail_to_end;
                cnt = n;
            }
        }
    }
    
    stats_.total_out += cnt;
    return cnt;
}