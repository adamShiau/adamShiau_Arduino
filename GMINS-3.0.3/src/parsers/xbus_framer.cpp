/**
 * @file xbus_framer.cpp
 * @brief XSENS XBUS 協議封包組合器實現
 * 
 * 實現 XSENS MTI 設備的 XBUS 協議封包識別和組合功能。
 */

#include "xbus_framer.h"
#include <cstring>

XbusFramer::XbusFramer() 
    : state_(WAIT_FA), pos_(0), data_length_(0) {
}

void XbusFramer::reset() {
    state_ = WAIT_FA;
    pos_ = 0;
    data_length_ = 0;
}

void XbusFramer::feed(const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        return;
    }
    
    stats_.total_bytes_processed += len;
    
    for (size_t i = 0; i < len; ++i) {
        uint8_t byte = data[i];
        
        switch (state_) {
            case WAIT_FA:
                if (byte == 0xFA) {
                    buffer_[0] = byte;
                    pos_ = 1;
                    state_ = WAIT_FF;
                } else {
                    stats_.sync_errors++;
                }
                break;
                
            case WAIT_FF:
                buffer_[pos_++] = byte;
                if (byte == 0xFF) {
                    state_ = READ_MID;
                } else {
                    // 不是預期的 0xFF，重新開始尋找 0xFA
                    stats_.sync_errors++;
                    if (byte == 0xFA) {
                        buffer_[0] = byte;
                        pos_ = 1;
                        state_ = WAIT_FF;
                    } else {
                        state_ = WAIT_FA;
                        pos_ = 0;
                    }
                }
                break;
                
            case READ_MID:
                buffer_[pos_++] = byte;
                state_ = READ_LEN;
                break;
                
            case READ_LEN:
                buffer_[pos_++] = byte;
                data_length_ = byte;
                
                if (data_length_ == 255) {
                    // 擴展長度格式（暫不支援）
                    // TODO: 實現擴展長度支援
                    stats_.oversized_frames++;
                    reset();
                } else if (data_length_ == 0) {
                    // 無數據，直接讀校驗和
                    state_ = READ_CHK;
                } else {
                    // 有數據需要讀取
                    state_ = READ_DATA;
                }
                break;
                
            case READ_DATA:
                buffer_[pos_++] = byte;
                
                // 檢查是否讀取完所有數據
                if (pos_ == 4 + data_length_) {  // 0xFA 0xFF MID LEN + DATA
                    state_ = READ_CHK;
                }
                
                // 防止緩衝區溢出
                if (pos_ >= MAX_FRAME_SIZE - 1) {
                    stats_.oversized_frames++;
                    reset();
                }
                break;
                
            case READ_CHK:
                buffer_[pos_++] = byte;
                processCompleteFrame();
                reset();
                break;
        }
    }
}

bool XbusFramer::verifyChecksum(const uint8_t* frame, size_t len) const {
    if (len < 5) {  // 至少需要 FA FF MID LEN CHK
        return false;
    }
    
    // XBUS 校驗和算法：總和為0驗證法 (正確算法)
    // 從 Bus ID (frame[1]) 開始計算到 checksum，總和低8位應為0
    uint8_t sum = 0;
    for (size_t i = 1; i < len; ++i) {  // 從 Bus ID 到 CHK (包含 CHK)
        sum += frame[i];
    }
    return (sum == 0);  // 總和低8位應為0
}

void XbusFramer::processCompleteFrame() {
    // 最小有效封包：0xFA 0xFF MID LEN CHK = 5 bytes
    if (pos_ < 5) {
        stats_.sync_errors++;
        return;
    }
    
    // 驗證校驗和
    if (!verifyChecksum(buffer_, pos_)) {
        stats_.checksum_errors++;
        return;
    }
    
    // 成功組合封包
    stats_.frames_completed++;
    
    // 觸發回調
    if (on_frame) {
        on_frame(buffer_, pos_);
    }
}

const char* XbusFramer::getStateDescription() const {
    return getStateName(state_);
}

const char* XbusFramer::getStateName(State state) const {
    switch (state) {
        case WAIT_FA: return "WAIT_FA";
        case WAIT_FF: return "WAIT_FF";
        case READ_MID: return "READ_MID";
        case READ_LEN: return "READ_LEN";
        case READ_DATA: return "READ_DATA";
        case READ_CHK: return "READ_CHK";
        default: return "UNKNOWN";
    }
}