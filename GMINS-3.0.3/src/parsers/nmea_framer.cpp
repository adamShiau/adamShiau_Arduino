/**
 * @file nmea_framer.cpp
 * @brief NMEA 0183 協議封包組合器實現
 * 
 * 實現 GNSS 設備的 NMEA 0183 協議句子識別和組合功能。
 */

#include "nmea_framer.h"
#include <cstring>
#include <cstdlib>

NmeaFramer::NmeaFramer() 
    : state_(WAIT_START), buffer_(nullptr), max_length_(DEFAULT_MAX_LENGTH), 
      pos_(0), validate_checksum_(true) {
    
    // 分配緩衝區
    buffer_ = static_cast<char*>(malloc(max_length_));
    if (!buffer_) {
        max_length_ = 0;
    }
}

void NmeaFramer::setMaxSentenceLength(size_t max_length) {
    if (max_length == max_length_) {
        return;
    }
    
    // 重新分配緩衝區
    char* new_buffer = static_cast<char*>(realloc(buffer_, max_length));
    if (new_buffer || max_length == 0) {
        buffer_ = new_buffer;
        max_length_ = max_length;
        
        // 如果當前位置超出新的最大長度，重置狀態
        if (pos_ >= max_length_) {
            reset();
        }
    }
}

void NmeaFramer::reset() {
    state_ = WAIT_START;
    pos_ = 0;
}

void NmeaFramer::feed(const uint8_t* data, size_t len) {
    if (!data || len == 0 || !buffer_ || max_length_ == 0) {
        return;
    }
    
    stats_.total_bytes_processed += len;
    
    for (size_t i = 0; i < len; ++i) {
        char c = static_cast<char>(data[i]);
        
        switch (state_) {
            case WAIT_START:
                if (c == '$') {
                    buffer_[0] = c;
                    pos_ = 1;
                    state_ = READ_SENTENCE;
                }
                // 忽略所有非 '$' 字符
                break;
                
            case READ_SENTENCE:
                // 檢查是否超出緩衝區
                if (pos_ >= max_length_ - 1) {
                    stats_.oversized_sentences++;
                    reset();
                    break;
                }
                
                buffer_[pos_++] = c;
                
                if (c == '\r') {
                    state_ = READ_CR;
                } else if (c == '\n') {
                    // 直接遇到 \n，沒有 \r
                    processCompleteSentence();
                    reset();
                }
                // 繼續讀取句子內容
                break;
                
            case READ_CR:
                if (c == '\n') {
                    // 標準的 \r\n 結尾
                    if (pos_ < max_length_) {
                        buffer_[pos_++] = c;
                    }
                    processCompleteSentence();
                    reset();
                } else {
                    // \r 後不是 \n，視為格式錯誤
                    stats_.malformed_sentences++;
                    reset();
                    
                    // 檢查當前字符是否為新句子開始
                    if (c == '$') {
                        buffer_[0] = c;
                        pos_ = 1;
                        state_ = READ_SENTENCE;
                    }
                }
                break;
        }
    }
}

bool NmeaFramer::verifyChecksum(const char* sentence, size_t len) const {
    if (len < 6) {  // 最小句子：$XXXXX 至少6字符
        return false;
    }
    
    // 尋找 '*' 字符
    const char* asterisk = nullptr;
    for (size_t i = 1; i < len; ++i) {  // 從第2個字符開始尋找
        if (sentence[i] == '*') {
            asterisk = &sentence[i];
            break;
        }
    }
    
    // 如果沒有找到 '*'，視為沒有校驗和的句子（某些設備會省略）
    if (!asterisk) {
        return true;  // 接受沒有校驗和的句子
    }
    
    // 檢查校驗和後是否有足夠的字符（2位十六進位）
    size_t checksum_pos = asterisk - sentence;
    if (checksum_pos + 3 > len) {  // '*' + 2位十六進位
        return false;
    }
    
    // 計算實際校驗和（從 '$' 後開始到 '*' 前）
    uint8_t calculated_checksum = 0;
    for (size_t i = 1; i < checksum_pos; ++i) {
        calculated_checksum ^= static_cast<uint8_t>(sentence[i]);
    }
    
    // 解析句子中的校驗和
    int hi = hexCharToValue(sentence[checksum_pos + 1]);
    int lo = hexCharToValue(sentence[checksum_pos + 2]);
    
    if (hi < 0 || lo < 0) {
        return false;  // 無效的十六進位字符
    }
    
    uint8_t sentence_checksum = static_cast<uint8_t>((hi << 4) | lo);
    
    return calculated_checksum == sentence_checksum;
}

void NmeaFramer::processCompleteSentence() {
    if (pos_ < 6) {  // 最小有效句子長度
        stats_.malformed_sentences++;
        return;
    }
    
    // 確保字串結尾
    if (pos_ < max_length_) {
        buffer_[pos_] = '\0';
    } else {
        buffer_[max_length_ - 1] = '\0';
    }
    
    // 驗證句子格式（必須以 '$' 開始）
    if (buffer_[0] != '$') {
        stats_.malformed_sentences++;
        return;
    }
    
    // 驗證校驗和（如果啟用）
    if (validate_checksum_ && !verifyChecksum(buffer_, pos_)) {
        stats_.checksum_errors++;
        return;
    }
    
    // 成功組合句子
    stats_.sentences_completed++;
    
    // 觸發回調
    if (on_frame) {
        on_frame(reinterpret_cast<const uint8_t*>(buffer_), pos_);
    }
}

int NmeaFramer::hexCharToValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;  // 無效字符
}

const char* NmeaFramer::getStateDescription() const {
    return getStateName(state_);
}

const char* NmeaFramer::getStateName(State state) const {
    switch (state) {
        case WAIT_START: return "WAIT_START";
        case READ_SENTENCE: return "READ_SENTENCE";
        case READ_CR: return "READ_CR";
        case READ_LF: return "READ_LF";
        default: return "UNKNOWN";
    }
}