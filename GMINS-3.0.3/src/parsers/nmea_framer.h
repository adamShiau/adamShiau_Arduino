/**
 * @file nmea_framer.h
 * @brief NMEA 0183 協議封包組合器
 * 
 * 實現 GNSS 設備的 NMEA 0183 協議句子識別和組合功能。
 * 支援標準格式：$TALKER,field1,field2,...*HH\r\n
 */

#pragma once
#include "IFramer.h"

/**
 * @brief NMEA 0183 協議封包組合器
 * 
 * 處理 GNSS 設備的 NMEA 0183 協議數據流，將原始字節組合成完整的 NMEA 句子。
 * 
 * NMEA 句子格式：
 * - $: 起始符號
 * - TALKER: 話者標識符（如 GP, GN, GL 等）
 * - MESSAGE: 訊息類型（如 GGA, RMC, VTG 等）
 * - ,field1,field2,...: 數據欄位（逗號分隔）
 * - *HH: 校驗和（可選，HH為16進位校驗和）
 * - \r\n: 行結束符
 */
class NmeaFramer : public IFramer {
public:
    NmeaFramer();
    ~NmeaFramer() = default;
    
    // IFramer 實現
    void feed(const uint8_t* data, size_t len) override;
    void reset() override;
    const char* getTypeName() const override { return "NmeaFramer"; }
    const char* getStateDescription() const override;
    
    /**
     * @brief 設定是否啟用校驗和驗證
     * @param enable true 啟用校驗和檢查，false 停用
     */
    void setChecksumValidation(bool enable) { validate_checksum_ = enable; }
    
    /**
     * @brief 設定最大句子長度
     * @param max_length 最大長度（預設 256）
     */
    void setMaxSentenceLength(size_t max_length);
    
    /**
     * @brief 獲取統計信息
     */
    struct Stats {
        uint32_t total_bytes_processed = 0;    // 處理的總字節數
        uint32_t sentences_completed = 0;      // 成功組合的句子數
        uint32_t checksum_errors = 0;          // 校驗和錯誤數
        uint32_t oversized_sentences = 0;      // 超長句子錯誤數
        uint32_t malformed_sentences = 0;      // 格式錯誤句子數
    };
    
    const Stats& getStats() const { return stats_; }
    void resetStats() { stats_ = {}; }

private:
    /**
     * @brief NMEA 句子處理狀態
     */
    enum State {
        WAIT_START,     // 等待起始符號 '$'
        READ_SENTENCE,  // 讀取句子內容
        READ_CR,        // 讀取 \r（可選）
        READ_LF         // 讀取 \n
    };
    
    State state_;                    // 當前狀態
    static constexpr size_t DEFAULT_MAX_LENGTH = 256;  // 預設最大句子長度
    char* buffer_;                   // 句子緩衝區（動態分配）
    size_t max_length_;              // 最大句子長度
    size_t pos_;                     // 當前位置
    bool validate_checksum_;         // 是否驗證校驗和
    Stats stats_;                    // 統計信息
    
    /**
     * @brief 驗證 NMEA 句子校驗和
     * @param sentence 完整句子（包含 $ 和 *HH）
     * @param len 句子長度
     * @return true 如果校驗和正確或沒有校驗和
     */
    bool verifyChecksum(const char* sentence, size_t len) const;
    
    /**
     * @brief 處理完整句子
     * 驗證格式和校驗和並觸發回調
     */
    void processCompleteSentence();
    
    /**
     * @brief 十六進位字符轉數值
     * @param c 十六進位字符
     * @return 對應數值，-1 表示無效字符
     */
    static int hexCharToValue(char c);
    
    /**
     * @brief 獲取狀態名稱（調試用）
     */
    const char* getStateName(State state) const;
};