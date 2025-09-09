#pragma once

#include "ITransport.h"
#include "../util/data_flow_monitor.h"
#include <functional>

/**
 * @brief 監控傳輸裝飾器
 * 
 * 裝飾現有的 ITransport 實例，在數據傳輸時進行監控
 * 不影響原有傳輸行為，純粹添加監控功能
 */
class MonitoredTransport : public ITransport {
public:
    /**
     * @brief 構造函數
     * @param transport 被裝飾的傳輸實例
     * @param monitor_name 監控器名稱
     */
    explicit MonitoredTransport(ITransport* transport, const char* monitor_name)
        : transport_(transport), monitor_(nullptr) {
        
        if (monitor_name) {
            monitor_ = monitor::createProtocolMonitor(monitor_name);
        }
    }
    
    ~MonitoredTransport() {
        if (monitor_) {
            delete monitor_;
            monitor_ = nullptr;
        }
    }
    
    // 禁止拷貝
    MonitoredTransport(const MonitoredTransport&) = delete;
    MonitoredTransport& operator=(const MonitoredTransport&) = delete;
    
    // 委派基本操作（無監控需求）
    bool initialize() override {
        return transport_ ? transport_->initialize() : false;
    }
    
    void cleanup() override {
        if (transport_) transport_->cleanup();
    }
    
    bool isReady() const override {
        return transport_ ? transport_->isReady() : false;
    }
    
    size_t available() const override {
        return transport_ ? transport_->available() : 0;
    }
    
    void flush() override {
        if (transport_) transport_->flush();
    }
    
    void clearRxBuffer() override {
        if (transport_) transport_->clearRxBuffer();
    }
    
    void resetBuffers() override {
        if (transport_) transport_->resetBuffers();
    }
    
    // 狀態查詢（委派給底層傳輸）
    const char* getTransportName() const override {
        return transport_ ? transport_->getTransportName() : "MonitoredTransport(null)";
    }
    
    uint32_t getBaudRate() const override {
        return transport_ ? transport_->getBaudRate() : 0;
    }
    
    bool setBaudRate(uint32_t new_baud_rate) override {
        return transport_ ? transport_->setBaudRate(new_baud_rate) : false;
    }
    
    // 監控核心：write 操作
    size_t write(const uint8_t* data, size_t length) override {
        if (!transport_) return 0;
        
        // 🚫 移除傳輸層追蹤日誌 - 避免死機問題
        // static uint32_t last_trace_log = 0;
        // if (length > 0 && millis() - last_trace_log >= 5000) {
        //     LOGI("🕵️ 傳輸層寫入追蹤: 長度=%zu bytes", length);
        //     LOGI("前8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X", 
        //          data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        //     last_trace_log = millis();
        // }
        
        size_t bytes_written = transport_->write(data, length);
        
        // MultiChannelMonitor: 監控實際輸出的協議數據
        if (monitor_ && bytes_written > 0) {
            monitor_->recordBytes(bytes_written);
            monitor_->recordPackets(1);  // 假設每次 write 是一個協議封包
            monitor_->recordOperations(1);
        }
        
        // 通知 TxMultiplexer 記錄協議統計（如果有設置回調）
        if (protocol_stats_callback_ && bytes_written > 0) {
            protocol_stats_callback_(bytes_written);
        }
        
        return bytes_written;
    }
    
    // read 操作（通常協議輸出不需要監控讀取）
    size_t read(uint8_t* buffer, size_t max_length) override {
        return transport_ ? transport_->read(buffer, max_length) : 0;
    }
    
    /**
     * @brief 獲取監控器
     * @return DataFlowMonitor* 監控器指針，用於集成到 MultiChannelMonitor
     */
    monitor::DataFlowMonitor* getMonitor() const {
        return monitor_;
    }
    
    /**
     * @brief 獲取被裝飾的傳輸實例
     * @return ITransport* 原始傳輸指針
     */
    ITransport* getUnderlyingTransport() const {
        return transport_;
    }
    
    /**
     * @brief 設置協議統計回調
     * @param callback 當有數據寫入時調用的回調函數
     */
    void setProtocolStatsCallback(std::function<void(size_t)> callback) {
        protocol_stats_callback_ = callback;
    }

private:
    ITransport* transport_;                    // 被裝飾的傳輸實例
    monitor::DataFlowMonitor* monitor_;        // 協議輸出監控器
    std::function<void(size_t)> protocol_stats_callback_;  // 協議統計回調
};