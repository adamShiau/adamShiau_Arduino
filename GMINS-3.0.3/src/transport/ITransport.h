#pragma once

#include <cstddef>
#include <cstdint>
#include "../util/log.h"

/**
 * @brief 傳輸層抽象介面
 * 
 * 支援 UART、UDP、USB 等不同傳輸媒介
 * 提供統一的讀寫介面和緩衝區管理
 */
class ITransport {
public:
    virtual ~ITransport() = default;
    
    // 基本傳輸操作
    virtual bool initialize() = 0;
    virtual void cleanup() = 0;
    virtual bool isReady() const = 0;
    
    // 數據傳輸
    virtual size_t write(const uint8_t* data, size_t length) = 0;
    virtual size_t read(uint8_t* buffer, size_t max_length) = 0;
    virtual size_t available() const = 0;
    
    // 緩衝區管理（關鍵：模式切換時需要清空）
    virtual void flush() = 0;           // 刷新發送緩衝區
    virtual void clearRxBuffer() = 0;   // 清空接收緩衝區
    virtual void resetBuffers() = 0;    // 重置所有緩衝區
    
    // 狀態查詢
    virtual const char* getTransportName() const = 0;
    virtual uint32_t getBaudRate() const = 0;
    
    // 波特率控制（預設為空實現，子類可選擇性重寫）
    virtual bool setBaudRate(uint32_t new_baud_rate) { 
        // 預設實現：不支援動態波特率調整
        return false; 
    }
    
protected:
    // 統一錯誤處理
    void logError(const char* operation, const char* details) const {
        // 🔇 暫時禁用Transport日誌避免死機
        // LOG_ERR("TRANSPORT", "[%s] %s: %s", getTransportName(), operation, details);
    }
    
    void logInfo(const char* operation, const char* details) const {
        // 🔇 暫時禁用Transport日誌避免死機  
        // LOG_INFO("TRANSPORT", "[%s] %s: %s", getTransportName(), operation, details);
    }
};

/**
 * @brief UART 傳輸實現
 * 
 * Arduino 環境下的串口封裝
 * 支援動態波特率和緩衝區重置
 */
class UARTTransport : public ITransport {
public:
    explicit UARTTransport(HardwareSerial& serial, uint32_t baud_rate = 115200)
        : serial_(serial), baud_rate_(baud_rate), initialized_(false) {}
    
    bool initialize() override {
        serial_.begin(baud_rate_);
        
        // 等待串口穩定
        delay(100);
        
        // 清空初始緩衝區
        resetBuffers();
        
        initialized_ = true;
        logInfo("initialize", "UART 初始化完成");
        return true;
    }
    
    void cleanup() override {
        if (initialized_) {
            flush();
            serial_.end();
            initialized_ = false;
            logInfo("cleanup", "UART 已關閉");
        }
    }
    
    bool isReady() const override {
        return initialized_ && serial_;
    }
    
    size_t write(const uint8_t* data, size_t length) override {
        if (!isReady()) {
            logError("write", "UART 未準備就緒");
            return 0;
        }
        
        return serial_.write(data, length);
    }
    
    size_t read(uint8_t* buffer, size_t max_length) override {
        if (!isReady()) {
            return 0;
        }
        
        size_t bytes_read = 0;
        while (serial_.available() && bytes_read < max_length) {
            buffer[bytes_read++] = serial_.read();
        }
        
        return bytes_read;
    }
    
    size_t available() const override {
        return isReady() ? serial_.available() : 0;
    }
    
    void flush() override {
        if (isReady()) {
            serial_.flush();  // 等待發送完成
        }
    }
    
    void clearRxBuffer() override {
        if (isReady()) {
            // 清空接收緩衝區
            while (serial_.available()) {
                serial_.read();
            }
            logInfo("clearRxBuffer", "RX 緩衝區已清空");
        }
    }
    
    void resetBuffers() override {
        flush();
        clearRxBuffer();
        logInfo("resetBuffers", "所有緩衝區已重置");
    }
    
    const char* getTransportName() const override {
        return "UART";
    }
    
    uint32_t getBaudRate() const override {
        return baud_rate_;
    }
    
    // UART 特有功能 - 覆寫基類的虛函數（非阻塞版本）
    bool setBaudRate(uint32_t new_baud_rate) override {
        if (new_baud_rate != baud_rate_) {
            baud_rate_ = new_baud_rate;
            if (initialized_) {
                serial_.end();
                // 移除阻塞延遲，讓系統繼續運行
                serial_.begin(baud_rate_);
                resetBuffers();
                logInfo("setBaudRate", "波特率已更新（非阻塞）");
            }
            return true;
        }
        return true; // 已經是目標波特率
    }

private:
    HardwareSerial& serial_;
    uint32_t baud_rate_;
    bool initialized_;
};