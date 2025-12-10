#pragma once
#include <Arduino.h>

// GNSS 控制類
class GnssControl {
public:
    // 初始化 GNSS 控制
    static bool initialize();

    // 發送 LOCOSYS 命令
    static bool sendLocosysCommand(const char* command);

    // 設定 GNSS 更新頻率
    static bool setUpdateFrequency(uint8_t frequency_hz);

private:
    // 計算 NMEA 校驗和
    static uint8_t calculateNmeaChecksum(const char* sentence);

    // 等待 GNSS 模組回應
    static bool waitForResponse(uint32_t timeout_ms = 1000);

    // 常數定義
    static const uint32_t RESPONSE_TIMEOUT_MS = 3000;
    static const size_t RESPONSE_BUFFER_SIZE = 256;
};