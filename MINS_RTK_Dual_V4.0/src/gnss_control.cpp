#include "gnss_control.h"
#include "myUART.h"
#include <string.h>

bool GnssControl::initialize() {
    Serial.println("🛰️ GNSS Control 初始化");
    return true;
}

bool GnssControl::sendLocosysCommand(const char* command) {
    if (!command) {
        Serial.println("❌ 發送失敗: 命令為空");
        return false;
    }

    // 構造完整的 NMEA 格式指令
    char full_command[128];
    snprintf(full_command, sizeof(full_command), "$%s", command);

    // 計算校驗和
    uint8_t checksum = calculateNmeaChecksum(command);

    // 添加校驗和和結束符
    char complete_command[140];
    snprintf(complete_command, sizeof(complete_command), "%s*%02X\r\n", full_command, checksum);

    // 發送到 GNSS 模組 (Serial4)
    size_t written = Serial4.write((const uint8_t*)complete_command, strlen(complete_command));

    // 顯示發送的指令
    Serial.print("📤 發送 LOCOSYS 配置: ");
    Serial.print(complete_command);

    bool send_success = (written == strlen(complete_command));
    if (!send_success) {
        Serial.print("❌ 發送失敗: 預期 ");
        Serial.print(strlen(complete_command));
        Serial.print(" bytes, 實際 ");
        Serial.print(written);
        Serial.println(" bytes");
        return false;
    }

    Serial.println("✅ 發送完成，等待模組回應...");

    // 等待模組回應
    return waitForResponse();
}

bool GnssControl::setUpdateFrequency(uint8_t frequency_hz) {
    if (frequency_hz < 1 || frequency_hz > 10) {
        Serial.println("❌ 頻率設定錯誤: 支援範圍 1-10Hz");
        return false;
    }

    char command[32];
    snprintf(command, sizeof(command), "PLSC,FIXRATE,%d", frequency_hz);

    Serial.print("設定更新頻率為 ");
    Serial.print(frequency_hz);
    Serial.println("Hz...");

    return sendLocosysCommand(command);
}

bool GnssControl::queryVersion() {
    Serial.println("查詢 GNSS 模組版本...");
    return sendLocosysCommand("PLSC,VER");
}

bool GnssControl::queryUpdateFrequency() {
    Serial.println("查詢當前更新頻率...");
    return sendLocosysCommand("PLSC,FIXRATE,?");
}

bool GnssControl::configureGnss(uint8_t frequency_hz) {
    Serial.println("🚀 開始 GNSS 配置流程");

    // 1. 查詢版本
    if (!queryVersion()) {
        Serial.println("⚠️ 版本查詢失敗，繼續執行...");
    }

    delay(500);  // 稍候等待

    // 2. 查詢當前頻率
    if (!queryUpdateFrequency()) {
        Serial.println("⚠️ 頻率查詢失敗，繼續執行...");
    }

    delay(500);  // 稍候等待

    // 3. 設定頻率
    if (!setUpdateFrequency(frequency_hz)) {
        Serial.println("❌ 頻率設定失敗");
        return false;
    }

    Serial.println("✅ GNSS 配置完成");
    return true;
}


uint8_t GnssControl::calculateNmeaChecksum(const char* sentence) {
    uint8_t checksum = 0;

    // 跳過開頭的 $ 符號，從第一個字符開始計算
    const char* p = sentence;
    if (*p == '$') {
        p++;  // 跳過 $
    }

    // 計算直到 * 或字串結束
    while (*p && *p != '*') {
        checksum ^= (uint8_t)*p;
        p++;
    }

    return checksum;
}

bool GnssControl::waitForResponse(uint32_t timeout_ms) {
    bool response_received = false;
    uint32_t start_time = millis();
    char response_buffer[RESPONSE_BUFFER_SIZE];
    int response_pos = 0;

    Serial.println("⏳ 等待 GNSS 模組回應...");

    while (millis() - start_time < timeout_ms && !response_received) {
        if (Serial4.available() > 0) {
            char c = Serial4.read();

            if (c == '$' && response_pos == 0) {
                response_buffer[response_pos++] = c;
            } else if (response_pos > 0 && response_pos < RESPONSE_BUFFER_SIZE - 1) {
                response_buffer[response_pos++] = c;

                if (c == '\n' || c == '\r') {
                    response_buffer[response_pos] = '\0';
                    if (strstr(response_buffer, "PLSC") || strstr(response_buffer, "PAIR") ||
                        strstr(response_buffer, "OK") || strstr(response_buffer, "ACK")) {
                        response_received = true;
                        Serial.print("📨 收到模組回應: ");
                        Serial.println(response_buffer);
                        break;
                    }
                    response_pos = 0;
                }
            } else if (response_pos >= RESPONSE_BUFFER_SIZE - 1) {
                response_pos = 0;
            }
        }
        delay(1);
    }

    if (response_received) {
        Serial.println("✅ 命令配置確認成功");
    } else {
        Serial.println("⚠️ 等待模組回應超時，但命令已發送");
    }

    return true;
}