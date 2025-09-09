#include "xsens_mti_driver.h"

#define LOG_TAG "MTI_DRV"

XsensMTIDriver::XsensMTIDriver(UARTInterface* uart_interface)
    : uart_interface_(uart_interface) {
}

void XsensMTIDriver::sendCommandSimple(uint8_t cmd_id, const uint8_t* data, uint8_t data_len) {
    
    // 1. 發送 HEADER [0xFA, 0xFF]
    uint8_t header[2] = {XBUS_PREAMBLE, XBUS_BUS_ID};
    uart_interface_->write(header, 2);
    
    // 2. 構建消息：[MID, LEN, [DATA], CHK]
    uint8_t message[256];
    size_t msg_len = 0;
    
    message[msg_len++] = cmd_id;         // MID
    message[msg_len++] = data_len;       // LEN
    
    // 添加數據（如果有）
    if (data && data_len > 0) {
        memcpy(&message[msg_len], data, data_len);
        msg_len += data_len;
    }
    
    // 計算校驗和
    uint8_t checksum_data[256];
    checksum_data[0] = XBUS_BUS_ID;
    memcpy(&checksum_data[1], message, msg_len);
    uint8_t checksum = calculateChecksum(checksum_data, msg_len + 1);
    message[msg_len++] = checksum;
    
    // 3. 發送消息
    uart_interface_->write(message, msg_len);
    
    // 詳細的除錯資訊
    LOG_INFO("📤 發送命令: MID=0x%02X, LEN=%u", cmd_id, data_len);
    
    // 顯示完整的封包內容
    if (data_len > 0) {
        char hex_str[128];
        char* ptr = hex_str;
        for (int i = 0; i < data_len && i < 16; i++) {
            ptr += sprintf(ptr, "%02X ", data[i]);
        }
        LOG_INFO("📋 數據內容: %s", hex_str);
    }
    
    // 顯示完整封包
    char full_packet[256];
    char* pkt_ptr = full_packet;
    pkt_ptr += sprintf(pkt_ptr, "FA FF ");  // Header
    for (size_t i = 0; i < msg_len && i < 32; i++) {
        pkt_ptr += sprintf(pkt_ptr, "%02X ", message[i]);
    }
    LOG_INFO("🔍 完整封包: %s", full_packet);
}

uint8_t XsensMTIDriver::calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(0x00 - sum); // Two's complement
}

void XsensMTIDriver::initialize() {
    LOG_INFO("🔧 MTI 設備初始化開始...");
    
    // 1. 進入配置模式
    LOG_INFO("   🔧 CMD_GOTO_CONFIG (0x30)");
    sendCommandSimple(0x30);  // CMD_GOTO_CONFIG
    delay(100);
    
    // 2. 執行初始化
    LOG_INFO("   🔧 CMD_INIT (0x02)");
    sendCommandSimple(0x02);  // CMD_INIT
    delay(200);
    
    // 3. 設定採樣頻率為 100Hz
    LOG_INFO("   🔧 CMD_SET_SAMPLE_PERIOD (0xD4) -> 100Hz");
    uint8_t sample_period[] = {0x00, 0x0A};  // 10ms = 100Hz (0x000A)
    sendCommandSimple(0xD4, sample_period, sizeof(sample_period));  // CMD_SET_SAMPLE_PERIOD
    delay(100);
    
    // 4. 設定輸出配置 @ 100Hz (完整配置：包含所有必需數據)
    LOG_INFO("   🔧 CMD_OUTPUT_CONFIG (0xC0) -> PackageCounter + SampleTimeFine + EulerAngles + Accel + Gyro + Mag @ 100Hz");
    uint8_t output_config[] = {
        0x10, 0x20, 0x00, 0x64,  // Package Counter @ 100 Hz
        0x10, 0x60, 0x00, 0x64,  // SampleTimeFine @ 100 Hz (時間對齊)
        0x20, 0x34, 0x00, 0x64,  // Euler Angles NED @ 100 Hz (0x2034) - 強制NED輸出
        0x40, 0x20, 0x00, 0x64,  // Acceleration @ 100 Hz (0x4020) - 品質檢查必需
        0x80, 0x20, 0x00, 0x64,  // Rate of Turn (Gyro) @ 100 Hz (0x8020) - 品質檢查必需
        0xC0, 0x20, 0x00, 0x64   // Magnetic Field @ 100 Hz (0xC020) - 提升品質分數
    };
    sendCommandSimple(0xC0, output_config, sizeof(output_config));  // CMD_OUTPUT_CONFIG
    delay(100);
    
    // 5. 進入測量模式
    LOG_INFO("   🔧 CMD_GOTO_MEASUREMENT (0x10)");
    sendCommandSimple(0x10);  // CMD_GOTO_MEASUREMENT
    delay(200);  // 等待測量模式穩定
    
    // 6. 陀螺儀校準 (必須在測量模式下執行) - 使用官方標準參數
    LOG_INFO("   🔧 GYRO_CALIBRATION (0x22) - 15 seconds (官方標準參數)");
    LOG_INFO("   ⚠️  請保持設備完全靜止 15 秒進行陀螺儀校準...");
    uint8_t calibration_time[] = {0x00, 0x0F};  // 15 seconds in big-endian format (0x0F = 15) - 官方標準參數
    
    // 詳細打印bias校準命令
    LOG_INFO("   📋 發送Bias校準命令:");
    LOG_INFO("      - 命令ID: 0x22 (CMD_SET_NO_ROTATION)");
    LOG_INFO("      - 校準時間: %d 秒 (0x%02X%02X) - 官方標準參數", 15, calibration_time[0], calibration_time[1]);
    LOG_INFO("      - 預期封包: FA FF 22 02 00 0F CE");
    
    sendCommandSimple(0x22, calibration_time, sizeof(calibration_time));  // CMD_SET_NO_ROTATION
    delay(20000);  // 等待20秒確保校準完成 (15秒校準 + 5秒緩衝)
    LOG_INFO("   ✅ 陀螺儀校準完成");
    
    LOG_INFO("✅ MTI 基本初始化完成 (Heading Reset 將單獨執行)");
}

void XsensMTIDriver::resetHeading(float current_yaw_deg) {
    LOG_INFO("   🔧 執行 Heading Reset - 當前 YAW: %.2f°", current_yaw_deg);
    
    uint8_t reset_data[] = {0x00, 0x04};
    sendCommandSimple(0xA4, reset_data, sizeof(reset_data));
    delay(300);
}