#include "AR1AFCProtocol.h"
#include <cstring>  // for memset

/**
 * AR1AFCProtocol 實現文件
 * 
 * 包含所有AR1AFC協議的具體實現，特別是封包發送和HEX輸出功能
 */

void AR1AFCProtocol::logNavigationDebugInfo(const NavigationState& nav_data) {
    // 調試：前10次立即顯示，然後改為每5秒
    static uint32_t debug_count = 0;
    static uint32_t last_debug_time = 0;
    uint32_t now = millis();
    
    if (debug_count < 10 || (now - last_debug_time >= 5000)) {
        debug_count++;
        last_debug_time = now;
        LOGE("AR1AFC輸入數據[%u]: gyro(%.3f,%.3f,%.3f) accel(%.3f,%.3f,%.3f)", 
             debug_count,
             nav_data.angular_velocity_x, nav_data.angular_velocity_y, nav_data.angular_velocity_z,
             nav_data.acceleration_east, nav_data.acceleration_north, nav_data.acceleration_down);
    }
}

bool AR1AFCProtocol::sendAR1AFCPacket(AR1AFCPacket& packet) {
    // TDD頻率監控：AR1AFC實際封包發送頻率
    // 移除：過度細節的頻率監控
    
    if (!isReady()) {
        logProtocolError("sendAR1AFCPacket", "協議未準備就緒");
        return false;
    }
    
    // 創建符合原始實現的封包格式 (direct memcpy approach)
    uint8_t buf[PACKET_SIZE];
    
    // 1) Header: 直接memcpy (符合原始代碼)
    memcpy(buf, PACKET_HEADER, 4);           // Header: 0xFE 0x81 0xFF 0x55
    
    // 2) 角速度：直接memcpy float數據 (符合原始 omg.bin_val)
    memcpy(buf + 4, &packet.gyro_x, 4);     // X-axis Rotational Data (DPS)
    memcpy(buf + 8, &packet.gyro_y, 4);     // Y-axis Rotational Data (DPS)  
    memcpy(buf + 12, &packet.gyro_z, 4);    // Z-axis Rotational Data (DPS)
    
    // 3) 加速度：直接memcpy float數據 (符合原始 acc.bin_val)
    memcpy(buf + 16, &packet.accel_x, 4);   // X-axis Acceleration Data (g)
    memcpy(buf + 20, &packet.accel_y, 4);   // Y-axis Acceleration Data (g)
    memcpy(buf + 24, &packet.accel_z, 4);   // Z-axis Acceleration Data (g)
    
    // 4) 溫度：實現 MINSPixhawk 風格的字節序反轉
    uint8_t temp_bytes[4];
    memcpy(temp_bytes, &packet.temperature, 4);
    uint8_t new_temp_bin[4];
    for (int i = 0; i < 4; i++) {
        new_temp_bin[i] = temp_bytes[3-i];  // 字節序反轉 (與 MINSPixhawk 一致)
    }
    memcpy(buf + 28, new_temp_bin, 4); // Temperature (°C) - 反轉字節序
    
    // 5) 時間計數器：直接使用原始時間 (移除 * 0.1 轉換)
    memcpy(buf + 32, &packet.time_counter, 4); // Time Counter (millisecond)
    
    // 6) 姿態角：直接memcpy (符合原始 corrected_ori.bin_val)
    memcpy(buf + 36, &packet.pitch, 4);     // Attitude Pitch (deg)
    memcpy(buf + 40, &packet.roll, 4);      // Attitude Roll (deg)
    memcpy(buf + 44, &packet.yaw, 4);       // Attitude Yaw (deg)
    
    // 7) CRC-32：使用 MINSPixhawk 風格的 CRC 計算 (完全相同的算法)
    MINSPixhawkCRC minsPixhawkCrc;
    minsPixhawkCrc.calCRC(buf, PACKET_SIZE);  // 直接寫入buffer最後4字節
    
    // 顯示要發送的原始hex數據 - 使用TDD頻率控制 (與MAVLink相同格式)
    static uint32_t hex_last_print = 0;
    uint32_t hex_now = millis();
    if (hex_now - hex_last_print >= 3000) {  // 3秒輸出一次
        hex_last_print = hex_now;
        char hex_buffer[100] = "🔍 發送HEX: ";
        char temp[4];
        for (int i = 0; i < PACKET_SIZE && i < 32; i++) {
            sprintf(temp, "%02X ", buf[i]);
            strcat(hex_buffer, temp);
        }
        TDD_PRINT("%s", hex_buffer);
    }
    
    // 8) 發送完整的官方規範格式封包
    if (!transport_ || !transport_->isReady()) {
        return false;
    }
    
    size_t sent = transport_->write(buf, PACKET_SIZE);
    
    if (sent != PACKET_SIZE) {
        logProtocolError("sendAR1AFCPacket", "封包發送不完整");
        return false;
    }
    
    packet_counter_++;
    
    // 降低日誌頻率：每5秒報告一次 (與MAVLink相同頻率)
    static uint32_t last_log_time = 0;
    uint32_t now = millis();
    if (now - last_log_time >= 5000) {
        char log_msg[64];
        snprintf(log_msg, sizeof(log_msg), "AR-1A-FC 封包發送正常 (累計 %lu 個)", packet_counter_);
        logProtocolInfo("sendAR1AFCPacket", log_msg);
        last_log_time = now;
    }
    
    return true;
}