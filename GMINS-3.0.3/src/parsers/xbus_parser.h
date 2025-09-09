#pragma once

/**
 * XBUS Data Parser
 * 
 * 職責：
 * - 解析 XBUS frame 為 IMUData 結構
 * - 提取四元數、加速度、角速度、磁力計數據
 * - 處理數據驗證和單位轉換
 * - 支援多種 XBUS 訊息類型
 * 
 * 設計原則：
 * - Header-only 實作，提高性能
 * - 無狀態設計，線程安全
 * - 完整的錯誤檢查和數據驗證
 * - 支援即時解析
 */

#include "../data/data_types.h"
#include "../util/log.h"
#include "parser_utils.h"
#include <cstring>
#include <cmath>
#include <Arduino.h>

namespace XbusParser {
    
    // 前置聲明
    inline bool parseDataPayload(const uint8_t* data, size_t length, IMUData& imu_data);
    inline uint8_t calculateDataQuality(const IMUData& imu_data);
    
    // XBUS 訊息類型
    enum class XbusMessageType : uint8_t {
        MT_DATA2 = 0x36,           // 主要數據訊息
        MT_DATA = 0x32,            // 傳統數據訊息
        INVALID = 0xFF
    };
    
    // XBUS 數據項 ID（XDI）
    enum class XbusDataID : uint16_t {
        // 時間戳
        PACKET_COUNTER = 0x1020,
        SAMPLE_TIME_FINE = 0x1060,
        
        // 溫度
        TEMPERATURE = 0x0810,
        
        // 加速度（m/s²）
        ACCELERATION = 0x4020,
        ACCELERATION_NED = 0x4024,
        
        // 角速度（rad/s）
        RATE_OF_TURN = 0x8020,
        RATE_OF_TURN_NED = 0x8024,
        
        // 磁力計（µT）
        MAGNETIC_FIELD = 0xC020,
        MAGNETIC_FIELD_NED = 0xC024,
        
        // 四元數
        QUATERNION = 0x2010,
        QUATERNION_NED = 0x2014,
        
        // 旋轉矩陣（3x3 float）
        ROTATION_MATRIX = 0x2020,
        
        // 歐拉角（rad）
        EULER_ANGLES = 0x2030,
        EULER_ANGLES_NED = 0x2034,
        
        // 狀態標誌
        STATUS_BYTE = 0xE010,
        STATUS_WORD = 0xE020,
        
        INVALID = 0xFFFF
    };
    
    // XBUS Frame 標頭結構
    struct XbusHeader {
        uint8_t preamble;          // 0xFA
        uint8_t bus_id;            // 通常為 0xFF
        uint8_t message_id;        // 訊息類型
        uint8_t length;            // 數據長度
    } __attribute__((packed));
    
    /**
     * @brief 解析 XBUS frame 為 IMUData (支援擴展長度 LEN=0xFF)
     * @param frame_data XBUS frame 數據（包含 header）
     * @param frame_length frame 總長度
     * @param imu_data 輸出的 IMU 數據結構
     * @return true 解析成功
     */
    bool parseXbusFrame(const uint8_t* frame_data, size_t frame_length, IMUData& imu_data);
    
    /**
     * @brief 解析 XBUS 數據載荷 (MTData2 格式: XDI(2B) + LEN(1B) + DATA)
     */
    bool parseDataPayload(const uint8_t* data, size_t length, IMUData& imu_data);
    
    
    /**
     * @brief 計算 IMU 數據品質分數
     */
    inline uint8_t calculateDataQuality(const IMUData& imu_data) {
        uint8_t quality = 0;
        
        // 基於可用數據類型評分
        if (imu_data.flags & IMU_ACCEL_VALID) quality += 30;
        if (imu_data.flags & IMU_GYRO_VALID) quality += 30;
        if (imu_data.flags & IMU_MAG_VALID) quality += 20;
        if (imu_data.flags & IMU_QUATERNION_VALID) quality += 15;
        if (imu_data.flags & IMU_CALIBRATED) quality += 5;
        
        return quality;
    }
    
    /**
     * @brief 檢查 XBUS 數據的基本有效性
     */
    inline bool isValidXbusData(const IMUData& imu_data) {
        // 只要有任何數據標誌都算有效（包括 IMU_DATA_FRESH）
        return (imu_data.flags != 0) && (imu_data.flags & IMU_DATA_FRESH);
    }
}