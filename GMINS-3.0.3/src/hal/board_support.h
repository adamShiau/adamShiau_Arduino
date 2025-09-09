#pragma once
#include <Arduino.h>
#include "../util/data_flow_monitor.h"
#include "../comm/IByteSource.h"
#include "../comm/uart_interface.h"

// === UART 通道枚舉（恢復正確工作版本的映射） ===
// 基於 312a6ea commit 的正確配置
enum class UARTChannel : uint8_t {
  DEBUG   = 0,  // Serial   - USB Debug
  GNSS    = 1,  // Serial4  - LOCOSYS GPS 輸入 ✅
  IMU     = 2,  // Serial2  - Xsens IMU 輸入
  MAVLINK = 3,  // Serial1  - Pixhawk PX4 輸出
  AUX     = 4,  // Serial3  - NMEA 輸出
};

struct UARTConfig {
  uint32_t    default_baud_rate;
  int8_t      tx_pin;            // 若無需動態用不到，可設 -1
  int8_t      rx_pin;            // 若無需動態用不到，可設 -1
  bool        enabled;
  uint16_t    rx_buffer_size;    // 視需求使用
  const char* description;
};

namespace hal {

// 全域配置表與數量
extern const UARTConfig UART_CONFIGS[5];
extern const size_t     UART_COUNT;

// Arduino SAMD 專用 Uart 實例（宣告）
extern Uart Serial1; // PX4 Output (SERCOM5)
extern Uart Serial2; // XSENS Input (SERCOM2)
extern Uart Serial3; // NMEA  Output (SERCOM1)
extern Uart Serial4; // GNSS   Input (SERCOM3)

// 基本初始化
void initPins();
void initUarts();
void initPeripherals();

// 存取介面
HardwareSerial& getUARTSerial(UARTChannel channel);
bool            isUARTReady(UARTChannel channel);                    // 可寫入
bool            resetUART(UARTChannel channel, uint32_t baud = 0);   // 重新開啟

// ================== 監控功能 ==================

/**
 * @brief 初始化 HAL UART 監控系統
 */
void initMonitoring();

/**
 * @brief 關閉 HAL UART 監控系統
 */
void shutdownMonitoring();

/**
 * @brief 記錄 UART 傳輸數據
 * @param channel UART 通道
 * @param bytes 傳輸字節數
 */
void recordUARTTransmit(UARTChannel channel, uint32_t bytes);

/**
 * @brief 記錄 UART 接收數據
 * @param channel UART 通道  
 * @param bytes 接收字節數
 */
void recordUARTReceive(UARTChannel channel, uint32_t bytes);

/**
 * @brief 更新所有 UART 監控器並檢查報告
 */
void updateUARTMonitoring();

/**
 * @brief 生成 HAL UART 監控報告
 */
void generateUARTMonitoringReport();

/**
 * @brief 重置 HAL UART 監控統計
 */
void resetUARTMonitoringStats();

/**
 * @brief 取得指定 UART 通道的統計數據
 * @param channel UART 通道
 * @return 數據流統計結構
 */
monitor::DataFlowStats getUARTStats(UARTChannel channel);

/**
 * @brief 檢查監控系統是否已初始化
 * @return true 如果已初始化
 */
bool isMonitoringInitialized();

// ================== 硬體資料讀取功能 ==================

/**
 * @brief 回調函數類型，用於接收硬體資料
 * @param data 資料緩衝區
 * @param len 資料長度
 * @param user 用戶數據指標
 */
using ByteSink = void(*)(const uint8_t* data, size_t len, void* user);

/**
 * @brief 註冊 IMU 資料接收器
 * @param fn 回調函數
 * @param user 用戶數據指標
 */
void attachIMUSink(ByteSink fn, void* user);

/**
 * @brief 註冊 GNSS 資料接收器
 * @param fn 回調函數
 * @param user 用戶數據指標
 */
void attachGNSSSink(ByteSink fn, void* user);

/**
 * @brief 執行一次硬體資料讀取
 * 從 Serial2 (IMU) 和 Serial4 (GNSS) 讀取資料並分發給註冊的接收器
 */
void pollOnce();

/**
 * @brief MTI 封包偵測和記錄
 * @param buffer 資料緩衝區
 * @param len 資料長度
 */
void detectAndRecordMTIPackets(const uint8_t* buffer, size_t len);

/**
 * @brief NMEA 句子偵測和記錄
 * @param buffer 資料緩衝區
 * @param len 資料長度
 */
void detectAndRecordNMEAPackets(const uint8_t* buffer, size_t len);

// ================== 感測器測試功能 ==================

/**
 * @brief 初始化 XSENS 感測器測試系統
 */
void initializeXSENSTest();

/**
 * @brief 處理所有感測器數據 (包含模擬數據)
 */
void processAllSensorData();

/**
 * @brief 產生所有感測器狀態報告
 */
void reportAllSensorStatus();

/**
 * @brief 獲取 IMU 數據源接口（抽象介面）
 * @return IMU 數據源指針，供 Parser 使用
 */
IByteSource* getIMUSource();

/**
 * @brief 獲取 GNSS 數據源接口（抽象介面）
 * @return GNSS 數據源指針，供 Parser 使用
 */
IByteSource* getGNSSSource();

/**
 * @brief 獲取 IMU UART 介面（用於命令發送）
 * @return IMU UART 介面指針，供 MTI 驅動程式使用
 */
UARTInterface* getIMUInterface();

/**
 * @brief 獲取 GNSS UART 介面（用於命令發送） 
 * @return GNSS UART 介面指針
 */
UARTInterface* getGNSSInterface();

// 調試命令功能已移除

// 模擬數據功能已移除

} // namespace hal

// ================== 便利宏定義 ==================

// HAL UART 監控記錄宏
#define HAL_RECORD_TX(channel, bytes)  hal::recordUARTTransmit(channel, bytes)
#define HAL_RECORD_RX(channel, bytes)  hal::recordUARTReceive(channel, bytes)

// HAL UART 監控報告宏
#define HAL_UPDATE_MONITORING()        hal::updateUARTMonitoring()
#define HAL_REPORT_MONITORING()        hal::generateUARTMonitoringReport()
#define HAL_RESET_MONITORING()         hal::resetUARTMonitoringStats()

// HAL UART 特定通道監控宏
#define HAL_RECORD_DEBUG_TX(bytes)     HAL_RECORD_TX(hal::UARTChannel::DEBUG, bytes)
#define HAL_RECORD_DEBUG_RX(bytes)     HAL_RECORD_RX(hal::UARTChannel::DEBUG, bytes)
#define HAL_RECORD_MAVLINK_TX(bytes)   HAL_RECORD_TX(hal::UARTChannel::MAVLINK, bytes)
#define HAL_RECORD_MAVLINK_RX(bytes)   HAL_RECORD_RX(hal::UARTChannel::MAVLINK, bytes)
#define HAL_RECORD_IMU_TX(bytes)       HAL_RECORD_TX(hal::UARTChannel::IMU, bytes)
#define HAL_RECORD_IMU_RX(bytes)       HAL_RECORD_RX(hal::UARTChannel::IMU, bytes)
#define HAL_RECORD_AUX_TX(bytes)       HAL_RECORD_TX(hal::UARTChannel::AUX, bytes)
#define HAL_RECORD_AUX_RX(bytes)       HAL_RECORD_RX(hal::UARTChannel::AUX, bytes)
#define HAL_RECORD_GNSS_TX(bytes)      HAL_RECORD_TX(hal::UARTChannel::GNSS, bytes)
#define HAL_RECORD_GNSS_RX(bytes)      HAL_RECORD_RX(hal::UARTChannel::GNSS, bytes)

// HAL 硬體資料讀取宏
#define HAL_POLL_SENSORS()             hal::pollOnce()
#define HAL_ATTACH_IMU_SINK(fn, user)  hal::attachIMUSink(fn, user)
#define HAL_ATTACH_GNSS_SINK(fn, user) hal::attachGNSSSink(fn, user)

// ================== Feed-based UART Interface 功能 ==================

/**
 * @brief 串接 Feed-based UART Interfaces 到 HAL sink
 * 
 * 此函數將 HAL 的 sink 機制與 UART interface 的 ring buffer 連接，
 * 實現「被餵食的 ring buffer」架構。
 * 應在 initPeripherals() 後調用一次。
 */
void wireUartInterfaces();

/**
 * @brief 報告 UART Interface 統計信息
 * 
 * 顯示各個 UART interface 的輸入/輸出/丟棄統計，
 * 以及當前 ring buffer 的使用情況。
 */
void reportUARTInterfaceStats();

/**
 * @brief 在 HAL 層記錄 MTI 原始資料 (可配置筆數)
 * @param data 原始資料緩衝區
 * @param length 資料長度
 */
void logMTIRawDataInHAL(const uint8_t* data, size_t length);

/**
 * @brief 在 HAL 層記錄 GNSS 原始資料 (可配置筆數)
 * @param data 原始資料緩衝區
 * @param length 資料長度
 */
void logGNSSRawDataInHAL(const uint8_t* data, size_t length);

/**
 * @brief 設定 MTI 記錄封包數量限制（用於 shift 角度分析）
 * @param limit 記錄封包數量限制 (1-255)
 */
void setMTIRecordLimit(uint8_t limit);

/**
 * @brief 設定 GNSS 記錄封包數量限制
 * @param limit 記錄封包數量限制 (1-255)
 */
void setGNSSRecordLimit(uint8_t limit);
