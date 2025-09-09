#include "board_support.h"
#include "../util/gmins_frequency_manager.h"
#include "../util/log.h"
#include "../comm/uart_interface.h"
#include "../comm/IByteSource.h"
#include "../util/tdd_flow_checker.h"
#include <string.h>

// 編譯期檢查：確保 UARTChannel 枚舉值與預期一致
static_assert((int)UARTChannel::DEBUG   == 0, "UARTChannel order changed");
static_assert((int)UARTChannel::GNSS    == 1, "UARTChannel order changed");  
static_assert((int)UARTChannel::IMU     == 2, "UARTChannel order changed");
static_assert((int)UARTChannel::MAVLINK == 3, "UARTChannel order changed");
static_assert((int)UARTChannel::AUX     == 4, "UARTChannel order changed");

#define LOG_TAG "HAL"

#ifndef LOG_LEVEL_LOCAL
#define LOG_LEVEL_LOCAL LOG_DEBUG
#endif

#ifndef LOG_LEVEL_MASK_LOCAL
#define LOG_LEVEL_MASK_LOCAL 0
#endif

// Arduino SAMD 核心的腳位週邊設定
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#include "wiring_private.h"
#endif

// ================== 配置表 ==================
namespace hal {

// SERCOM 對應: SERCOM0=I2C, SERCOM1=Serial3, SERCOM2=Serial2,
//               SERCOM3=Serial4, SERCOM4=SPI,  SERCOM5=Serial1
// 恢復 312a6ea commit 的正確配置
const UARTConfig UART_CONFIGS[5] = {
  {115200,  0,   0,   true,  512,  "USB Debug"},          // DEBUG (Serial)    - 通道 0
  {115200,  10,  9,   true,  256,  "LOCOSYS GPS Input"},  // GNSS (Serial4)    - 通道 1 ✅
  {115200,  25,  24,  true,  1024, "Xsens IMU Input"},    // IMU (Serial2)     - 通道 2 ✅
  {460800,  23,  22,  true,  1024, "Pixhawk PX4 Output"}, // MAVLINK (Serial1) - 通道 3 ✅
  {115200,  13,  8,   true,  512,  "NMEA Output"}         // AUX (Serial3)     - 通道 4 ✅
};

const size_t UART_COUNT = 5;

// ================== 監控管理器 ==================
static monitor::DataFlowMonitor* uart_monitors_[UART_COUNT] = {nullptr};
static bool monitoring_initialized_ = false;

// 索引轉換函數：解決 UARTChannel 枚舉與 UART_CONFIGS 順序不一致的問題
static inline size_t idx(UARTChannel ch) {
  // 確保 monitoring 索引與 UART_CONFIGS 順序一致
  switch (ch) {
    case UARTChannel::DEBUG:   return 0;  // UART_CONFIGS[0] - USB Debug
    case UARTChannel::GNSS:    return 1;  // UART_CONFIGS[1] - LOCOSYS GPS ✅
    case UARTChannel::IMU:     return 2;  // UART_CONFIGS[2] - Xsens IMU
    case UARTChannel::MAVLINK: return 3;  // UART_CONFIGS[3] - Pixhawk PX4
    case UARTChannel::AUX:     return 4;  // UART_CONFIGS[4] - NMEA Output
  }
  return 0; // fallback
}

// 靜態字串儲存，避免指針失效
static char monitor_names_[UART_COUNT][64];

// ================== Uart 實例 ==================
// 注意：這些建構子簽名依 ArduinoCore-samd 的 Uart 類別而定
Uart Serial1(&sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX); // PX4 Output
Uart Serial2(&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);  // XSENS Input
Uart Serial3(&sercom1, 13, 8,  SERCOM_RX_PAD_1, UART_TX_PAD_2);  // NMEA Output
Uart Serial4(&sercom3, 10, 9,  SERCOM_RX_PAD_3, UART_TX_PAD_2);  // GNSS Input

// ================== Feed-based UART Interfaces ==================
// 全域實例供Parser/Router使用，被HAL sink餵食
static UARTInterface gIMU_Interface;
static UARTInterface gGNSS_Interface;

// ================== Feed-based UART Interface 串接 ==================

void wireUartInterfaces() {
  LOGI("🔌 正在串接 Feed-based UART Interfaces...");
  
  // 設置 IMU sink：將 HAL 讀取的數據餵給 IMU interface
  attachIMUSink([](const uint8_t* data, size_t len, void* user) {
    static_cast<UARTInterface*>(user)->feed(data, len);
  }, &gIMU_Interface);
  
  // 設置 GNSS sink：將 HAL 讀取的數據餵給 GNSS interface  
  attachGNSSSink([](const uint8_t* data, size_t len, void* user) {
    static_cast<UARTInterface*>(user)->feed(data, len);
  }, &gGNSS_Interface);
  
  // 設置溢位策略（建議丟最舊的數據）
  gIMU_Interface.setDropOldestOnOverflow(true);
  gGNSS_Interface.setDropOldestOnOverflow(true);
  
  // 設置可選的TX函數（如果需要回寫到硬體）
  gIMU_Interface.setTx([](const uint8_t* data, size_t len) -> size_t {
    HardwareSerial& serial = getUARTSerial(UARTChannel::IMU);
    return serial.write(data, len);
  });
  
  gGNSS_Interface.setTx([](const uint8_t* data, size_t len) -> size_t {
    HardwareSerial& serial = getUARTSerial(UARTChannel::GNSS);
    return serial.write(data, len);
  });
  
  LOGI("✅ Feed-based UART Interfaces 串接完成");
  LOGI("  📡 IMU Interface: Ring buffer size = %d bytes", UART_IF_RING_SIZE);
  LOGI("  📡 GNSS Interface: Ring buffer size = %d bytes", UART_IF_RING_SIZE);
}

// 獲取 UART Interface 統計信息（用於監控和調試）
void reportUARTInterfaceStats() {
  // 靜態變量用於計算5秒區間統計
  static uint64_t last_imu_in = 0, last_imu_out = 0, last_imu_drops = 0;
  static uint64_t last_gnss_in = 0, last_gnss_out = 0, last_gnss_drops = 0;
  static bool first_report = true;
  
  auto imu_stats = gIMU_Interface.getStats();
  auto gnss_stats = gGNSS_Interface.getStats();
  
  LOGI("📊 === UART Interface 統計報告 (5秒區間) ===");
  
  if (first_report) {
    // 第一次報告：顯示當前累積值
    LOGI("  🧭 IMU Interface (初始化):");
    LOGI("    📥 累積輸入: %lu bytes", (unsigned long)imu_stats.total_in);
    LOGI("    📤 累積輸出: %lu bytes", (unsigned long)imu_stats.total_out);
    LOGI("    🗑️ 累積丟棄: %lu bytes", (unsigned long)imu_stats.drops);
    
    LOGI("  🛰️ GNSS Interface (初始化):");
    LOGI("    📥 累積輸入: %lu bytes", (unsigned long)gnss_stats.total_in);
    LOGI("    📤 累積輸出: %lu bytes", (unsigned long)gnss_stats.total_out);
    LOGI("    🗑️ 累積丟棄: %lu bytes", (unsigned long)gnss_stats.drops);
    
    first_report = false;
  } else {
    // 計算5秒區間增量
    uint64_t imu_in_delta = imu_stats.total_in - last_imu_in;
    uint64_t imu_out_delta = imu_stats.total_out - last_imu_out;
    uint64_t imu_drops_delta = imu_stats.drops - last_imu_drops;
    
    uint64_t gnss_in_delta = gnss_stats.total_in - last_gnss_in;
    uint64_t gnss_out_delta = gnss_stats.total_out - last_gnss_out;
    uint64_t gnss_drops_delta = gnss_stats.drops - last_gnss_drops;
    
    LOGI("  🧭 IMU Interface (5秒區間):");
    LOGI("    📥 區間輸入: %lu bytes (%.2f KB/s)", (unsigned long)imu_in_delta, imu_in_delta / 5000.0);
    LOGI("    📤 區間輸出: %lu bytes (%.2f KB/s)", (unsigned long)imu_out_delta, imu_out_delta / 5000.0);
    LOGI("    🗑️ 區間丟棄: %lu bytes", (unsigned long)imu_drops_delta);
    LOGI("    📊 緩衝可用: %u bytes", (unsigned int)gIMU_Interface.available());
    
    LOGI("  🛰️ GNSS Interface (5秒區間):");
    LOGI("    📥 區間輸入: %lu bytes (%.2f KB/s)", (unsigned long)gnss_in_delta, gnss_in_delta / 5000.0);
    LOGI("    📤 區間輸出: %lu bytes (%.2f KB/s)", (unsigned long)gnss_out_delta, gnss_out_delta / 5000.0);
    LOGI("    🗑️ 區間丟棄: %lu bytes", (unsigned long)gnss_drops_delta);
    LOGI("    📊 緩衝可用: %u bytes", (unsigned int)gGNSS_Interface.available());
  }
  
  // 更新上次快照
  last_imu_in = imu_stats.total_in;
  last_imu_out = imu_stats.total_out;
  last_imu_drops = imu_stats.drops;
  last_gnss_in = gnss_stats.total_in;
  last_gnss_out = gnss_stats.total_out;
  last_gnss_drops = gnss_stats.drops;
  
  LOGI("=========================================");
}

} // namespace hal

// ================== SERCOM ISR ==================
extern "C" {
  void SERCOM1_Handler() { hal::Serial3.IrqHandler(); } // Serial3
  void SERCOM2_Handler() { hal::Serial2.IrqHandler(); } // Serial2
  void SERCOM3_Handler() { hal::Serial4.IrqHandler(); } // Serial4
  void SERCOM5_Handler() { hal::Serial1.IrqHandler(); } // Serial1
}

// ================== 基本初始化 ==================
namespace hal {

void initPins() {
  // 預留 GPIO 初始化
  // 可在此設定 LED、按鈕等 GPIO
}

void initUarts() {
  // 基於原始 myUART_init() 函數
  Serial.begin(115200);   // Debug USB
  Serial1.begin(230400);  // Pixhawk PX4 輸出 - 使用AR-1A-FC預設波特率
  Serial2.begin(115200);  // Xsens IMU 輸入
  Serial3.begin(115200);  // NMEA 輸出  
  Serial4.begin(115200);  // LOCOSYS GPS 輸入
  
  // 配置 PIN 外設功能 (恢復原始配置)
  pinPeripheral(24, PIO_SERCOM);    // Serial2 TX (Xsens)
  pinPeripheral(25, PIO_SERCOM);    // Serial2 RX (Xsens)
  
  pinPeripheral(8, PIO_SERCOM);     // Serial3 TX (NMEA_OUT)  
  pinPeripheral(13, PIO_SERCOM);    // Serial3 RX (NMEA_OUT)
  
  pinPeripheral(10, PIO_SERCOM_ALT); // Serial4 TX (GPS_IN)
  pinPeripheral(9, PIO_SERCOM_ALT);  // Serial4 RX (GPS_IN)
}

void initPeripherals() {
  initPins();
  initUarts();
  initMonitoring();
  
  // 串接 Feed-based UART Interfaces
  wireUartInterfaces();
}

// ================== 監控功能 ==================

void initMonitoring() {
  if (monitoring_initialized_) {
    LOGW("HAL monitoring already initialized");
    return;
  }
  
  LOGI("Initializing HAL UART monitoring...");
  
  // 為每個 UART 通道創建監控器
  for (size_t i = 0; i < UART_COUNT; i++) {
    const UARTConfig& config = UART_CONFIGS[i];
    // 使用靜態陣列儲存名稱，避免指針失效
    snprintf(monitor_names_[i], sizeof(monitor_names_[i]), "HAL-UART%d-%s", (int)i, config.description);
    
    uart_monitors_[i] = monitor::createUARTMonitor(monitor_names_[i]);
    if (uart_monitors_[i]) {
      LOGI("✅ Created monitor for %s", config.description);
    } else {
      LOGE("❌ Failed to create monitor for %s", config.description);
    }
  }
  
  monitoring_initialized_ = true;
  LOGI("✅ HAL UART monitoring initialized for %d channels", UART_COUNT);
}

void shutdownMonitoring() {
  if (!monitoring_initialized_) return;
  
  LOGI("Shutting down HAL UART monitoring...");
  
  for (size_t i = 0; i < UART_COUNT; i++) {
    delete uart_monitors_[i];
    uart_monitors_[i] = nullptr;
  }
  
  monitoring_initialized_ = false;
  LOGI("HAL UART monitoring shutdown complete");
}

void recordUARTTransmit(UARTChannel channel, uint32_t bytes) {
  size_t index = idx(channel);  // ✅ 使用正確的索引映射
  if (index < UART_COUNT && uart_monitors_[index] && monitoring_initialized_) {
    uart_monitors_[index]->recordBytes(bytes);     // ✅ 只記 bytes
    // ❌ 不要 recordPackets / recordOperations - 這些由 Parser 完包時記錄
  }
}

void recordUARTReceive(UARTChannel channel, uint32_t bytes) {
  size_t index = idx(channel);  // ✅ 使用正確的索引映射
  if (index < UART_COUNT && uart_monitors_[index] && monitoring_initialized_) {
    uart_monitors_[index]->recordBytes(bytes);     // ✅ 只記 bytes
    // ❌ 不要 recordPackets / recordOperations - 這些由 Parser 完包時記錄
  }
}

void updateUARTMonitoring() {
  if (!monitoring_initialized_) return;
  
  // 只更新監控器的內部統計，不觸發任何輸出
  for (size_t i = 0; i < UART_COUNT; i++) {
    if (uart_monitors_[i]) {
      uart_monitors_[i]->shouldReport(); // 僅更新內部區間統計
    }
  }
}

void generateUARTMonitoringReport() {
  if (!monitoring_initialized_) {
    LOGW("HAL monitoring not initialized");
    return;
  }
  
  // IMU (XSENS) 狀態報告 - 符合文檔格式
  if (uart_monitors_[2]) { // IMU channel
    monitor::DataFlowStats stats = uart_monitors_[2]->getStats();
    uart_monitors_[2]->shouldReport();
    
    LOGD("--- XSENS 狀態報告 (5秒區間) ---");
    LOGD("區間封包: %lu 個", (uint32_t)stats.total_packets);
    LOGD("區間位元組: %lu bytes", (uint32_t)stats.total_bytes);  
    LOGD("封包頻率: %.1f Hz", stats.packet_rate_hz);
    LOGD("區間速率: %.2f bytes/sec", stats.byte_rate_Bps);
    LOGD("累積總計: %lu 封包, %lu bytes", 
         (uint32_t)stats.total_packets, (uint32_t)stats.total_bytes);
    LOGD("IMU 緩衝區: %u bytes", (unsigned int)gIMU_Interface.available());
  }
  
  // GNSS 狀態報告 - 符合文檔格式  
  if (uart_monitors_[1]) { // GNSS channel
    monitor::DataFlowStats stats = uart_monitors_[1]->getStats();
    uart_monitors_[1]->shouldReport();
    
    LOGD("--- GNSS 狀態報告 (5秒區間) ---");
    LOGD("區間句子: %lu 句", (uint32_t)stats.total_packets);
    LOGD("區間位元組: %lu bytes", (uint32_t)stats.total_bytes);
    LOGD("句子頻率: %.1f sentences/sec", stats.packet_rate_hz);
    LOGD("區間速率: %.2f bytes/sec", stats.byte_rate_Bps);
    LOGD("累積總計: %lu 句, %lu bytes",
         (uint32_t)stats.total_packets, (uint32_t)stats.total_bytes);
    LOGD("GNSS UART: %s", isUARTReady(UARTChannel::GNSS) ? "準備就緒" : "未就緒");
    LOGD("GNSS 緩衝區: %u bytes", (unsigned int)gGNSS_Interface.available());
  }
  
  // 統合 HAL UART 通道監控報告
  LOGI("🔌 ===== HAL UART 通道監控報告 =====");
  
  for (size_t i = 0; i < UART_COUNT; i++) {
    if (uart_monitors_[i]) {
      const char* channel_name = UART_CONFIGS[i].description;
      
      // 確保統計數據是最新的
      uart_monitors_[i]->shouldReport();
      monitor::DataFlowStats stats = uart_monitors_[i]->getStats();
      
      LOGI("  📡 [%d] %s:", (int)i, channel_name);
      LOGI("    📊 字節頻率: %.2f Bytes/sec", stats.byte_rate_Bps);
      LOGI("    📦 封包頻率: %.2f Hz", stats.packet_rate_hz);
      LOGI("    ⚙️ 操作頻率: %.2f Hz", stats.operation_rate_hz);
      
      // Arduino 兼容的 64 位整數格式化
      uint32_t bytes_high = (uint32_t)(stats.total_bytes >> 32);
      uint32_t bytes_low = (uint32_t)(stats.total_bytes & 0xFFFFFFFF);
      uint32_t packets_high = (uint32_t)(stats.total_packets >> 32);
      uint32_t packets_low = (uint32_t)(stats.total_packets & 0xFFFFFFFF);
      
      if (bytes_high > 0 || packets_high > 0) {
        LOGI("    📈 累積: %lu%08lu bytes, %lu%08lu packets", bytes_high, bytes_low, packets_high, packets_low);
      } else {
        LOGI("    📈 累積: %lu bytes, %lu packets", bytes_low, packets_low);
      }
      
      LOGI("    🔄 狀態: %s", stats.is_active ? "Active" : "Inactive");
    }
  }
  
  LOGI("=====================================");
}

void resetUARTMonitoringStats() {
  if (!monitoring_initialized_) return;
  
  LOGI("Resetting HAL UART monitoring statistics...");
  for (size_t i = 0; i < UART_COUNT; i++) {
    if (uart_monitors_[i]) {
      uart_monitors_[i]->resetStats();
    }
  }
  LOGI("HAL UART monitoring statistics reset complete");
}

monitor::DataFlowStats getUARTStats(UARTChannel channel) {
  monitor::DataFlowStats empty_stats = {};
  size_t index = idx(channel);  // ✅ 使用正確的索引映射
  
  if (index < UART_COUNT && uart_monitors_[index] && monitoring_initialized_) {
    return uart_monitors_[index]->getStats();
  }
  
  return empty_stats;
}

bool isMonitoringInitialized() {
  return monitoring_initialized_;
}

// ================== 硬體資料讀取功能 ==================

namespace {
  // 定義回調函數類型
  using ByteSink = void(*)(const uint8_t* data, size_t len, void* user);
  
  // 回調結構
  struct Sink { 
    ByteSink fn = nullptr; 
    void* user = nullptr; 
  };
  
  // 靜態回調容器
  static Sink imu_sink, gnss_sink;
  
} // namespace anonymous

// ==================== 封包偵測輔助函數 ==================

// MTI 封包偵測狀態
static struct {
  uint8_t state = 0;  // 0=等待0xFA, 1=等待0xFF, 2=讀取長度, 3=讀取數據
  uint8_t packet_length = 0;
  uint8_t bytes_read = 0;
  uint32_t total_bytes = 0;  // 當前封包的總字節數
} mti_detector;

// NMEA 句子偵測狀態
static struct {
  char line_buffer[128];
  uint8_t line_pos = 0;
  bool in_sentence = false;
} nmea_detector;

// MTI 封包偵測函數 (移到 hal 命名空間)
void detectAndRecordMTIPackets(const uint8_t* buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
      uint8_t byte_data = buffer[i];
      
      switch (mti_detector.state) {
        case 0: // 等待 0xFA
          if (byte_data == 0xFA) {
            mti_detector.state = 1;
            mti_detector.total_bytes = 1;
          }
          break;
          
        case 1: // 等待 0xFF
          if (byte_data == 0xFF) {
            mti_detector.state = 2;
            mti_detector.total_bytes = 2;
          } else {
            mti_detector.state = 0; // 重置
          }
          break;
          
        case 2: // 讀取封包長度 (跳過 BID)
          mti_detector.state = 3;
          mti_detector.total_bytes = 3;
          break;
          
        case 3: // 讀取長度字節
          mti_detector.packet_length = byte_data;
          mti_detector.bytes_read = 0;
          mti_detector.total_bytes = 4;
          mti_detector.state = (byte_data > 0) ? 4 : 5; // 有數據就讀數據，沒數據就等校驗
          break;
          
        case 4: // 讀取數據字節
          mti_detector.bytes_read++;
          mti_detector.total_bytes++;
          if (mti_detector.bytes_read >= mti_detector.packet_length) {
            mti_detector.state = 5; // 等校驗碼
          }
          break;
          
        case 5: // 讀取校驗碼，封包完成
          mti_detector.total_bytes++;
          // 記錄完整的 MTI 封包
          GMINS_RECORD_MTI(mti_detector.total_bytes);
          // 重置狀態
          mti_detector.state = 0;
          mti_detector.total_bytes = 0;
          break;
      }
    }
}
  
// NMEA 句子偵測函數 (移到 hal 命名空間)
void detectAndRecordNMEAPackets(const uint8_t* buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
      char c = (char)buffer[i];
      
      if (c == '$' && !nmea_detector.in_sentence) {
        // 開始新句子
        nmea_detector.in_sentence = true;
        nmea_detector.line_pos = 0;
        nmea_detector.line_buffer[nmea_detector.line_pos++] = c;
      }
      else if (nmea_detector.in_sentence && nmea_detector.line_pos < sizeof(nmea_detector.line_buffer) - 1) {
        nmea_detector.line_buffer[nmea_detector.line_pos++] = c;
        
        if (c == '\n' || c == '\r') {
          // 句子結束
          nmea_detector.line_buffer[nmea_detector.line_pos] = '\0';
          
          // 分析句子類型並記錄
          if (strstr(nmea_detector.line_buffer, "GNGGA") != nullptr) {
            GMINS_RECORD_GNSS_GNGGA(nmea_detector.line_pos);
          }
          else if (strstr(nmea_detector.line_buffer, "GNRMC") != nullptr) {
            GMINS_RECORD_GNSS_GNRMC(nmea_detector.line_pos);
          }
          else if (strstr(nmea_detector.line_buffer, "PLSHD") != nullptr) {
            GMINS_RECORD_GNSS_PLSHD(nmea_detector.line_pos);
          }
          // 可以在這裡添加更多 NMEA 句子類型
          
          // 重置狀態
          nmea_detector.in_sentence = false;
          nmea_detector.line_pos = 0;
        }
      }
      else if (nmea_detector.line_pos >= sizeof(nmea_detector.line_buffer) - 1) {
        // 緩衝區滿，重置狀態
        nmea_detector.in_sentence = false;
        nmea_detector.line_pos = 0;
      }
    }
}

void attachIMUSink(ByteSink fn, void* user) { 
  imu_sink.fn = fn;
  imu_sink.user = user;
  LOGI("✅ IMU 資料接收器已註冊");
}

void attachGNSSSink(ByteSink fn, void* user) { 
  gnss_sink.fn = fn;
  gnss_sink.user = user;
  LOGI("✅ GNSS 資料接收器已註冊");
}

// ================== MTI 原始資料記錄 ==================

// 可配置的 MTI 記錄封包數量（用於 shift 角度分析）
static uint8_t mti_record_limit = 30;  // 可在運行時調整

// 全域變數用於記錄 MTI 原始資料  
static uint8_t hal_mti_raw_count = 0;
static bool hal_mti_logging = true;

// GNSS 原始資料記錄變數
static uint8_t hal_gnss_raw_count = 0;
static bool hal_gnss_logging = true;
static uint8_t gnss_record_limit = 10;  // GNSS記錄較少

// TDD Flow Checker for MTI data processing in HAL layer (已升級為頻率計算版本)
static TDDFlowChecker mti_hal_checker("board_support:logMTIRawDataInHAL", "logMTIRawDataInHAL", 5000, true);  // MTI 原始數據處理頻率監控

// TDD Flow Checker for GNSS data processing in HAL layer
static TDDFlowChecker gnss_hal_checker("board_support:logGNSSRawDataInHAL", "logGNSSRawDataInHAL", 5000, true);  // GNSS 原始數據處理頻率監控

// XBUS 封包重組緩衝區
static uint8_t xbus_packet_buffer[256];
static size_t xbus_buffer_pos = 0;
static bool xbus_packet_started = false;

void logMTIRawDataInHAL(const uint8_t* data, size_t length) {
  mti_hal_checker.recordIn();  // TDD: 記錄 MTI HAL 層數據流入
  
  // 只記錄前 N 筆（可配置）
  if (!hal_mti_logging || hal_mti_raw_count >= mti_record_limit) {
    // mti_hal_checker.update();  // TDD: 暫停避免死機
    return;
  }
  
  /*// 格式化顯示原始位元組
  String hex_string = "";
  for (size_t i = 0; i < length && i < 32; i++) {  // 限制顯示前32字節
    if (hex_string.length() > 0) hex_string += " ";
    if (data[i] < 0x10) hex_string += "0";
    hex_string += String(data[i], HEX);
    hex_string.toUpperCase();
  }*/
  
  hal_mti_raw_count++;
  /*LOGI("📊 HAL-MTI 完整封包 #%d (%d bytes): %s", 
       hal_mti_raw_count, length, hex_string.c_str());*/
  
  /*// 檢查是否為 XBUS 格式並解析
  if (length >= 4 && data[0] == 0xFA && data[1] == 0xFF) {
    uint8_t mid = data[2];
    uint8_t len = data[3];
    LOGI("   🎯 XBUS 封包: MID=0x%02X, LEN=%d, 總長度=%d bytes", mid, len, length);
  }*/
  
  // 完成指定筆數後停止記錄
  if (hal_mti_raw_count >= mti_record_limit) {
    LOGI("✅ HAL 層已記錄 %d 筆 MTI 原始資料", mti_record_limit);
    hal_mti_logging = false;
  }
  
  // mti_hal_checker.update();  // TDD: 暫停避免死機
}

// GNSS資料記錄函數
void logGNSSRawDataInHAL(const uint8_t* data, size_t length) {
  gnss_hal_checker.recordIn();  // TDD: 記錄 GNSS HAL 層數據流入
  
  // 只記錄前 N 筆GNSS數據  
  if (!hal_gnss_logging || hal_gnss_raw_count >= gnss_record_limit) {
    // gnss_hal_checker.update();  // TDD: 暫停避免死機
    return;
  }
  
  // 格式化顯示原始NMEA數據
  String ascii_string = "";
  for (size_t i = 0; i < length && i < 64; i++) {  // NMEA可讀性更好
    char c = (char)data[i];
    if (c >= 32 && c <= 126) { // 可顯示字符
      ascii_string += c;
    } else if (c == '\r') {
      ascii_string += "\\r";
    } else if (c == '\n') {
      ascii_string += "\\n";
    } else {
      ascii_string += ".";
    }
  }
  
  hal_gnss_raw_count++;
  LOGI("📊 HAL-GNSS 數據片段 #%d (%d bytes): %s", 
       hal_gnss_raw_count, length, ascii_string.c_str());
  
  // 完成指定筆數後停止記錄
  if (hal_gnss_raw_count >= gnss_record_limit) {
    LOGI("✅ HAL 層已記錄 %d 筆 GNSS 原始資料", gnss_record_limit);
    hal_gnss_logging = false;
  }
  
  // gnss_hal_checker.update();  // TDD: 暫停避免死機
}

// 設定 MTI 記錄封包數量限制
void setMTIRecordLimit(uint8_t limit) {
  if (limit == 0) limit = 1;  // 至少記錄 1 筆
  mti_record_limit = limit;
  LOGI("📝 MTI 記錄限制已設定為 %d 筆", limit);
}

// 設定 GNSS 記錄封包數量限制
void setGNSSRecordLimit(uint8_t limit) {
  if (limit == 0) limit = 1;  // 至少記錄 1 筆
  gnss_record_limit = limit;
  LOGI("📝 GNSS 記錄限制已設定為 %d 筆", limit);
}

// XBUS 封包重組函數
void reassembleXbusPackets(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    
    // 偵測 XBUS 前導碼 0xFA 0xFF
    if (!xbus_packet_started) {
      if (byte == 0xFA) {
        xbus_packet_buffer[0] = byte;
        xbus_buffer_pos = 1;
        xbus_packet_started = true;
      }
    } else if (xbus_buffer_pos == 1) {
      if (byte == 0xFF) {
        xbus_packet_buffer[1] = byte;
        xbus_buffer_pos = 2;
      } else {
        // 不是有效的 XBUS 前導碼，重置
        xbus_packet_started = false;
        xbus_buffer_pos = 0;
        // 重新檢查這個字節是否是 0xFA
        if (byte == 0xFA) {
          xbus_packet_buffer[0] = byte;
          xbus_buffer_pos = 1;
          xbus_packet_started = true;
        }
      }
    } else {
      // 收集封包資料
      if (xbus_buffer_pos < sizeof(xbus_packet_buffer)) {
        xbus_packet_buffer[xbus_buffer_pos++] = byte;
        
        // 檢查是否收到完整封包
        // XBUS 格式: FA FF MID LEN [DATA...] CHK
        if (xbus_buffer_pos >= 4) { // 至少有 FA FF MID LEN
          uint8_t packet_length = xbus_packet_buffer[3]; // LEN 欄位
          size_t expected_total = 4 + packet_length + 1; // FA FF MID LEN + DATA + CHK
          
          if (xbus_buffer_pos >= expected_total) {
            // 完整封包接收完成，記錄它（保留 shift 分析功能）
            logMTIRawDataInHAL(xbus_packet_buffer, expected_total);
            
            // 🔧 修復：傳遞給上層處理（IngressManager → framer）
            if (imu_sink.fn) {
              imu_sink.fn(xbus_packet_buffer, expected_total, imu_sink.user);
              // TDD: 記錄成功傳遞給上層的數據流出（在 logMTIRawDataInHAL 中會記錄 IN）
              mti_hal_checker.recordOut();
            }
            
            // 重置緩衝區
            xbus_packet_started = false;
            xbus_buffer_pos = 0;
          }
        }
      } else {
        // 緩衝區溢出，重置
        xbus_packet_started = false;
        xbus_buffer_pos = 0;
      }
    }
  }
}

static inline void pumpPort(HardwareSerial& serial, UARTChannel channel, Sink sink) {
  uint8_t buffer[256];
  while (serial.available() > 0) {
    // 避免阻塞：只讀取目前可用的資料量
    size_t available = serial.available();
    size_t to_read = (available > sizeof(buffer)) ? sizeof(buffer) : available;
    size_t bytes_read = serial.readBytes(buffer, to_read);
    if (!bytes_read) break;
    
    // 🚨 停用所有累加操作，只保留核心數據流轉
    // recordUARTReceive(channel, bytes_read);
    // detectAndRecordMTIPackets(buffer, bytes_read);
    // detectAndRecordNMEAPackets(buffer, bytes_read);  
    // logGNSSRawDataInHAL(buffer, bytes_read);
    // gnss_hal_checker.recordOut();
    
    // 🚨 暫時停用XBUS重組，測試是否也有累加問題
    // if (channel == UARTChannel::IMU) {
    //   reassembleXbusPackets(buffer, bytes_read);
    // }
    
    // 推送給上層（核心功能）
    if (sink.fn) {
      sink.fn(buffer, bytes_read, sink.user);
    }
  }
}

void pollOnce() {
  // 唯一讀者：把硬體資料拉進來
  pumpPort(getUARTSerial(UARTChannel::IMU),  UARTChannel::IMU,  imu_sink);
  pumpPort(getUARTSerial(UARTChannel::GNSS), UARTChannel::GNSS, gnss_sink);
  
}

// ================== 感測器測試功能 ==================

void initializeXSENSTest() {
  LOGI("🧭 正在初始化 XSENS 感測器測試...");
  
  // 初始化全域頻率管理器
  if (!gmins::initializeGlobalFrequencyManager(5000)) {
    LOGE("❌ 無法初始化全域頻率管理器");
    return;
  }
  
  // 設定感測器頻率目標
  gmins::FrequencyManager* freq_mgr = gmins::getGlobalFrequencyManager();
  if (freq_mgr) {
    freq_mgr->setFrequencyTargets(
      100.0f,  // MTI 目標 100Hz
      6.0f,    // GNSS 目標 6Hz (GNGGA: 2Hz + GNRMC: 2Hz + PLSHD: 2Hz)  
      100.0f,   // MAVLink 目標 50Hz
      20.0f    // Custom 目標 20Hz
    );
    LOGI("✅ 感測器頻率目標已設定");
  }
  
  LOGI("✅ XSENS 感測器測試系統初始化完成");
}

void processAllSensorData() {
  static uint32_t last_process_time = 0;
  uint32_t current_time = millis();
  
  // 每 10ms 處理一次感測器數據
  if (current_time - last_process_time >= 10) {
    
    // 🚨 測試1: 只啟用硬體數據讀取
    pollOnce();
    
    // 🚨 測試2: 停用 UART 監控更新 - 確認是死機源
    // if (monitoring_initialized_) {
    //   updateUARTMonitoring();
    // }
    
    // 🚨 測試3: 停用頻率管理器更新 - 確認也是死機源
    // gmins::FrequencyManager* freq_mgr = gmins::getGlobalFrequencyManager();
    // if (freq_mgr) {
    //   freq_mgr->updateAll();
    // }
    
    last_process_time = current_time;
  }
}

void reportAllSensorStatus() {
  static uint32_t last_report_time = 0;
  uint32_t current_time = millis();
  
  // 每 5 秒報告一次狀態 (配合5秒區間統計)
  if (current_time - last_report_time >= 5000) {
    
    LOGI("🔄 ===== 感測器狀態報告 =====");
    
    // HAL UART 監控報告
    if (monitoring_initialized_) {
      generateUARTMonitoringReport();
    }
    
    // 頻率管理器報告
    gmins::FrequencyManager* freq_mgr = gmins::getGlobalFrequencyManager();
    if (freq_mgr) {
      freq_mgr->generateFrequencyReport();
    }
    
    // UART Interface 統計報告
    reportUARTInterfaceStats();
    
    LOGI("================================");
    
    last_report_time = current_time;
  }
}

HardwareSerial& getUARTSerial(UARTChannel channel) {
  // 恢復 312a6ea commit 的正確映射
  switch (channel) {
    case UARTChannel::DEBUG:   return Serial;   // 通道 0 - Serial
    case UARTChannel::GNSS:    return Serial4;  // 通道 1 - Serial4 ✅ (SERCOM3)
    case UARTChannel::IMU:     return Serial2;  // 通道 2 - Serial2 ✅ (SERCOM2)
    case UARTChannel::MAVLINK: return Serial1;  // 通道 3 - Serial1 ✅ (SERCOM5)
    case UARTChannel::AUX:     return Serial3;  // 通道 4 - Serial3 ✅ (SERCOM1)
    default:                   return Serial;   // fallback
  }
}

bool isUARTReady(UARTChannel channel) {
  return getUARTSerial(channel).availableForWrite() > 0;
}

bool resetUART(UARTChannel channel, uint32_t baud) {
  HardwareSerial& serial = getUARTSerial(channel);
  const UARTConfig& cfg  = UART_CONFIGS[(int)channel];

  serial.end();
  delay(100);
  serial.begin(baud ? baud : cfg.default_baud_rate);
  return true;
}

// ================== Feed-based UART Interface Getters ==================

/**
 * @brief 獲取 IMU 數據源接口（抽象介面）
 * @return IMU 數據源指針，供 Parser 使用
 */
IByteSource* getIMUSource() { 
  return &gIMU_Interface; 
}

/**
 * @brief 獲取 GNSS 數據源接口（抽象介面）
 * @return GNSS 數據源指針，供 Parser 使用
 */
IByteSource* getGNSSSource() { 
  return &gGNSS_Interface; 
}

/**
 * @brief 獲取 IMU UART 介面（用於命令發送）
 * @return IMU UART 介面指針，供 MTI 驅動程式使用
 */
UARTInterface* getIMUInterface() {
  return &gIMU_Interface;
}

/**
 * @brief 獲取 GNSS UART 介面（用於命令發送）
 * @return GNSS UART 介面指針
 */
UARTInterface* getGNSSInterface() {
  return &gGNSS_Interface;
}


} // namespace hal
