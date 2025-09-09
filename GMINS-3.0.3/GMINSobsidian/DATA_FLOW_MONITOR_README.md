# 數據流量監控模組 (Data Flow Monitor)

## 概述

這是一個通用的數據流量與頻率監控模組，專為嵌入式系統設計，特別適用於UART通訊、感測器數據處理和通訊協議監控。

## 功能特色

- ✅ **多維度統計**: 字節數、封包數、操作次數
- ✅ **區間統計**: 可設定報告間隔 (預設5秒)
- ✅ **頻率計算**: 自動計算 bytes/sec 和 Hz
- ✅ **多通道管理**: 支援同時監控多個數據源
- ✅ **低資源消耗**: 針對Arduino/嵌入式平台優化
- ✅ **靈活配置**: 可啟用/停用、重置、自訂間隔
- ✅ **詳細日誌**: 整合日誌系統，支援不同等級

## 文件結構

```
src/util/
├── data_flow_monitor.h      # 標頭檔案 (介面定義)
├── data_flow_monitor.cpp    # 實作檔案
examples/
├── data_flow_monitor_example.cpp  # 使用範例
docs/
├── DATA_FLOW_MONITOR_README.md    # 此文件
```

## 核心類別

### 1. `DataFlowMonitor` - 單一通道監控器

```cpp
// 創建監控器
monitor::DataFlowMonitor* uart_monitor = monitor::createUARTMonitor("GNSS");

// 記錄數據
uart_monitor->recordBytes(59);     // 記錄字節數
uart_monitor->recordPackets(1);    // 記錄封包數
uart_monitor->recordOperations(1); // 記錄操作次數

// 檢查並生成報告
if (uart_monitor->shouldReport()) {
    uart_monitor->generateReport();
}
```

### 2. `MultiChannelMonitor` - 多通道管理器

```cpp
// 創建管理器
monitor::MultiChannelMonitor channel_manager;

// 註冊多個監控器
channel_manager.registerMonitor(uart_monitor);
channel_manager.registerMonitor(imu_monitor);

// 統一管理
channel_manager.updateAll();           // 更新所有通道
channel_manager.generateSummaryReport(); // 生成摘要報告
```

## 工廠函數

提供三種預設的監控器類型：

```cpp
// UART通訊監控
monitor::DataFlowMonitor* uart_mon = monitor::createUARTMonitor("GNSS");

// 感測器數據監控  
monitor::DataFlowMonitor* sensor_mon = monitor::createSensorMonitor("XSENS");

// 通訊協議監控
monitor::DataFlowMonitor* proto_mon = monitor::createProtocolMonitor("MAVLink");
```

## 典型輸出範例

```
=== UART-GNSS 數據流量報告 (5.0秒區間) ===
📥 區間字節: 1587 bytes | 頻率: 317.40 bytes/sec
📦 區間封包: 135 個 | 頻率: 27.00 Hz
⚙️ 區間操作: 135 次 | 頻率: 27.00 Hz
📊 累積總計: 3174 bytes, 270 packets, 270 ops
⏱️ 監控時間: 10 秒

=== SENSOR-XSENS 數據流量報告 (5.0秒區間) ===
📥 區間字節: 10300 bytes | 頻率: 2060.00 bytes/sec
📦 區間封包: 500 個 | 頻率: 100.00 Hz
⚙️ 區間操作: 500 次 | 頻率: 100.00 Hz
📊 累積總計: 20600 bytes, 1000 packets, 1000 ops
⏱️ 監控時間: 10 秒
```

## 整合到現有代碼

### 在HAL層整合

```cpp
#include "util/data_flow_monitor.h"

// 全域監控器
static monitor::DataFlowMonitor* gnss_monitor = nullptr;
static monitor::DataFlowMonitor* imu_monitor = nullptr;

void initMonitoring() {
    gnss_monitor = monitor::createUARTMonitor("GNSS");
    imu_monitor = monitor::createSensorMonitor("IMU");
}

void processGNSSData() {
    HardwareSerial& gnss_serial = getUARTSerial(UARTChannel::GNSS);
    
    while (gnss_serial.available()) {
        char incoming_char = gnss_serial.read();
        
        // 整合監控
        gnss_monitor->recordBytes(1);
        gnss_monitor->recordOperations(1);
        
        // 原有處理邏輯
        if (nmea_parser.processIncomingChar(incoming_char)) {
            gnss_monitor->recordPackets(1);
        }
    }
    
    // 報告檢查
    if (gnss_monitor->shouldReport()) {
        gnss_monitor->generateReport();
    }
}
```

### 在UART Interface層整合

```cpp
void ArduinoUARTInterface::updateReadStats(size_t bytes_read) {
    stats_.total_bytes_read += bytes_read;
    stats_.total_read_calls++;
    
    // 整合監控模組
    if (flow_monitor_) {
        flow_monitor_->recordBytes(bytes_read);
        flow_monitor_->recordOperations(1);
    }
}
```

## 配置選項

### 報告間隔

```cpp
monitor->setReportInterval(3000); // 改為3秒間隔
```

### 啟用/停用

```cpp
monitor->setEnabled(false); // 暫停監控
monitor->setEnabled(true);  // 恢復監控
```

### 重置統計

```cpp
monitor->resetStats(); // 清除所有累積數據
```

## 日誌控制

在 `data_flow_monitor.cpp` 開頭修改日誌等級：

```cpp
// 開發調試：顯示所有訊息
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_DEBUG | LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR)

// 正常運行：僅INFO以上
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR)

// 生產環境：僅警告以上
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_WARN | LOG_BIT_ERROR)
```

## 效能考量

- **記憶體使用**: 每個監控器約 100 bytes
- **CPU負擔**: 每次記錄操作 < 1μs
- **建議通道數**: ≤ 8個 (受限於 MultiChannelMonitor)

## 故障排除

### 1. 頻率不匹配

```cpp
// 確保在正確的位置記錄數據
uart_monitor->recordBytes(actual_bytes_read);  // 不是buffer大小
uart_monitor->recordPackets(1);                // 每個完整封包/句子
```

### 2. 沒有報告輸出

```cpp
// 檢查監控器是否啟用
if (!monitor->isEnabled()) {
    monitor->setEnabled(true);
}

// 手動觸發報告
if (monitor->shouldReport()) {
    monitor->generateReport();
}
```

### 3. 統計數據異常

```cpp
// 重置並重新開始
monitor->resetStats();
```

## 未來擴展

- [ ] 支援統計數據持久化
- [ ] 網路傳輸統計
- [ ] 自動異常檢測
- [ ] 圖形化監控介面
- [ ] 統計數據導出功能

## 重要改進 (基於專家回饋)

### ✅ 實際區間頻率計算
使用 `(current_time - last_report_time_) / 1000.0f` 而非固定間隔，確保頻率計算準確性。

### ✅ 統計唯一來源原則
**關鍵**：只在 `UARTHub::push()` 中調用 `recordBytes()`，避免 Support/Parser 重複統計。

### ✅ ISR安全性警告  
`record*()` 函數包含LOG輸出，**不適合在中斷服務程序中調用**。

### ✅ 64位累積計數器
避免長時間運行時的溢位問題，累積統計使用 `uint64_t`。

### ✅ LOG等級保護
使用 `#ifndef` 保護避免被上層重定義覆蓋。

## 監控模組使用準則

```md
### 監控模組使用準則
- **統計唯一來源**：只在 `UARTHub::push()` 呼叫 `recordBytes(len)`；Parser 組包完成呼叫 `recordPackets(1)`。
- **頻率分母**：用「實際區間秒數」(now - last_report_time_) 計算。
- **ISR 安全**：ISR 不呼叫 `record*()`；主循環統一記帳。
- **所有權**：`create*Monitor()` 配對 `delete` 或改用 `unique_ptr`。
- **通用化**：必要時把累積計數升級為 `uint64_t`。
```

## 版本歷史

- **v1.1** (2025-08-13): 專家回饋改進版本
  - 實際區間頻率計算
  - 64位累積計數器
  - ISR安全性警告
  - LOG等級保護
  - UART Hub整合範例
- **v1.0** (2025-08-13): 初始版本，基礎監控功能
  - 支援字節、封包、操作統計
  - 多通道管理
  - Arduino平台優化

---

**作者**: GMINS Team  
**聯絡**: 請透過項目GitHub Issues報告問題
● Update(GMINS.ino)
  ⎿  Updated GMINS.ino with 5 additions and 7 removals
       203      mti_driver.sendCommandSimple(0xD4, sample_period, sizeof(sample_period));  // CMD_SET_SAMPLE_PERIOD
       204      delay(100);
       205
       206 -    // 3. 設定輸出配置 (基本 XBUS 格式)
       207 -    LOGI("   🔧 CMD_OUTPUT_CONFIG (0xC0) -> 包含計數器、時間戳、四元數、加速度、角速度");
       206 +    // 3. 設定輸出配置 (簡化版本 - 只保留核心資料)
       207 +    LOGI("   🔧 CMD_OUTPUT_CONFIG (0xC0) -> 只包含計數器和四元數");
       208      uint8_t output_config[] = {
       209 -      0x10, 0x20, 0x00, 0x04,  // Package Counter
       210 -      0x10, 0x60, 0x00, 0x04,  // SampleTimeFine
       211 -      0x20, 0x30, 0x00, 0x10,  // Quaternion
       212 -      0x40, 0x20, 0x00, 0x0C,  // Acceleration
       213 -      0x80, 0x20, 0x00, 0x0C   // Angular Velocity
       209 +      0x10, 0x20, 0x00, 0x04,  // Package Counter (4 bytes)
       210 +      0x20, 0x30, 0x00, 0x10   // Quaternion (16 bytes)
       211 +      // 移除: SampleTimeFine, Acceleration, Angular Velocity 以提高頻率
       212      };
       213      mti_driver.sendCommandSimple(0xC0, output_config, sizeof(output_config));  // CMD_OUTPUT_CONFIG