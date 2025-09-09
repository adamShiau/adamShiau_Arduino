# UART 配置說明

> **功能**: GMINS 系統多串列埠硬體配置和通訊參數設定  
> **適用系統**: GMINS v2.0  
> **版本**: v2.0  
> **最後更新**: 2025-08-08  

## 🔌 硬體連接概述

GMINS 系統繼承 MINSPixhawk 的硬體架構優勢，使用 Arduino MCU 的 SERCOM 功能，同時支援 4 路 UART 通訊，實現多設備並行資料處理。系統採用星型拓撲，Arduino 作為中央處理器，連接所有外部設備。

### 連接拓撲圖
```
                    ┌─────────────────┐
                    │   Arduino MCU   │
                    │  (GMINS 核心)   │
                    └─────────┬───────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        │                    │                    │
   ┌────▼────┐         ┌────▼────┐         ┌────▼────┐
   │ Serial1 │         │ Serial2 │         │ Serial3 │
   │460800bps│         │115200bps│         │115200bps│
   │SERCOM5  │         │SERCOM2  │         │SERCOM1  │
   └────┬────┘         └────┬────┘         └────┬────┘
        │                   │                   │
        │                   │                   │
   ┌────▼────┐         ┌────▼────┐         ┌────▼────┐
   │Pixhawk  │         │MTi-680  │         │MTi-680  │
   │ PX4 FC  │         │ XBUS    │         │ NMEA    │
   │(MAVLink)│         │ Input   │         │ Output  │
   └─────────┘         └─────────┘         └─────────┘
                                                 │
                                                 │
                                           ┌────▼────┐
                                           │ Serial4 │
                                           │115200bps│
                                           │SERCOM3  │
                                           └────┬────┘
                                                │
                                           ┌────▼────┐
                                           │LOCOSYS  │
                                           │GPS NMEA │
                                           │ Input   │
                                           └─────────┘
```

## 📊 UART 端口配置表

### 完整配置參數
| 串列埠 | SERCOM | 鮑率 | 資料位 | 停止位 | 校驗位 | 用途 | 連接設備 | 協定 |
|-------|--------|------|--------|--------|--------|------|----------|------|
| Serial1 | SERCOM5 | 460800 | 8 | 1 | None | MAVLink 通訊 | Pixhawk PX4 | MAVLink |
| Serial2 | SERCOM2 | 115200 | 8 | 1 | None | 感測器資料 | Xsens MTi-680 | XBUS |
| Serial3 | SERCOM1 | 115200 | 8 | 1 | None | NMEA 輸出 | MTi-680 GNSS | NMEA |
| Serial4 | SERCOM3 | 115200 | 8 | 1 | None | NMEA 輸入 | LOCOSYS GPS | NMEA |

## 📡 各串列埠詳細說明

### 1. Serial1 - Pixhawk MAVLink 通訊
```cpp
// GMINS v2.0 配置參數
串列埠: Serial1 (SERCOM5)
鮑率: 460800 bps
用途: 雙向 MAVLink 協定通訊
資料方向: GMINS Core ↔ Pixhawk PX4

// 主要功能
- 發送 ODOMETRY 訊息 (#331) - 100Hz
- 發送 GPS_RAW_INT 訊息 (#24) - 10Hz  
- 發送 GPS_INPUT 訊息 (#232) - 10Hz
- 發送 TIMESYNC 訊息 (#111) - 1Hz
- 接收 Pixhawk 系統狀態和參數

// 資料格式
協定: MAVLink v2.0
封包大小: 可變 (典型 30-50 bytes)
錯誤檢測: CRC-16 checksum
流控制: Communication Manager 統一管理
```

### 2. Serial2 - MTi-680 XBUS 感測器通訊
```cpp
// GMINS v2.0 配置參數
串列埠: Serial2 (SERCOM2)  
鮑率: 115200 bps
用途: 接收 MTi-680 感測器資料
資料方向: MTi-680 → GMINS Core (單向)

// 主要功能
- 接收四元數姿態資料 (100Hz)
- 接收角速度資料 (100Hz)
- 接收加速度資料 (100Hz)
- 接收 GPS 位置和速度 (1-10Hz)
- 接收溫度和氣壓資料

// 資料格式  
協定: XSENS XBUS (二進制)
封包大小: 可變 (典型 50-200 bytes)
錯誤檢測: Checksum
同步標記: 0xFA (XBUS preamble)
```

### 3. Serial3 - NMEA 輸出到 MTi-680
```cpp
// GMINS v2.0 配置參數
串列埠: Serial3 (SERCOM1)
鮑率: 115200 bps  
用途: 轉發 NMEA 資料給 MTi-680 GNSS 輔助
資料方向: GMINS Core → MTi-680 (單向)

// 主要功能
- 轉發 LOCOSYS GPS 的 NMEA 句子
- 提供 MTi-680 高精度 GNSS 輔助定位
- 支援 9 種 NMEA 句子類型轉發
- 即時資料流轉發 (無緩衝延遲)

// 支援的 NMEA 句子
$GPGGA - GPS 定位資料
$GPRMC - GPS 推薦最小資料  
$GPGST - GPS 偽距誤差統計
$GPGSA - GPS 衛星狀態
$GPVTG - GPS 速度資料
$GPZDA - GPS 日期時間
$GPHDT - GPS 真航向
$PLSHD - 雙天線航向 (專用)
$GPGSV - GPS 衛星可見資訊
```

### 4. Serial4 - LOCOSYS GPS NMEA 輸入
```cpp
// GMINS v2.0 配置參數
串列埠: Serial4 (SERCOM3)
鮑率: 115200 bps
用途: 接收 LOCOSYS GPS 的 NMEA 資料
資料方向: LOCOSYS GPS → GMINS Core (單向)

// 主要功能  
- 接收高精度 GPS 位置資料
- 接收 GPS 速度和航向資料
- 接收衛星狀態和精度資訊
- 接收雙天線 GNSS 航向資料 (如支援)

// LOCOSYS 特色功能
- 支援雙天線配置 ($PLSHD 句子)
- 高更新率 (最高 10Hz)
- 多星座支援 (GPS/GLONASS/BeiDou/Galileo)
- RTK 差分定位支援 (如配置)
```

## ⚡ GMINS v2.0 改進特性

### 1. 統一通訊管理
- **Communication Manager**: 統一管理所有 UART 通訊
- **智能緩衝**: 動態緩衝區管理，避免資料遺失
- **流量控制**: 自動流量控制和優先級管理
- **錯誤恢復**: 自動檢測和恢復通訊錯誤

### 2. 效能優化
```cpp
// 改進的緩衝區監控 (Communication Manager)
class UARTManager {
private:
    struct UARTChannel {
        HardwareSerial* serial;
        uint32_t baud_rate;
        uint16_t buffer_size;
        uint16_t buffer_threshold;
        unsigned long last_activity;
        bool is_active;
    };
    
public:
    void monitorAllChannels();
    void optimizeBufferUsage();
    bool detectCommunicationError();
    void recoverFromError(int channel);
};
```

### 3. 診斷增強
```cpp
// 通訊診斷功能
struct CommDiagnostics {
    uint32_t bytes_sent[4];
    uint32_t bytes_received[4];
    uint32_t packets_sent[4];
    uint32_t packets_received[4];
    uint32_t errors_detected[4];
    uint32_t recoveries_performed[4];
    float throughput_mbps[4];
    float error_rate_percent[4];
};

void generateCommReport();
void logCommStats();
bool validateCommHealth();
```

### 4. 自動配置
- **自動鮑率檢測**: 啟動時自動檢測最佳鮑率
- **設備自動識別**: 自動識別連接的設備類型
- **智能重新連接**: 設備斷線時自動重新連接
- **配置持久化**: 成功配置自動儲存

## 🔧 故障排除和診斷

### 診斷命令 (通過 USB Serial)
```cpp
// GMINS v2.0 診斷命令
"comm_status"    - 顯示所有通訊通道狀態
"comm_stats"     - 顯示通訊統計資料
"comm_test"      - 執行通訊連接測試
"comm_reset"     - 重置所有通訊通道
"comm_config"    - 顯示通訊配置
"comm_optimize"  - 執行通訊優化
```

### 自動故障恢復
```cpp
// 自動恢復機制
enum CommError {
    NO_ERROR = 0,
    TIMEOUT_ERROR,
    BUFFER_OVERFLOW,
    CHECKSUM_ERROR,
    DEVICE_DISCONNECTED,
    BAUD_RATE_MISMATCH
};

class CommRecovery {
public:
    bool handleError(int channel, CommError error);
    void resetChannel(int channel);
    bool reconnectDevice(int channel);
    void fallbackToSafeMode(int channel);
};
```

## 🔗 相關文件連結

### 硬體設計
- [[2.硬體架構/硬體架構整合文件]] - 完整硬體架構說明
- [[4.源代碼架構/通訊模組設計]] - Communication Manager 詳細設計

### 軟體實作
- [[3.軟體架構重新設計/新軟體架構設計]] - 整體軟體架構
- [[5.實作指導/開發環境設置]] - 開發環境配置

### 測試和診斷
- [[5.實作指導/測試策略]] - 通訊測試方法
- [[4.源代碼架構/核心模組設計]] - 各模組介面設計

---

**最後更新**: 2025-08-08  
**適用版本**: GMINS v2.0  
**維護者**: GMINS 開發團隊  
**硬體相容性**: 完全兼容 MINSPixhawk v1.0 硬體