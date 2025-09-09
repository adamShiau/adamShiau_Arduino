# GMINS Hardware Abstraction Layer (HAL) 硬體抽象層

> **文檔版本**: 1.1  
> **最後更新**: 2025-08-12  
> **專案**: GMINS 導航系統  
> **描述**: GMINS 硬體抽象層架構與使用說明

---

## 📋 目錄

1. [系統概述](#系統概述)
2. [硬體配置](#硬體配置)
3. [架構設計](#架構設計)
4. [LOG系統設定](#LOG系統設定)
5. [API接口說明](#API接口說明)
6. [除錯診斷工具](#除錯診斷工具)
7. [使用範例](#使用範例)
8. [效能參考數據](#效能參考數據)

---

## 🎯 系統概述

GMINS HAL層提供統一的硬體介面抽象，
主要負責串口 (UART) 管理與感測器通訊協調。

### 核心功能
- **UART串口管理**: 5組串口統一配置
- **感測器介面**: XSENS IMU 與 LOCOSYS GPS 資料處理
- **MAVLink輸出**: Pixhawk PX4 導航系統介面
- **診斷工具**: 系統除錯與監控功能
- **LOG系統**: 分級日誌與除錯資訊輸出

---

## 🔌 硬體配置

### UART串口配置

| 通道名稱    | UARTChannel  | 實體串口    | SERCOM  | 腳位 (RX/TX) | 波特率    | 緩衝區  | 功能描述          |
| ------- | ------------ | ------- | ------- | ---------- | ------ | ---- | ------------- |
| DEBUG   | `DEBUG(0)`   | Serial  | USB     | USB        | 115200 | 512  | USB除錯輸出       |
| GNSS    | `GNSS(1)`    | Serial4 | SERCOM3 | 10/9       | 115200 | 256  | LOCOSYS GPS輸入 |
| IMU     | `IMU(2)`     | Serial2 | SERCOM2 | 25/24      | 115200 | 1024 | Xsens IMU輸入   |
| MAVLINK | `MAVLINK(3)` | Serial1 | SERCOM5 | PB23/PB22  | 460800 | 1024 | Pixhawk PX4輸出 |
| AUX     | `AUX(4)`     | Serial3 | SERCOM1 | 13/8       | 115200 | 512  | NMEA格式輸出      |

### 硬體設計原理
```cpp
// 基於原始 myUART.h 的 SERCOM 配置
// SERCOM0=I2C, SERCOM1=Serial3, SERCOM2=Serial2, 
// SERCOM3=Serial4, SERCOM4=SPI, SERCOM5=Serial1
```

### 通訊特性
- **高速通道**: Serial1 (460800) 專用於 MAVLink 高頻輸出
- **標準通道**: Serial2/3/4 (115200) 用於感測器除錯與格式輸出
- **緩衝區管理**: 依據通訊頻率分配緩衝區大小 (256-1024 bytes)

---

## 🏗️ 架構設計

### 核心結構定義

```cpp
namespace hal {
    // 串口通道枚舉
    enum class UARTChannel {
        DEBUG = 0,    // USB除錯
        GNSS = 1,     // GPS資料接收  
        IMU = 2,      // IMU感測器
        MAVLINK = 3,  // MAVLink輸出
        AUX = 4       // 輔助輸出
    };
    
    // UART配置結構
    struct UARTConfig {
        uint32_t default_baud_rate;
        uint8_t rx_pin, tx_pin;
        bool enabled;
        uint16_t buffer_size;
        const char* description;
    };
}
```

### 設計特點
- **通道抽象**: 透過 `getUARTSerial(channel)` 管理
- **配置驅動**: 硬體配置表驅動初始化
- **統計監控**: 即時資料流統計與監控
- **除錯介面**: 系統除錯指令與資訊輸出

---

## 📊 LOG系統設定

### LOG等級配置

HAL層支援分級LOG輸出，可依據除錯需求動態調整：

```cpp
// 🔧 開發調試模式 - 顯示所有詳細資訊
#define LOG_LEVEL_LOCAL LOG_DEBUG
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_DEBUG | LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)

// ⚙️ 正常運行模式 - 標準操作資訊  
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)

// ⚠️ 生產環境模式 - 僅警告與錯誤
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)

// 🔇 完全靜音模式
#define LOG_LEVEL_MASK_LOCAL 0
```

### 監控輸出格式

HAL層採用 **5秒區間統計** 模式進行資料監控：

#### XSENS IMU 監控輸出
```
[D][HAL] --- XSENS 狀態報告 (5秒區間) ---

[D][HAL] 區間封包: 500 個

[D][HAL] 區間位元組: 51504 bytes

[D][HAL] 封包頻率: 100.0 Hz

[D][HAL] 區間速率: 10300.80 bytes/sec

[D][HAL] 累積總計: 5999 封包, 618017 bytes

[D][HAL] IMU 緩衝區: 17 bytes
```

#### GNSS GPS 監控輸出 (重點NMEA解析 - 5秒區間統計)
```
[D][HAL] --- GNSS 狀態報告 (5秒區間) ---

[D][HAL] 區間句子: 135 句

[D][HAL] 區間位元組: 7934 bytes

[D][HAL] 句子頻率: 27.0 sentences/sec

[D][HAL] 區間速率: 1586.80 bytes/sec

[D][HAL] === NMEA 5秒區間分析 ===

[D][HAL] PLSHD: 10 句 (7.4%) | 頻率: 2.0/sec

[D][HAL] GNGGA: 10 句 (7.4%) | 頻率: 2.0/sec

[D][HAL] GNRMC: 10 句 (7.4%) | 頻率: 2.0/sec

[D][HAL] Others: 105 句 (77.8%) | 頻率: 21.0/sec

[D][HAL] 累積總計: 1752 句, 101235 bytes

[D][HAL] GNSS UART: 準備就緒

[D][HAL] GNSS 緩衝區: 0 bytes
```

#### NMEA重點句子即時解析輸出
```
[D][HAL] NMEA-GNGGA [128]: $GNGGA,013428.000,2503.8212800,N,12116.8426100,E,2,35,0.87,63.98,M,14.77,M,,*4C

[D][HAL] NMEA-GNRMC [128]: $GNRMC,013428.000,A,2503.8212800,N,12116.8426100,E,0.02,0.00,130825,,,D,V*05

[D][HAL] NMEA-PLSHD [128]: $PLSHD,1,22,21,0.423,239.024,3.787*5F

[D][HAL] NMEA-GNGGA [129]: $GNGGA,013428.500,2503.8212700,N,12116.8426100,E,2,35,0.87,63.99,M,14.77,M,,*47

[D][HAL] NMEA-GNRMC [129]: $GNRMC,013428.500,A,2503.8212700,N,12116.8426100,E,0.03,0.00,130825,,,D,V*0E

[D][HAL] NMEA-PLSHD [129]: $PLSHD,1,22,21,0.423,242.099,5.480*57

[D][HAL] NMEA-GNGGA [130]: $GNGGA,013429.000,2503.8212800,N,12116.8426100,E,2,35,0.87,63.98,M,14.77,M,,*4D

[D][HAL] NMEA-GNRMC [130]: $GNRMC,013429.000,A,2503.8212800,N,12116.8426100,E,0.02,0.00,130825,,,D,V*04

[D][HAL] NMEA-PLSHD [130]: $PLSHD,1,22,21,0.418,238.792,2.323*57
```

#### 句子類型說明
- **PLSHD**: 專用格式句子 - 顯示完整內容與計數
- **GNGGA**: GNSS 定位資料 - 顯示完整內容與計數  
- **GNRMC**: 建議最小資料集 - 顯示完整內容與計數
- **Others**: GSA/GSV/VTG/GLL等其他類型 - 靜默計數，不顯示個別內容

### 統計實作原理

#### NMEA 5秒區間統計實作
```cpp
// NMEA 5秒區間統計變數
static NMEASentenceParser::NMEAStats last_nmea_snapshot;

// 計算5秒區間內的NMEA句子增量
auto current_nmea_stats = nmea_parser.getStatistics();
uint32_t interval_plshd = current_nmea_stats.plshd_count - last_nmea_snapshot.plshd_count;
uint32_t interval_gngga = current_nmea_stats.gngga_count - last_nmea_snapshot.gngga_count;
uint32_t interval_gnrmc = current_nmea_stats.gnrmc_count - last_nmea_snapshot.gnrmc_count;
uint32_t interval_others = current_nmea_stats.others_count - last_nmea_snapshot.others_count;

// 更新快照
last_nmea_snapshot = current_nmea_stats;
```

#### NMEA句子解析邏輯
```cpp
// CRLF處理 - 避免重複解析
if (incoming_char == '\n') {
    if (buffer_pos > 5 && !sentence_ready) {
        sentence_buffer[buffer_pos] = '\0';
        sentence_ready = true;
        parseSentenceType();
        return true;
    }
}
// 跳過 \r 字符，避免 \r\n 雙重觸發
if (incoming_char == '\r') return false;
```

---

## 🛠️ API接口說明

### 初始化接口

```cpp
// 硬體初始化
void initPins();          // GPIO初始化
void initUarts();         // UART初始化
void initPeripherals();   // 統合初始化
```

### UART管理接口

```cpp
// 取得UART串口物件
HardwareSerial& getUARTSerial(UARTChannel channel);

// 狀態檢查
bool isUARTReady(UARTChannel channel);

// 錯誤處理
void handleUARTError(UARTChannel channel, const char* error_msg);
void resetUART(UARTChannel channel);
```

### 感測器處理接口

```cpp
// XSENS IMU處理
void processXSENSData();         // 處理IMU資料流
void reportXSENSStatus();        // 報告IMU狀態
void resetXSENSStatistics();     // 重置統計數據
XSENSStatistics getXSENSStatistics(); // 取得統計資料

// GNSS GPS處理  
void processGNSSData();          // 處理GPS資料流
void reportGNSSStatus();         // 報告GPS狀態
void resetGNSSStatistics();      // 重置統計數據

// 統合感測器處理
void processAllSensorData();     // 處理所有感測器
void reportAllSensorStatus();    // 報告所有狀態
void resetAllStatistics();       // 重置所有統計數據
```

### 診斷接口

```cpp
// 狀態診斷
void printUARTStatus();                    // 顯示UART狀態
void testUARTLoopback(UARTChannel channel); // 迴路測試

// 除錯指令
void handleDebugCommand(const String& cmd); // 處理除錯指令
void printHelpMessage();                   // 顯示說明
```

---

## 🔍 除錯診斷工具

### 除錯指令系統

HAL層提供互動式除錯指令系統：

| 指令 | 功能描述 | 輸出內容 |
|------|----------|----------|
| `status` | 顯示所有UART狀態 | 波特率、就緒狀態、緩衝區 |
| `reset` | 重置所有統計數據 | 清除歷史統計資料 |  
| `test` | 執行UART迴路測試 | 測試IMU與MAVLink串口 |
| `xsens` | 顯示XSENS狀態 | 5秒區間IMU統計報告 |
| `gnss` | 顯示GNSS狀態 | 5秒區間GPS統計報告與NMEA分析 |
| `nmea` | 顯示NMEA句子統計 | 詳細NMEA句子類型分布與計數 |
| `sensors` | 顯示所有感測器狀態 | 統合感測器報告 |
| `help` | 顯示說明 | 可用指令列表 |

### NMEA句子解析器 (重點類型解析)

```cpp
class NMEASentenceParser {
    // 重點NMEA句子類型檢測與5秒區間統計
    // 支援: PLSHD, GNGGA, GNRMC + Others歸類
    // 顯示完整句子內容 (僅限重點類型)
    // 120字符緩衝區防止溢出
    // CRLF處理避免重複解析
    // 5秒區間頻率與百分比分析
};
```

#### 重點NMEA句子類型 (完整內容顯示)
- **PLSHD** - 專用格式句子 (Priority Display)
- **GNGGA** - GNSS 全球定位系統定位資料 (Priority Display)  
- **GNRMC** - GNSS 建議最小資料集 (Priority Display)

#### 其他NMEA句子類型 (靜默統計)
- **Others** - 包含 GSA, GSV, VTG, GLL, GPHDT 等所有其他類型
- 僅進行計數統計，不顯示個別句子內容
- 在5秒區間報告中顯示總計數與頻率

### XSENS封包檢測器

```cpp
class XSENSPacketDetector {
    // XBUS標頭檢測: 0xFA 0xFF 序列
    // 動態封包長度計算 (最大256 bytes)
    // 防止緩衝區溢出保護
    // 即時封包解析與狀態顯示
};
```

### 緩衝區狀態評估

- **正常狀態**: 0-50 bytes (正常資料流)
- **繁忙狀態**: 50-100 bytes  (高頻資料期間)
- **警告狀態**: >100 bytes (處理延遲)
- **錯誤狀態**: >500 bytes (嚴重阻塞)

---

## 💡 使用範例

### 基本初始化

```cpp
#include "src/hal/board_support.h"

void setup() {
    // 初始化HAL層
    hal::initPeripherals();
    
    // 啟動感測器測試模式
    hal::initializeXSENSTest();
    
    Serial.println("HAL系統準備就緒");
}
```

### 主程式循環

```cpp
void loop() {
    // 處理所有感測器資料
    hal::processAllSensorData();
    
    // 報告狀態 (每5秒自動)
    hal::reportAllSensorStatus();
    
    // 處理除錯指令
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        hal::handleDebugCommand(cmd);
    }
}
```

### 特定通道操作

```cpp
// 取得GPS資料流
HardwareSerial& gps = hal::getUARTSerial(hal::UARTChannel::GNSS);
if (gps.available()) {
    String nmea = gps.readStringUntil('\n');
    // 處理NMEA資料
}

// 輸出MAVLink資料流  
HardwareSerial& mavlink = hal::getUARTSerial(hal::UARTChannel::MAVLINK);
if (hal::isUARTReady(hal::UARTChannel::MAVLINK)) {
    mavlink.write(mavlink_packet, packet_size);
}
```

---

## 📈 效能參考數據

### 典型運行狀況

- **XSENS IMU**: 100Hz, ~9.3 KB/s, 103字節封包
- **LOCOSYS GPS**: 17 Hz, ~450 B/s, NMEA標準格式
- **MAVLink輸出**: 460800波特率專用於高頻輸出
- **統計監控**: 區間統計每5秒更新，無需額外CPU負擔

### 資源使用

- **RAM**: ~2KB緩衝區 + 統計數據
- **CPU**: 極低處理負擔，主要為非阻塞I/O
- **I/O**: 5個UART通道並發處理

---

## 📝 組態調校建議

### 開發階段最佳化

#### 1. LOG等級調校
```cpp
// 完整除錯配置
#define LOG_LEVEL_LOCAL LOG_DEBUG        // 顯示詳細資訊
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_DEBUG) // 允許除錯訊息
```

#### 2. UART資料流調校
```cpp
// 增加緩衝區容量以應對高頻資料
const UARTConfig UART_CONFIGS[5] = {
    {115200, 25, 24, true, 1024, "Xsens IMU Input"}, // 增加緩衝區
};
```

#### 3. 感測器監控調校
```cpp
// 使用診斷指令
hal::handleDebugCommand("status");  // 檢查UART狀態
hal::handleDebugCommand("test");    // 執行迴路測試
```

### 生產環境最佳化

#### 減少LOG負擔
```cpp
// 生產環境配置
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)
```

#### 調整緩衝區策略
- **高頻資料** (XSENS): 1024 bytes
- **中頻資料** (DEBUG): 512 bytes  
- **低頻資料** (GPS): 256 bytes

---

## 🎯 效能基準

### 實測性能數據

- **XSENS IMU**: 90 Hz, ~9.3 KB/s, 103字節封包
- **LOCOSYS GPS**: 17 Hz, ~450 B/s, NMEA標準格式
- **MAVLink輸出**: 460800波特率/高頻專用輸出
- **統計系統**: 區間統計每5秒自動更新無需人工干預

### 系統資源

- **RAM**: ~2KB緩衝區 + 統計資料
- **CPU**: 非阻塞I/O主導，極低處理負擔 
- **I/O**: 5個UART通道同步併發處理

---

## 📚 版本紀錄

| 版本 | 日期 | 變更內容 |
|------|------|----------|
| 1.3 | 2025-08-13 | 重點NMEA解析與5秒區間統計 - 專注PLSHD/GNGGA/GNRMC三種類型，顯示完整句子內容，其他類型靜默統計，修正CRLF重複解析問題 |
| 1.2 | 2025-08-13 | 新增NMEA句子解析功能 - 支援GGA/RMC/GSV/GSA/VTG/GLL句子類型檢測與統計，添加`nmea`除錯指令 |
| 1.1 | 2025-08-12 | 完整重寫文檔 - 修正編碼問題，補齊所有HAL層功能區間統計 |
| 1.0 | 2025-08-12 | 初始版本 - 建立HAL層架構與區間統計 |

---

## 🔗 相關文檔

- [GMINS Log.h 系統使用說明](../FOUNDATION'/GMINS%20Log.h%20系統使用說明.md)
- [myUART.h 原始設計參考](../../legacy/myUART.h)
- [XSENS MTI 協定說明](../sensors/XSENS_protocol.md)
- [MAVLink 協定實作](../communication/MAVLink_implementation.md)

---

> **重要提醒**: 此文檔描述HAL層核心架構，請嚴格遵循配置原則避免系統不穩定。修改前請先備份。