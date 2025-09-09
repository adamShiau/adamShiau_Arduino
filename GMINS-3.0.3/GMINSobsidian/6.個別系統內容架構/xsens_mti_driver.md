# Xsens MTI Driver 技術文件

## 基本資訊

**日期**: 2025-01-17  
**版本**: v2.0 (簡化命令發送器)  
**狀態**: 生產穩定版本

---

## 概述

**純命令發送驅動程式** - 專門負責向 MTI 設備發送 XBUS 配置命令
- ✅ 單純命令發送 (無狀態管理)
- ✅ 標準 XBUS 協議封包
- ✅ 完整 MTI 初始化序列
- ✅ 100Hz 高頻數據輸出配置

---

## 核心功能

### 🎯 主要職責
1. **MTI 設備初始化** - 完整的 7 步驟配置序列
2. **XBUS 命令封包** - 標準協議格式處理
3. **設備校準控制** - 陀螺儀校準和 Heading Reset
4. **高頻數據配置** - 100Hz 完整導航數據輸出

### 📡 數據流向
```
setupMTIDevice() → XsensMTIDriver → Serial2 → MTI 設備
                     ↓
              [初始化完成後]
                     ↓
MTI 設備 → Serial2 → HAL → IngressManager → XbusParser → DFI
```

### 🔧 驅動程式架構
```cpp
class XsensMTIDriver {
public:
    // 核心方法
    void initializeStandardConfiguration();  // 完整初始化序列
    void resetHeading();                     // YAW 重置為 0
    void sendCommandSimple(cmd, data, len);  // 基礎命令發送

private:
    UARTInterface* uart_interface_;          // Serial2 介面
    uint8_t calculateChecksum();             // XBUS 校驗和
};
```

---

## MTI 初始化命令序列

### 🚀 完整 7 步驟初始化 (`initializeStandardConfiguration()`)

| 步驟 | 命令 | 參數 | 等待時間 | 說明 |
|------|------|------|----------|------|
| 1 | **CMD_GOTO_CONFIG** (0x30) | 無 | 100ms | 進入配置模式 |
| 2 | **CMD_INIT** (0x02) | 無 | 200ms | 設備初始化 |
| 3 | **CMD_SET_SAMPLE_PERIOD** (0xD4) | `{0x00, 0x0A}` | 100ms | 100Hz 採樣率 |
| 4 | **CMD_OUTPUT_CONFIG** (0xC0) | 完整配置 | 100ms | 數據輸出配置 |
| 5 | **CMD_GOTO_MEASUREMENT** (0x10) | 無 | 2000ms | 進入測量模式 |
| 6 | **陀螺儀校準** (0x22) | `{0x00, 0x0F}` | 20000ms | 15 秒靜止校準 |
| 7 | **Heading Reset** (0x22) | `{0x00, 0x03}` | 5000ms | YAW 重置為 0 |

### 📊 100Hz 完整數據輸出配置
```cpp
uint8_t output_config[] = {
    0x10, 0x20, 0x00, 0x64,  // Package Counter @ 100Hz
    0x10, 0x60, 0x00, 0x64,  // SampleTimeFine @ 100Hz (時間對齊)
    0x20, 0x34, 0x00, 0x64,  // Euler Angles NED @ 100Hz
    0x40, 0x20, 0x00, 0x64,  // Acceleration @ 100Hz  
    0x80, 0x20, 0x00, 0x64,  // Rate of Turn (Gyro) @ 100Hz
    0xC0, 0x20, 0x00, 0x64   // Magnetic Field @ 100Hz
};
```

### 🔧 XBUS 封包格式
```
發送格式: [0xFA] [0xFF] [MID] [LEN] [DATA...] [CHECKSUM]
校驗和: Two's complement (0x00 - sum)

範例: 進入配置模式
FA FF 30 00 CF
├─ 0xFA 0xFF: XBUS 前導碼
├─ 0x30: CMD_GOTO_CONFIG  
├─ 0x00: 無數據 (LEN=0)
└─ 0xCF: 校驗和
```

---

## 使用方式

### 🏗️ 系統整合
```cpp
// GMINS.ino - 系統啟動時調用
void setupMTIDevice() {
    UARTInterface* mti_interface = hal::getIMUInterface();
    XsensMTIDriver mti_driver(mti_interface);
    mti_driver.initializeStandardConfiguration();  // 執行完整初始化
}
```

### 🔄 傳輸路徑
```
指令發送: setupMTIDevice() → XsensMTIDriver → gIMU_Interface → Serial2 → MTI
數據接收: MTI → Serial2 → HAL → IngressManager → XbusParser → DFI
```

### ⚙️ 硬體配置
```cpp
// src/hal/board_support.cpp
IMU (Serial2):     通道 2, pins 24/25, 115200 baud, 1024 buffer
描述:              "Xsens IMU Input" 
SERCOM:           SERCOM2
```

---

## 設計特點

### 🎯 極簡設計原則
- **無狀態**: 不追蹤設備狀態，每次調用獨立
- **Fire-and-Forget**: 發送命令後不等待 ACK
- **單一職責**: 純命令發送，不處理數據接收
- **零錯誤檢查**: 最大化性能和簡潔性

### ⚡ 關鍵優勢
- **啟動速度快**: 直接發送命令序列
- **穩定可靠**: 無複雜狀態機或錯誤恢復
- **易於維護**: 代碼極簡，邏輯清晰
- **高性能**: 最小化 CPU 負載

---

## 技術要點

### 📡 XBUS 協議核心
```
封包格式: [0xFA] [0xFF] [MID] [LEN] [DATA] [CHECKSUM]
校驗算法: Two's complement (0x00 - sum)
輸出配置: [XDI_HIGH] [XDI_LOW] [RATE_HIGH] [RATE_LOW]
頻率編碼: 0x0064 = 100Hz, 0x0032 = 50Hz
```

### 🔧 實現細節
```cpp
void sendCommandSimple(uint8_t cmd_id, const uint8_t* data, uint8_t data_len) {
    // 1. 發送前導碼 [0xFA, 0xFF]
    // 2. 構建訊息 [MID, LEN, DATA]  
    // 3. 計算校驗和
    // 4. 發送完整封包
}
```

---

## 文件資訊

### 📁 核心文件
```
src/drivers/xsens_mti_driver.h    - 驅動程式介面 (66 行)
src/drivers/xsens_mti_driver.cpp  - 驅動程式實現 (112 行)  
GMINS.ino                         - setupMTIDevice() 調用
src/hal/board_support.cpp         - Serial2 硬體配置
```

### 🎯 重要常數
```cpp
// XBUS 命令 ID
CMD_GOTO_CONFIG      (0x30)  // 進入配置模式
CMD_INIT            (0x02)  // 設備初始化  
CMD_SET_SAMPLE_PERIOD (0xD4)  // 採樣頻率
CMD_OUTPUT_CONFIG   (0xC0)  // 輸出配置
CMD_GOTO_MEASUREMENT (0x10)  // 測量模式
CMD_SET_NO_ROTATION (0x22)  // 校準/重置

// 輸出頻率編碼
100Hz: {0x00, 0x64}    50Hz: {0x00, 0x32}
25Hz:  {0x00, 0x19}    16Hz: {0x00, 0x10}
```

---

**版本**: v2.0 (生產穩定版)  
**更新日期**: 2025-01-17  
**狀態**: ✅ 部署完成