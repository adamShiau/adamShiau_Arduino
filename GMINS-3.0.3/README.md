# GMINS v2.0 - GPS/IMU Navigation System

> **版本**: 2.0.0  
> **平台**: Arduino SAMD21 (Arduino Zero)  
> **開發狀態**: 架構重構中  
> **最後更新**: 2025-08-11

## 🚀 專案概覽

GMINS (GPS/IMU Navigation System) 是一個高精度導航系統，整合 GPS、IMU 和氣壓計等多種感測器，提供可靠的位置、速度和姿態估計。v2.0 版本採用全新的模組化架構設計，提升程式碼的可維護性和擴展性。

## 📁 專案結構

```
GMINS/
├── src/                    # 源代碼目錄
│   ├── core/              # 核心系統模組
│   ├── communication/     # 通信模組
│   ├── data/              # 數據處理模組
│   ├── utils/             # 工具函數庫
│   ├── hardware/          # 硬體抽象層
│   └── external/          # 外部函式庫
├── tests/                 # 測試檔案
├── docs/                  # 文檔
├── tools/                 # 開發工具
├── GMINS.ino             # 主程式
└── README.md             # 本檔案
```

## 🔧 核心功能

### 感測器融合
- **IMU 數據處理**: 加速度計、陀螺儀、磁力計數據融合
- **GPS 整合**: 高精度位置和速度資訊
- **氣壓計支援**: 高度資訊補強
- **多感測器融合**: 卡爾曼濾波器實現最優估計

### 通信協議
- **MAVLink 協議**: 與飛控系統和地面站通信
- **XSENS XBUS**: XSENS IMU 專用協議
- **NMEA**: 標準 GPS 協議支援
- **多通道管理**: 同時支援多個通信通道

### 座標系統
- **ENU 座標轉換**: 東北天座標系統
- **四元數姿態表示**: 避免萬向節死鎖
- **多種座標系支援**: WGS84, ENU, Body Frame

## 🛠️ 硬體需求

### 主控制器
- Arduino SAMD21 (Arduino Zero 或相容板)
- 最少 32KB SRAM, 256KB Flash

### 感測器
- XSENS IMU (MTi 系列) - UART 介面
- GPS 接收器 - UART 介面  
- 氣壓計 (可選) - I2C/SPI 介面

### 連接方式
- Serial1 (Pin 16/17): MAVLink 通信
- Serial2 (Pin 10/12): XSENS IMU
- Serial3 (Pin 14/15): GPS 接收器
- USB: 調試和配置

## 🚀 快速開始

### 1. 環境設置

```bash
# 安裝 PlatformIO (推薦)
pip install platformio

# 或者使用 Arduino IDE
# 安裝 Arduino SAMD Boards 套件包
```

### 2. 編譯和上傳

```bash
# 使用 PlatformIO
pio run --target upload

# 或在 Arduino IDE 中:
# 選擇板子: Arduino Zero (Native USB Port)
# 選擇連接埠並上傳
```

### 3. 基本配置

```cpp
// 在 src/hardware/pin_definitions.h 中調整引腳定義
// 在 src/core/system_controller.cpp 中配置系統參數
```

## 📊 API 參考

### 核心類別

```cpp
// 系統控制器
SystemController controller;
controller.initialize();
controller.setMode(SystemMode::NAVIGATION);

// 融合引擎
FusionEngine fusion;
FusionConfig config;
fusion.initialize(config);

// 通信管理
CommunicationManager comm;
comm.initialize();
comm.sendData(ChannelId::MAVLINK_UART, data, size);
```

## 🧪 測試

### 單元測試
```bash
# 執行單元測試
pio test -e native_test

# 或個別測試
pio test -e native_test --filter test_fusion_engine
```

### 硬體測試
```bash
# 硬體在環測試
pio test -e arduino_zero --filter hardware_tests
```

## 📈 性能指標

- **更新率**: 100Hz 系統循環
- **感測器頻率**: 100Hz 資料讀取
- **通信頻率**: 100Hz MAVLink 輸出
- **記憶體使用**: <28KB SRAM
- **計算延遲**: <5ms 感測器融合

## 🔍 故障排除

### 常見問題

1. **IMU 無數據**
   - 檢查 UART2 連線和波特率
   - 確認 XSENS 設備電源和配置

2. **GPS 無定位**
   - 確保天線良好放置
   - 檢查 UART3 連線
   - 等待冷啟動完成 (約 30-60 秒)

3. **MAVLink 連接失敗**
   - 檢查波特率設置 (預設 57600)
   - 確認地面站配置c:\Users\user\.aa專案管理\GMINS\GMINS.ino
   - 檢查 UART1 連線

### 除錯資訊

```cpp
// 啟用除錯輸出
Logger::setLogLevel(Logger::DEBUG);
Logger::enableSerial(true);
```

## 🤝 貢獻指南

1. Fork 專案
2. 建立功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交變更 (`git commit -m 'Add AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 開啟 Pull Request

## 📄 授權

本專案採用 MIT 授權 - 詳見 [LICENSE](LICENSE) 檔案

## 📞 聯繫資訊

- **專案維護**: GMINS 開發團隊
- **問題回報**: 請使用 GitHub Issues
- **技術討論**: 請建立 GitHub Discussions

## 📚 相關文件

- [架構設計文件](GMINSobsidian/3.軟體架構重新設計/新軟體架構設計.md)
- [源代碼結構](GMINSobsidian/4.源代碼架構/src目錄結構設計.md)
- [API 參考手冊](docs/api/)
- [開發者指南](docs/developer_guide.md)

---

**最後更新**: 2025-08-11  
**版本**: v2.0.0-dev  
**狀態**: 🚧 開發中