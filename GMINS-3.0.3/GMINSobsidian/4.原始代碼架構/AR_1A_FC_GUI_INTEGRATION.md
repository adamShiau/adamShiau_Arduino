# AR-1A-FC GUI整合模組

> **Documentation Version**: 1.0  
> **Last Updated**: 2025-07-22  
> **Project**: MINSPixhawk Arduino  
> **Branch**: AHRSGUI  
> **Description**: AR-1A-FC格式數據與自家GUI系統的整合模組

## 🎯 模組概述

此模組負責將MINSPixhawk系統的AHRS數據轉換為AR-1A-FC標準格式，並透過串列埠傳輸給自家開發的GUI系統。該模組基於原有的二進制輸出功能擴展而來。

## 📊 AR-1A-FC 數據格式規範

### 基本資訊
- **數據包長度**: 52 bytes
- **更新頻率**: 100Hz (可調整)
- **數據類型**: IEEE 754 單精度浮點數 (SFFP)
- **字節序**: 大端序 (Big Endian)
- **校驗機制**: CRC-32

### 數據包結構

| 位置 | 字段名稱 | 數據類型 | 描述 |
|------|----------|----------|------|
| 1-4 | Header | UINT32 | 數據包頭 |
| 5-8 | X-axis Rotational Data | SFFP | LSB (Byte 5) 為輸出的第一個 Unit_DPS |
| 9-12 | Y-axis Rotational Data | SFFP | LSB (Byte 9) 為輸出的第一個 Unit_DPS |
| 13-16 | Z-axis Rotational Data | SFFP | LSB (Byte 13) 為輸出的第一個 Unit_DPS |
| 17-20 | X-axis Acceleration Data | SFFP | LSB (Byte 17) 為輸出的第一個 Unit_g |
| 21-24 | Y-axis Acceleration Data | SFFP | LSB (Byte 21) 為輸出的第一個 Unit_g |
| 25-28 | Z-axis Acceleration Data | SFFP | LSB (Byte 25) 為輸出的第一個 Unit_g |
| 29-32 | Temperature | SFFP | LSB (Byte 29) 為輸出的第一個 Unit_℃ |
| 33-36 | Time Counter | UINT32 | LSB (Byte 33) 為輸出的第一個 Unit_millisecond |
| 37-40 | Attitude Pitch | SFFP | LSB (Byte 37) 為輸出的第一個 Unit_deg |
| 41-44 | Attitude Roll | SFFP | LSB (Byte 41) 為輸出的第一個 Unit_deg |
| 45-48 | Attitude Yaw | SFFP | LSB (Byte 45) 為輸出的第一個 Unit_deg |
| 49-52 | CRC | UINT32 | 循環冗餘校驗 (Cyclic Redundancy Check) |

## 🔧 啟用方式

### 預設配置
系統啟動時的預設配置：
```cpp
current_output_mode = OUT_MODE_BIN;     // 二進制輸出模式
current_Xsens_mode = MODE_AHRS;         // AHRS模式 (52字節輸出)
is_run = true;                          // 系統運行狀態
```

### 手動切換命令
透過Serial命令可以切換模式：
```bash
# 切換到AR-1A-FC格式輸出
AHRS_BIN

# 切換回MAVLink格式
AHRS_ML_ODOM
```

### 串列埠配置
- **數據輸出埠**: PIXHAWK_SERIAL (Serial1)
- **控制命令埠**: Serial (USB)
- **波特率**: 115200 bps

## ⚙️ 實現細節

### 核心函數位置
- **主程序**: `Arduinopixhawkv00.ino:841-851`
- **數據封裝**: 使用 `memcpy()` 進行字節複製
- **CRC計算**: `MyCRC().calCRC(buffer, 52)`
- **數據輸出**: `PIXHAWK_SERIAL.write(buffer, 52)`

### 數據流程
1. **數據採集**: 從Xsens IMU獲取原始數據
2. **格式轉換**: 將數據轉換為AR-1A-FC格式
3. **CRC計算**: 計算並添加校驗碼
4. **串列傳輸**: 透過Serial1發送52字節數據包

### 關鍵修復
**問題**: 數據流傳輸中斷  
**原因**: 錯誤的Serial緩衝區檢查條件  
**修復**: 移除 `PIXHAWK_SERIAL.available() >= 0x20` 條件  
**位置**: `Arduinopixhawkv00.ino:459`

## 📈 未來規劃

### Phase 1: 基礎整合 ✅
- [x] 修復數據流傳輸問題
- [x] 驗證AR-1A-FC格式符合性
- [x] 確認串列埠通信穩定性

### Phase 2: 功能擴展 🔄
- [ ] 增加數據品質監控
- [ ] 實現動態頻率調整
- [ ] 添加數據包遺失檢測
- [ ] 優化CRC計算效能

### Phase 3: GUI整合 📋
- [ ] GUI接收端協議實現
- [ ] 實時數據可視化
- [ ] 參數調整介面
- [ ] 系統狀態監控

### Phase 4: 進階功能 📋
- [ ] 數據錄製與回放
- [ ] 自動校準功能
- [ ] 故障診斷系統
- [ ] 遠程配置能力

## 🔍 除錯與監控

### 數據流狀態檢查
```cpp
// 監控串列埠狀態 (每2秒)
if (millis() - last_serial_check > 2000) {
    Serial.println("[DEBUG] PIXHAWK_SERIAL available: " + 
                   String(PIXHAWK_SERIAL.availableForWrite()) + " bytes");
}
```

### 常見問題排除
1. **數據流中斷**: 檢查 `is_run` 狀態和輸出模式
2. **格式錯誤**: 驗證字節序和數據類型
3. **CRC失敗**: 確認數據完整性和計算邏輯
4. **頻率不穩**: 檢查主循環時序和串列埠緩衝

## 📚 相關文檔
- [ADF043-AR-1A-EC姿態航向參考系統介面控制文件](./ADF043-AR-1A-EC姿態航向參考系統介面控制文件.pdf)
- [GNSS衛星顯示實現](./GNSS_SATELLITE_DISPLAY.md)
- [命令參考手冊](./COMMANDS_REFERENCE.md)
- [進度報告](./PROGRESS_REPORT.md)

## 📝 工作日誌

### 2025-07-22
- **修復**: 移除錯誤的Serial緩衝區檢查條件
- **驗證**: 確認啟動條件和數據輸出流程正確
- **文檔**: 建立AR-1A-FC GUI整合模組文檔
- **狀態**: 數據流問題已解決，準備進行GUI整合測試

### 未來更新記錄
*此處將記錄後續的開發進度和重要變更*

---

**注意事項**:
1. 此模組目前在 `AHRSGUI` 分支開發
2. 穩定版本已保護在 `V1-pixhawk` 標籤
3. 所有GUI相關開發應在此分支進行，避免影響主線穩定性