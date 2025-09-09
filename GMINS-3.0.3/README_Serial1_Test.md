# Serial1 測試工具使用指南

## 概述
`test_serial1_sender.py` 是用於測試 GMINS 系統 Serial1 命令接收功能的 Python 腳本。

## 安裝需求

### 1. 安裝 Python
確保您的電腦已安裝 Python 3.x

### 2. 安裝 pyserial
```bash
pip install pyserial
```

## 使用步驟

### 1. 準備 GMINS 系統
- 確保 GMINS 系統正常運行
- 確認已添加 Serial1 除錯代碼（會顯示 `🔍 Serial1 收到資料！`）
- 查看系統 log 輸出

### 2. 執行測試程式
```bash
cd /mnt/c/Users/user/.aaproject/GMINS
python test_serial1_sender.py
```

### 3. 選擇 COM port
程式會列出所有可用的 COM ports，選擇對應 Serial1 的端口。

### 4. 選擇測試模式
- **模式 1**: 自動發送測試命令（TEST, AR1AFC, MAVLINK 等）
- **模式 2**: 手動輸入命令
- **模式 3**: 兩個模式都執行

## 預期結果

### 如果連接正常
在 GMINS 系統的 Serial 輸出中應該看到：
```
🔍 Serial1 收到資料！
0x54 0x45 0x53 0x54 0x0A
原始資料字串: 'TEST'
處理後命令: 'TEST'
⚠️ Serial1 未知命令: 'TEST' (長度: 4)
```

### 協議切換命令
發送 `AR1AFC` 或 `MAVLINK` 應該看到：
```
🔍 Serial1 收到資料！
0x41 0x52 0x31 0x41 0x46 0x43 0x0A
原始資料字串: 'AR1AFC'
處理後命令: 'AR1AFC'
📡 Serial1 命令已執行: AR1AFC
```

## 🆕 智能模式切換功能

### 工作原理
- **協議切換命令**：使用固定波特率確保切換成功
  - `MAVLINK` 命令：230400 baud
  - `AR1AFC` 命令：460800 baud
- **其他測試命令**：根據當前協議模式自動選擇波特率
  - MAVLINK 模式下：460800 baud
  - AR1AFC 模式下：230400 baud

### 測試流程示例
```
1. [AR1AFC模式] TEST → 230400 baud
2. [AR1AFC模式] MAVLINK → 230400 baud (切換命令)
3. [MAVLINK模式] HELLO → 460800 baud (新模式)
4. [MAVLINK模式] AR1AFC → 460800 baud (切換命令)
5. [AR1AFC模式] STATUS → 230400 baud (新模式)
```

### 優勢
✅ **隨時測試通信** - 在任何協議模式下都能測試其他命令的接收  
✅ **自動波特率** - 無需手動切換，腳本自動處理  
✅ **狀態透明** - 隨時顯示當前協議模式和使用的波特率

## 故障排除

### 1. 找不到 COM port
- 檢查 Serial1 的硬體連接
- 確認 USB-to-Serial 轉換器正常工作
- 查看設備管理器中的 COM port

### 2. 連接失敗
- 確認 COM port 沒有被其他程式佔用
- 檢查波特率是否正確（AR1AFC: 460800, MAVLink: 230400）
- 確認硬體連接穩定

### 3. GMINS 沒有反應
- 檢查是否選擇了正確的 COM port
- 確認 GMINS 系統中的除錯代碼已添加
- 查看 GMINS 的 Serial 輸出 log

### 4. 資料格式問題
如果收到資料但格式不正確，檢查：
- 換行符設定
- 字元編碼
- 波特率匹配

## 測試命令說明

- `TEST`: 基本連通性測試
- `AR1AFC`: 切換到 AR1AFC 協議
- `MAVLINK`: 切換到 MAVLink 協議
- `STATUS`: 測試未知命令處理
- 自訂命令: 可以在手動模式下輸入任意命令

## 注意事項

1. **波特率**: 確保使用 230400 baud
2. **換行符**: 程式自動添加 `\n`
3. **TxMultiplexer**: 這個工具只測試接收，不會被 TxMultiplexer 影響
4. **同時連接**: 確保沒有其他程式同時使用同一個 COM port