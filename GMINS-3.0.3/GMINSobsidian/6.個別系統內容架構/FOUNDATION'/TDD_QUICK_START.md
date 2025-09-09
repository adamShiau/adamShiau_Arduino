# TDD Flow Checker 快速使用指南

## 🚀 **立即開始**

TDD Flow Checker 已整合到 GMINS 中，現在您可以：

### **1. 直接編譯運行**
```bash
# 在 Arduino IDE 中直接編譯 GMINS.ino
# 或使用 PlatformIO 編譯
```

### **2. 觀察輸出**
串列監控器中每5秒會顯示：
```
[I][TDD_FLOW] GMINS::mainLoop IN=1 OUT=1
[I][TDD_FLOW] HAL::processAllSensorData IN=1 OUT=1  
[I][TDD_FLOW] SystemController::tick IN=1 OUT=1
[I][TDD_FLOW] MTI::dataProcessing IN=1 OUT=1
[I][TDD_FLOW] Navigation::callback IN=1 OUT=1
```

## 🔍 **快速故障檢測**

### **正常狀態**: 所有顯示 `IN=1 OUT=1`
- ✅ 系統運行正常
- ✅ 數據流暢通

### **發現問題**: 出現 `IN=1 OUT=0` 或 `IN=0 OUT=0`
- ⚠️ `GMINS::mainLoop IN=1 OUT=0` → 主循環異常
- ⚠️ `MTI::dataProcessing IN=1 OUT=0` → MTI 數據格式問題
- ⚠️ `Navigation::callback IN=0 OUT=0` → 無導航數據

## 🛠️ **在您的模組中使用**

### **步驟1**: 包含標頭檔
```cpp
#include "util/tdd_flow_checker.h"
```

### **步驟2**: 創建檢測器
```cpp
// 全局或靜態
TDDFlowChecker my_checker("MyModule", "myFunction");
```

### **步驟3**: 加入檢測點
```cpp
void myFunction() {
    my_checker.recordIn();        // 記錄流入
    
    // 您的邏輯
    if (success) {
        my_checker.recordOut();   // 記錄成功流出
    }
    // 失敗時不調用 recordOut()
    
    my_checker.update();          // 定期更新（5秒顯示）
}
```

## 📊 **實用範例**

### **UART 數據處理**
```cpp
void processUARTData() {
    static TDDFlowChecker uart_checker("UART", "processData");
    
    uart_checker.recordIn();
    
    if (uart.available()) {
        // 處理數據
        uart_checker.recordOut();
    }
    
    uart_checker.update();
}
```

### **協議發送**
```cpp  
void sendProtocolData(const uint8_t* data, size_t len) {
    static TDDFlowChecker protocol_checker("Protocol", "send");
    
    protocol_checker.recordIn();
    
    if (data && len > 0 && send_success) {
        protocol_checker.recordOut();
    }
    
    protocol_checker.update();
}
```

### **暫時除錯**
```cpp
void debugMyFunction() {
    TDDFlowChecker debug_checker("DEBUG", "tempCheck", 1000);  // 1秒檢查
    
    debug_checker.recordIn();
    // 測試代碼
    if (test_passed) {
        debug_checker.recordOut();
    }
    debug_checker.update();
}
```

## ⚙️ **調整設定**

### **改變報告週期**
```cpp
TDDFlowChecker checker("Module", "func", 3000);  // 3秒週期
```

### **控制顯示**
在 `log.h` 設定中：
```cpp
// 只顯示 TDD 輸出
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_INFO)

// 關閉 TDD 輸出  
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_WARN | LOG_BIT_ERROR)
```

## 🎯 **最佳實踐**

### **✅ 建議做法**
- 在關鍵函數進入點加 `recordIn()`
- 只在成功情況下調用 `recordOut()`
- 定期調用 `update()` (如在主循環中)
- 使用有意義的模組名和函數名

### **❌ 避免做法**
- 不要在 ISR 中使用
- 不要過度使用（每個小函數都加）
- 不要忘記調用 `update()`

## 🏃‍♂️ **30秒快速整合**

```cpp
// 1. 加入標頭檔
#include "util/tdd_flow_checker.h"

// 2. 創建檢測器
static TDDFlowChecker my_checker("YourModule", "yourFunction");

// 3. 在您的函數中
void yourFunction() {
    my_checker.recordIn();
    
    // 您的代碼
    if (success) {
        my_checker.recordOut(); 
    }
    
    my_checker.update();
}
```

**🎉 完成！現在您可以看到函數的流入/流出狀態了！**

**享受輕量級的流程監控吧！** 🚀