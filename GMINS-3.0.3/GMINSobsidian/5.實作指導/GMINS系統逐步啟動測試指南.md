# GMINS系統逐步啟動測試指南

> **創建日期**: 2025-08-31  
> **版本**: v1.0  
> **目的**: 系統功能逐步驗證與問題追蹤指南

## 📋 測試概覽

### 🎯 測試目標
- 驗證MAVLink協議數據流正確性
- 驗證AR1AFC自家GUI協議功能
- 驗證協議動態切換功能
- 確保測試數據→實際感測器數據的平滑過渡

### 📊 測試數據配置
- **測試位置**: 洛杉磯市中心 (34.0522°, -118.2437°)
- **預期輸出**: `✅ TX: GPS_INPUT 座標: 34.0522000°, -118.2437000°`
- **數據源**: `ProtocolManagerDualMode.h` 中的固定測試數據

---

## 🚀 Phase 1: 基礎系統初始化

### 目標
- 啟用setup() + SystemController
- 確認系統基本運作
- 建立穩定的測試環境

### 執行步驟
1. **啟用基礎setup函數**
   ```cpp
   void setup() {
     Serial.begin(115200);
     delay(2000);
     
     // 初始化SystemController
     system_controller = new SystemController();
     if (system_controller->initialize()) {
       LOGI("✅ SystemController 初始化成功");
     }
   }
   ```

2. **修改測試loop**
   ```cpp
   void loop() {
     if (system_controller && system_controller->getState() == SystemState::RUNNING) {
       system_controller->tick(); // 觸發數據流
     }
     delay(1000);
   }
   ```

### 預期結果
- ✅ Arduino編譯成功
- ✅ 看到SystemController初始化訊息
- ✅ 系統進入RUNNING狀態

### 問題排除
- **編譯錯誤**: 檢查setup/loop函數註解狀態
- **初始化失敗**: 檢查HAL層依賴
- **系統卡住**: 檢查delay設置和Serial輸出

---

## 🧪 Phase 2A: MAVLink數據流測試

### 目標
- 測試ProtocolManagerDualMode → MAVLink數據流
- 驗證洛杉磯測試數據正確輸出
- 確認MAVLink解析器工作正常

### 執行步驟
1. **確認測試數據配置**
   - 檔案: `/src/core/ProtocolManagerDualMode.h:68-80`
   - 驗證洛杉磯座標: `34.0522, -118.2437`

2. **啟動MAVLink協議**
   - SystemController會自動初始化ProtocolManagerDualMode
   - 預設使用MAVLink協議

3. **觀察解析輸出**
   - 位置: MAVLinkProtocol.cpp:181
   - 格式: `printf("✅ TX: GPS_INPUT (Seq:%d) 座標: %.7f°, %.7f°\n")`

### 預期結果
```
✅ TX: GPS_INPUT (Seq:1) 座標: 34.0522000°, -118.2437000°
✅ TX: GPS_INPUT (Seq:2) 座標: 34.0522000°, -118.2437000°
...
```

### 成功標準
- ✅ 看到GPS_INPUT解析輸出
- ✅ 座標顯示洛杉磯位置 (34.05°, -118.24°)
- ✅ 序號遞增表示持續發送
- ✅ 無編譯或運行錯誤

### 問題排除
| 問題 | 可能原因 | 解決方案 |
|------|----------|----------|
| 沒有解析輸出 | SystemController未啟動 | 檢查setup()初始化 |
| 座標錯誤 | 測試數據配置錯誤 | 檢查ProtocolManagerDualMode.h |
| 序號不變 | 數據流停止 | 檢查loop()中的tick()調用 |

---

## 🏠 Phase 2B: AR1AFC自家GUI數據流測試

### 目標
- 測試ProtocolManagerDualMode → AR1AFCProtocol
- 驗證自家GUI協議封包輸出
- 確認IMU/姿態數據正確傳輸

### 執行步驟
1. **切換到AR1AFC協議**
   ```cpp
   // 在setup()中添加
   if (system_controller->setProtocolMode("AR1AFC")) {
     LOGI("✅ 切換到AR1AFC協議");
   }
   ```

2. **添加AR1AFC解析器**
   - 需要在AR1AFCProtocol.cpp中添加類似的解析功能
   - 解析52字節的AR1AFC封包格式

3. **觀察輸出差異**
   - AR1AFC主要發送NavigationState數據
   - 包含姿態四元數、位置、速度等

### 預期結果
```
✅ TX: AR1AFC 姿態: qw=0.9998, qx=0.01, qy=0.005, qz=0.0
✅ TX: AR1AFC 位置: N=100.0m, E=50.0m, D=-87.0m
```

### 成功標準
- ✅ 看到AR1AFC封包解析輸出
- ✅ IMU姿態數據正確
- ✅ 位置數據對應洛杉磯高度(87m)
- ✅ 封包格式符合52字節規格

---

## 🔄 Phase 2C: 協議切換功能測試

### 目標
- 測試動態協議切換功能
- 驗證TxMultiplexer協議管理
- 確保切換過程穩定無數據丟失

### 執行步驟
1. **添加切換測試代碼**
   ```cpp
   void loop() {
     static uint32_t last_switch = 0;
     static bool use_mavlink = true;
     
     // 每10秒切換一次協議
     if (millis() - last_switch > 10000) {
       if (use_mavlink) {
         system_controller->setProtocolMode("AR1AFC");
         LOGI("🔄 切換到AR1AFC");
       } else {
         system_controller->setProtocolMode("MAVLINK");
         LOGI("🔄 切換到MAVLink");
       }
       use_mavlink = !use_mavlink;
       last_switch = millis();
     }
     
     system_controller->tick();
     delay(100);
   }
   ```

2. **監控切換過程**
   - 觀察協議切換日誌
   - 確認數據流連續性
   - 檢查是否有錯誤或卡住

### 預期結果
```
🔄 切換到AR1AFC
✅ TX: AR1AFC 姿態: qw=0.9998...
✅ TX: AR1AFC 姿態: qw=0.9998...
🔄 切換到MAVLink
✅ TX: GPS_INPUT (Seq:15) 座標: 34.0522000°...
✅ TX: GPS_INPUT (Seq:16) 座標: 34.0522000°...
```

### 成功標準
- ✅ 協議切換日誌正常
- ✅ 兩種協議數據都正確輸出
- ✅ 切換過程無卡住或錯誤
- ✅ TxMultiplexer控制權轉移正常

---

## 🔄 Phase 3: 數據源切換階段 (測試→實際感測器)

### 目標
- 從固定測試數據切換到實際感測器數據
- 保持協議切換功能正常
- 驗證實際GPS位置和IMU數據
 根據我的分析，以下是從測試數據切換到真實感測器數據的關鍵連接點：

  🔄 測試數據 ↔ 真實數據切換位置

  1️⃣ 主要切換點：system_controller.cpp 第513-530行

  目前使用測試數據的位置：
  // 第513-530行：processingLoop() 函數中
  // ✅ Phase 2A測試：使用固定的洛杉磯測試數據
  GNSSData current_gnss = {};
  current_gnss.latitude = 34.0522;      // 洛杉磯測試數據
  current_gnss.longitude = -118.2437;   // 洛杉磯測試數據
  current_gnss.altitude_msl = 87.0f;
  current_gnss.fix_type = (GNSSFixType)3;
  current_gnss.satellites_visible = 16;
  // ...
  protocol_manager_->continuousUpdate(current_gnss, current_nav, current_imu);

  2️⃣ 真實數據來源：數據流回調系統

  真實感測器數據流路徑：
  1. IMU數據：IngressManager → XbusFrameCallback → dfi_->updateIMU()
  2. GPS數據：IngressManager → NmeaFrameCallback → dfi_->updateGNSS()

  切換方法：
  // 【測試模式 → 實際模式】只需要註釋/取消註釋這一段：

  void SystemController::processingLoop() {
      // === 測試模式（目前使用） ===
      /*
      GNSSData current_gnss = {};
      current_gnss.latitude = 34.0522;  // 測試數據
      // ... 其他測試數據
      protocol_manager_->continuousUpdate(current_gnss, current_nav, current_imu);
      */

      // === 實際模式（使用真實感測器） ===
      // protocol_manager_->continuousUpdate() 改為由 DFI 回調觸發
      // 數據來源：ingress_manager_->process() → 感測器回調 → DFI → 協議輸出
  }

  3️⃣ 已存在的真實數據路徑

  系統已經有完整的真實感測器支援：
  - IMU: Serial2 → XbusParser → IMUData → DFI → 協議輸出
  - GPS: Serial4 → NmeaParser → GNSSData → DFI → 協議輸出

  目前狀況：
  - ✅ 真實感測器數據已經在背景運作（第502-504行）
  - ✅ 測試數據是額外注入的（第513-530行）
  - 🔄 只需要移除測試數據注入，系統就會切換到真實數據

  4️⃣ 切換步驟

  Phase 3/4 時的切換動作：
  // system_controller.cpp 第513-530行：
  void SystemController::processingLoop() {
      // 1) 處理真實輸入數據（已存在）
      if (ingress_manager_ && data_flow_enabled_) {
          ingress_manager_->process();  // ← 真實感測器數據處理
      }

      // 2) 移除這整段測試數據注入即可！
      // if (protocol_manager_ && data_flow_enabled_) {
      //     GNSSData current_gnss = {}; // 測試數據
      //     // ...
      //     protocol_manager_->continuousUpdate(current_gnss, current_nav, current_imu);
      // }
  }

  總結：要切換到真實數據，只需要：
  - 移除 processingLoop() 中第513-530行的測試數據注入
  - 保留 第502-504行的真實感測器處理
  - 系統會自動使用 IMU(Serial2) + GPS(Serial4) 的真實數據
### 預期結果
```
// 切換前 (測試數據)
✅ TX: GPS_INPUT 座標: 34.0522000°, -118.2437000° (洛杉磯)

// 切換後 (實際位置，例如台灣)
✅ TX: GPS_INPUT 座標: 25.0338000°, 121.5645000° (你的實際位置)
```

### 成功標準
- ✅ 座標顯示從洛杉磯變為實際位置
- ✅ IMU數據反映實際設備狀態
- ✅ 兩種協議都使用實際感測器數據
- ✅ 協議切換功能繼續正常

---

## 🎯 Phase 4: 實際感測器運行

### 目標
- 完整系統運行
- 長期穩定性測試
- 性能監控和優化

### 執行步驟
1. **啟用完整功能**
   - 完整的HAL層
   - TDD檢查器
   - 系統統計監控

2. **長期運行測試**
   - 持續運行1小時以上
   - 監控記憶體使用
   - 檢查數據流穩定性

3. **性能評估**
   - 檢查數據更新頻率
   - 監控協議切換延遲
   - 確認CPU使用率合理

### 成功標準
- ✅ 系統穩定運行超過1小時
- ✅ 記憶體使用保持穩定
- ✅ 數據更新頻率符合預期
- ✅ 協議切換響應時間<1秒

---

## 📊 測試記錄表

### Phase 1: 基礎系統初始化
- [ ] Arduino編譯成功
- [ ] SystemController初始化成功
- [ ] 系統進入RUNNING狀態
- **完成時間**: ___________
- **問題記錄**: ___________

### Phase 2A: MAVLink數據流測試
- [ ] 看到GPS_INPUT解析輸出
- [ ] 座標顯示洛杉磯位置
- [ ] 序號遞增正常
- **完成時間**: ___________
- **問題記錄**: ___________

### Phase 2B: AR1AFC數據流測試
- [ ] AR1AFC封包解析正常
- [ ] IMU姿態數據正確
- [ ] 位置數據正確
- **完成時間**: ___________
- **問題記錄**: ___________

### Phase 2C: 協議切換測試
- [ ] 協議切換日誌正常
- [ ] 兩種協議數據正確
- [ ] 切換過程無錯誤
- **完成時間**: ___________
- **問題記錄**: ___________

### Phase 3: 數據源切換
- [ ] 座標從洛杉磯變為實際位置
- [ ] IMU數據反映實際狀態
- [ ] 兩種協議都使用實際數據
- **完成時間**: ___________
- **問題記錄**: ___________

### Phase 4: 實際感測器運行
- [ ] 系統穩定運行>1小時
- [ ] 記憶體使用穩定
- [ ] 性能符合預期
- **完成時間**: ___________
- **問題記錄**: ___________

---

## 🔧 常見問題排除

### 編譯問題
```bash
# 常見錯誤1: 'setupMTIDevice' was not declared
解決方案: 註解掉setup()中的setupMTIDevice()調用

# 常見錯誤2: expected unqualified-id before '/' token
解決方案: 檢查註解符號配對 /* */
```

### 運行問題
```bash
# 問題1: 沒有解析輸出
檢查項目: SystemController是否初始化、tick()是否被調用

# 問題2: 座標錯誤
檢查項目: ProtocolManagerDualMode.h中的測試數據配置

# 問題3: 協議切換失敗
檢查項目: TxMultiplexer初始化、協議實例創建
```

### 性能問題
```bash
# 問題1: 記憶體不足
解決方案: 檢查大結構體分配、優化緩衝區使用

# 問題2: 數據更新慢
解決方案: 調整loop()中的delay、檢查阻塞函數

# 問題3: 協議切換延遲
解決方案: 優化switchToProtocol()實現、減少初始化時間
```

---

## 📝 測試完成檢查清單

### 功能驗證
- [ ] MAVLink GPS_INPUT封包正確輸出
- [ ] AR1AFC自家協議封包正確輸出
- [ ] 協議動態切換功能正常
- [ ] 測試數據→實際感測器數據切換正常
- [ ] 系統長期穩定運行

### 代碼品質
- [ ] 無編譯警告或錯誤
- [ ] 解析器輸出格式正確
- [ ] 錯誤處理機制完善
- [ ] 記憶體使用優化

### 文檔記錄
- [ ] 測試過程完整記錄
- [ ] 問題和解決方案文檔化
- [ ] 性能基準數據記錄
- [ ] 後續改進建議整理

---

## 🎯 下一步計劃

1. **Phase 1-2完成後**: 評估是否需要Phase 3-4
2. **協議測試完成後**: 考慮添加更多協議支持
3. **穩定性驗證後**: 進行壓力測試和邊界測試
4. **全功能驗證後**: 部署到實際應用環境

---

**測試指南版本**: v1.0  
**最後更新**: 2025-08-31  
**維護責任**: GMINS開發團隊