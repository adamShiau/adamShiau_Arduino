# GMINS Adapter 系統筆記

> **文件版本**: 1.0  
> **建立日期**: 2025-08-12  
> **路徑**: `C:\Users\user\.aa專案管理\GMINS\src\adapter`  
> **用途**: 提供各種數據結構與數學工具之間的統一適配介面

## 📋 目錄概覽

```
src/adapter/
├── coord_adapter.h         # 座標轉換適配器
├── gnss_adapter.h          # GNSS 數據適配器  
├── imu_adapter.h           # IMU 數據適配器
└── navigation_adapter.h    # 導航狀態適配器
```

## 🎯 系統設計原則

### 統一設計模式
- **Header-only 設計**: 所有功能都以 inline 函數實現
- **無狀態架構**: 純函數設計，不維護內部狀態  
- **一致的錯誤處理**: 使用 bool 回傳值表示操作成功/失敗
- **標準化介面**: 統一的函數命名和參數傳遞方式

### 依賴結構
```
Adapter Layer
    ↓
data_types.h + math_utils.h
    ↓
底層數學和數據結構
```

## 📊 各個 Adapter 詳細說明

### 1. coord_adapter.h - 座標轉換適配器

**主要職責:**
- 統一座標系轉換介面 (NED ↔ ENU ↔ Body)
- 四元數與歐拉角相互轉換
- 旋轉矩陣操作封裝
- 地理座標與局部座標轉換

**核心功能模組:**

#### 🔄 座標系轉換
```cpp
// 座標系類型枚舉
enum class CoordinateFrame {
    BODY,       // 機體座標系 (X前Y右Z下)
    NED,        // 北東地座標系 (X北Y東Z下)
    ENU,        // 東北天座標系 (X東Y北Z上)
    ECEF        // 地心地固座標系
};

// 主要轉換函數
nedToEnu()          // NED → ENU 座標轉換
enuToNed()          // ENU → NED 座標轉換  
bodyToNed()         // Body → NED 座標轉換
nedToBody()         // NED → Body 座標轉換
```

#### 📐 四元數與歐拉角處理
```cpp
eulerToQuaternion()         // 歐拉角 → 四元數 (NED系)
quaternionToEuler()         // 四元數 → 歐拉角 (NED系)
normalizeQuaternion()       // 四元數正規化
multiplyQuaternions()       // 四元數乘法
```

#### 🔧 Shift 校正專用工具
```cpp
calculateShiftOffset()      // 計算 GNSS-IMU Shift 偏移量
applyShiftCorrection()      // 應用 Shift 校正
```

**使用場景:**
- IMU 與 GNSS 數據融合時的座標系統一
- 姿態角度計算和轉換
- Shift 校正算法實現

---

### 2. gnss_adapter.h - GNSS 數據適配器

**主要職責:**
- GNSS 數據有效性檢查和提取
- 航向精度評估
- Shift 校正計算支援
- GNSS 數據品質評分

**核心功能模組:**

#### 🧭 航向數據處理
```cpp
getHeading()                // 提取航向角（弧度）
getHeadingDegrees()         // 提取航向角（度數）
calculateShiftOffset()      // 計算與 IMU 的 Shift 偏移
```

#### 📍 位置和速度數據
```cpp
getPosition()               // 提取 WGS84 位置數據
getVelocityVector()         // 提取速度向量 (NED)
getGroundTrack()            // 提取地面速度和航跡角
```

#### 🎯 品質評估系統
```cpp
calculateDataQuality()      // 計算數據品質分數 (0-100)
isValidForShiftCalibration() // 檢查是否適合 Shift 校正
getPositionAccuracy()       // 獲取位置精度指標
```

**品質評分機制:**
- **基本定位品質 (40分)**: RTK Fixed(40) → RTK Float(35) → DGPS(25) → 3D(15) → 2D(5)
- **衛星數量 (20分)**: ≥8個(20) → ≥6個(15) → ≥4個(10)
- **精度稀釋因子 (20分)**: HDOP <1.0(20) → <2.0(15) → <5.0(10) → <10.0(5)
- **數據完整性 (20分)**: 各種數據有效標誌

---

### 3. imu_adapter.h - IMU 數據適配器

**主要職責:**
- IMU 感測器數據提取和驗證
- 姿態四元數處理
- 感測器數據品質評估
- 單位轉換和格式統一

**核心功能模組:**

#### 📊 姿態數據提取
```cpp
extractEulerAngles()        // 從四元數提取歐拉角
extractQuaternion()         // 提取並驗證四元數
getYawAngle()              // 提取偏航角（弧度）
getYawAngleDegrees()       // 提取偏航角（度數）
```

#### ⚡ 感測器數據處理
```cpp
getAccelerationVector()     // 提取加速度向量 (Body frame)
getGyroVector()            // 提取角速度向量 (Body frame)
getMagnetometerVector()    // 提取磁力計向量 (Body frame)
```

#### 🔍 數據驗證機制
```cpp
isDataValid()              // 綜合數據有效性檢查
calculateDataQuality()      // 數據品質分數計算
isDataStale()              // 數據時效性檢查
```

**數據驗證標準:**
- 加速度範圍: 0.1-50.0 m/s²
- 角速度範圍: <20.0 rad/s
- 溫度範圍: -40°C 到 85°C
- 四元數有效性檢查

---

### 4. navigation_adapter.h - 導航狀態適配器

**主要職責:**
- 導航狀態數據的統一存取介面
- 融合引擎輸出處理
- Shift 校正應用和管理
- 不確定性估計更新

**核心功能模組:**

#### 🎯 姿態狀態管理
```cpp
extractQuaternion()         // 提取導航狀態四元數
extractEulerAngles()        // 提取導航狀態歐拉角
updateQuaternion()          // 更新四元數狀態
updateFromEulerAngles()     // 使用歐拉角更新狀態
```

#### 🚀 運動狀態處理
```cpp
getPositionVector()         // 獲取位置向量 (NED)
getVelocityVector()         // 獲取速度向量 (NED)
getAccelerationVector()     // 獲取加速度向量 (NED)
updatePosition()            // 更新位置狀態
updateVelocity()            // 更新速度狀態
```

#### 🔧 Shift 校正管理
```cpp
applyYawShiftToQuaternion() // 四元數模式 Shift 校正
applyYawShiftToEuler()      // 歐拉角模式 Shift 校正
getCurrentShiftOffset()     // 獲取當前 Shift 偏移
updateShiftConfidence()     // 更新校正置信度
```

#### 🔄 融合引擎介面
```cpp
updateFromFusionOutput()    // 從融合輸出更新狀態
updateUncertainties()       // 更新不確定性估計
updateTimestamp()           // 更新時間戳
```

## 🔧 使用指導

### 典型使用流程

#### 1. GNSS 數據處理範例
```cpp
// 檢查 GNSS 數據品質
if (GNSSAdapter::calculateDataQuality(gnss_data) > 70) {
    float heading;
    if (GNSSAdapter::getHeading(gnss_data, heading)) {
        // 使用有效的航向數據
    }
}
```

#### 2. IMU-GNSS Shift 校正範例
```cpp
// 計算 Shift 偏移
float shift_offset;
if (GNSSAdapter::calculateShiftOffset(gnss_data, imu_yaw, shift_offset)) {
    // 應用到導航狀態
    NavigationAdapter::applyYawShiftToQuaternion(nav_state, shift_offset);
}
```

#### 3. 座標系轉換範例
```cpp
// NED → ENU 轉換
MathUtils::Vector3f ned_vector(1.0f, 2.0f, 3.0f);
MathUtils::Vector3f enu_vector = CoordAdapter::nedToEnu(ned_vector);
```

### ⚠️ 注意事項

#### 數據有效性檢查
- **必須先檢查標誌位**: 所有 Adapter 都會檢查對應的有效性標誌
- **數值有效性驗證**: 使用 `isfinite()` 檢查浮點數有效性
- **範圍合理性檢查**: 各 Adapter 都有內建的合理性檢查

#### 時間同步處理
- **時間戳檢查**: 使用 `isDataStale()` 檢查數據時效性
- **預設超時設定**: 
  - IMU: 100ms
  - GNSS: 1000ms  
  - Navigation: 200ms

#### 座標系約定
- **IMU Body Frame**: X前Y右Z下
- **NED Frame**: X北Y東Z下  
- **ENU Frame**: X東Y北Z上
- **角度單位**: 統一使用弧度，提供度數轉換函數

## 🛠️ 修改和維護指導

### 添加新功能
1. **選擇正確的 Adapter**: 根據數據類型選擇對應的適配器
2. **遵循命名約定**: 使用動詞開頭的函數名 (get, extract, update, apply)
3. **保持一致的錯誤處理**: 使用 bool 回傳值和可選指標參數
4. **添加調試工具**: 為新功能添加對應的 debugString 輸出

### 性能優化建議
1. **避免重複計算**: 使用 inline 函數減少函數呼叫開銷
2. **合理使用快取**: 對於昂貴的計算結果進行快取
3. **最小化記憶體分配**: 使用棧上變數和引用傳遞

### 調試和測試
- 每個 Adapter 都提供 `debugString()` 函數用於除錯輸出
- 使用 `calculateDataQuality()` 監控數據品質
- 定期檢查 `isDataValid()` 和 `isDataStale()` 回傳值

## 📈 系統整合注意事項

### 與其他模組的互動
- **Fusion Engine**: 主要透過 NavigationAdapter 進行數據交換
- **Data Pipeline**: 使用各 Adapter 進行數據格式統一
- **Math Utils**: 所有 Adapter 都依賴 MathUtils 提供的基礎數學工具

### 執行緒安全
- **無狀態設計**: 所有 Adapter 都是無狀態的，天然執行緒安全
- **數據修改**: 只有 NavigationAdapter 會修改傳入的數據結構
- **共享資源**: 注意 NavigationState 的並發存取

---

*此文件記錄了 GMINS 系統中 Adapter 層的完整設計和使用方式，便於後續開發和維護時參考。*