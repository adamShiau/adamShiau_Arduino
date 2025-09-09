# GMINS 核心數據類型設計文件
# `src/data/data_types.h` 完整規格說明

> **版本**: v1.0  
> **更新日期**: 2025-08-12  
> **階段**: 完整規格實現 - 基於 data_types_design.md v1.0  
> **檔案**: `src/data/data_types.h`

## 🎯 設計概述

此文件僅定義數據結構和枚舉、時間基準類型、狀態標誌（使用 bitmask），不包含：
- 數學轉換函數（→ math_utils.h）
- 數據驗證邏輯（→ validation.h/.cpp）
- 協議封包格式（→ proto/*.h/.cpp）
- 設備配置（→ drivers/*.h/.cpp）

### 核心設計原則
- **統一時間基準**：全系統使用 `timestamp_us_t` (uint64_t) microseconds since boot
- **標準化單位**：SI 標準單位 + 內部統一使用弧度制
- **座標系定義**：支援多種座標系（BODY/NED/ENU/ECEF/WGS84）
- **狀態標誌使用 bitmask**：使用位元 OR 運算避免手動計算錯誤
- **編譯時檢查**：靜態斷言確保數據結構大小、對齊和精度要求

## 📐 基礎類型定義

### 時間基準定義
```cpp
// 統一時間基準
using timestamp_us_t = uint64_t;  // microseconds since boot
using timestamp_ms_t = uint32_t;  // milliseconds（相容性支援）

// 數據結構版本控制（用於未來相容性）
constexpr uint32_t DATA_SCHEMA_VERSION = 1;
```

### 座標系枚舉
```cpp
enum CoordinateFrame : uint8_t {
    FRAME_BODY              = 0,  // 機體座標系
    FRAME_NED               = 1,  // North-East-Down
    FRAME_ENU               = 2,  // East-North-Up
    FRAME_ECEF              = 3,  // Earth-Centered Earth-Fixed
    FRAME_WGS84_LLH         = 4   // WGS84 經緯高
};
```

## 🏷️ 狀態標誌定義（使用 bitmask 和 OR 運算）

### IMU 數據狀態標誌
使用位元 OR 運算定義 ALL_VALID，避免手動計算錯誤
```cpp
enum IMUDataFlags : uint32_t {
    IMU_ACCEL_VALID         = 1u << 0,
    IMU_GYRO_VALID          = 1u << 1,
    IMU_MAG_VALID           = 1u << 2,
    IMU_TEMPERATURE_VALID   = 1u << 3,
    IMU_QUATERNION_VALID    = 1u << 4,
    IMU_CALIBRATED          = 1u << 5,
    IMU_DATA_FRESH          = 1u << 6,
    // 使用位元 OR 運算，未來加欄位不會忘記更新
    IMU_ALL_VALID           = IMU_ACCEL_VALID | IMU_GYRO_VALID | IMU_MAG_VALID |
                              IMU_TEMPERATURE_VALID | IMU_QUATERNION_VALID
};
```

### GNSS 數據狀態標誌
```cpp
enum GNSSDataFlags : uint32_t {
    GNSS_POSITION_VALID     = 1u << 0,
    GNSS_VELOCITY_VALID     = 1u << 1,
    GNSS_HEADING_VALID      = 1u << 2,
    GNSS_ACCURACY_VALID     = 1u << 3,
    GNSS_FIX_VALID          = 1u << 4,
    GNSS_RTK_FIXED          = 1u << 5,
    GNSS_RTK_FLOAT          = 1u << 6,
    GNSS_DGPS_CORRECTION    = 1u << 7,
    // 使用位元 OR 運算定義核心有效性
    GNSS_ALL_VALID          = GNSS_POSITION_VALID | GNSS_VELOCITY_VALID |
                              GNSS_HEADING_VALID | GNSS_ACCURACY_VALID | GNSS_FIX_VALID
};
```

### 導航狀態標誌
```cpp
enum NavigationStateFlags : uint32_t {
    NAV_POSITION_VALID      = 1u << 0,
    NAV_VELOCITY_VALID      = 1u << 1,
    NAV_ATTITUDE_VALID      = 1u << 2,
    NAV_ANGULAR_VEL_VALID   = 1u << 3,
    NAV_FUSION_ACTIVE       = 1u << 4,
    NAV_SHIFT_CORRECTED     = 1u << 5,
    NAV_USER_ROTATION_APPLIED = 1u << 6,
    NAV_READY_FOR_OUTPUT    = 1u << 7,
    // 核心導航數據有效性
    NAV_ALL_VALID           = NAV_POSITION_VALID | NAV_VELOCITY_VALID | 
                              NAV_ATTITUDE_VALID | NAV_ANGULAR_VEL_VALID
};
```

### GPS 定位類型枚舉
內部使用，與 MAVLink GPS_FIX_TYPE 不同，需在 proto layer 做映射
```cpp
enum GNSSFixType : uint8_t {
    GNSS_FIX_NONE           = 0,  // 無定位
    GNSS_FIX_2D             = 1,  // 2D 定位 
    GNSS_FIX_3D             = 2,  // 3D 定位
    GNSS_FIX_DGPS           = 3,  // DGPS 差分定位
    GNSS_FIX_RTK_FLOAT      = 4,  // RTK 浮點解
    GNSS_FIX_RTK_FIXED      = 5,  // RTK 固定解
    // 為未來擴展預留
    GNSS_FIX_PPP            = 6,  // PPP 精密單點定位
    GNSS_FIX_STATIC         = 7   // 靜態定位
};
```

### GNSS Fix Type 到 MAVLink 映射函數
```cpp
inline uint8_t gnss_fix_type_to_mavlink(GNSSFixType fix_type) {
    switch (fix_type) {
        case GNSS_FIX_NONE:      return 0;  // GPS_FIX_TYPE_NO_GPS
        case GNSS_FIX_2D:        return 2;  // GPS_FIX_TYPE_2D_FIX  
        case GNSS_FIX_3D:        return 3;  // GPS_FIX_TYPE_3D_FIX
        case GNSS_FIX_DGPS:      return 4;  // GPS_FIX_TYPE_DGPS
        case GNSS_FIX_RTK_FLOAT: return 5;  // GPS_FIX_TYPE_RTK_FLOAT
        case GNSS_FIX_RTK_FIXED: return 6;  // GPS_FIX_TYPE_RTK_FIXED
        case GNSS_FIX_STATIC:    return 7;  // GPS_FIX_TYPE_STATIC
        case GNSS_FIX_PPP:       return 8;  // GPS_FIX_TYPE_PPP
        default:                 return 0;  // 未知類型映射為 NO_GPS
    }
}
```

## 📦 核心數據結構定義

### 1. IMU 原始數據結構
```cpp
/**
 * IMU 原始數據結構
 * 座標系：BODY FRAME (機體座標系) - 右手座標系
 * 角度單位：內部統一使用弧度（rad）
 * 四元數慣例：Hamilton 乘法，w,x,y,z 順序，q_body_to_ned
 * 其他單位：SI 標準單位
 */
struct IMUData {
    uint32_t schema = DATA_SCHEMA_VERSION;  // 版本控制，用於未來相容性
    timestamp_us_t timestamp_us;            // 數據時間戳
    
    // 加速度數據（Body frame 機體座標系，m/s²）
    float accel_x;                      // X軸加速度（通常為前進方向）
    float accel_y;                      // Y軸加速度（通常為右側方向）
    float accel_z;                      // Z軸加速度（通常為下方）
    
    // 陀螺儀數據（Body frame 機體座標系，rad/s）
    float gyro_x;                       // 繞X軸角速度（roll rate）
    float gyro_y;                       // 繞Y軸角速度（pitch rate）
    float gyro_z;                       // 繞Z軸角速度（yaw rate）
    
    // 磁力計數據（Body frame 機體座標系，微特斯拉 µT）
    float mag_x, mag_y, mag_z;
    
    // 姿態四元數（Hamilton 慣例，從 Body frame 到 NED frame 的旋轉）
    // 順序：w, x, y, z（實部在前）
    // 旋轉方向：q_body_to_ned，將 Body frame 向量轉到 NED frame
    float quat_w, quat_x, quat_y, quat_z;
    
    // 環境數據
    float temperature;                  // 溫度（攝氏度）
    
    // 品質指標
    uint16_t packet_counter;            // 封包計數器（升級為 uint16_t 避免快速繞回）
    uint8_t data_quality;               // 數據品質 0-255
    
    // 狀態標誌（使用 bitmask）
    uint32_t flags;                     // 組合 IMUDataFlags
};
```

### 2. GNSS 原始數據結構
```cpp
/**
 * GNSS 原始數據結構
 * 支援 NMEA 和 PLSHD 等格式
 * 位置：WGS84 座標系
 * 注意：heading_accuracy 單位為弧度，其他 accuracy 為米或 m/s
 */
struct GNSSData {
    uint32_t schema = DATA_SCHEMA_VERSION;  // 版本控制
    timestamp_us_t timestamp_us;
    
    // WGS84 位置數據
    double latitude;                    // 緯度（度）
    double longitude;                   // 經度（度）
    float altitude_msl;                 // 海拔高度（米，平均海平面）
    float altitude_ellipsoid;           // 橢球面高度（米）
    
    // 速度數據（NED 座標系，m/s）
    float velocity_north;               // 北向速度
    float velocity_east;                // 東向速度
    float velocity_down;                // 垂直速度（向下為正）
    
    // 地面速度和航向
    float ground_speed;                 // 地面速度（m/s）
    float course_over_ground;           // 地面航跡角（弧度，從北向順時針）
    
    // GNSS 航向數據（用於 Shift 校正參考，弧度）
    float gnss_heading;                 // GNSS 航向（弧度，從北向順時針）
    
    // 精度和品質指標（單位注意：heading_accuracy 為弧度！）
    float horizontal_accuracy;          // 水平精度（米）
    float vertical_accuracy;            // 垂直精度（米）
    float speed_accuracy;               // 速度精度（m/s）
    float heading_accuracy;             // 航向精度（弧度）
    
    float hdop, vdop, pdop;             // 精度稀釋因子
    
    // 衛星資訊
    uint8_t satellites_used;            // 參與定位的衛星數
    uint8_t satellites_visible;         // 可見衛星數
    GNSSFixType fix_type;               // 定位類型
    
    // 時間來源（增強功能）
    uint16_t gps_week;                  // GPS 週數
    uint32_t tow_ms;                    // 週內時間（毫秒）
    uint64_t unix_time_us;              // UNIX 時間戳（微秒）
    
    // 多感測器支援
    uint8_t source_id;                  // 感測器 ID（0=主，1=備援...）
    
    // 原始 NMEA 數據（用於除錯和記錄）
    char nmea_sentence[256];            // 原始 NMEA 句子
    uint16_t nmea_length;               // NMEA 句子長度
    
    // 狀態標誌（使用 bitmask）
    uint32_t flags;                     // 組合 GNSSDataFlags
};
```

### 3. 融合導航狀態
```cpp
/**
 * 融合導航狀態
 * 這是系統的最終輸出，包含所有處理和校正後的導航信息
 * 位置和速度：NED 座標系（相對於初始化基準點）
 * 姿態：四元數表示（Hamilton 慣例，從 Body frame 到 NED frame）
 * 角速度：Body frame（機體座標系）
 */
struct NavigationState {
    uint32_t schema = DATA_SCHEMA_VERSION;  // 版本控制
    timestamp_us_t timestamp_us;
    
    // 位置（NED 座標系，米，相對於初始化基準點）
    float position_north;               // 北向位置
    float position_east;                // 東向位置  
    float position_down;                // 垂直位置（向下為正）
    
    // 速度（NED 座標系，m/s）
    float velocity_north;               // 北向速度
    float velocity_east;                // 東向速度
    float velocity_down;                // 垂直速度（向下為正）
    
    // 加速度（NED 座標系，m/s²）
    float acceleration_north;           // 北向加速度
    float acceleration_east;            // 東向加速度
    float acceleration_down;            // 垂直加速度（向下為正）
    
    // 姿態四元數（Hamilton 慣例，從 Body frame 到 NED frame 的旋轉）
    // 順序：w, x, y, z（實部在前）
    // 旋轉方向：q_body_to_ned
    float quat_w, quat_x, quat_y, quat_z;
    
    // 角速度（Body frame 機體座標系，rad/s）
    float angular_velocity_x;           // 繞機體X軸角速度（roll rate）
    float angular_velocity_y;           // 繞機體Y軸角速度（pitch rate）  
    float angular_velocity_z;           // 繞機體Z軸角速度（yaw rate）
    
    // 不確定性估計（1-sigma 標準差）
    float position_std_north, position_std_east, position_std_down;     // 米
    float velocity_std_north, velocity_std_east, velocity_std_down;     // m/s
    float attitude_std_roll, attitude_std_pitch, attitude_std_yaw;      // 弧度
    
    // Shift 校正資訊
    float applied_shift_offset;         // 已應用的 Shift 偏移量（弧度）
    float shift_confidence;             // Shift 校正置信度 0.0-1.0
    
    // 品質指標
    uint8_t overall_quality;            // 整體導航品質 0-255
    uint8_t fusion_mode;                // 當前融合模式
    
    // 狀態標誌（使用 bitmask）
    uint32_t flags;                     // 組合 NavigationStateFlags
};
```

### 4. NED 座標系參考原點配置
```cpp
/**
 * NED 座標系參考原點配置
 * 記錄 NED 位置計算的基準點，使 NavigationState 位置可追溯
 */
struct NEDReferencePoint {
    uint32_t schema = DATA_SCHEMA_VERSION;
    
    // WGS84 基準點
    double origin_latitude;             // 原點緯度（度）
    double origin_longitude;            // 原點經度（度）
    float origin_altitude;              // 原點海拔（米）
    
    // 基準點設定狀態
    bool origin_set;                    // 是否已設定原點
    timestamp_us_t origin_set_time;     // 原點設定時間
    
    // 基準點品質
    float origin_accuracy;              // 原點精度（米）
    uint8_t origin_source;              // 原點來源（0=GNSS首次定位，1=手動設定）
};
```

### 5. 用戶自定義旋轉配置
```cpp
/**
 * 用戶自定義旋轉配置
 * 用於在原始IMU數據上應用用戶定義的座標系轉換
 */
struct UserRotationConfig {
    uint32_t schema = DATA_SCHEMA_VERSION;
    
    // 旋轉矩陣（3x3，從原始IMU座標系到目標座標系）
    float rotation_matrix[9];           // 按行主序存儲
    
    // 旋轉角度（弧度，ZYX 歐拉角序列）
    float euler_roll;                   // 繞X軸旋轉
    float euler_pitch;                  // 繞Y軸旋轉
    float euler_yaw;                    // 繞Z軸旋轉
    
    // 配置狀態
    bool is_enabled;                    // 是否啟用用戶旋轉
    bool is_valid;                      // 配置是否有效
    timestamp_us_t last_update_time;    // 最後更新時間
};
```

### 6. 系統配置常數
```cpp
/**
 * 系統配置常數
 */
struct SystemConstants {
    // 物理常數
    static constexpr float GRAVITY = 9.80665f;           // 標準重力加速度 m/s²
    static constexpr double EARTH_RADIUS = 6378137.0;    // 地球半徑（米，WGS84）
    
    // 傳感器限制
    static constexpr float MAX_VALID_ACCEL = 50.0f;      // 最大有效加速度 m/s²
    static constexpr float MAX_VALID_GYRO = 10.0f;       // 最大有效角速度 rad/s
    static constexpr float MAX_VALID_ALTITUDE = 10000.0f; // 最大有效高度 m
    
    // 時間相關
    static constexpr uint32_t MAX_TIME_GAP_US = 1000000; // 最大時間間隔 1秒
};
```

## ⚡ 編譯時檢查

### 靜態斷言檢查
```cpp
// 確保時間戳大小足夠
static_assert(sizeof(timestamp_us_t) >= 8, "時間戳必須至少 8 字節");

// 確保浮點數精度
static_assert(sizeof(float) == 4, "浮點數必須為 4 字節");
static_assert(sizeof(double) == 8, "雙精度浮點數必須為 8 字節");

// 確保枚舉大小
static_assert(sizeof(GNSSFixType) == 1, "GNSSFixType 必須為 1 字節");
static_assert(sizeof(CoordinateFrame) == 1, "CoordinateFrame 必須為 1 字節");

// 確保結構大小合理（防止意外膨脹）
static_assert(sizeof(IMUData) <= 128, "IMUData 結構應保持緊湊");
static_assert(sizeof(GNSSData) <= 512, "GNSSData 結構應保持合理大小");
static_assert(sizeof(NavigationState) <= 256, "NavigationState 結構應保持緊湊");

// 確保結構對齊（提升性能）
static_assert(alignof(IMUData) >= 4, "IMUData 應 4 字節對齊");
static_assert(alignof(GNSSData) >= 8, "GNSSData 應 8 字節對齊（因包含 double）");
static_assert(alignof(NavigationState) >= 4, "NavigationState 應 4 字節對齊");

// 確保關鍵欄位偏移量合理（便於高效存取）
// timestamp_us 為 64-bit，會有 4-byte 對齊 padding
static_assert(offsetof(IMUData, timestamp_us) == 8, "timestamp_us 偏移應為 8（含 padding）");
static_assert(offsetof(GNSSData, timestamp_us) == 8, "timestamp_us 偏移應為 8（含 padding）");
static_assert(offsetof(NavigationState, timestamp_us) == 8, "timestamp_us 偏移應為 8（含 padding）");
```

## 🛠️ 核心設計理念

### 1. 職責分離
- **data_types.h**: 只定義數據結構、枚舉、常數和 schema
- **math_utils.h**: 數學轉換函數（歐拉角↔四元數、座標轉換）
- **validation.h/.cpp**: 數據驗證邏輯和範圍檢查
- **proto/*.h/.cpp**: 協議封包格式和映射
- **drivers/*.h/.cpp**: 設備配置和通訊協議

### 2. 版本控制策略
- 每個結構都包含 `schema` 欄位用於版本控制
- `DATA_SCHEMA_VERSION` 統一管理版本號
- 支援未來的向下/向上相容性

### 3. 狀態管理
- 使用 `flags` bitmask 替代多個 boolean 欄位
- 使用位元 OR 運算定義複合狀態，避免手動計算錯誤
- 統一的狀態檢查邏輯

### 4. 座標系一致性
- 明確定義每個欄位的座標系
- IMU 使用 Body frame（機體座標系）
- GNSS 使用 WGS84 座標系
- NavigationState 使用 NED 座標系（相對於基準點）
- 四元數統一使用 Hamilton 慣例

### 5. 單位標準化
- 內部統一使用弧度制（rad）
- 時間統一使用微秒 (timestamp_us_t)
- 所有物理量使用 SI 標準單位

## 📋 實現注意事項

### 平台相容性
- 使用標準 C++ 類型和特性
- 避免平台特定的代碼
- 考慮大端/小端字節序問題

### 性能優化
- 結構對齊優化記憶體存取
- 使用適當的數據類型大小
- 編譯時檢查防止結構膨脹

### 除錯和維護
- 豐富的註解說明座標系和單位
- 版本控制支援未來擴展
- 靜態斷言確保結構完整性

---

## 📝 變更記錄

| 版本 | 日期 | 變更內容 |
|------|------|----------|
| v1.0 | 2025-08-12 | 基於實際 data_types.h 實現的完整設計文檔 |

---

**注意**: 本文件嚴格對應 `src/data/data_types.h` 的實際實現，任何修改都應該保持設計文檔與實現的一致性。