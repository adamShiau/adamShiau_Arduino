# Parser 解析器系統

## 系統概述

GMINS 的 Parser 解析器系統負責處理原始感測器數據的解析和結構化，包含：
- **XBUS Parser**: 解析 MTI IMU 設備的 XBUS 協議封包
- **NMEA Parser**: 解析 GNSS 設備的 NMEA 0183 協議封包

## XBUS Parser 詳細說明

### 功能特性
- 解析 XBUS frame 為 `IMUData` 結構
- 支援多種數據識別碼
- 自動校驗和驗證
- 處理多種 XBUS 封包格式

### 支援的數據識別碼 (XDI)

#### **基本資訊**
| XDI | 名稱 | 長度 | 說明 |
|-----|------|------|------|
| 0x1020 | PACKET_COUNTER | 2B | 封包計數器 |
| 0x1060 | SAMPLE_TIME_FINE | 4B | 精確時間戳 (10kHz基準) |

#### **感測器數據**
| XDI | 名稱 | 長度 | 單位 | 說明 |
|-----|------|------|------|------|
| 0x4020 | ACCELERATION | 12B | m/s² | 加速度 (Body frame) |
| 0x4024 | ACCELERATION_NED | 12B | m/s² | 加速度 (NED frame) |
| 0x8020 | RATE_OF_TURN | 12B | rad/s | 角速度 (Body frame) |
| 0x8024 | RATE_OF_TURN_NED | 12B | rad/s | 角速度 (NED frame) |
| 0xC020 | MAGNETIC_FIELD | 12B | μT | 磁場 (Body frame) |
| 0xC024 | MAGNETIC_FIELD_NED | 12B | μT | 磁場 (NED frame) |
| 0x0810 | TEMPERATURE | 4B | °C | 溫度 |

#### **姿態數據**
| XDI | 名稱 | 長度 | 格式 | 說明 |
|-----|------|------|------|------|
| 0x2010 | QUATERNION | 16B | 4×float32 | 四元數 (w,x,y,z) |
| 0x2014 | QUATERNION_NED | 16B | 4×float32 | 四元數 NED frame |
| 0x2020 | ROTATION_MATRIX | 36B | 9×float32 | 旋轉矩陣 (3×3) |
| 0x2030 | EULER_ANGLES | 12B/24B | 3×float32/64 | 歐拉角 (Roll,Pitch,Yaw) |

#### **狀態資訊**
| XDI | 名稱 | 長度 | 說明 |
|-----|------|------|------|
| 0xE010 | STATUS_BYTE | 1B | 狀態位元組 |
| 0xE020 | STATUS_WORD | 2B | 狀態字 |

### XBUS Frame 格式

#### **標準 Frame 結構**
```
FA FF MID LEN [DATA...] CHK
                  └── 數據內容 (LEN bytes)
              └── 數據長度 (0-254)
          └── 訊息ID (0x36=MT_DATA2, 0x32=MT_DATA)
      └── Bus ID (固定 0xFF)
  └── 前導碼 (固定 0xFA)
```

#### **擴展長度 Frame (LEN=0xFF)**
```
FA FF MID FF Hi Lo [DATA...] CHK
               └─┬─┘ 16位長度
                 └── 長度高位元組
             └── 長度低位元組
```

#### **校驗和計算**
```cpp
// 計算範圍: Bus ID + Message ID + Length + Data
uint8_t sum = 0;
for (size_t i = 1; i < header_size + data_length; i++) {
    sum += frame_data[i];
}
uint8_t checksum = 0xFF - sum;
```

### MTData2 資料格式

每個數據項目格式：
```
XDI_Hi XDI_Lo LEN [DATA...]
               └── 數據內容 (LEN bytes)
           └── 數據長度
       └── XDI 高位元組
   └── XDI 低位元組
```

### 解析後數據結構 (IMUData)

#### **元資料**
```cpp
uint32_t schema;           // 數據架構版本
timestamp_us_t timestamp_us; // 時間戳 (微秒)
uint16_t packet_counter;   // 封包計數器
uint32_t flags;           // 狀態旗標 (bitmask)
```

#### **感測器數據 (Body Frame)**
```cpp
// 加速度 (m/s²)
float accel_x, accel_y, accel_z;

// 角速度 (rad/s)
float gyro_x, gyro_y, gyro_z;

// 磁場 (μT)
float mag_x, mag_y, mag_z;

// 溫度 (°C)
float temperature;
```

#### **姿態數據**
```cpp
// 四元數 (Hamilton 慣例, q_body_to_ned)
float quat_w, quat_x, quat_y, quat_z;

// 歐拉角 (弧度, ZYX順序)
float euler_roll;   // X軸旋轉 (橫滾)
float euler_pitch;  // Y軸旋轉 (俯仰)
float euler_yaw;    // Z軸旋轉 (偏航)
```

#### **品質**
```cpp
uint8_t data_quality;  // 數據品質 0-255
```

### 關鍵演算法

#### **1. 旋轉矩陣轉歐拉角**
從 ROTATION_MATRIX (0x2020) 計算歐拉角：
```cpp
// ZYX convention (Roll-Pitch-Yaw)
pitch = asin(-r31);
if (cos(pitch) > 1e-6) {
    roll = atan2(r32, r33);
    yaw = atan2(r21, r11);
} else {
    // 萬向節鎖情況
    roll = atan2(-r23, r22);
    yaw = 0.0f;
}
```

#### **2. 四元數正規化**
```cpp
float norm = sqrt(w*w + x*x + y*y + z*z);
if (norm > 0.1f && isfinite(norm)) {
    quat_w /= norm;
    quat_x /= norm;
    quat_y /= norm;
    quat_z /= norm;
}
```

#### **3. 數據品質評估**
```cpp
uint8_t quality = 0;
if (flags & IMU_ACCEL_VALID) quality += 30;
if (flags & IMU_GYRO_VALID) quality += 30;
if (flags & IMU_MAG_VALID) quality += 20;
if (flags & IMU_QUATERNION_VALID) quality += 15;
if (flags & IMU_CALIBRATED) quality += 5;
```

### 狀態旗標定義

| 旗標名稱 | 值 | 說明 |
|------|----|----- |
| IMU_ACCEL_VALID | 0x01 | 加速度數據有效 |
| IMU_GYRO_VALID | 0x02 | 角速度數據有效 |
| IMU_MAG_VALID | 0x04 | 磁場數據有效 |
| IMU_TEMPERATURE_VALID | 0x08 | 溫度數據有效 |
| IMU_QUATERNION_VALID | 0x10 | 四元數數據有效 |
| IMU_CALIBRATED | 0x20 | 感測器已校準 |
| IMU_DATA_FRESH | 0x40 | 數據為最新 |
| IMU_EULER_VALID | 0x80 | 歐拉角數據有效 |

---

## NMEA Parser 詳細說明

### 功能特性
- 解析 NMEA sentence 為 `GNSSData` 結構
- 支援 GGA、RMC、VTG、GSA、GSV 等標準訊息
- 支援 PLSHD 專有航向訊息
- 自動校驗和驗證機制

### 支援的 NMEA 訊息類型

#### **1. GGA - Global Positioning System Fix Data**
```
$GPGGA,time,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt,alt_unit,height,height_unit,dgps,dgps_id*checksum
```

**重要欄位:**
- **位置**: 經緯度 (DDMM.MMMM 格式轉換十進制)
- **定位品質**: 0=無定位, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
- **衛星數**: 參與定位的衛星數量
- **HDOP**: 水平精度稀釋因子
- **高度**: MSL 海拔高度 (公尺)

#### **2. RMC - Recommended Minimum Course**
```
$GPRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir*checksum
```

**重要欄位:**
- **數據有效性**: 'A'=有效, 'V'=無效
- **速度**: 節 (需轉換為 m/s, ×0.514444)
- **航向**: 真北方位角 (需轉換為弧度, ×π/180)

#### **3. VTG - Track Made Good and Ground Speed**
```
$GPVTG,course1,T,course2,M,speed1,N,speed2,K,mode*checksum
```

**重要欄位:**
- **航跡角**: 相對真北的航跡角 (需轉換為弧度)
- **地面速度**: km/h 轉 m/s (÷3.6)

#### **4. GSA - GPS DOP and Active Satellites**
```
$GPGSA,mode1,mode2,sat1,sat2,...,sat12,pdop,hdop,vdop*checksum
```

**重要欄位:**
- **PDOP**: 位置精度稀釋因子
- **HDOP**: 水平精度稀釋因子  
- **VDOP**: 垂直精度稀釋因子

#### **5. PLSHD - Proprietary Heading Data**
```
$PLSHD,heading,accuracy*checksum
```

**重要欄位:**
- **航向**: GPS 航向 (需轉換為弧度)
- **精度**: 航向精度 (需轉換為弧度)

### 解析後數據結構 (GNSSData)

#### **位置資訊 (WGS84)**
```cpp
double latitude;          // 緯度 (度)
double longitude;         // 經度 (度)
float altitude_msl;       // 海拔高度 (公尺, MSL)
float altitude_ellipsoid; // 橢球高度 (公尺)
```

#### **速度資訊 (NED 座標)**
```cpp
float velocity_north;     // 北向速度 (m/s)
float velocity_east;      // 東向速度 (m/s)
float velocity_down;      // 下向速度 (m/s, 負值)
float ground_speed;       // 地面速度 (m/s)
float course_over_ground; // 地面航跡角 (弧度)
```

#### **航向資訊**
```cpp
float gnss_heading;       // GNSS 航向 (弧度, 真北基準)
```

#### **精度資訊**
```cpp
float horizontal_accuracy; // 水平精度 (公尺)
float vertical_accuracy;   // 垂直精度 (公尺)
float speed_accuracy;      // 速度精度 (m/s)
float heading_accuracy;    // 航向精度 (弧度)
float hdop, vdop, pdop;    // 精度稀釋因子
```

#### **衛星資訊**
```cpp
uint8_t satellites_used;    // 參與定位的衛星數
uint8_t satellites_visible; // 可見衛星數
GNSSFixType fix_type;       // 定位類型
```

#### **時間資訊**
```cpp
uint16_t gps_week;         // GPS 週數
uint32_t tow_ms;           // 週內時間 (毫秒)
uint64_t unix_time_us;     // UNIX 時間戳 (微秒)
```

#### **原始數據**
```cpp
char nmea_sentence[256];   // 原始 NMEA 句子
uint16_t nmea_length;      // 句子長度
```

### 關鍵單位轉換

#### **座標轉換**
```cpp
// DDMM.MMMM 格式轉十進制度
float degrees = int(ddmm_value / 100);
float minutes = ddmm_value - (degrees * 100);
float decimal_degrees = degrees + (minutes / 60.0);
```

#### **速度轉換**
```cpp
// 節 轉 m/s
float speed_ms = speed_knots * 0.514444f;

// km/h 轉 m/s  
float speed_ms = speed_kmh / 3.6f;
```

#### **角度轉換**
```cpp
// 度 轉 弧度
float angle_rad = angle_deg * M_PI / 180.0f;
```

### GNSS 狀態旗標定義

| 旗標名稱 | 值 | 說明 |
|------|----|----- |
| GNSS_POSITION_VALID | 0x01 | 位置數據有效 |
| GNSS_VELOCITY_VALID | 0x02 | 速度數據有效 |
| GNSS_HEADING_VALID | 0x04 | 航向數據有效 |
| GNSS_ACCURACY_VALID | 0x08 | 精度數據有效 |
| GNSS_FIX_VALID | 0x10 | 定位狀態有效 |
| GNSS_RTK_FIXED | 0x20 | RTK 固定解 |
| GNSS_RTK_FLOAT | 0x40 | RTK 浮動解 |
| GNSS_DGPS_CORRECTION | 0x80 | DGPS 差分修正 |

### 支援的定位類型

```cpp
enum GNSSFixType : uint8_t {
    GNSS_FIX_NONE = 0,      // 無定位
    GNSS_FIX_2D = 1,        // 2D 定位
    GNSS_FIX_3D = 2,        // 3D 定位
    GNSS_FIX_DGPS = 3,      // DGPS 差分定位
    GNSS_FIX_RTK_FLOAT = 4, // RTK 浮動解
    GNSS_FIX_RTK_FIXED = 5, // RTK 固定解
    GNSS_FIX_PPP = 6,       // PPP 精密單點定位
    GNSS_FIX_STATIC = 7     // 靜態定位
};
```

---

## Parser 系統設計原則

### 1. **Header-Only 設計**
- 方便整合
- 減少編譯依賴
- 提高移植性

### 2. **零動態記憶體配置**
- 避免記憶體碎片
- 提高即時性
- 降低系統風險

### 3. **強類型檢查**
- 編譯期驗證
- 數據有效性檢查
- 校驗和驗證

### 4. **模組化設計**
- 獨立功能模組
- 清晰介面定義
- 易於擴展和維護

### 5. **效能導向**
- 最小化運算開銷
- 記憶體友善設計
- 適合嵌入式環境

---

## 使用範例

### XBUS Parser 使用
```cpp
// 解析 XBUS frame
IMUData imu_data;
bool success = XbusParser::parseXbusFrame(frame_data, frame_length, imu_data);

if (success && XbusParser::isValidXbusData(imu_data)) {
    // 使用姿態數據
    if (imu_data.flags & IMU_EULER_VALID) {
        float roll = imu_data.euler_roll;
        float pitch = imu_data.euler_pitch; 
        float yaw = imu_data.euler_yaw;
    }
    
    if (imu_data.flags & IMU_ACCEL_VALID) {
        float ax = imu_data.accel_x;
        float ay = imu_data.accel_y;
        float az = imu_data.accel_z;
    }
}
```

### NMEA Parser 使用
```cpp
// 解析 NMEA sentence
GNSSData gnss_data;
bool success = NmeaParser::parseNmeaSentence(sentence_data, length, gnss_data);

if (success && NmeaParser::isValidNmeaData(gnss_data)) {
    // 使用位置數據
    if (gnss_data.flags & GNSS_POSITION_VALID) {
        double lat = gnss_data.latitude;   // 緯度
        double lon = gnss_data.longitude;  // 經度
        float alt = gnss_data.altitude_msl; // 高度
    }
    
    // 使用航向數據
    if (gnss_data.flags & GNSS_HEADING_VALID) {
        float heading = gnss_data.gnss_heading; // 弧度
    }
}
```

---

## 相關檔案位置

- **XBUS Parser**: `/src/parsers/xbus_parser.h`
- **NMEA Parser**: `/src/parsers/nmea_parser.h`
- **Parser 工具**: `/src/parsers/parser_utils.h`
- **數據類型**: `/src/data/data_types.h`