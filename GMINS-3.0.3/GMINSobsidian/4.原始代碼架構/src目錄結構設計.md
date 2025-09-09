# src 目錄結構設計

> **功能**: GMINS v2.0 源代碼組織和目錄結構設計  
> **版本**: v2.0  
> **最後更新**: 2025-08-08  
> **設計原則**: 模組化、可維護、可擴展

## 🏗️ 總體設計理念

GMINS v2.0 採用全新的源代碼組織結構，基於模組化設計原則，將系統功能清晰分離，提高代碼的可讀性、可維護性和可測試性。

### 設計原則
1. **職責分離**: 每個模組職責明確，避免功能混合
2. **介面標準化**: 模組間通過標準介面通訊
3. **依賴最小化**: 減少模組間的耦合度
4. **擴展性**: 易於添加新功能和模組
5. **可測試性**: 每個模組都可以獨立測試


## 🔧 模組詳細說明

### 1. core/ - 核心功能模組

#### SystemController (系統控制器)
```cpp
// system_controller.h
class SystemController {
public:
    bool initialize();
    void run();
    void shutdown();
    bool setMode(SystemMode mode);
    SystemStatus getStatus();
    
private:
    SystemMode current_mode_;
    SystemStatus system_status_;
    bool is_initialized_;
};
```

#### FusionEngine (融合引擎)
```cpp
// fusion_engine.h
class FusionEngine {
public:
    bool initialize(const FusionConfig& config);
    void processSensorData(const SensorData& data);
    NavigationState getNavigationState();
    void updateDynamicWeights();
    
private:
    FusionConfig config_;
    NavigationState nav_state_;
    DynamicWeights weights_;
};
```

#### SafetyGuardian (安全守護)
```cpp
// safety_guardian.h
class SafetyGuardian {
public:
    bool validateSensorData(const SensorData& data);
    bool checkSystemHealth();
    void handleFault(FaultType fault);
    SafetyStatus getSafetyStatus();
    
private:
    SafetyConfig config_;
    FaultHistory fault_history_;
    SafetyStatus status_;
};
```

### 2. communication/ - 通訊模組

#### CommunicationManager (通訊管理器)
```cpp
// comm_manager.h
class CommunicationManager {
public:
    bool initialize();
    void processIncomingData();
    bool sendData(ChannelId channel, const uint8_t* data, size_t size);
    CommStats getStatistics();
    
private:
    std::array<CommChannel, 4> channels_;
    MAVLinkHandler mavlink_handler_;
    XBUSHandler xbus_handler_;
    NMEAHandler nmea_handler_;
};
```

#### 協定處理器
```cpp
// mavlink_handler.h
class MAVLinkHandler {
public:
    void processMessage(const mavlink_message_t& msg);
    bool sendOdometry(const NavigationState& nav_state);
    bool sendGPSData(const GPSData& gps_data);
};

// xbus_handler.h  
class XBUSHandler {
public:
    bool parseXBUSData(const uint8_t* buffer, size_t size);
    XsensData getLatestData();
    bool isDataValid();
};

// nmea_handler.h
class NMEAHandler {
public:
    bool parseNMEASentence(const std::string& sentence);
    void forwardSentence(const std::string& sentence);
    GPSData extractGPSData(const std::string& sentence);
};
```

### 3. data/ - 資料處理模組

#### DataProcessor (資料處理器)
```cpp
// data_processor.h
class DataProcessor {
public:
    void processIMUData(const IMUData& imu_data);
    void processGPSData(const GPSData& gps_data);
    ProcessedData getProcessedData();
    bool validateDataQuality(const SensorData& data);
    
private:
    DataQualityChecker quality_checker_;
    DataBuffer data_buffer_;
};
```

#### CoordinateTransform (座標轉換)
```cpp
// coordinate_transform.h
class CoordinateTransform {
public:
    Vector3d llhToEnu(const LLHPosition& llh, const LLHPosition& origin);
    LLHPosition enuToLlh(const Vector3d& enu, const LLHPosition& origin);
    Quaternion eulerToQuaternion(const EulerAngles& euler);
    EulerAngles quaternionToEuler(const Quaternion& quat);
    
private:
    static constexpr double EARTH_RADIUS = 6378137.0;
    LLHPosition reference_origin_;
};
```

### 4. utils/ - 工具程式庫

#### Logger (日誌系統)
```cpp
// logger.h
class Logger {
public:
    enum LogLevel { DEBUG, INFO, WARNING, ERROR, FATAL };
    
    static void log(LogLevel level, const std::string& message);
    static void logf(LogLevel level, const char* format, ...);
    static void setLogLevel(LogLevel level);
    static void enableFileLogging(const std::string& filename);
    
private:
    static LogLevel current_level_;
    static bool file_logging_enabled_;
    static std::string log_filename_;
};
```

#### ConfigManager (配置管理)
```cpp
// config_manager.h
class ConfigManager {
public:
    bool loadConfig(const std::string& config_file);
    bool saveConfig(const std::string& config_file);
    
    template<typename T>
    T getParameter(const std::string& key, const T& default_value);
    
    template<typename T>
    void setParameter(const std::string& key, const T& value);
    
private:
    std::map<std::string, ConfigValue> parameters_;
};
```

### 5. hardware/ - 硬體抽象層

#### UARTInterface (UART 介面)
```cpp
// uart_interface.h
class UARTInterface {
public:
    bool initialize(int uart_id, int baud_rate);
    int available();
    int read(uint8_t* buffer, size_t size);
    int write(const uint8_t* buffer, size_t size);
    void flush();
    
private:
    int uart_id_;
    int baud_rate_;
    bool is_initialized_;
};
```

## 📊 模組依賴關係

### 依賴關係圖
```
SystemController
    ├── CommunicationManager
    │   ├── MAVLinkHandler
    │   ├── XBUSHandler
    │   └── NMEAHandler
    ├── FusionEngine
    │   └── DataProcessor
    │       └── CoordinateTransform
    ├── SafetyGuardian
    └── Utils
        ├── Logger
        ├── ConfigManager
        ├── MathUtils
        └── TimeUtils

Hardware Layer
    └── UARTInterface
```

## 🔍 編譯和建構

### PlatformIO 配置
```ini
; platformio.ini
[env:arduino_zero]
platform = atmelsam
board = zeroUSB
framework = arduino

; 編譯選項
build_flags = 
    -DGMINS_VERSION=2.0
    -DDEBUG_LEVEL=2
    -std=c++14

; 函式庫依賴
lib_deps = 
    SPI
    Wire
    
; 源代碼過濾
src_filter = 
    +<*>
    -<tests/>
    -<docs/>
    -<tools/>

; 測試配置
[env:native_test]
platform = native
test_framework = unity
test_filter = unit_tests/*
```

### CMake 配置
```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(GMINS_v2.0)

set(CMAKE_CXX_STANDARD 14)

# 包含目錄
include_directories(src/)

# 源文件收集
file(GLOB_RECURSE SOURCES 
    "src/core/*.cpp"
    "src/communication/*.cpp" 
    "src/data/*.cpp"
    "src/utils/*.cpp"
    "src/hardware/*.cpp"
)

# 創建函式庫
add_library(gmins_core ${SOURCES})

# 創建可執行文件
add_executable(gmins_test tests/test_main.cpp)
target_link_libraries(gmins_test gmins_core)
```

## 🧪 測試策略

### 測試組織結構
```cpp
// tests/unit_tests/test_fusion_engine.cpp
#include <unity.h>
#include "core/fusion_engine.h"

void test_fusion_engine_initialization() {
    FusionEngine engine;
    FusionConfig config;
    
    TEST_ASSERT_TRUE(engine.initialize(config));
    TEST_ASSERT_EQUAL(FUSION_STATE_READY, engine.getState());
}

void test_sensor_data_processing() {
    // 測試感測器資料處理
}

void setUp() {
    // 測試設置
}

void tearDown() {
    // 測試清理
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_fusion_engine_initialization);
    RUN_TEST(test_sensor_data_processing);
    return UNITY_END();
}
```

## 🔗 相關文件連結

### 設計文件
- [[3.軟體架構重新設計/新軟體架構設計]] - 整體軟體架構
- [[4.源代碼架構/核心模組設計]] - 各核心模組詳細設計
- [[4.源代碼架構/通訊模組設計]] - 通訊模組實作細節

### 開發指導  
- [[5.實作指導/編碼規範]] - 代碼風格和開發標準
- [[5.實作指導/測試策略]] - 測試方法和品質保證
- [[5.實作指導/開發環境設置]] - 開發環境配置

### 其他參考
- [[1.系統概覽/GMINS專案架構總覽]] - 專案整體架構
- [[2.硬體架構/硬體架構整合文件]] - 硬體架構說明

---

**最後更新**: 2025-08-08  
**適用版本**: GMINS v2.0  
**維護者**: GMINS 開發團隊  
**設計狀態**: 架構設計完成，準備實作