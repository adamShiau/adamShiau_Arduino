# CLAUDE.md - MINSPixhawk

> **Documentation Version**: 1.3  
> **Last Updated**: 2025-08-15  
> **Project**: MINSPixhawk  
> **Description**: Integrated framework for Xsens MTi-680 sensor and Pixhawk (PX4) flight control system with MAVLink protocol  
> **Features**: GitHub auto-backup, Task agents, technical debt prevention, TDD workflow integration

This file provides essential guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 🚨 CRITICAL RULES - READ FIRST

> **⚠️ RULE ADHERENCE SYSTEM ACTIVE ⚠️**  
> **Claude Code must explicitly acknowledge these rules at task start**  
> **These rules override all other instructions and must ALWAYS be followed:**

### 🔄 **RULE ACKNOWLEDGMENT REQUIRED**
> **Before starting ANY task, Claude Code must respond with:**  
> "✅ CRITICAL RULES ACKNOWLEDGED - I will follow all prohibitions and requirements listed in CLAUDE.md"

### 🧪 **TDD OPERATING MODE**
> **For development tasks involving new functionality, Claude Code must also respond with:**  
> "✅ TDD RULES ACKNOWLEDGED – I will write tests first, implement the minimal code to pass them, and avoid any out‑of‑scope changes."

#### **TDD 三鐵律（覆蓋一切其他指示）**
1. **先測試**：沒有測試就不寫功能
2. **最小實作**：只寫到「讓測試轉綠」所需的最少程式碼
3. **不越界**：不改未指定檔案、不新增抽象層、不動 HAL 介面與公開 API

### ❌ ABSOLUTE PROHIBITIONS
- **NEVER** create new files in root directory → use proper module structure
- **NEVER** write output files directly to root directory → use designated output folders
- **NEVER** create documentation files (.md) unless explicitly requested by user
- **NEVER** use git commands with -i flag (interactive mode not supported)
- **NEVER** use `find`, `grep`, `cat`, `head`, `tail`, `ls` commands → use Read, LS, Grep, Glob tools instead
- **NEVER** create duplicate files (manager_v2.py, enhanced_xyz.py, utils_new.js) → ALWAYS extend existing files
- **NEVER** create multiple implementations of same concept → single source of truth
- **NEVER** copy-paste code blocks → extract into shared utilities/functions
- **NEVER** hardcode values that should be configurable → use config files/environment variables
- **NEVER** use naming like enhanced_, improved_, new_, v2_ → extend original files instead
- **NEVER** block or limit data flow rates → maintain sensor throughput integrity
- **NEVER** add excessive print/debug statements in main loop → preserve performance
- **NEVER** create functions that interfere with hardware data streams → single reader principle
- **NEVER** implement functionality without tests first (TDD violation)
- **NEVER** write code beyond minimal implementation to pass tests
- **NEVER** modify HAL interfaces or public APIs during TDD implementation
- **NEVER** create abstractions not required by current tests

### 📝 MANDATORY REQUIREMENTS
- **COMMIT** after every completed task/phase - no exceptions
- **GITHUB BACKUP** - Push to GitHub after every commit to maintain backup: `git push origin main`
- **USE TASK AGENTS** for all long-running operations (>30 seconds) - Bash commands stop when context switches
- **TODOWRITE** for complex tasks (3+ steps) → parallel agents → git checkpoints → test validation
- **READ FILES FIRST** before editing - Edit/Write tools will fail if you didn't read the file first
- **DEBT PREVENTION** - Before creating new files, check for existing similar functionality to extend  
- **SINGLE SOURCE OF TRUTH** - One authoritative implementation per feature/concept
- **DATA INTEGRITY** - Keep all sensor data flows clean and unimpeded
- **MONITOR-FIRST DESIGN** - Use DataFlowMonitor for all detection/statistics instead of inline checks
- **MODULAR LOGGING** - Every module must implement proper LOG_TAG and structured logging
- **MAIN LOOP CLEANLINESS** - Keep GMINS.ino minimal, delegate complex logic to HAL/modules
- **TDD WORKFLOW** - Follow Red → Green → Refactor cycle for all new functionality
- **TEST FIRST** - Create tests for Cases A (正常), B (邊界), C (異常) before implementation
- **MINIMAL IMPLEMENTATION** - Only write code necessary to pass current tests
- **TRACE VALIDATION** - Implement IN/OUT/FAIL traces for all critical functions

### ⚡ EXECUTION PATTERNS
- **PARALLEL TASK AGENTS** - Launch multiple Task agents simultaneously for maximum efficiency
- **SYSTEMATIC WORKFLOW** - TodoWrite → Parallel agents → Git checkpoints → GitHub backup → Test validation
- **GITHUB BACKUP WORKFLOW** - After every commit: `git push origin main` to maintain GitHub backup
- **BACKGROUND PROCESSING** - ONLY Task agents can run true background operations
- **PROBLEM-SOLVING WORKFLOW** - When encountering problems, ALWAYS ask user for clarification before proceeding

### 🔍 MANDATORY PRE-TASK COMPLIANCE CHECK
> **STOP: Before starting any task, Claude Code must explicitly verify ALL points:**

**Step 1: Rule Acknowledgment**
- [ ] ✅ I acknowledge all critical rules in CLAUDE.md and will follow them

**Step 2: Task Analysis**  
- [ ] Will this create files in root? → If YES, use proper module structure instead
- [ ] Will this take >30 seconds? → If YES, use Task agents not Bash
- [ ] Is this 3+ steps? → If YES, use TodoWrite breakdown first
- [ ] Am I about to use grep/find/cat? → If YES, use proper tools instead

**Step 3: Technical Debt Prevention (MANDATORY SEARCH FIRST)**
- [ ] **SEARCH FIRST**: Use Grep pattern="<functionality>.*<keyword>" to find existing implementations
- [ ] **CHECK EXISTING**: Read any found files to understand current functionality
- [ ] Does similar functionality already exist? → If YES, extend existing code
- [ ] Am I creating a duplicate class/manager? → If YES, consolidate instead
- [ ] Will this create multiple sources of truth? → If YES, redesign approach
- [ ] Have I searched for existing implementations? → Use Grep/Glob tools first
- [ ] Can I extend existing code instead of creating new? → Prefer extension over creation
- [ ] Am I about to copy-paste code? → Extract to shared utility instead

**Step 4: Session Management**
- [ ] Is this a long/complex task? → If YES, plan context checkpoints
- [ ] Have I been working >1 hour? → If YES, consider /compact or session break

**Step 5: Problem-Solving Protocol**
- [ ] If I encounter ANY problems, ambiguity, or uncertainty during task execution
- [ ] I must IMMEDIATELY ask the user for clarification before making assumptions
- [ ] This applies to: unclear requirements, missing dependencies, errors, or design choices
- [ ] I will NOT proceed with guesswork - user guidance is mandatory

> **⚠️ DO NOT PROCEED until all checkboxes are explicitly verified**

## 🧪 TDD DEVELOPMENT WORKFLOW

### 🔄 **TDD 工作流程 (Red → Green → Refactor)**

#### **Red（先寫測試）**
- 建立 3 類案例：**A 正常** / **B 邊界** / **C 異常**（含 NaN、長度 0、時間回繞…）
- 若與 UART/封包相關：建立 FakeTransport 攔截輸出、使用 **黃金樣本（golden bytes）** memcmp
- 追蹤輸出（TRACE）：入口 IN、出口 OUT、失敗 FAIL 必須可驗證

#### **Green（最小實作）**
- 只修改 target_file、只實作 function_or_class（不要影響其他檔案/模組）
- 僅為通過測試而寫，不提前最佳化，不新增參數

#### **Refactor（保綠重構）**
- 僅做命名/抽重複碼/移 .cpp 依賴；禁止改行為
- 測試須保持全綠

### 📋 **任務宣告格式（強制欄位）**

提交任務給 Claude 時，必須附上：

```
target_file: <相對路徑，例如 src/protocol/AR1AFCProtocol.cpp>
function_or_class: <精確符號，例如 AR1AFCProtocol::buildAndSend>

allowed_includes:
  - Arduino.h  (只允許於 HAL 邊界檔案)
  - src/transport/ITransport.h
  - src/util/trace.h

forbidden_changes:
  - src/hal/*
  - 公開 API 與既有介面簽名
  - 新增抽象層/新協定檔

trace_spec:
  - IN/OUT/FAIL 格式與觸發條件
```

### 🧪 **測試規格模板（Case A/B/C）**

#### **Case A – 正常**
- 輸入為合理值（可人工推導）
- 期望：回傳值正確、IN/OUT=1、封包長度/CRC 正確（若有）

#### **Case B – 邊界**
- 空/0 長度/極值
- 期望：策略化處理（拒送或飽和），OUT=0 或 FAIL 可觀測

#### **Case C – 異常**
- NaN/Inf、時間回繞、下層 write()<len
- 期望：回 0 + FAIL，不得悄悄通過

#### **UART/封包（52B）加碼**
- 用 FakeTransport 收集 bytes
- 黃金樣本：把固定假資料序列化產出的 52B hex 硬編於測試，memcmp(expected, actual, 52)
- 再做 CRC 與「反序列化回原值」的誤差容忍比對

### 📊 **追蹤（TRACE）與假資料（FAKE）守則**

#### **TRACE_LEVEL 設定**
- **1**=模組層 IN/OUT
- **2**=含工具層（math）

#### **ENABLE_FAKE_DATA 控制**
- 預設 0；測試時才開
- 生產編譯若 ENABLE_FAKE_DATA==1 → 編譯失敗（#error 雙重保險）

#### **TDD 檢測器命名規則**
**目的**: 讓 TDD 檢測器的 **print 輸出內容** 符合 `模組名:函數名` 模式，便於識別和追蹤

**格式**: 第一參數使用 `"模組名:函數名"` + 後綴註解 `// 量測內容簡介`

**範例**:
```cpp
// ✅ 正確命名
static TDDFlowChecker board_support_checker("board_support:logMTIRawDataInHAL", "logMTIRawDataInHAL", 5000, true); // MTI原始數據記錄頻率監控
static TDDFlowChecker xbus_parser_checker("XBUSParser:parseFrame", "parseFrame", 5000, true); // IMU解析頻率監控
static TDDFlowChecker protocol_manager_checker("ProtocolManager:sendNavigation", "sendNavigation", 5000, true); // 協議輸出頻率監控

// ❌ 避免的命名
static TDDFlowChecker checker1("module", "func");  // 太模糊
static TDDFlowChecker data_checker("Data", "process");  // 不明確
```

**命名規範**:
1. **第一參數**: 使用 `"模組名:函數名"` 格式 (如 `"board_support:logMTIRawDataInHAL"`, `"XBUSParser:parseFrame"`)
2. **第二參數**: 重複函數名稱 (如 `"logMTIRawDataInHAL"`, `"parseFrame"`, `"sendNavigation"`)
3. **註解**: 簡潔描述監控目的，包含關鍵字如「頻率監控」、「數據記錄」、「解析」等
4. **變數名**: 使用 `模組名_功能_checker` 格式，統一後綴便於識別

#### **實際 Print 輸出效果**
遵循此命名規則後，TDD 檢測器將產生清晰的輸出格式：
```
board_support:logMTIRawDataInHAL IN=1 OUT=1         // MTI原始數據記錄
XBUSParser:parseFrame IN=1 OUT=1                    // IMU解析頻率  
DataFlowIntegrator:processIMU IN=1 OUT=1            // 數據整合
ProtocolManager:sendNavigation IN=1 OUT=1           // 協議輸出
IngressManager:process IN=1 OUT=1                   // 數據接收處理
```

#### **輸出格式統一**
所有輸出以 **模組名:函數名 + IN/OUT/FAIL** 統一格式，方便機器化比對和問題追蹤

### 🛠️ **預任工具（可重用片段）**

#### **FakeTransport 測試工具**
```cpp
class FakeTransport : public ITransport {
public:
  std::vector<uint8_t> last;
  size_t write(const uint8_t* buf, size_t n) override {
    last.assign(buf, buf + n); 
    return n;
  }
};
```

#### **52B 封包測試範例**
```cpp
REQUIRE(bytes.size() == 52);
REQUIRE(bytes[0]==0xFE && bytes[1]==0x81 && bytes[2]==0xFF && bytes[3]==0x55); // Header
REQUIRE(memcmp(bytes.data(), GOLDEN, 52) == 0); // 黃金樣本
REQUIRE(crc32_le(bytes.data(), 48) == readU32LE(&bytes[48])); // CRC 檢查
```

#### **TRACE 宏等級控制**
```cpp
#if TRACE_LEVEL >= 1
  #define TRACE_IO(mod,dir,ok)  Serial.printf("[TRACE] %s %s=%d\n", mod, dir, (int)((ok)?1:0))
  #define TRACE_FAIL(mod,why)   Serial.printf("[TRACE] %s FAIL:%s\n", mod, why)
#else
  #define TRACE_IO(mod,dir,ok)  do{}while(0)
  #define TRACE_FAIL(mod,why)   do{}while(0)
#endif
```

#### **FAKE 總開關保護**
```cpp
#if ENABLE_FAKE_DATA && defined(PRODUCTION_BUILD)
  #error "Fake data mode is ON in production build!"
#endif
```

---

## 🏗️ PROJECT OVERVIEW - MINSPixhawk

### 🎯 **PROJECT PURPOSE**
MINSPixhawk is an integrated framework that bridges **Xsens MTi-680 sensors** with **Pixhawk (PX4) flight control systems** using MAVLink protocol. This project enables external high-precision sensors to replace or augment PX4's built-in IMU/GPS as the primary navigation and state estimation data source.

### 🏛️ **CORE ARCHITECTURE PRINCIPLES**

#### 📊 **Data Flow Integrity**
- **Single Reader Pattern**: Only HAL layer reads from hardware Serial ports
- **No Rate Limiting**: Never throttle or limit sensor data throughput
- **Clean Data Streams**: Avoid interference with real-time sensor data flow
- **Monitor-First Design**: Use DataFlowMonitor for statistics, not inline detection

#### 🧩 **Modular Design Standards**  
- **LOG_TAG Requirement**: Every module must define unique LOG_TAG for structured logging
- **HAL Abstraction**: All hardware access through Hardware Abstraction Layer
- **Monitor Integration**: Statistics and health checks via centralized monitoring system
- **Clean Separation**: Business logic separate from hardware interface layer

#### 🎯 **Performance Guidelines**
- **Minimal Main Loop**: GMINS.ino stays clean, delegates to specialized modules
- **Efficient Monitoring**: Use background monitoring threads, not polling loops
- **Memory Management**: Static allocation preferred, avoid dynamic memory in loops
- **Interrupt Safety**: Hardware interrupts handled at HAL level only

### 🔧 **HARDWARE ARCHITECTURE**
- **Arduino MCU (SERCOM 配置)** with multiple UART ports:
  - `Serial (Debug)`: USB 序列監控器 (115200 baud) - 調試用
  - `Serial1 (SERCOM5)`: Pixhawk MAVLink 通訊 (460800 baud) - 🚀 已優化
  - `Serial2 (SERCOM2)`: Xsens MTi-680 感測器 (115200 baud) - MTI 100Hz 輸出
  - `Serial3 (SERCOM1)`: NMEA 輸出至 MTi-680 (115200 baud) - GNSS 輔助定位  
  - `Serial4 (SERCOM3)`: GNSS NMEA 輸入 (115200 baud) - LOCOSYS 4Hz 輸入

### 📡 **KEY FEATURES**
- **GNSS-AHRS 智能融合**: 動態信任機制，Trust Factor (0.1-1.0) 根據運動狀態調整
- **多格式感測器數據處理**: 方向、速度、加速度、角速度、磁場、GPS、溫度/壓力
- **MAVLink 訊息支援**: ODOMETRY (#331), GPS_RAW_INT (#24), GPS_INPUT (#232), TIMESYNC (#111)
- **多協定通訊**: MAVLink, NMEA (GGA/RMC/GST/GSA/VTG/ZDA/HDT/PLSHD/GSV), XBUS
- **座標轉換系統**: LLH ↔ ENU 轉換，四元數/DCM 處理
- **安全防護機制**: 多層認證、硬體完整性檢查、防 DoS 攻擊
- **AR-1A-FC GUI 整合**: 52-byte 二進制格式，CRC-32 校驗，100Hz 輸出
- **動態協方差調整**: 1e-6 ↔ 1e-1 範圍，即時 EKF 信任度控制

### 🧩 **MODULE STRUCTURE**
```
src/
├── hal/                     # 硬體抽象層 (Hardware Abstraction Layer)
│   ├── board_support.*      # 硬體初始化、UART 管理、監控整合
│   └── uart_interface.*     # UART 介面抽象 (可選)
├── util/                    # 核心工具模組
│   ├── log.h                # 結構化日誌系統 (LOG_TAG support)
│   ├── data_flow_monitor.*  # 資料流監控系統
│   └── gmins_frequency_manager.* # 頻率管理和統計
├── comm/                    # 通訊層
│   └── uart_interface.*     # UART 通訊抽象
├── core/                    # 核心演算法模組 (保留現有)
│   ├── gnss_ahrs_fusion.*   # GNSS-AHRS 融合演算法 (動態協方差)
│   └── Orientation.*        # 座標轉換 (LLH↔ENU, 四元數/DCM)
├── communication/           # 通訊協定模組 (保留現有)  
│   ├── myUARTSensor.*       # Xsens XBUS 協定解析器
│   ├── myMessage.*          # MAVLink 訊息封裝
│   └── uartRT.*             # 即時串列通訊處理
└── external/                # 外部庫
    └── Xsens/               # MTi-680 SDK

GMINS.ino                    # 主程式 (保持簡潔，僅初始化和主循環)
docs/                        # 技術文檔和資源
├── AR_1A_FC_GUI_INTEGRATION.md  # AR-1A-FC 整合文檔
├── GNSS_SATELLITE_DISPLAY.md    # GPS 衛星顯示功能
├── DATA_FLOW_MONITOR_README.md  # 資料流監控系統說明
└── PROGRESS_REPORT.md           # 開發進度報告
```


**技術模組記憶點：**
- `#xsens-debug` → XSENS 感測器調試狀態
- `#mavlink-config` → MAVLink 協定配置狀態
- `#nmea-parser` → NMEA 解析器開發狀態
- `#time-sync` → 時序同步機制開發狀態
- `#coordinate-transform` → 座標轉換演算法狀態

**問題解決記憶點：**
- `#checksum-fix` → NMEA checksum 修復進度
- `#frequency-issue` → 頻率問題解決方案
- `#buffer-overflow` → 緩衝區溢出解決狀態
- `#hardware-test` → 硬體測試結果記錄

**專案里程碑記憶點：**
- `#milestone-alpha` → Alpha 版本開發完成
- `#milestone-beta` → Beta 版本測試完成
- `#milestone-release` → 正式版本發布狀態
- `#integration-success` → 系統整合成功狀態

#### 🔧 **自定義斜杠指令**
> **使用方式**: 輸入 /指令名 載入特定開發環境

**技術模組指令：**
- `/xsens` → 載入 XSENS MTi-680 開發環境
- `/mavlink` → 載入 MAVLink 協定開發環境
- `/debug-nmea` → 載入 NMEA 調試工具環境
- `/hardware` → 載入硬體整合開發環境

**每個指令會自動載入：**
- 📋 相關技術規格和配置
- 🔧 硬體連接和設定資訊
- 🐛 常見問題和解決方案
- 📁 相關檔案和程式碼片段
- 🧪 測試工具和除錯指令

### 🎛️ **個人設定**
- **代碼風格**: C++/Arduino, 4空格縮排, 中英混合註解, CamelCase函數名, snake_case變數名
- **版本管理**: feature-* 分支策略, 中英混合commit訊息, 每日增量提交
- **工作習慣**: 嵌入式系統專注, 硬體整合導向, 即時資料處理, 模組化設計
- **工具偏好**: Arduino IDE, C++標準, MAVLink協定, 硬體抽象層設計

---

## 📝 **文檔筆記模板規範**

當用戶要求創建或更新技術文檔時，使用以下模板結構：

### **標準模板結構**
```markdown
# [模組名稱] - 實現狀態更新

> **狀態**: ✅/🔄/⏳ **[狀態描述]** (日期)  
> **階段**: [在整體架構中的位置]

---

## ✅ **當前實現狀況**

### 📊 **運行統計** (實測數據)
- **[關鍵指標]**: [數值] ([狀態描述])
- **處理效率**: [性能表現]

### 📋 **實際運行日誌** (如有)
\```log
[實際的系統日誌輸出，加上關鍵註解]
\```

### 🏗️ **已實現組件**
\```cpp
✅ [組件名稱]    // [功能描述]
\```

---

## 🔄 **數據流架構**

### **實際路徑**
\```mermaid
flowchart LR
    classDef io fill:#FFF1F0,stroke:#F5222D,rx:8,ry:8
    classDef adapter fill:#FFF7E6,stroke:#FA8C16,rx:8,ry:8
    classDef module fill:#F9F0FF,stroke:#722ED1,rx:8,ry:8
    
    [節點定義和連接]
\```

---

## 📤 **輸出接口定義**

### **當前階段輸出**
\```cpp
[具體的函數簽名或結構]
\```

### **下階段目標**
\```cpp
[下個階段需要的接口]
\```

**重要規格**:
- **坐標系**: [坐標系統]
- **單位**: [物理單位]
- **時間**: [時間基準]

---

## 🎯 **下一步: [下階段名稱]**

### **目標架構** (完整流程)
\```mermaid
flowchart TD
    classDef current fill:#E6F7FF,stroke:#1890FF,rx:8,ry:8
    classDef next fill:#F6FFED,stroke:#52C41A,rx:8,ry:8
    classDef output fill:#FFF0F6,stroke:#EB2F96,rx:8,ry:8
    
    [當前/下階段/輸出的節點和連接]
\```

### **需要實現**
1. **[任務1]**: [描述]
2. **[任務2]**: [描述]

### **準備就緒**
- ✅ [已完成的前置條件]

---

## 📋 **技術參考**

### **原始設計** vs **實際實現**
對比展示設計演進

---

**結論**: [總結當前狀態，下階段準備情況] 🚀
```

### **模板使用原則**
1. **簡潔明了** - 突出重點，避免冗餘
2. **視覺化** - 必須包含 mermaid 圖表
3. **實證數據** - 包含實際運行日誌和統計
4. **階段性** - 明確當前位置和下步目標
5. **規格明確** - 單位、坐標系、接口定義清晰

---

