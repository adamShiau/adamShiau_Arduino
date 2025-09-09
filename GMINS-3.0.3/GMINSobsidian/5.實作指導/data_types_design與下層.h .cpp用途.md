讚，現在就把三層的分工講清楚：**設計文件（data_types_design）→ 介面（.h）→ 實作（.cpp）**。你照這個切就很穩。

# 1) data_types_design（設計層 / 規格書）

**性質：** Markdown/Doc/註解為主，**不參與編譯**。  
**責任：**定義「長相與規範」，不碰記憶體與程式流程。

你要在這裡**一次講清楚**：

- 共同約定：座標系（NED/ENU/Body）、右手規則、角度單位（內部 rad）、時間基準（`timestamp_us` 單調時鐘）
    
- 資料模型：`IMUData / GNSSData / NavigationState` 欄位、單位、有效旗標（`flags`）、品質指標
    
- 相容性：`schema` 版本策略、向下/向上相容的規則
    
- 契約（Contract）：每個欄位「何時有效」、誤差範圍、NaN/缺值處理、邊界條件
    
- 測試要點：尺寸/對齊、角度包絡、四元數安全、Log 率限等
    
- 範例資料流圖：Driver→Filter→Fusion→Protocol→Transport
    

> 這是**唯一真相**的文件；.h/.cpp 都要跟它對齊。

超好的問題！把「什麼時候該動 `data_types.h`」講清楚，後面就不會亂改壞掉。下面是實務準則＋小清單。

# 什麼情況「需要」修改 `data_types.h`

1. **資料契約有變**（加欄位 / 改語意 / 改單位）
    
    - 新增要跨模組傳遞的數據（例如在 `NavigationState` 加 `acceleration_*`）。
        
    - 既有欄位的**單位或座標系**變更（例如 deg→rad、ENU→NED）。
        
    - 欄位語意改動（例如 `course_over_ground` 定義從「真北」改為「磁北」）。
        
2. **有效旗標（VALID flags）改變**
    
    - 新增/移除旗標（bitmask）。
        
    - 既有旗標的成立條件改變（要更新註解與設計書）。
        
    - 將新旗標納入/移出 `*_ALL_VALID`。
        
3. **相容性或版本控管**
    
    - 需要**保留向下相容**或宣告**破壞式變更**（bump `DATA_SCHEMA_VERSION`）。
        
    - 準備**棄用欄位**（加 `// DEPRECATED` 註解與替代方案，標註最後版本）。
        
4. **效能/記憶體配置原因**
    
    - 整體結構大小超過你的限制（例如 `NavigationState > 256 bytes`）→ 重整欄位或型別。
        
    - 對齊/填充造成浪費（調整欄位順序、用 `uint8_t` 取代多個 `bool`、改用 flags）。
        
5. **擴充新的資料領域**
    
    - 導入新的感測器或資料域，且需要**跨模組共享**（例如新增 `BaroData`、`VisionPoseData`）。
        

---

# 什麼情況「不需要」修改 `data_types.h`

- **演算法/計算方式**改了，但輸入/輸出**型態不變**（放 `.cpp` / `fusion/*.cpp`）。
    
- **驗證規則**調整（放 `validation.h/.cpp`）。
    
- **數學轉換**、角度包絡、四元數運算（放 `math_utils.h`）。
    
- **協定打包/傳輸**（MAVLink、自訂 52-byte 協議；放 `proto/*`）。
    
- **裝置/驅動設定**（Xsens/PLSHD；放 `drivers/*`）。
    
- **log 行為**或率限（放 `util/log.h` 或 `.cpp`）。
    

---

# 修改流程（建議固定化）

1. **先改規格書**：在 `data_types_design.md` 清楚寫「為何要改、欄位/單位/座標、成立條件、相容性影響」。
    
2. **決定是否破壞式**：
    
    - 只新增欄位/旗標 → 多半是**非破壞式**（schema 可不升級或小升）。
        
    - 改動型別/語意/單位 → **破壞式**（`DATA_SCHEMA_VERSION++`）。
        
3. **改 `data_types.h`**：
    
    - 加/改欄位、更新注釋（單位、座標、語意）、新增/調整 flags（用 OR 組合）。
        
    - 必要時調整欄位順序以控制大小與對齊。
        
4. **跑編譯期守門**：
    
    - `static_assert(sizeof(Struct) <= …)` 檢查。
        
    - 新枚舉/旗標值與 `*_ALL_VALID`。
        
5. **更新周邊橋接層**（不改 .h）：
    
    - MAVLink / 協議 pack 函式同步映射。
        
    - 驅動/融合讀寫那些欄位的地方。
        
6. **測試**：
    
    - 單元測試：序列化、flags 判斷、尺寸/對齊。
        
    - 串接測試：Producer→Consumer 走一遍。
        

---

# 判斷小抄（快速決策）

- **這個變更會不會影響別的模組「怎麼讀或寫資料」？**  
    會 → 改 `data_types.h`。不會 → 放實作層。
    
- **需要新旗標來表示有效性或狀態嗎？**  
    需要 → 加 bitmask。
    
- **只是計算方法更好/更快？**  
    是 → 不動 `data_types.h`。
    
- **單位或座標換了？**  
    是 → 一定要改 `data_types.h`（並加粗註解）。
    

---

# 範例

- 新增「GNSS 水平速度向量」→ 已有 `velocity_north/east/down`，**不用改**（用現有欄位）。
    
- 想把 `heading_accuracy` 由「度」改成「弧度」→ **要改**（單位是契約）。
    
- 想新增 `NAV_ACCELERATION_VALID` 旗標→ **要改**（加 bitmask；是否放入 `NAV_ALL_VALID` 視需求）。
    
- 換演算法，把姿態直接由四元數平滑→ **不用改**（型別不變）。

---

# 2) .h（介面層 / API 與型別）

**性質：**被別人 `#include` 的「**可見承諾**」。少依賴、明確邊界。  
**責任：**宣告「能用什麼」、型別長什麼樣，不實作細節。

常見內容：

- `data_types.h`：**型別定義**（struct/enum/常數、`schema`、`flags`），**不含業務邏輯**
    
- `log.h`、`math_utils.h`：**header-only** 小工具（`inline/constexpr`）
    
- 模組 API：`xsens.h`、`gnss.h`、`fusion.h`、`MAVLinkProtocol.h`
    
    - 函式/類別**宣告**、必要的前置型別
        
    - 外部可用的設定結構（`Config`）、返回碼（`Result`）、錯誤列舉
        
    - 需要共用的資料**用 `extern` 宣告**（唯一定義放 .cpp）
        

**設計原則：**

- **不 include 重物**：在 .h 盡量 forward declaration，實作再 `#include` 重型標頭（如 MAVLink C library）
    
- **不定義全域變數**：只 `extern`；避免 multiple definition
    
- **不暴露私有細節**：把 helper、buffer、狀態機藏到 .cpp
    
- **單一職責**：.h 專注在「能呼叫什麼」、輸入輸出是什麼
    

---

# 3) .cpp（實作層 / 行為與邏輯）

**性質：**每個 .cpp 是一個編譯單元；**藏細節**。  
**責任：**把 .h 的承諾做出來；管理狀態、連硬體、做演算法。

常見內容：

- 真正的**函式/方法實作**
    
- **內部狀態**（`static` 變數、環形緩衝、狀態機）
    
- **協定/硬體細節**（串口讀寫、MAVLink 打包/解析）
    
- `extern` 變數的**唯一定義**
    
- 單元測試用**內部 helper**（以 `static` 或匿名命名空間保護）
    

**實務守則：**

- **黏緊 contracts**：輸入前檢、輸出後檢（例如 flags/NaN/範圍）
    
- **率限/節流**：Log、串口 I/O 做速率控管
    
- **邊界安全**：長度檢查、除零防呆、浮點容差
    
- **可測性**：解析與 I/O 拆開（可離線餵檔測試）
    

---

# 放在你專案的地圖（建議）

```
/docs/data_types_design.md        ← 你的規格書（唯一真相）
/src/data/data_types.h            ← 型別與常數（不含策略）
// 小工具（header-only）
/src/util/log.h
/src/util/math_utils.h
// 驅動
/src/drivers/xsens.h
/src/drivers/xsens.cpp
/src/drivers/gnss.h
/src/drivers/gnss.cpp
// 融合/策略
/src/fusion/ekf.h
/src/fusion/ekf.cpp
// 協定/傳輸
/src/proto/MAVLinkProtocol.h
/src/proto/MAVLinkProtocol.cpp
// 全域共享（如需）
/src/core/globals.h        ← extern 宣告
/src/core/globals.cpp      ← 唯一定義
// 入口
/GMINS.ino                 ← setup/loop 串接流程（不寫細節）
```

---

# 各層的「Definition of Done」

**data_types_design.md**

- 座標/單位/時間/flags/schema/契約寫明，有資料流圖與測試要點
    
- 任一欄位的來源、消費者與生效條件可追溯
    

**.h**

- 無重型依賴、無全域定義、API 完整註解（單位、座標、有效條件、錯誤碼）
    
- 能被獨立 `#include` 編譯（最少 include）
    

**.cpp**

- 通過基本單元測試與串接測試
    
- 邊界檢查、率限生效；私有細節不外漏
    

---

# 一個超簡短例子（對照）

**MAVLinkProtocol.h**

```cpp
#pragma once
#include "src/data/data_types.h"
#include <stdint.h>

namespace proto {
struct MavCfg { uint8_t sysid=1, compid=200; };
void init(void* serial_port, const MavCfg& cfg);  // 不在 .h 引入重型串口類別
void send_nav(const datamodel::NavigationState& nav);
}
```

**MAVLinkProtocol.cpp**

```cpp
#include "MAVLinkProtocol.h"
#include "mavlink.h"         // 重型依賴放這裡
#include <Arduino.h>         // 實作才 include 硬體

namespace {
  HardwareSerial* s_port = nullptr;
  uint8_t sid=1, cid=200;
}

void proto::init(void* serial_port, const MavCfg& cfg){
  s_port = static_cast<HardwareSerial*>(serial_port);
  sid = cfg.sysid; cid = cfg.compid;
}
void proto::send_nav(const datamodel::NavigationState& nav){
  // 打包 nav → mavlink_message_t → 寫 s_port
}
```

---

## 一句話總結

- **design（規格）**決定「長相與規則」
    
- **.h（介面）**承諾「怎麼用」
    
- **.cpp（實作）**兌現「如何做」
    

你現在只要確保 `.h/.cpp` 完全遵守 `data_types_design` 的契約，後面擴充協定、替換感測器或搬演算法都會很順。需要的話，我可以把你的現有檔案樹貼上來，我幫你逐一標註該放什麼、哪裡要瘦身或拆分。