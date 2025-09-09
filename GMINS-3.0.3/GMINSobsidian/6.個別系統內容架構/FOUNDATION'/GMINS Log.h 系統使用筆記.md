
---

## 快速觀念

- **位元遮罩（mask）** 精準選要顯示的等級：`LOG_BIT_DEBUG/INFO/WARN/ERROR/CRITICAL`。
    
- **每支 .cpp/.ino** 在 `#include "src/util/log.h"` **之前** 定義：`LOG_TAG`、`LOG_LEVEL_MASK_LOCAL`（可選）、`HEARTBEAT_ENABLE`（可選）。
    
- 預設門檻 `LOG_LEVEL_GLOBAL = LOG_INFO`（在 `log.h` 內），主要以 **遮罩** 為主控。
    
- **輸出埠** 可在編譯期或執行期選擇：`LOG_DEFAULT_PORT` / `LOG_SET_PORT(...)`。
    

---

## 每檔最小模板

```
#define LOG_TAG "MAIN"                    // 本檔標籤
// #define LOG_LEVEL_MASK_LOCAL (...)      // 可選：本檔遮罩，不設則用全域預設
#include "src/util/log.h"
```

> 沒定義 `LOG_LEVEL_MASK_LOCAL` 時，會採用 `LOG_LEVEL_MASK_GLOBAL`（預設 I/W/E/C）。

---

## 遮罩快速表（選你要的字母）

- `LOG_BIT_DEBUG`
    
- `LOG_BIT_INFO`
    
- `LOG_BIT_WARN`
    
- `LOG_BIT_ERROR`
    
- `LOG_BIT_CRITICAL`
    

**範例:**

```
// 只要 INFO + ERROR
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_INFO | LOG_BIT_ERROR)

// 全開（D/I/W/E/C）
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_DEBUG | LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)

// 只要 WARNING 以上（W/E/C）
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)

// 只要 DEBUG + ERROR（忽略 I/W/C）
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_DEBUG | LOG_BIT_ERROR)

// 完全靜音（本檔）
#define LOG_LEVEL_MASK_LOCAL 0
```

---

## 心跳（Heartbeat）

- 開啟：`#define HEARTBEAT_ENABLE`，使用：`HEARTBEAT(period_ms)`。
    
- **預設**：走 `LOG_INFO`，受遮罩/門檻影響。
    
- **只想要心跳**（其他全關）→ 繞過過濾：
    

```
#define LOG_LEVEL_MASK_LOCAL 0
#define HEARTBEAT_ENABLE
#define HEARTBEAT_BYPASS_FILTER   // 讓 heartbeat 不受遮罩/門檻影響
#include "src/util/log.h"
```

---

## 輸出埠選擇

**編譯期設定預設埠**（在包含 `log.h` 前）：

```
#define LOG_DEFAULT_PORT Serial1
#include "src/util/log.h"
```

**執行期切換**（例如在 `setup()`）：

```
Serial1.begin(921600);
LOG_SET_PORT(Serial1);
```

> 預設使用 `Serial`；非 Arduino 平台走 `stdout`。

---

## 全域預設 vs. 本檔覆寫

若你不想每檔都寫遮罩，可以集中在 `src/util/log_config.h`：

```
// src/util/log_config.h
#pragma once
#define LOG_LEVEL_MASK_GLOBAL (LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)
// （可選）也可設定門檻：#define LOG_LEVEL_GLOBAL LOG_DEBUG
```

各檔使用：

```
#define LOG_TAG "IMU"
#include "src/util/log_config.h"
#include "src/util/log.h"
// 本檔若要再放寬：
// #define LOG_LEVEL_MASK_LOCAL (LOG_LEVEL_MASK_GLOBAL | LOG_BIT_DEBUG)
```

---

## 常用巨集

- 印出：`LOGD/LOGI/LOGW/LOGE/LOGC`
    
- 率限印出：
    

```
LOG_EVERY_MS(LOG_INFO, 5000, "Alive: %lu ms", millis());
```

- 十六進位傾印：
    

```
LOG_HEX(LOG_DEBUG, buffer, length);
```

- 區塊計時：
    

```
{
  SCOPED_TIMER("sensor_update");
  // do work ...
}
```

- 原味輸出（不帶前綴，走同一輸出埠）：
    

```
LOG_RAW("raw value=%d", v);
```

---

## 情境小抄

**(1) 模組只看警告以上**

```
#define LOG_TAG "FUSION"
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)
#include "src/util/log.h"
```

**(2) 除錯中特定檔全開**

```
#define LOG_TAG "GNSS"
#define LOG_LEVEL_MASK_LOCAL (LOG_BIT_DEBUG | LOG_BIT_INFO | LOG_BIT_WARN | LOG_BIT_ERROR | LOG_BIT_CRITICAL)
#include "src/util/log.h"
```

**(3) 發布前完全靜音**

```
#define LOG_TAG "TX"
#define LOG_LEVEL_MASK_LOCAL 0
#include "src/util/log.h"
```

**(4) 長期監控：只要心跳**

```
#define LOG_TAG "MON"
#define LOG_LEVEL_MASK_LOCAL 0
#define HEARTBEAT_ENABLE
#define HEARTBEAT_BYPASS_FILTER
#include "src/util/log.h"
```

---

## 疑難排解

- **沒有輸出**：
    
    1. 確認 `LOG_LEVEL_MASK_LOCAL` 或 `LOG_LEVEL_MASK_GLOBAL` 是否允許該等級。
        
    2. Arduino 記得 `Serial.begin(...)`；若走 `Serial1`，要 `LOG_SET_PORT(Serial1)`。
        
    3. AVR 平台 `vsnprintf` 可能不支援 `%f`，請改用 `Serial.print(value, 2)` 組字串後輸出。
        
- **想全專案一鍵關閉**：在 `log_config.h` 設 `#define LOG_LEVEL_MASK_GLOBAL 0`，或各檔設 `LOG_LEVEL_MASK_LOCAL 0`。
    

---

## 版本對應

- `log.h`：位元遮罩 + 可切換輸出埠 + 心跳可繞過（本筆記版本）。
    

> 建議：將本筆記存放於 `docs/GMINS_Log_Notes.md` 或專案根目錄 README 區塊，方便團隊共用。