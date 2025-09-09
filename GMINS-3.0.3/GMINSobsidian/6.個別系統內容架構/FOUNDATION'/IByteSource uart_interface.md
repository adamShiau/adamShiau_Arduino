太好了～走 **「被餵食的 ring buffer」** 這條線。下面是我幫你整理好的「製作過程」給 Claude 直接照做。內容依你 Obsidian 習慣用 **Markdown + Mermaid** 撰寫；每一步都有檔案路徑、介面定義、實作重點、驗收標準與小心事項。

---

# #️⃣ 目標

把 `uart_interface.*` 從「直接讀硬體」改造成**被 HAL 餵食的 ring buffer**，讓下游 Parser/Router 仍用 `available()/read()` 拉資料，同時維持 **HAL 單一讀者** 架構，不再有雙重讀取衝突。

---

# 🧭 分層架構（最終形態）

```mermaid
flowchart TD
  %% 樣式
  classDef io fill:#FFF1F0,stroke:#F5222D,rx:8,ry:8
  classDef adapter fill:#FFF7E6,stroke:#FA8C16,rx:8,ry:8
  classDef module fill:#F9F0FF,stroke:#722ED1,rx:8,ry:8
  classDef mon fill:#F6FFED,stroke:#52C41A,rx:8,ry:8

  subgraph HW[IO / 硬體 UART]
    S2[Serial2 / IMU(MTI)]:::io
    S4[Serial4 / GNSS(NMEA)]:::io
  end

  subgraph HAL[board_support（唯一讀者）]
    P[pollOnce(): 讀 bytes → sink]:::module
    M[DataFlowMonitor / FrequencyManager]:::mon
  end

  subgraph ADP[Adapter（餵食式轉接器）]
    UIF_IMU[UARTInterface(IMU)<br/>feed-based RingBuffer]:::adapter
    UIF_GNSS[UARTInterface(GNSS)<br/>feed-based RingBuffer]:::adapter
  end

  subgraph NEXT[Parser/Router]
    IMUP[IMU XBUS Parser]:::module
    GNSSP[GNSS NMEA Parser]:::module
  end

  S2 --> P --> UIF_IMU --> IMUP
  S4 --> P --> UIF_GNSS --> GNSSP
  P -.統計.-> M
```

---

# ✅ 變更總覽（給 Claude）

## 1) 新增/重寫：抽象讀源介面

**檔案**：`src/comm/IByteSource.h`

```cpp
#pragma once
#include <cstddef>
#include <cstdint>

struct IByteSource {
  virtual ~IByteSource() = default;
  virtual size_t available() const = 0;
  virtual size_t read(uint8_t* dst, size_t n) = 0; // 回傳實際讀取的 byte 數
};
```

---

## 2) 重寫：`uart_interface.h`（改為 feed-based）

**檔案**：`src/comm/uart_interface.h`

```cpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include "IByteSource.h"

#ifndef UART_IF_RING_SIZE
#define UART_IF_RING_SIZE 8192  // IMU 建議 >= 8KB；GNSS 4~8KB
#endif

// 單執行緒假設：feed() 與 read() 在主循環呼叫；如需 ISR 安全，之後再加鎖。
class UARTInterface final : public IByteSource {
public:
  UARTInterface() = default;

  // 舊 initialize 不再觸碰硬體；保留為 no-op 以減少改動面
  bool initialize(uint32_t /*baud*/ = 0) { return true; }

  // ★ HAL sink 餵資料入口
  void feed(const uint8_t* data, size_t len);

  // IByteSource
  size_t available() const override;
  size_t read(uint8_t* dst, size_t n) override;

  // 可選：TX 由外部注入（例如走 HAL 寫到 Serial1）
  void setTx(std::function<size_t(const uint8_t*, size_t)> tx) { tx_ = std::move(tx); }
  size_t write(const uint8_t* src, size_t n) { return tx_ ? tx_(src, n) : 0; }

  // 監控用：簡單統計（僅 bytes）
  struct Stats { uint64_t total_in = 0, total_out = 0, drops = 0; };
  Stats getStats() const { return stats_; }

  // 溢位策略：true=丟舊資料並插入新資料（建議）；false=丟新資料
  void setDropOldestOnOverflow(bool v) { drop_oldest_ = v; }

private:
  // 簡易 ring buffer（單生產者/單消費者）
  uint8_t buf_[UART_IF_RING_SIZE]{};
  size_t head_ = 0, tail_ = 0; // head=寫入位置, tail=讀取位置
  bool drop_oldest_ = true;
  std::function<size_t(const uint8_t*, size_t)> tx_;
  Stats stats_{};

  size_t freeSpace() const;
  size_t usedSpace() const;
  void pushByte(uint8_t b);
};
```

---

## 3) 新增/重寫：`uart_interface.cpp`

**檔案**：`src/comm/uart_interface.cpp`

```cpp
#include "uart_interface.h"
#include <cstring>

size_t UARTInterface::freeSpace() const {
  return UART_IF_RING_SIZE - usedSpace() - 1; // 空一格避開head==tail歧義
}

size_t UARTInterface::usedSpace() const {
  if (head_ >= tail_) return head_ - tail_;
  return UART_IF_RING_SIZE - (tail_ - head_);
}

void UARTInterface::pushByte(uint8_t b) {
  size_t next = (head_ + 1) % UART_IF_RING_SIZE;
  if (next == tail_) {
    // 滿了
    if (drop_oldest_) {
      // 丟最舊：前進 tail
      tail_ = (tail_ + 1) % UART_IF_RING_SIZE;
      stats_.drops++;
    } else {
      // 丟最新：直接放棄
      stats_.drops++;
      return;
    }
  }
  buf_[head_] = b;
  head_ = next;
}

void UARTInterface::feed(const uint8_t* data, size_t len) {
  if (!data || !len) return;
  // 快速路徑：如果空間足夠，做一次或兩次 memcpy（處理環狀收尾）
  size_t space = freeSpace();
  if (space >= len && head_ >= tail_) {
    size_t tail_free = UART_IF_RING_SIZE - head_;
    if (tail_free >= len) {
      std::memcpy(&buf_[head_], data, len);
      head_ = (head_ + len) % UART_IF_RING_SIZE;
    } else {
      std::memcpy(&buf_[head_], data, tail_free);
      std::memcpy(&buf_[0], data + tail_free, len - tail_free);
      head_ = (len - tail_free);
    }
    stats_.total_in += len;
    return;
  }
  // 慢路徑：逐 byte push（處理溢位策略）
  for (size_t i = 0; i < len; ++i) pushByte(data[i]);
  stats_.total_in += len;
}

size_t UARTInterface::available() const {
  return usedSpace();
}

size_t UARTInterface::read(uint8_t* dst, size_t n) {
  if (!dst || n == 0) return 0;
  size_t cnt = 0;
  while (cnt < n && tail_ != head_) {
    dst[cnt++] = buf_[tail_];
    tail_ = (tail_ + 1) % UART_IF_RING_SIZE;
  }
  stats_.total_out += cnt;
  return cnt;
}
```

> 注意：這個版本**完全不包含** `Arduino.h` / `HardwareSerial`，不會碰硬體。

---

## 4) 串接 HAL：在 sink 內 `feed()` 到介面

**檔案**：`src/hal/board_support.cpp`（你已有 attach sink，這裡只要把介面接起來）

```cpp
#include "board_support.h"
#include "../comm/uart_interface.h"

// 全域或靜態，讓 Parser 可拿到
static UARTInterface gIMU_IF;
static UARTInterface gGNSS_IF;

// setup 或初始化時：掛上 sink，餵資料
void wireUartInterfaces() {
  hal::attachIMUSink([](const uint8_t* d, size_t n, void*){
    gIMU_IF.feed(d, n);
  }, nullptr);

  hal::attachGNSSSink([](const uint8_t* d, size_t n, void*){
    gGNSS_IF.feed(d, n);
  }, nullptr);

  // 可選：指定溢位策略
  gIMU_IF.setDropOldestOnOverflow(true);
  gGNSS_IF.setDropOldestOnOverflow(true);
}

// 提供 Getter 讓 Parser/Router 取得來源
UARTInterface* getIMUSource()  { return &gIMU_IF;  }
UARTInterface* getGNSSSource() { return &gGNSS_IF; }
```

> 在 `setup()` 或 `initPeripherals()` 後呼叫 `wireUartInterfaces()` 一次即可。

---

## 5) Parser 接法（不改呼叫風格）

**示意**：`parser_gnss.cpp`

```cpp
void loopParserGNSS() {
  auto* src = getGNSSSource();   // 取得被餵食介面
  uint8_t buf[256];
  while (src->available() > 0) {
    size_t n = src->read(buf, sizeof(buf));
    // 這裡做 NMEA framing（累積到 \n / \r\n）
    gnssFramer.feed(buf, n);
  }
}
```

**示意**：`parser_mti.cpp`

```cpp
void loopParserMTI() {
  auto* src = getIMUSource();
  uint8_t buf[256];
  while (src->available() > 0) {
    size_t n = src->read(buf, sizeof(buf));
    // 這裡做 XBUS framing（找 0xFA 0xFF + len）
    mtiFramer.feed(buf, n);
  }
}
```

---

## 6) 移除舊硬體讀取行為（避免「雙讀」）

- 刪除或停用舊版 `uart_interface.cpp` 中所有 `HardwareSerial` 相關程式碼（`begin()/read()/write()` 等）。
    
- **確保**沒有其他地方還在 `Serial2/Serial4.read*()`（可用 grep 或 CI 規則擋掉）。
    
- 保留 TX 的情境（例如 MAVLink 走 Serial1）→ 透過 `UARTInterface::setTx()` 注入一個寫入函式（呼叫 HAL 實作）即可。
    

---

# 🧪 測試與驗收

## 單元測試（host/emb）

1. **Ring 行為**
    
    - 餵入 `N=1000` byte，`available()==1000`，讀出 1000，FIFO 順序一致。
        
    - 餵入 `N=RING_SIZE+100`，若 `drop_oldest=true`，`drops>0` 且 `available()==RING_SIZE-1`；資料尾段保留。
        
2. **零拷貝快路徑**
    
    - 單次 `feed()` 長度小於尾端剩餘空間，應走 memcpy（非逐 byte）。可用時間或內部計數驗證。
        

## 整合驗收（板上）

1. 開機後 `hal::pollOnce()` 在跑；**不再存在任何 `SerialX.read*()` 除 HAL**。
    
2. HAL monitor 顯示：
    
    - **IMU**：bytes/sec ≈ 5 KB/s（100 Hz、~50B/包）
        
    - **GNSS**：bytes/sec 依設定（1–10 Hz，數百～數千 B/s）
        
3. FrequencyManager 報表：
    
    - **MTI**：接近 100 Hz
        
    - **GNSS**：接近設定值（例如 6 Hz）
        
4. Parser 能正常組包（XBUS/ NMEA），無漏包/亂序。
    

---

# ⚠️ 注意事項

- `feed()` 為熱路徑，**不要**在裡面做 log 或重運算；統計交給現有 Monitor。
    
- Ring 溢位策略：**GNSS 建議丟最舊**（避免卡死在舊資料），IMU 依需求可調整。
    
- 如果未來要從 **檔案回放**，直接造一個 `FileReplaySource : IByteSource`，Parser 介面不需改。
    
- 若之後引入 ISR 層 `feed()`，再補自旋鎖或禁用中斷的臨界區；目前設計在主循環呼叫，無需鎖。
    

---

# 🗂 檔案與步驟清單（給 Claude 執行）

1. 新增 `src/comm/IByteSource.h`（如上）。
    
2. 覆蓋/重寫 `src/comm/uart_interface.h` 與 `.cpp`（如上）。
    
3. 在 `src/hal/board_support.cpp`：
    
    - 新增 `wireUartInterfaces()` 與 `getIMUSource()/getGNSSSource()`（如上）。
        
    - 在初始化流程中呼叫 `wireUartInterfaces()`。
        
4. 刪除舊 `uart_interface.cpp` 任何 `HardwareSerial` 讀寫邏輯（保留可選 TX 綁定）。
    
5. 確認 **無其他檔案**使用 `Serial2/Serial4.read*()`（若有，改為從 `UARTInterface` 或 Hub 讀）。
    
6. 跑板上測試，檢查 Monitor 與 Frequency 報表是否符合預期。
    

---

需要我再補「XBUS/NMEA Framer 的最小骨架」嗎？我可以加上 `feed()` 狀態機模板，直接能接你現在的 `UARTInterface`。