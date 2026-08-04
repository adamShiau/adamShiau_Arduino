# IRIS1 MCU 新硬體移植 — 修改與 Bug 追蹤紀錄

專案：IRIS1_MCU_V1.7（AP2 EM0）
本文件用途：本專案程式碼是從舊硬體架構 copy 過來的，目前正針對新硬體進行修改。此檔案用來持續記錄「改了哪裡、為什麼改、遇到什麼 bug、後來怎麼解」，方便追蹤與交接。

> 標示「〔觀察〕」的內容是我瀏覽程式碼後的初步發現，尚未經你確認是否為本次異動範圍或是否正確，請協助確認/修正。

---

## 1. 新硬體 vs 舊硬體 — 已知差異範圍

根據目前確認，主要異動有兩塊：

1. **UART / 通訊介面改變**
2. **從 FPGA 獲取的內容變更**（payload 內容/格式）

其餘細節（例如實際腳位、鮑率、payload 欄位定義如何變化）待補充。

---

## 2. 〔觀察〕程式碼中與硬體相關、可能需要留意的位置

這些是我初步檢視 `src/` 後找到、看起來與硬體綁定的地方，先列出來當作 checklist，實際是否需要修改請你確認。

### 2.1 UART 設定 — `src/myUART.cpp` / `src/myUART.h`

- 檔案註解已寫明「Minimal-change UART helpers for Arduino SAMD21 (e.g., Nano 33 IoT)」，看起來先前已經有一輪為了新板子（SAMD21 / Nano 33 IoT）做的搬移。
- 目前的埠號配置（`myUART.cpp` / `myUART.h` 註解）：
  | 名稱 | SERCOM | 用途 | 鮑率 | 腳位 |
  |---|---|---|---|---|
  | Serial（USB CDC） | - | Debug/PC | 230400 | - |
  | Serial1 | SERCOM5 | PC（由 Arduino core 提供） | 115200 | - |
  | Serial2 | SERCOM2 | 待確認 | 115200 | TX=24, RX=25 |
  | Serial3 | SERCOM1 | 待確認 | 115200 | TX=8, RX=13 |
  | Serial4 | SERCOM3 | **FPGA** | 115200 | TX=9, RX=10 (ALT) |
- 〔待確認〕新硬體的 UART 改變，是否就是指這份腳位/SERCOM 對應表？還是鮑率、協定格式（header/trailer）也有變？

### 2.2 從 FPGA 讀取資料 — `src/acq_imu.cpp`、`src/acq_ahrs.cpp`、`src/common.cpp`

- FPGA 資料走 `Serial4`，透過 `readDataStream(HDR_ABBA, 2, TRL_5556, 2, SENSOR_PAYLOAD_LEN, &try_cnt)` 解析，`SENSOR_PAYLOAD_LEN = 44` bytes（11 個 float，不含 header/CRC）。
- `common.cpp: update_raw_data()`（約 line 160）用**固定順序**把這 44 bytes 拆進 `my_sensor_t`：
  1. `fog.fogz.step`
  2. `fog.fogy.step`
  3. `fog.fogx.step`
  4. `adxl357.ax`
  5. `adxl357.ay`
  6. `adxl357.az`
  7. `temp.tempz`
  8. `temp.tempy`
  9. `temp.tempx`
  10. `adxl357.temp`
  11. `time.time`
- 〔待確認〕「從 FPGA 獲取內容變更」指的是不是這 11 個欄位的**順序、數量、或定義**有變？如果新 FPGA 韌體輸出的欄位順序/內容不同，這個函式（以及對稱的 `pack_sensor_payload_from_cali()`，約 line 1488）就是需要同步修改的地方。
- `acq_imu.cpp` / `acq_ahrs.cpp` 兩處都各自呼叫 `readDataStream` + `update_raw_data`，兩邊都要一起改，避免漏掉其中一個。

### 2.3 其他待你補充的地方

- 感測器型號/軸向定義是否也隨新硬體改變（例如 `軸向說明.pptx` 是否需要更新）？
- 電源/reset 電路（`acq_rst.cpp`）是否也有異動？

---

## 3. 修改記錄

| 日期 | 檔案 / 位置 | 修改內容 | 原因（新硬體差異） | 狀態 |
|---|---|---|---|---|
| | | | | |

## 4. Bug 追蹤

| 日期 | 現象 / 錯誤訊息 | 相關檔案 / 函式 | 推測原因 | 目前狀態 / 解法 |
|---|---|---|---|---|
| | | | | |

## 5. 待確認事項（Open Questions）

- [ ] UART 改變的具體內容（腳位？鮑率？協定？）
- [ ] FPGA payload 新格式的完整定義（欄位順序、新增/刪除欄位、資料型態）
- [ ] 是否有新硬體的 datasheet / pinout 文件可參考

---

*最後更新：2026-08-04*
