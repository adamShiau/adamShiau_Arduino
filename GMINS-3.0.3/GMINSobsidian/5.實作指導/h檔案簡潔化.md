● 📋 Header檔案複雜實現分析報告

  根據全面搜尋分析，發現以下.h檔案包含應該移到.cpp的複雜實現：

  ---
  🚨 高優先級問題（必須立即處理）

  1. ProtocolManagerDualMode.h

  問題函數:
  - continuousUpdate() - 30+行，包含LOG和static變數
  - switchToProtocol() - 70+行，大量LOG輸出和複雜邏輯

  2. xbus_parser.h ⚠️ 最嚴重

  問題函數:
  - parseXbusFrame() - 80+行複雜實現
  - parseDataPayload() - 300+行巨大函數！ 包含大量LOG和複雜計算

  3. multi_rate_publisher.h

  問題函數:
  - publish() - 20+行，包含迴圈和LOG
  - printStats() - 15+行，多個LOGI輸出

  ---
  🟡 中等優先級問題

  4. crc_calculator.h

  問題函數:
  - initTable() - 雙重迴圈計算CRC表
  - updateCRC32() - 迴圈處理邏輯

  5. minspixhawk_crc.h

  問題函數:
  - calCRC() - 20+行雙重迴圈計算
  - isCRCPass() - 10+行迴圈驗證

  6. ITransport.h

  問題函數:
  - read() - while迴圈實現
  - clearRxBuffer() - 包含LOG和迴圈

  ---
  🟢 低優先級（可考慮處理）

  7. parser_utils.h

  問題函數:
  - tokenize() - 25+行，static緩衝區和while迴圈
  - calculateNmeaChecksum() - for迴圈計算

  ---
  📊 統計摘要

  - 發現問題檔案: 7個
  - 需要處理的函數: 13個
  - 最嚴重問題: xbus_parser.h 的300行函數
  - 總計需移動代碼: 約500+行

  🎯 建議處理順序

  1. 緊急: xbus_parser.h (影響最大)
  2. 高優先級: ProtocolManagerDualMode.h
  3. 標準: CRC相關檔案
  4. 最後: 工具類函數

  您希望我先從哪一個檔案開始處理？

● 📋 需要重構的Header檔案詳細報告

  根據分析，以下是包含複雜實現需要移到.cpp的檔案清單：

  ---
  🚨 第一級：緊急處理

  1. /src/parsers/xbus_parser.h

  問題最嚴重 - 包含巨大的內聯函數
  - parseDataPayload() - 300+行 巨大函數
  - parseXbusFrame() - 80+行複雜邏輯
  - 包含大量LOG輸出、static變數、複雜迴圈

  2. /src/core/ProtocolManagerDualMode.h

  高頻調用函數
  - continuousUpdate() - 30+行，static變數，多LOG
  - switchToProtocol() - 70+行，大量LOG和錯誤處理

  ---
  🟡 第二級：重要處理

  3. /src/core/multi_rate_publisher.h

  發布器統計邏輯
  - publish() - 20+行迴圈和統計
  - printStats() - 15+行多LOG輸出

  4. /src/util/crc_calculator.h

  CRC計算邏輯
  - initTable() - 雙重迴圈初始化
  - updateCRC32() - 迴圈計算

  5. /src/util/minspixhawk_crc.h

  另一個CRC實現
  - calCRC() - 20+行雙重迴圈
  - isCRCPass() - 10+行驗證邏輯

  ---
  🟢 第三級：標準處理

  6. /src/transport/ITransport.h

  傳輸介面
  - read() - while迴圈實現
  - clearRxBuffer() - 迴圈+LOG

  7. /src/parsers/parser_utils.h

  解析工具
  - tokenize() - 25+行static緩衝+迴圈
  - calculateNmeaChecksum() - for迴圈計算

  ---
  📊 影響評估

  | 檔案                        | 函數數量 | 代碼行數 | 複雜度 | 優先級   |
  |---------------------------|------|------|-----|-------|
  | xbus_parser.h             | 2    | 380+ | 極高  | 🚨 緊急 |
  | ProtocolManagerDualMode.h | 2    | 100+ | 高   | 🚨 緊急 |
  | multi_rate_publisher.h    | 2    | 35+  | 中   | 🟡 重要 |
  | crc_calculator.h          | 2    | 25+  | 中   | 🟡 重要 |
  | minspixhawk_crc.h         | 2    | 30+  | 中   | 🟡 重要 |
  | ITransport.h              | 2    | 20+  | 低   | 🟢 標準 |
  | parser_utils.h            | 2    | 30+  | 低   | 🟢 標準 |

  🎯 建議處理順序

  1. xbus_parser.h - 最大影響，300行函數
  2. ProtocolManagerDualMode.h - 核心協議管理
  3. multi_rate_publisher.h - 發布器邏輯
  4. 其他CRC和工具類
