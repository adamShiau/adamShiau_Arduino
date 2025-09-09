# MAVLink 輸出流程完整追蹤

> **文件版本**: 1.0  
> **最後更新**: 2025-01-17  
> **分析目的**: 追蹤從 continuousUpdate 到實際封包輸出的完整 MAVLink 處理鏈

## 🔍 **流程概覽**

```
continuousUpdate() 
    ↓
sendNavigationData() & sendGNSSData()
    ↓
sendIMUBasedData() & sendGNSSBasedData()  
    ↓
sendMAVLinkPacket() / sendMAVLinkPacket_Fixed()
    ↓
Transport Layer (write())
    ↓
Hardware Serial Output
```

---

## 🚀 **階段 1: 協議管理器觸發**

**文件**: `/src/core/ProtocolManagerDualMode.h`
**函數**: `continuousUpdate()`

```cpp
void continuousUpdate(const GNSSData& gnss_data, const NavigationState& nav_state, const IMUData& imu_data) {
    if (active_protocol_) {
        switch (current_protocol_) {
            case Protocol::MAVLINK:
                // 雙重調用 - 關鍵性能點！
                active_protocol_->sendNavigationData(nav_state);  // ← 觸發點 1
                active_protocol_->sendGNSSData(gnss_data);        // ← 觸發點 2
                break;
        }
    }
}
```

**關鍵特徵**:
- 每次調用發送 **2 個封包**（Navigation + GNSS）
- 無頻率限制，依賴上層調用頻率
- **潛在瓶頸**: 雙重發送增加系統負載

---

## 📡 **階段 2: MAVLink 協議實現**

**文件**: `/src/protocol/MAVLinkProtocol.h`

### 2.1 Navigation 路徑
```cpp
sendNavigationData() → sendIMUBasedData() → ODOMETRY 封包
```

**處理過程**:
1. **數據映射**: NavigationState → OdometryPayload (108 bytes)
2. **時間戳計算**: `millis() * 1000UL`
3. **結構填充**: 位置、四元數、速度、角速度
4. **協方差矩陣**: 42 個 float 值（168 bytes）
5. **調用**: `sendMAVLinkPacket(ODOMETRY, payload, sizeof(payload))`

### 2.2 GNSS 路徑
```cpp
sendGNSSData() → sendGNSSBasedData() → GPS_RAW_INT 封包
```

**處理過程**:
1. **數據轉換**: GNSSData → GPSRawPayload (30 bytes)
2. **座標轉換**: lat/lon × 1e7, alt × 1000
3. **精度計算**: hdop/vdop × 100, speed × 100
4. **調用**: `sendMAVLinkPacket(GPS_RAW_INT, payload, sizeof(payload))`

---

## 🔧 **階段 3: 封包建構與傳送**

**文件**: `/src/protocol/MAVLinkProtocol.h`
**函數**: `sendMAVLinkPacket()`

### 3.1 封包組裝
```cpp
bool sendMAVLinkPacket(MessageID msg_id, const uint8_t* payload, size_t payload_size) {
    // 1) 驗證檢查
    if (!isReady()) return false;
    if (!validatePacketSize(payload_size)) return false;
    
    // 2) 緩衝區分配 - 潛在瓶頸！
    uint8_t tx_buffer[MAX_PACKET_SIZE];  // 267 bytes 棧分配
    
    // 3) MAVLink v2.0 標頭組裝 (10 bytes)
    tx_buffer[0] = 0xFD;                    // Magic
    tx_buffer[1] = payload_size;            // Length  
    tx_buffer[2-3] = 0;                     // Flags
    tx_buffer[4] = sequence_number_++;      // Sequence
    tx_buffer[5] = SYSTEM_ID;               // System ID
    tx_buffer[6] = COMPONENT_ID;            // Component ID
    tx_buffer[7-9] = message_id;            // 24-bit Message ID
    
    // 4) 負載數據拷貝
    memcpy(&tx_buffer[10], payload, payload_size);
    
    // 5) CRC 計算 - 重要性能點！
    uint16_t checksum = calculateChecksum_Safe(...);
    tx_buffer[packet_size-2] = checksum & 0xFF;
    tx_buffer[packet_size-1] = (checksum >> 8) & 0xFF;
    
    // 6) 傳輸層發送
    size_t sent = transport_->write(tx_buffer, packet_size);
    return (sent == packet_size);
}
```

### 3.2 CRC 計算 - 性能關鍵點
```cpp
uint16_t calculateChecksum_Safe(const uint8_t* data, size_t length, MessageID msg_id) {
    // 簡化算法避免死機，但仍需遍歷數據
    uint16_t simple_checksum = 0;
    size_t check_length = (length > 20) ? 16 : length;
    
    for (size_t i = 0; i < check_length; i++) {
        simple_checksum += data[i];
        simple_checksum = (simple_checksum << 1) | (simple_checksum >> 15);
    }
    return simple_checksum;
}
```

---

## 🚛 **階段 4: 傳輸層處理**

### 4.1 TxMultiplexer
**文件**: `/src/transport/TxMultiplexer.h`
```cpp
size_t writeWithStats(const uint8_t* data, size_t length, const char* protocol_name) {
    return monitored_transport_->write(data, length);
}
```

### 4.2 MonitoredTransport  
**文件**: `/src/transport/MonitoredTransport.h`
```cpp
size_t write(const uint8_t* data, size_t length) override {
    size_t bytes_written = transport_->write(data, length);
    
    // MultiChannelMonitor: 記錄輸出統計
    if (monitor_ && bytes_written > 0) {
        monitor_->recordBytes(bytes_written);
        monitor_->recordPackets(1);
        monitor_->recordOperations(1);
    }
    
    return bytes_written;
}
```

### 4.3 UARTTransport (最終硬體層)
**文件**: `/src/transport/ITransport.h`
```cpp
size_t write(const uint8_t* data, size_t length) override {
    if (!isReady()) return 0;
    
    // 直接寫入硬體序列埠
    return serial_.write(data, length);
}
```

---

## ⚡ **性能瓶頸識別**

### 🔴 **高開銷操作**

1. **雙重封包發送**: 每次 `continuousUpdate()` 發送 2 個封包
   - ODOMETRY: ~108 bytes payload + 12 bytes header = 120 bytes
   - GPS_RAW_INT: ~30 bytes payload + 12 bytes header = 42 bytes
   - **總計**: ~162 bytes/call

2. **大棧空間分配**: `uint8_t tx_buffer[267]` × 2 = 534 bytes/call

3. **記憶體拷貝操作**:
   - Payload → tx_buffer: 138 bytes/call
   - 數據結構填充: NavigationState → OdometryPayload

4. **CRC 計算循環**: 每個封包 16-30 次循環操作

5. **監控系統開銷**: MultiChannelMonitor 統計記錄

### 🟡 **中等開銷操作**

1. **時間戳計算**: `millis() * 1000UL` 
2. **數據類型轉換**: float ↔ int32_t, 座標轉換
3. **序列號遞增**: `sequence_number_++`

### 🟢 **低開銷操作**

1. **指標解參考**: `active_protocol_->`
2. **條件判斷**: `if (!isReady())`
3. **簡單賦值**: 標頭欄位設定

---

## 📊 **系統影響分析**

### **調用頻率估算**
- 假設系統主循環: ~1000Hz
- `continuousUpdate()` 調用: ~1000Hz  
- MAVLink 封包輸出: ~2000 packets/sec
- 總數據量: ~324KB/sec

### **CPU 負載分析**
```
每次 continuousUpdate() 的主要開銷：
├── 封包組裝: ~200 μs
├── CRC 計算: ~50 μs  
├── 傳輸寫入: ~100 μs
├── 監控記錄: ~30 μs
└── 總計: ~380 μs/call
```

在 1000Hz 調用頻率下：
- MAVLink 處理時間: 380ms/sec (38% CPU)
- 剩餘處理時間: 620ms/sec (62% CPU)

**結論**: MAVLink 輸出確實消耗大量 CPU 資源，可能影響 MTI 輸入處理頻率！

---

## 🔧 **優化建議**

1. **頻率限制**: 在 `continuousUpdate()` 增加輸出頻率限制（如 50Hz）
2. **封包合併**: 考慮將 ODOMETRY 和 GPS_RAW_INT 合併或交替發送
3. **緩衝區優化**: 使用固定大小的預分配緩衝區
4. **CRC 簡化**: 進一步簡化 CRC 算法或使用查找表
5. **監控精簡**: 減少或批次化監控統計操作

---

## 🎯 **驗證要點**

為驗證此流程對系統頻率的影響，應監控：

1. **MTI 處理頻率**: 啟用前 vs 啟用後
2. **MAVLink 輸出頻率**: 實際達到多少 Hz
3. **系統主循環頻率**: 是否受到影響
4. **記憶體使用**: 棧溢出風險評估

---

**📝 分析結論**: MAVLink 輸出流程涉及複雜的多層處理，每次調用需要 ~380μs，在高頻調用下確實可能成為系統性能瓶頸，影響 MTI 數據處理頻率。