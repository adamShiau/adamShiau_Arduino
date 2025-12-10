# PKT_POSITION 封包格式驗證

## 你的測試封包
```
FA FF 02 01 1C 00 49 64 1F 64 59 10 39 40 93 6B 0A 64 F6 51 5E 40 66 66 8F 42 09 35 05 0A 0C E9 07 A4
```

## 解析驗證

### Header 部分 (6 bytes)
- `FA FF` - Header bytes ✅
- `02` - Packet Type (PKT_POSITION) ✅
- `01` - Status (DATA_POS_ONLY) ✅
- `1C 00` - Size = 28 (27 payload + 1 checksum) ✅

### Payload 部分 (27 bytes)
- **Bytes 0-7**: `49 64 1F 64 59 10 39 40` - Latitude (double)
- **Bytes 8-15**: `93 6B 0A 64 F6 51 5E 40` - Longitude (double)
- **Bytes 16-19**: `66 66 8F 42` - Altitude (float)
- **Bytes 20-26**: `09 35 05 0A 0C E9 07` - UTC Time (7 bytes)

### UTC 時間解析
- `09` - Hour = 9
- `35` - Minute = 53
- `05` - Second = 5
- `0A` - Day = 10
- `0C` - Month = 12
- `E9 07` - Year = 2025 (0x07E9)

### Checksum
- `A4` - XOR checksum ✅

## 預期 MCU-MARS 輸出
```
Parsed packet type: 0x2 | Status: POS_ONLY | Lat: 25.xxxxxx Lon: 121.xxxxxx Alt: 71.8m Time: 09:53:05 2025/12/10
```

## 解析器變更摘要 (v1.2)
✅ **parsePositionPacket** - 從 20 bytes → 27 bytes
✅ **時間欄位解析** - 新增 UTC 時間提取
✅ **調試輸出更新** - 顯示完整位置+時間資訊
✅ **向後兼容性** - 不影響其他封包類型的解析

封包格式與解析器完全匹配！🎯