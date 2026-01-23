// src/drivers/link/hins_link.cpp
#include "hins_link.h"
#include <Arduino.h>
// #include "../../common.h"
#include "../../utils/endian.h"

// ---- MIP constants (from HBK/MicroStrain DCP examples) ----
static constexpr uint8_t MIP_SYNC1    = 0x75;
static constexpr uint8_t MIP_SYNC2    = 0x65;
static constexpr uint8_t MIP_BASE_SET = 0x01;
static constexpr uint8_t MIP_ACK_DESC = 0xF1;   // ACK/NACK field descriptor

// Fletcher-16 (2-byte) checksum.
// DCP examples show 2 bytes at end; we verify as (sum1<<8 | sum2).
static uint16_t mip_fletcher16(const uint8_t* data, size_t len)
{
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (size_t i = 0; i < len; ++i) {
    sum1 = (uint8_t)(sum1 + data[i]);
    sum2 = (uint8_t)(sum2 + sum1);
  }
  return (uint16_t(sum1) << 8) | sum2; // MSB first
}

// Parse the first field descriptor from a TX command packet.
// Packet: 75 65 [desc_set] [payload_len] [field_len] [field_desc] ...
static bool mip_get_first_field_desc(const uint8_t* tx, uint16_t tx_len,
                                     uint8_t* out_desc_set, uint8_t* out_cmd_desc)
{
  if (!tx || tx_len < 6) return false;
  if (tx[0] != MIP_SYNC1 || tx[1] != MIP_SYNC2) return false;

  uint8_t desc_set = tx[2];
  uint8_t payload_len = tx[3];

  // total = header(4) + payload + checksum(2)
  uint16_t total = (uint16_t)(4 + payload_len + 2);
  if (tx_len < total) return false;

  uint8_t field_len = tx[4];
  if (field_len < 2) return false; // must at least include [len][desc]
  // field descriptor is tx[5]
  *out_desc_set = desc_set;
  *out_cmd_desc = tx[5];
  return true;
}

// Read one MIP packet into buf (blocking until deadline).
// Returns true if a packet is read; false on timeout/invalid.
static bool mip_read_packet(Stream& s,
                            uint8_t* buf, uint16_t buf_cap,
                            uint16_t* out_len,
                            uint32_t deadline_ms)
{
  if (!buf || buf_cap < 8) return false;

  // find sync 0x75 0x65
  while (millis() < deadline_ms) {
    if (s.available() <= 0) continue;
    int b = s.read();
    if (b < 0) continue;
    if ((uint8_t)b != MIP_SYNC1) continue;

    // SYNC2
    while (millis() < deadline_ms && s.available() <= 0) {}
    if (millis() >= deadline_ms) return false;
    int b2 = s.read();
    if (b2 < 0) continue;
    if ((uint8_t)b2 != MIP_SYNC2) continue;

    // desc_set + payload_len
    while (millis() < deadline_ms && s.available() < 2) {}
    if (millis() >= deadline_ms) return false;

    uint8_t desc_set   = (uint8_t)s.read();
    uint8_t payload_len = (uint8_t)s.read();

    uint16_t total = (uint16_t)(4 + payload_len + 2);
    if (total > buf_cap) {
      // Drain payload+checksum to resync.
      uint16_t to_drain = (uint16_t)(payload_len + 2);
      while (to_drain-- && millis() < deadline_ms) {
        while (s.available() <= 0 && millis() < deadline_ms) {}
        if (s.available() > 0) (void)s.read();
      }
      return false;
    }

    buf[0] = MIP_SYNC1;
    buf[1] = MIP_SYNC2;
    buf[2] = desc_set;
    buf[3] = payload_len;

    // read payload + checksum
    uint16_t need = (uint16_t)(payload_len + 2);
    uint16_t got = 0;
    while (got < need && millis() < deadline_ms) {
      if (s.available() <= 0) continue;
      int v = s.read();
      if (v < 0) continue;
      buf[4 + got] = (uint8_t)v;
      got++;
    }
    if (got < need) return false;

    *out_len = total;
    // --- MIP 底層監控印出 ---
    // Serial.print("MIP RAW_RX: ");
    // for(uint16_t i=0; i<total; i++) {
    //     if (buf[i] < 0x10) Serial.print("0");
    //     Serial.print(buf[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();
// -----------------------
    return true;
  }
  return false;
}

// Find ACK field (0xF1) in packet and match echo cmd.
// ACK field format (example): [field_len=0x04][desc=0xF1][cmd_echo][ack_code]
static bool mip_find_ack_field(const uint8_t* pkt, uint16_t pkt_len,
                               uint8_t expected_cmd_desc,
                               uint8_t* out_ack_code)
{
  if (!pkt || pkt_len < 8) return false;
  if (pkt[0] != MIP_SYNC1 || pkt[1] != MIP_SYNC2) return false;

  uint8_t payload_len = pkt[3];
  uint16_t total = (uint16_t)(4 + payload_len + 2);
  if (pkt_len < total) return false;

  // Verify checksum
  uint16_t calc = mip_fletcher16(pkt, 4 + payload_len);
  uint16_t got  = (uint16_t(pkt[4 + payload_len]) << 8) | pkt[4 + payload_len + 1];
  if (calc != got) return false;

  // Walk fields in payload
  const uint8_t* p = &pkt[4];
  uint16_t remain = payload_len;
  while (remain >= 2) {
    uint8_t flen = p[0];
    if (flen < 2 || flen > remain) return false;

    uint8_t fdesc = p[1];
    if (fdesc == MIP_ACK_DESC && flen >= 4) {
      uint8_t cmd_echo = p[2];
      uint8_t ack_code = p[3];
      if (cmd_echo == expected_cmd_desc) {
        if (out_ack_code) *out_ack_code = ack_code;
        return true;
      }
    }

    p += flen;
    remain = (uint16_t)(remain - flen);
  }
  return false;
}

UsecaseResult hins_send_and_wait_ack_base(Stream& port_hins,
                                          const uint8_t* tx, uint16_t tx_len,
                                          uint32_t timeout_ms,
                                          uint8_t* out_ack_code)
{
  UsecaseResult r;

  uint8_t desc_set = 0;
  uint8_t cmd_desc = 0;
  if (!mip_get_first_field_desc(tx, tx_len, &desc_set, &cmd_desc)) {
    r.status = Status::BAD_PARAM;
    return r;
  }

  // Base only (0x01)
  if (desc_set != MIP_BASE_SET) {
    r.status = Status::BAD_PARAM;
    return r;
  }

  // Send
  port_hins.write(tx, tx_len);

  // Wait reply packets until timeout
  uint32_t deadline = millis() + timeout_ms;
  uint8_t pkt[256];
  uint16_t pkt_len = 0;

  while (millis() < deadline) {
    if (!mip_read_packet(port_hins, pkt, sizeof(pkt), &pkt_len, deadline)) {
      continue;
    }
    DEBUG_PRINT("[HINS_RX] len=%u: ", (unsigned)pkt_len);
    for (uint16_t i = 0; i < pkt_len; ++i) {
      DEBUG_PRINT("%02X ", pkt[i]);
    }
    DEBUG_PRINT("\r\n");

    // Only care base replies
    if (pkt[2] != MIP_BASE_SET) {
      continue;
    }

    uint8_t ack_code = 0xFF;
    if (mip_find_ack_field(pkt, pkt_len, cmd_desc, &ack_code)) {
      if (out_ack_code) *out_ack_code = ack_code;

      // 0x00 = ACK, non-zero = NACK (for now map to BAD_PARAM)
      r.status = (ack_code == 0x00) ? Status::OK : Status::BAD_PARAM;
      return r;
    }
  }

  r.status = Status::TIMEOUT;
  return r;
}

// Parse first field: returns pointers to field data and length.
static bool mip_get_first_field_info(const uint8_t* tx, uint16_t tx_len,
                                     uint8_t* out_desc_set,
                                     uint8_t* out_cmd_desc,
                                     const uint8_t** out_field_data,
                                     uint8_t* out_field_data_len)
{
  if (!tx || tx_len < 6) return false;
  if (tx[0] != MIP_SYNC1 || tx[1] != MIP_SYNC2) return false;

  uint8_t desc_set = tx[2];
  uint8_t payload_len = tx[3];
  uint16_t total = (uint16_t)(4 + payload_len + 2);
  if (tx_len < total) return false;

  uint8_t flen = tx[4];                 // includes [len][desc]...
  if (flen < 2) return false;
  if (4 + flen > 4 + payload_len) return false;

  uint8_t cmd_desc = tx[5];
  uint8_t data_len = (uint8_t)(flen - 2);
  const uint8_t* data = (data_len > 0) ? &tx[6] : nullptr;

  if (out_desc_set) *out_desc_set = desc_set;
  if (out_cmd_desc) *out_cmd_desc = cmd_desc;
  if (out_field_data) *out_field_data = data;
  if (out_field_data_len) *out_field_data_len = data_len;
  return true;
}

static bool mip_verify_packet_checksum(const uint8_t* pkt, uint16_t pkt_len)
{
  if (!pkt || pkt_len < 8) return false;
  if (pkt[0] != MIP_SYNC1 || pkt[1] != MIP_SYNC2) return false;
  uint8_t payload_len = pkt[3];
  uint16_t total = (uint16_t)(4 + payload_len + 2);
  if (pkt_len < total) return false;

  uint16_t calc = mip_fletcher16(pkt, 4 + payload_len);
  uint16_t got  = (uint16_t(pkt[4 + payload_len]) << 8) | pkt[4 + payload_len + 1];
  return calc == got;
}

// Walk fields; call cb for each field.
// field layout: [flen][fdesc][data...(flen-2)]
template <typename CB>
static bool mip_for_each_field(const uint8_t* pkt, uint16_t pkt_len, CB cb)
{
  if (!pkt || pkt_len < 8) return false;
  uint8_t payload_len = pkt[3];
  uint16_t total = (uint16_t)(4 + payload_len + 2);
  if (pkt_len < total) return false;

  const uint8_t* p = &pkt[4];
  uint16_t remain = payload_len;
  while (remain >= 2) {
    uint8_t flen = p[0];
    if (flen < 2 || flen > remain) return false;
    uint8_t fdesc = p[1];
    const uint8_t* fdata = (flen > 2) ? &p[2] : nullptr;
    uint8_t fdata_len = (uint8_t)(flen - 2);

    cb(fdesc, fdata, fdata_len);

    p += flen;
    remain = (uint16_t)(remain - flen);
  }
  return true;
}

Status hins_mip_transact(Stream& port_hins,
                         const uint8_t* tx, uint16_t tx_len,
                         uint32_t timeout_ms,
                         uint8_t* out_desc_set,
                         uint8_t* out_cmd_desc,
                         uint8_t* out_ack_code,
                         uint8_t* out_ack_echo,
                         uint8_t* out_resp_desc,
                         uint8_t* out_resp_data, uint16_t resp_cap,
                         uint16_t* out_resp_len)
{
  if (out_resp_len) *out_resp_len = 0;
  if (out_resp_desc) *out_resp_desc = 0;
  if (out_ack_code) *out_ack_code = 0xFF;
  if (out_ack_echo) *out_ack_echo = 0x00;

  uint8_t desc_set = 0, cmd_desc = 0;
  const uint8_t* field_data = nullptr;
  uint8_t field_data_len = 0;

  if (!mip_get_first_field_info(tx, tx_len, &desc_set, &cmd_desc, &field_data, &field_data_len)) {
    return Status::BAD_PARAM;
  }

  if (out_desc_set) *out_desc_set = desc_set;
  if (out_cmd_desc) *out_cmd_desc = cmd_desc;

  // Decide want response based on Function Selector (first byte of field data) == READ(0x01)
  bool want_resp = false;
  if (field_data && field_data_len >= 1) {
    const uint8_t function_selector = field_data[0];
    want_resp = (function_selector == 0x02); // READ
  }

  const uint8_t expected_resp_desc = (uint8_t)(cmd_desc | 0x80);

  // Send TX
  port_hins.write(tx, tx_len);

  bool got_ack = false;
  uint8_t ack_code = 0xFF, ack_echo = 0x00;

  bool got_resp = false;
  uint8_t resp_desc = 0;
  uint16_t resp_len = 0;

  uint32_t deadline = millis() + timeout_ms;
  uint8_t pkt[512];
  uint16_t pkt_len = 0;

  while (millis() < deadline) {
    if (!mip_read_packet(port_hins, pkt, sizeof(pkt), &pkt_len, deadline)) {
      continue;
    }

    DEBUG_PRINT("[HINS_RX] len=%u: ", (unsigned)pkt_len);
    for (uint16_t i = 0; i < pkt_len; ++i) DEBUG_PRINT("%02X ", pkt[i]);
    DEBUG_PRINT("\r\n");

    // Filter by same descriptor set
    if (pkt[2] != desc_set) continue;
    if (!mip_verify_packet_checksum(pkt, pkt_len)) continue;

    // Scan fields
    mip_for_each_field(pkt, pkt_len, [&](uint8_t fdesc, const uint8_t* fdata, uint8_t fdata_len) {
      if (fdesc == MIP_ACK_DESC) {
        if (fdata_len >= 2) {
          uint8_t echo = fdata[0];
          uint8_t code = fdata[1];
          if (echo == cmd_desc) {
            got_ack = true;
            ack_echo = echo;
            ack_code = code;
          }
        }
        return;
      }

      // Response field: only useful if want_resp and ACK is OK
      if (!want_resp) return;

      // Prefer exact response desc match (cmd_desc|0x80). If not found, accept first non-ACK field.
      if (!got_resp) {
        if (fdesc == expected_resp_desc || resp_desc == 0) {
          resp_desc = fdesc;
          resp_len = (uint16_t)min<uint16_t>(fdata_len, resp_cap);
          if (out_resp_data && resp_len > 0) memcpy(out_resp_data, fdata, resp_len);
          got_resp = true;
        }
      }
    });

    if (got_ack) {
      if (out_ack_code) *out_ack_code = ack_code;
      if (out_ack_echo) *out_ack_echo = ack_echo;

      // NACK: finish immediately (no need to wait for resp)
      if (ack_code != 0x00) {
        return Status::BAD_PARAM; // 你也可以 map 成 Status::ERR 或自訂 NACK
      }

      if (!want_resp) {
        return Status::OK;
      }
      if (want_resp && got_resp) {
        if (out_resp_desc) *out_resp_desc = resp_desc;
        if (out_resp_len) *out_resp_len = resp_len;
        return Status::OK;
      }
    }
  }

  // Timeout: if no ACK -> TIMEOUT, if want_resp but no resp -> TIMEOUT
  return Status::TIMEOUT;
}

bool hins_send_mip_raw(Stream& port_hins, const uint8_t* mip)
{
  if (!mip) return false;
  if (mip[0] != 0x75 || mip[1] != 0x65) return false;

  const uint8_t payload_len = mip[3];
  const uint16_t total_len = 4u + payload_len + 2u;

  port_hins.write(mip, total_len);  // ← 整包送出
  port_hins.flush();
  return true;
}

// Read a fixed-length HINS streaming payload after aligning to a fixed header.
// - Align to 'header' bytes (e.g. 75 65 85 13 13 49)
// - Read 'payload_len' bytes payload
// - Read 2 bytes checksum (sum1, sum2), MSB first
// - Verify Fletcher-16 over (header + payload), excluding checksum
// - On success, copy payload into out_payload and return true
//
// This function is best-effort and non-blocking-friendly (short timeout).
bool hins_read_stream_payload(
    Stream& port_hins,
    const uint8_t* header,
    uint16_t header_len,
    uint16_t payload_len,
    uint8_t* out_payload,
    uint16_t out_cap,
    uint32_t timeout_ms
) {
    if (!header || header_len == 0) return false;
    if (!out_payload || out_cap == 0) return false;
    if (payload_len > out_cap) return false;

    const uint32_t t0 = millis();

    // Helper: read one byte with timeout
    auto read_byte = [&](uint8_t &b) -> bool {
        while ((millis() - t0) < timeout_ms) {
            if (port_hins.available() > 0) {
                int v = port_hins.read();
                if (v >= 0) { b = (uint8_t)v; return true; }
            }
            yield(); // ok on Arduino; remove if unavailable
        }
        return false;
    };

    // 1) Align to header
    uint16_t matched = 0;
    while ((millis() - t0) < timeout_ms) {
        uint8_t b = 0;
        if (!read_byte(b)) return false;

        if (b == header[matched]) {
            matched++;
            if (matched == header_len) break; // header matched
        } else {
            matched = (b == header[0]) ? 1 : 0;
        }
    }
    if (matched != header_len) return false;

    // 2) Read payload
    for (uint16_t i = 0; i < payload_len; i++) {
        uint8_t b = 0;
        if (!read_byte(b)) return false;
        out_payload[i] = b;
    }

    // 3) Read checksum (2 bytes, MSB first: sum1, sum2)
    uint8_t rx_sum1 = 0, rx_sum2 = 0;
    if (!read_byte(rx_sum1)) return false;
    if (!read_byte(rx_sum2)) return false;
    const uint16_t rx_ck = (uint16_t(rx_sum1) << 8) | rx_sum2;

    // 4) Verify Fletcher-16 over (header + payload) ㄇ
    const uint16_t total = (uint16_t)(header_len + payload_len);
    uint8_t tmp[64];
    if (total > sizeof(tmp)) return false;

    for (uint16_t i = 0; i < header_len; i++) tmp[i] = header[i];
    for (uint16_t i = 0; i < payload_len; i++) tmp[header_len + i] = out_payload[i];

    const uint16_t calc_ck = mip_fletcher16(tmp, total);

    if (calc_ck != rx_ck) {
        return false;
    }

    return true;
}

// True Heading Aiding (0x13,0x31) via hins_mip_transact()
// Variant: u32 ns to fit Field Length = 0x13 (field data = 17 bytes)
Status hins_true_heading_transact_u32ns(
    Stream& port_hins,
    uint32_t timeout_ms,
    const true_heading_t* th,
    uint8_t* out_ack_code,
    uint8_t* out_ack_echo
) {
  if (!th) return Status::ERR; // 你的 Status enum 若不同，改成你專案的 error

  // Packet: SYNC(2) + DS(0x13) + PL(0x13) + Field(0x13 bytes) + CK(2)
  uint8_t pkt[4 + 0x13 + 2];

  // Header
  pkt[0] = 0x75;
  pkt[1] = 0x65;
  pkt[2] = 0x13;
  pkt[3] = 0x13;

  // Field
  uint8_t* f = &pkt[4];
  f[0] = 0x13;   // field len
  f[1] = 0x31;   // field desc

  // time: timebase + reserved + ns(u32)
  f[2] = th->ts.timebase;
  f[3] = th->ts.reserved;

  // use low 32-bit of nanosecs (mod 2^32)
  const uint32_t ns32 = (uint32_t)(th->ts.nanosecs & 0xFFFFFFFFu);
  write_be_u32(&f[4], ns32);

  // frame id
  f[8] = th->Frame_id;

  // heading + uncertainty (radians)
  write_be_f32(&f[9],  th->Heading.float_val);
  write_be_f32(&f[13], th->Uncertainty.float_val);

  // valid flags
  write_be_u16(&f[17], th->valid_flag);

  // checksum over [SYNC..end of payload]
  const uint16_t ck = mip_fletcher16(pkt, 4 + 0x13);
  pkt[4 + 0x13 + 0] = (uint8_t)(ck >> 8);   // sum1
  pkt[4 + 0x13 + 1] = (uint8_t)(ck & 0xFF); // sum2

  uint8_t out_desc_set = 0, out_cmd_desc = 0;
  uint8_t ack_code = 0xFF, ack_echo = 0xFF;
  uint8_t resp_desc = 0;
  uint8_t resp_buf[8];
  uint16_t resp_len = 0;

  Status st = hins_mip_transact(
      port_hins,
      pkt, (uint16_t)sizeof(pkt),
      timeout_ms,
      &out_desc_set,
      &out_cmd_desc,
      &ack_code,
      &ack_echo,
      &resp_desc,
      resp_buf, (uint16_t)sizeof(resp_buf),
      &resp_len
  );

  if (out_ack_code) *out_ack_code = ack_code;
  if (out_ack_echo) *out_ack_echo = ack_echo;

  return st;
}

Status hins_true_heading_transact_u64ns(
    Stream& port_hins,
    uint32_t timeout_ms,
    const true_heading_t* th,
    uint8_t* out_ack_code,
    uint8_t* out_ack_echo
) {
  // Packet: SYNC(2) + DS(0x13) + PL(0x16) + Field(0x16 bytes) + CK(2)
  const uint8_t PL = 0x16;
  const uint8_t FL = 0x16;

  uint8_t pkt[4 + PL + 2];

  pkt[0] = 0x75;
  pkt[1] = 0x65;
  pkt[2] = 0x13;
  pkt[3] = PL;

  uint8_t* f = &pkt[4];
  f[0] = FL;
  f[1] = 0x31;

  // time: timebase + reserved + ns(u64)
  f[2] = th->ts.timebase;
  f[3] = th->ts.reserved;
  write_be_u64(&f[4], th->ts.nanosecs);

  // frame id
  f[12] = th->Frame_id;

  // heading + uncertainty
  write_be_f32(&f[13], th->Heading.float_val);
  write_be_f32(&f[17], th->Uncertainty.float_val);

  // valid flags
  write_be_u16(&f[21], th->valid_flag);

  const uint16_t ck = mip_fletcher16(pkt, 4 + PL);
  pkt[4 + PL + 0] = (uint8_t)(ck >> 8);
  pkt[4 + PL + 1] = (uint8_t)(ck & 0xFF);

  uint8_t out_desc_set = 0, out_cmd_desc = 0;
  uint8_t ack_code = 0xFF, ack_echo = 0xFF;
  uint8_t resp_desc = 0;
  uint8_t resp_buf[8];
  uint16_t resp_len = 0;

  Status st = hins_mip_transact(
      port_hins,
      pkt, (uint16_t)sizeof(pkt),
      timeout_ms,
      &out_desc_set,
      &out_cmd_desc,
      &ack_code,
      &ack_echo,
      &resp_desc,
      resp_buf, (uint16_t)sizeof(resp_buf),
      &resp_len
  );

  if (out_ack_code) *out_ack_code = ack_code;
  if (out_ack_echo) *out_ack_echo = ack_echo;

  return st;
}

Status hins_capture_raw_mip(Stream& port_hins, 
                            uint8_t* out_buf, uint16_t buf_cap, 
                            uint16_t* out_len, uint32_t timeout_ms) 
{
    uint32_t deadline = millis() + timeout_ms;
    uint16_t captured_len = 0;

    // 直接調用現有的封包讀取邏輯
    if (mip_read_packet(port_hins, out_buf, buf_cap, &captured_len, deadline)) {
        if (out_len) *out_len = captured_len;
        return Status::OK;
    }
    
    return Status::TIMEOUT;
}
