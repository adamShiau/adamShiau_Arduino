// src/drivers/link/hins_link.cpp
#include "hins_link.h"
#include <Arduino.h>
#include "../../common.h"

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
      serial_printf("%02X ", pkt[i]);
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
