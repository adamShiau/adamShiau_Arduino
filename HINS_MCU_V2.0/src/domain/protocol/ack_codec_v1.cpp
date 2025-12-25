#include "ack_codec_v1.h"
#include "../../utils/crc32.h"
#include "../../utils/checksum16.h"


namespace {
  // Frame format (minimal, independent from sendCmd):
  // [0]  SOF = 0xFA
  // [1]  TYPE (0xA1=ACK, 0xA2=RESULT)
  // [2]  cmd_id
  // [3]  status
  // [4]  len_L
  // [5]  len_H
  // [6..] payload
  // [..]  checksum16 (2 bytes, little-endian)
  //
  // Checksum coverage [type..payload] => buf[1], length = 5 + len

  constexpr uint8_t SOF = 0xFA;
  constexpr uint8_t TYPE_ACK = 0xA1;
  constexpr uint8_t TYPE_RESULT = 0xA2;

  // 最大 payload
  constexpr uint16_t MAX_PAYLOAD = 1024;

  static bool send_frame(Stream& port, uint8_t type, uint8_t cmd_id,
                         uint8_t status, const uint8_t* payload, uint16_t len)
  {
    if (len > MAX_PAYLOAD) return false;

    uint8_t buf[1 + 1 + 1 + 1 + 2 + MAX_PAYLOAD + 2]; // SOF + (type..len2) + payload + checksum16
    uint16_t idx = 0;

    buf[idx++] = SOF;
    buf[idx++] = type;
    buf[idx++] = cmd_id;
    buf[idx++] = status;
    buf[idx++] = (uint8_t)(len & 0xFF);        // len_L
    buf[idx++] = (uint8_t)((len >> 8) & 0xFF); // len_H

    for (uint16_t  i = 0; i < len; ++i) buf[idx++] = payload[i];

    // checksum over [type..payload] => buf[1], length = 5 + len: type,cmd,status,len_L,len_H
    const uint16_t cs = fletcher16(&buf[1], (size_t)(5 + len));

    // on-wire: little endian (low byte first)
    buf[idx++] = (uint8_t)(cs & 0xFF);
    buf[idx++] = (uint8_t)((cs >> 8) & 0xFF);

    port.write(buf, idx);
    return true;
  }
} // namespace

bool send_ack_v1(Stream& pc_port, uint8_t cmd_id, AckStatus st)
{
  return send_frame(pc_port, TYPE_ACK, cmd_id, (uint8_t)st, nullptr, 0);
}

bool send_result_v1(Stream& pc_port, uint8_t cmd_id, AckStatus st)
{
  return send_frame(pc_port, TYPE_RESULT, cmd_id, (uint8_t)st, nullptr, 0);
}

bool send_result_v1(Stream& pc_port,
                    uint8_t cmd_id,
                    AckStatus st,
                    const uint8_t* payload,
                    uint16_t payload_len)
{
  if (payload_len > MAX_PAYLOAD) return false; 
  return send_frame(pc_port, TYPE_RESULT, cmd_id, (uint8_t)st, payload, (uint8_t)payload_len);
}