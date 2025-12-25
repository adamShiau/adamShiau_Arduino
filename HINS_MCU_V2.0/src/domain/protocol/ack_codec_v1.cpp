#include "ack_codec_v1.h"
#include "../../utils/crc32.h"
#include "../../utils/checksum16.h"


namespace {
  // Frame format (minimal, independent from sendCmd):
  // [0]  SOF = 0xFA
  // [1]  TYPE (0xA1=ACK, 0xA2=RESULT)
  // [2]  cmd_id
  // [3]  status
  // [4]  len (payload length, 0..255)
  // [5..] payload (optional)
  // [..]  checksum16 (2 bytes, little-endian on wire)
  //
  // Checksum coverage [type..payload] i.e. buf[1] length = 4 + len

  constexpr uint8_t SOF = 0xFA;
  constexpr uint8_t TYPE_ACK = 0xA1;
  constexpr uint8_t TYPE_RESULT = 0xA2;

  // 最大 payload
  constexpr uint16_t MAX_PAYLOAD = 128;

  static bool send_frame(Print& port, uint8_t type, uint8_t cmd_id,
                         uint8_t status, const uint8_t* payload, uint8_t len)
  {
    if (len > MAX_PAYLOAD) return false;

    uint8_t buf[1 + 1 + 1 + 1 + 1 + MAX_PAYLOAD + 2]; // SOF + (type..len) + payload + checksum16
    uint16_t idx = 0;

    buf[idx++] = SOF;
    buf[idx++] = type;
    buf[idx++] = cmd_id;
    buf[idx++] = status;
    buf[idx++] = len;

    for (uint8_t i = 0; i < len; ++i) buf[idx++] = payload[i];

    // checksum over [type..payload] => buf[1], length = 4 + len
    const uint16_t cs = fletcher16(&buf[1], (size_t)(4 + len));

    // on-wire: little endian (low byte first)
    buf[idx++] = (uint8_t)(cs & 0xFF);
    buf[idx++] = (uint8_t)((cs >> 8) & 0xFF);

    port.write(buf, idx);
    return true;
  }
} // namespace

bool send_ack_v1(Print& pc_port, uint8_t cmd_id, AckStatus st)
{
  return send_frame(pc_port, TYPE_ACK, cmd_id, (uint8_t)st, nullptr, 0);
}

bool send_result_v1(Print& pc_port, uint8_t cmd_id, AckStatus st)
{
  return send_frame(pc_port, TYPE_RESULT, cmd_id, (uint8_t)st, nullptr, 0);
}

bool send_result_v1(Print& pc_port,
                    uint8_t cmd_id,
                    AckStatus st,
                    const uint8_t* payload,
                    uint16_t payload_len)
{
  if (payload_len > 255) return false; // len field is 1 byte
  return send_frame(pc_port, TYPE_RESULT, cmd_id, (uint8_t)st, payload, (uint8_t)payload_len);
}