#include "nios_link.h"

size_t sendCmd(Stream& port, const uint8_t header[2], const uint8_t trailer[2], uint8_t cmd, 
  int32_t value, uint8_t ch)
{
  size_t n = 0;

  // header (2 bytes)
  n += port.write(header, 2);

  // cmd (1 byte)
  n += port.write(&cmd, 1);

  // value (4 bytes, big-endian)
  uint32_t v = (uint32_t)value;  // cast to avoid sign-propagation on shifts
  uint8_t b[4] = {
    (uint8_t)((v >> 24) & 0xFF),
    (uint8_t)((v >> 16) & 0xFF),
    (uint8_t)((v >>  8) & 0xFF),
    (uint8_t)((v >>  0) & 0xFF)
  };
  n += port.write(b, 4);

  // ch (1 byte)
  n += port.write(&ch, 1);

  // trailer (2 bytes)
  n += port.write(trailer, 2);

  return n;  // expected 10
}

bool sendSN(Stream& port, const uint8_t header[2], const uint8_t trailer[2], uint8_t cmd,
            const uint8_t sn_ascii[12])
{
    // Packet format:
    //   header[2] + cmd[1] + SN[12] + trailer[2]
    uint8_t pkt[2 + 1 + 12 + 2];
    uint8_t idx = 0;

    // Header
    pkt[idx++] = header[0];
    pkt[idx++] = header[1];

    // Command (CMD_WRITE_SN = 0x6E)
    pkt[idx++] = cmd;

    // SN payload (12 bytes, already padded with ASCII space if needed)
    for (uint8_t i = 0; i < 12; i++) {
        pkt[idx++] = sn_ascii[i];
    }

    // Trailer
    pkt[idx++] = trailer[0];
    pkt[idx++] = trailer[1];

    // Sanity check (debug friendly)
    if (idx != sizeof(pkt)) {
        return false;
    }

    // Send out
    port.write(pkt, sizeof(pkt));
    // port.flush();   // 與 sendCmd 行為一致（若你原本有）

    return true;
}

