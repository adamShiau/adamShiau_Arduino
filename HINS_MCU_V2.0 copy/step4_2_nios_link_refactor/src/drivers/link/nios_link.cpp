#include "drivers/link/nios_link.h"

size_t sendCmd(Print& port, const uint8_t header[2], const uint8_t trailer[2], uint8_t cmd, 
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
