#include "checksum16.h"

uint16_t fletcher16(const uint8_t* data, size_t len)
{
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (size_t i = 0; i < len; ++i) {
    sum1 += data[i];
    sum1 %= 255;
    sum2 += sum1;
    sum2 %= 255;
  }

  // Fletcher-16 convention: (sum2 << 8) | sum1
  return (uint16_t)((sum2 << 8) | sum1);
}
