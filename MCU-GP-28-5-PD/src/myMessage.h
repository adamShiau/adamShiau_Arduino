#ifndef MYMESSAGE
#define MYMESSAGE
#include <Arduino.h>

class MyCRC {
private:
    uint32_t WIDTH;
    uint32_t POLYNOMIAL;
    uint32_t REMAINDER;
    uint32_t crcFailCnt;
    int NUM_BYTE;

public:
    // Constructor
    MyCRC(uint32_t width = 32, uint32_t polynomial = 0x04C11DB7, uint32_t initial_remainder = 0xFFFFFFFF);

    // Calculate CRC
    void calCRC(uint8_t *messagem, int num_byte);
};


#endif


#ifndef MYDATAFORMAT
#define MYDATAFORMAT

typedef union{
  float float_val[3];
  uint8_t bin_val[12];
  unsigned long ulong_val[3];
} my_data_3f;

typedef union{
  float float_val[4];
  uint8_t bin_val[16];
  unsigned long ulong_val[4];
} my_data_4f;

typedef union{
  float float_val[2];
  uint8_t bin_val[8];
  unsigned long ulong_val[2];
} my_data_2f;

typedef union{
  float float_val;
  uint8_t bin_val[4];
  unsigned long ulong_val;
} my_data_u4;

typedef union{
  uint8_t bin_val[2];
  uint16_t ushort_val;
} my_data_u2;

#endif