# include "myMessage.h"

// 沒用到
MyCRC::MyCRC(uint32_t width, uint32_t polynomial, uint32_t initial_remainder)
    : WIDTH(width), POLYNOMIAL(polynomial), REMAINDER(initial_remainder), crcFailCnt(0) {
    NUM_BYTE = (WIDTH + 7) / 8;
}

// Calculate CRC
void MyCRC::calCRC(uint8_t *message, int num_msg) {
    uint32_t TOPBIT = (1 << (WIDTH - 1));
    uint32_t mask = (1 << WIDTH) - 1;
    uint32_t remainder = REMAINDER;

    for (int n = 0; n < num_msg - NUM_BYTE; n++) {
        remainder ^= (message[n] << (WIDTH - 8));

        for (int i = 0; i < 8; i++) {
            if (remainder & TOPBIT) {
                remainder = ((remainder << 1) & mask) ^ POLYNOMIAL;
            } else {
                remainder = (remainder << 1) & mask;
            }
        }
    }

    for (int i = 0; i < NUM_BYTE; i++) {
        message[num_msg - NUM_BYTE + i] = (remainder >> (8 * (NUM_BYTE - 1 - i))) & 0xFF;
    }
}


