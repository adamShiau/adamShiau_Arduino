#pragma once
#include <stdint.h>

/**
 * @brief MINSPixhawk兼容的CRC計算類
 * 
 * 完全複製MINSPixhawk的CRC實現邏輯，確保產生相同的CRC值
 */
class MINSPixhawkCRC {
private:
    uint32_t WIDTH;
    uint32_t POLYNOMIAL;
    uint32_t REMAINDER;
    uint32_t NUM_BYTE;
    uint32_t crcFailCnt;

public:
    MINSPixhawkCRC(uint32_t width = 32, uint32_t polynomial = 0x04C11DB7, uint32_t initial_remainder = 0xFFFFFFFF)
        : WIDTH(width), POLYNOMIAL(polynomial), REMAINDER(initial_remainder), crcFailCnt(0) {
        NUM_BYTE = (WIDTH + 7) / 8;
    }

    // 計算CRC，num_msg 包含CRC字節
    void calCRC(uint8_t *message, int num_msg) {
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

    // 驗證CRC，num_msg 包含CRC字節
    bool isCRCPass(uint8_t *message, int num_msg) {
        uint8_t temp_msg[num_msg + NUM_BYTE];
        for (int i = 0; i < num_msg; i++) { 
            temp_msg[i] = message[i]; 
        }
        calCRC(temp_msg, num_msg + NUM_BYTE);
        for (int i = 0; i < NUM_BYTE; i++){
            if (temp_msg[num_msg + i] != 0) { 
                return false; 
            }
        }
        return true;
    }
};