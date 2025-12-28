// src/utils/endian.h
#pragma once
#include <stdint.h>

static inline int32_t be_bytes_to_i32(uint8_t b3, uint8_t b2,
                                      uint8_t b1, uint8_t b0)
{
    return ((int32_t)b3 << 24) |
           ((int32_t)b2 << 16) |
           ((int32_t)b1 <<  8) |
           ((int32_t)b0);
}

static inline uint32_t be_bytes_to_u32(uint8_t b3, uint8_t b2,
                                       uint8_t b1, uint8_t b0)
{
    return ((uint32_t)b3 << 24) |
           ((uint32_t)b2 << 16) |
           ((uint32_t)b1 <<  8) |
           ((uint32_t)b0);
}
