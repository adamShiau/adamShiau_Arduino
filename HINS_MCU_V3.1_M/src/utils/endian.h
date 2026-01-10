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

static inline float be_f32(const uint8_t* p)
{
    my_float_t u;
    u.bin_val[0] = p[3];
    u.bin_val[1] = p[2];
    u.bin_val[2] = p[1];
    u.bin_val[3] = p[0];
    return u.float_val;
}

static inline uint16_t be_u16(const uint8_t* p)
{
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

static inline void write_be_u16(uint8_t* p, uint16_t v){
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

static inline void write_be_u32(uint8_t* p, uint32_t v){
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)(v & 0xFF);
}

static inline void write_be_u64(uint8_t* p, uint64_t v){
  p[0] = (uint8_t)(v >> 56);
  p[1] = (uint8_t)(v >> 48);
  p[2] = (uint8_t)(v >> 40);
  p[3] = (uint8_t)(v >> 32);
  p[4] = (uint8_t)(v >> 24);
  p[5] = (uint8_t)(v >> 16);
  p[6] = (uint8_t)(v >> 8);
  p[7] = (uint8_t)(v & 0xFF);
}

static inline void write_be_f32(uint8_t* p, float f){
  my_float_t u;
  u.float_val = f;
  p[0] = u.bin_val[3];
  p[1] = u.bin_val[2];
  p[2] = u.bin_val[1];
  p[3] = u.bin_val[0];
}

