#pragma once
#include <Arduino.h>

// ====== 型別 ======
typedef union {
  float     float_val;
  uint8_t   bin_val[4];
  uint32_t  ulong_val;
} my_time_t;

typedef union {
  float    float_val[3];
  uint8_t  bin_val[12];
  int      int_val[3];
} my_acc_t;

typedef union {
  float    float_val;
  uint8_t  bin_val[4];
  int      int_val;
} my_float_t;

typedef union {
  struct {
    float ax, ay, az;
    float a11,a12,a13, a21,a22,a23, a31,a32,a33;
    float gx, gy, gz;
    float g11,g12,g13, g21,g22,g23, g31,g32,g33;
  } _f;
  struct {
    int ax, ay, az;
    int a11,a12,a13, a21,a22,a23, a31,a32,a33;
    int gx, gy, gz;
    int g11,g12,g13, g21,g22,g23, g31,g32,g33;
  } _d;
} my_misalignment_cali_t;

typedef union {
  struct { float std_wx, std_wy, std_wz; } _f;
  struct { int   std_wx, std_wy, std_wz; } _d;
} my_attitude_cali_t;



// ====== 函式指標型別（共用的話放這裡）======
typedef void (*fn_ptr)(byte&, unsigned int, byte);


