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

struct FletcherChecksumBuffer {
    uint8_t last_valid_buf[18]; // 儲存 16 bytes data + 2 bytes checksum
    bool has_first_valid = false;
    uint32_t error_count = 0;   // 用於監控通訊品質

    // 驗證並更新內部 Buffer
    void verifyAndLock(uint8_t* new_data) {
        if (new_data == nullptr) return;

        // 1. 計算前 16 bytes 的 Fletcher16
        uint8_t sum1 = 0, sum2 = 0;
        for (int i = 0; i < 16; i++) {
            sum1 += new_data[i];
            sum2 += sum1;
        }
        uint16_t calculated = (uint16_t)((uint16_t)sum1 << 8) | sum2;

        // 2. 提取原始 Checksum
        uint16_t received = (uint16_t)(new_data[16] << 8) | new_data[17];

        // 3. 比對
        if (calculated == received) {
            memcpy(last_valid_buf, new_data, 18);
            has_first_valid = true;
        } else {
            error_count++;
            // 這裡不更新 last_valid_buf，保持舊值
        }
    }

    // 提供給 if(fog) 使用的輸出介面
    uint8_t* getOutput() {
        return has_first_valid ? last_valid_buf : nullptr;
    }
};


