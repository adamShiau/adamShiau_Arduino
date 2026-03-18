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
    uint8_t last_valid_buf[64]; 
    uint8_t payload_len;        
    bool has_first_valid = false; // 標記是否曾經存入過正確資料
    uint32_t error_count = 0;

    FletcherChecksumBuffer(uint8_t len) : payload_len(len) {
        memset(last_valid_buf, 0, sizeof(last_valid_buf));
    }

    void verifyAndLock(uint8_t* raw_input) {
        if (!raw_input) return;

        // 計算 Checksum (包含手動補回 0xABBA)
        uint8_t sum1 = 0, sum2 = 0;
        uint8_t header[2] = {0xAB, 0xBA};
        for (int i=0; i<2; i++) { sum1 += header[i]; sum2 += sum1; }
        for (int i=0; i<payload_len-2; i++) { sum1 += raw_input[i]; sum2 += sum1; }

        uint16_t calculated = (uint16_t)((uint16_t)sum1 << 8) | sum2;
        uint16_t received = (uint16_t)(raw_input[payload_len-2] << 8) | raw_input[payload_len-1];

        if (calculated == received) {
            // 校驗成功：更新快取
            memcpy(last_valid_buf, raw_input, payload_len);
            has_first_valid = true;
        } else {
            // 校驗失敗：不更新快取，僅紀錄錯誤
            error_count++;
            // 自動打印除錯資訊
            Serial.print(" Cksum Fail! Calc:"); Serial.print(calculated, HEX);
            Serial.print(" Recv:"); Serial.println(received, HEX);
        }
    }

    uint8_t* getOutput() {
        // 只要 has_first_valid 為 true，就回傳快取位址。
        // 即使 verifyAndLock 剛才失敗了，這裏回傳的依然是「上一次正確」的資料位址。
        return has_first_valid ? last_valid_buf : nullptr; 
    }
};


