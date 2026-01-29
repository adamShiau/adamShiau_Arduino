#pragma once

#include <stdint.h>
#include "../../common.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

typedef struct {
    float ax_lp, ay_lp, az_lp;

    // coning states
    float w_prev_dps[3];
    float dt_prev_s;
    uint8_t have_prev;
    uint32_t last_ts_us;

    uint8_t  needs_alignment; // 新增：啟動時對齊旗標

    // gimbal-lock yaw hold
    uint8_t yaw_gl_locked;
    float   yaw_hold_deg;

    // still-detect bias boost
    uint8_t  just_still;
    uint32_t still_ts_ms;
    float    bias_alpha_base;

    // circular MA buffers (pitch/roll/yaw)
    // NOTE: buffer size is fixed in the .cpp implementation.
    int pitch_idx, pitch_count;
    int roll_idx,  roll_count;
    int yaw_idx,   yaw_count;
    float pitch_sin_sum, pitch_cos_sum;
    float roll_sin_sum,  roll_cos_sum;
    float yaw_sin_sum,   yaw_cos_sum;
    
    float out_th;    // 對應 GUI 的 OUT_TH
    uint8_t out_th_en;  // 對應 GUI 的 OUT_EN (0: 關閉, 1: 開啟)

    float nominal_fs_hz;
} ahrs_att_stage_ctx_t;

// Initialize (must be called once before first update)
void ahrs_att_stage_init(ahrs_att_stage_ctx_t* ctx, float fs_hz);

// Reset runtime state (does NOT reset the global ahrs_attitude object)
void ahrs_att_stage_reset(ahrs_att_stage_ctx_t* ctx);

// Prepare for a new streaming run (keeps behavior identical to the original code):
// - set global gyro-bias learning alpha to the library's base value
// - clear any still/boost transient state
void ahrs_att_stage_prepare_run(ahrs_att_stage_ctx_t* ctx);

// Run one attitude update tick.
// Inputs are the calibrated sensor values used by the original implementation:
//   - gyro in dps
//   - accel in m/s^2
// Output is roll/pitch/yaw degrees packed in my_att_t (same as original).
void ahrs_att_stage_update(ahrs_att_stage_ctx_t* ctx,
                           const my_att_t* gyro_cali_dps,
                           const my_att_t* accl_cali_mps2,
                           my_att_t* att_out);

// #ifdef __cplusplus
// } // extern "C"
// #endif
