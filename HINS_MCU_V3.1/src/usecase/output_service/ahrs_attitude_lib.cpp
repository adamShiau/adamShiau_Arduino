#include "ahrs_attitude_lib.h"

#include <math.h>
#include <string.h>

// Need access to the global `ahrs_attitude` instance and its methods.
#include "../../app/app_state.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD ((float)M_PI / 180.0f)
#define RAD2DEG (180.0f / (float)M_PI)

// ===== Keep constants identical to acq_ahrs.cpp =====
#define ACC_MIN        (0.05f * 9.80665f)
#define ACC_MAX        (16.0f * 9.80665f)
#define GYRO_MIN_DPS   (0.01f)
#define GYRO_MAX_DPS   (1000.0f)
#define ACC_LP_ALPHA   (0.2f)

#define G0_MPS2               (9.80665f)
#define GYRO_STILL_THRESH_DPS (2.0f)
#define ACC_STILL_TOL_G       (0.06f)

#define BIAS_ALPHA_BOOST     (0.010f)
#define BIAS_ALPHA_BOOST_MS  (800u)
#define BIAS_ALPHA_BASE      (0.002f)

#define GL_ENTER_DEG  (88.0f)
#define GL_EXIT_DEG   (86.0f)

// circular moving average window
#define ATT_MA_N 5

// internal MA buffers (kept in ctx via indices/sums; buffers here)
static float pitch_sin_buf[ATT_MA_N] = {0};
static float pitch_cos_buf[ATT_MA_N] = {0};
static float roll_sin_buf[ATT_MA_N]  = {0};
static float roll_cos_buf[ATT_MA_N]  = {0};
static float yaw_sin_buf[ATT_MA_N]   = {0};
static float yaw_cos_buf[ATT_MA_N]   = {0};

static inline float apply_deadband_and_sat(float x, float min_abs, float max_abs)
{
    if (!isfinite(x)) return 0.0f;
    float ax = fabsf(x);
    if (ax < min_abs) return 0.0f;
    if (ax > max_abs) return (x > 0.0f ? max_abs : -max_abs);
    return x;
}

static inline bool is_still_app(float gx_dps, float gy_dps, float gz_dps,
                                float ax_mps2, float ay_mps2, float az_mps2)
{
    float gsum = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);
    if (gsum > GYRO_STILL_THRESH_DPS) return false;
    float anorm = sqrtf(ax_mps2*ax_mps2 + ay_mps2*ay_mps2 + az_mps2*az_mps2);
    float dev   = fabsf(anorm - G0_MPS2) / G0_MPS2;
    return (dev <= ACC_STILL_TOL_G);
}

static inline void coning_two_sample_dps(const float w1_dps[3], float dt1,
                                         const float w2_dps[3], float dt2,
                                         float weq_dps[3])
{
    const float D2R = 0.01745329252f;
    const float R2D = 57.295779513f;

    float d1[3] = { (w1_dps[0]*D2R)*dt1, (w1_dps[1]*D2R)*dt1, (w1_dps[2]*D2R)*dt1 };
    float d2[3] = { (w2_dps[0]*D2R)*dt2, (w2_dps[1]*D2R)*dt2, (w2_dps[2]*D2R)*dt2 };

    float cx[3] = {
        0.5f * (d1[1]*d2[2] - d1[2]*d2[1]),
        0.5f * (d1[2]*d2[0] - d1[0]*d2[2]),
        0.5f * (d1[0]*d2[1] - d1[1]*d2[0])
    };

    float dc[3] = { d1[0]+d2[0]+cx[0], d1[1]+d2[1]+cx[1], d1[2]+d2[2]+cx[2] };

    float dt = dt1 + dt2;
    if (dt < 1e-9f) {
        weq_dps[0]=w2_dps[0]; weq_dps[1]=w2_dps[1]; weq_dps[2]=w2_dps[2];
        return;
    }

    float weq_rad[3] = { dc[0]/dt, dc[1]/dt, dc[2]/dt };
    weq_dps[0] = weq_rad[0]*R2D;
    weq_dps[1] = weq_rad[1]*R2D;
    weq_dps[2] = weq_rad[2]*R2D;
}

static inline void quat_to_Rwb(float w,float x,float y,float z, float R[9])
{
    float ww=w*w, xx=x*x, yy=y*y, zz=z*z;
    float wx=w*x, wy=w*y, wz=w*z, xy=x*y, xz=x*z, yz=y*z;
    R[0]=ww+xx-yy-zz; R[1]=2*(xy-wz);    R[2]=2*(xz+wy);
    R[3]=2*(xy+wz);   R[4]=ww-xx+yy-zz; R[5]=2*(yz-wx);
    R[6]=2*(xz-wy);   R[7]=2*(yz+wx);   R[8]=ww-xx-yy+zz;
}

static inline float circle_ma_filter_deg(float angle_deg,
                                        float *buf_sin, float *buf_cos,
                                        float *sum_sin, float *sum_cos,
                                        int *idx, int *count,
                                        int N)
{
    if (!isfinite(angle_deg)) angle_deg = 0.0f;

    float rad = angle_deg * DEG2RAD;
    float new_s = sinf(rad);
    float new_c = cosf(rad);

    float old_s = buf_sin[*idx];
    float old_c = buf_cos[*idx];

    *sum_sin += (new_s - old_s);
    *sum_cos += (new_c - old_c);

    buf_sin[*idx] = new_s;
    buf_cos[*idx] = new_c;

    if (*count < N) (*count)++;
    *idx = (*idx + 1) % N;

    if (*count == 0) return angle_deg;

    float avg_s = (*sum_sin) / (float)(*count);
    float avg_c = (*sum_cos) / (float)(*count);

    float norm2 = avg_s*avg_s + avg_c*avg_c;
    if (norm2 < 1e-6f) return angle_deg;

    float filt_rad = atan2f(avg_s, avg_c);
    return filt_rad * RAD2DEG;
}

void ahrs_att_stage_init(ahrs_att_stage_ctx_t* ctx, float fs_hz)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->nominal_fs_hz = (fs_hz > 1.0f ? fs_hz : 100.0f);
    ctx->bias_alpha_base = BIAS_ALPHA_BASE;

    // Clear buffers
    memset(pitch_sin_buf, 0, sizeof(pitch_sin_buf));
    memset(pitch_cos_buf, 0, sizeof(pitch_cos_buf));
    memset(roll_sin_buf,  0, sizeof(roll_sin_buf));
    memset(roll_cos_buf,  0, sizeof(roll_cos_buf));
    memset(yaw_sin_buf,   0, sizeof(yaw_sin_buf));
    memset(yaw_cos_buf,   0, sizeof(yaw_cos_buf));
}

void ahrs_att_stage_reset(ahrs_att_stage_ctx_t* ctx)
{
    if (!ctx) return;
    float fs = (ctx->nominal_fs_hz > 1.0f ? ctx->nominal_fs_hz : 100.0f);
    ahrs_att_stage_init(ctx, fs);
    ctx->needs_alignment = 1; // 確保 reset 後下次運算會重新對齊
}

void ahrs_att_stage_prepare_run(ahrs_att_stage_ctx_t* ctx)
{
    // Keep the original behavior: each start forces the bias-learning alpha to base.
    (void)ctx;
    ahrs_attitude.setGyroBiasAlpha(BIAS_ALPHA_BASE);
    if (ctx) {
        ctx->bias_alpha_base = BIAS_ALPHA_BASE;
        ctx->just_still = 0;
        ctx->still_ts_ms = 0;
        ctx->needs_alignment = 1; // 標記下一筆數據進來時進行對齊
    }
}

void ahrs_att_stage_update(ahrs_att_stage_ctx_t* ctx,
                           const my_att_t* gyro_cali_dps,
                           const my_att_t* accl_cali_mps2,
                           my_att_t* att_out)
{
    if (!ctx || !gyro_cali_dps || !accl_cali_mps2 || !att_out) return;

    // 1) threshold + saturation
    my_att_t gyro_att = {};
    my_att_t acc_att  = {};
    for (int i=0;i<3;++i) {
        gyro_att.float_val[i] = apply_deadband_and_sat(gyro_cali_dps->float_val[i], GYRO_MIN_DPS, GYRO_MAX_DPS);
        acc_att.float_val[i]  = apply_deadband_and_sat(accl_cali_mps2->float_val[i], ACC_MIN, ACC_MAX);
    }

    // 新增：初始對齊邏輯
    if (ctx->needs_alignment) {
        // 使用第一筆 Case Frame 加速度計數據進行對齊
        ahrs_attitude.initQuaternionFromAccel(acc_att.float_val[0], 
                                              acc_att.float_val[1], 
                                              acc_att.float_val[2]);
        
        // 同步更新低通濾波器初始值，避免濾波器從 0 開始拉扯
        ctx->ax_lp = acc_att.float_val[0];
        ctx->ay_lp = acc_att.float_val[1];
        ctx->az_lp = acc_att.float_val[2];
        
        ctx->needs_alignment = 0;
    }
    

    // 2) accel low-pass for correction
    if (ctx->ax_lp==0 && ctx->ay_lp==0 && ctx->az_lp==0) {
        ctx->ax_lp = acc_att.float_val[0];
        ctx->ay_lp = acc_att.float_val[1];
        ctx->az_lp = acc_att.float_val[2];
    } else {
        ctx->ax_lp = (1-ACC_LP_ALPHA)*ctx->ax_lp + ACC_LP_ALPHA*acc_att.float_val[0];
        ctx->ay_lp = (1-ACC_LP_ALPHA)*ctx->ay_lp + ACC_LP_ALPHA*acc_att.float_val[1];
        ctx->az_lp = (1-ACC_LP_ALPHA)*ctx->az_lp + ACC_LP_ALPHA*acc_att.float_val[2];
    }

    // 3) still detect + bias boost (uses un-thresholded calibrated values, same as original)
    bool still_now = is_still_app(
        gyro_cali_dps->float_val[0], gyro_cali_dps->float_val[1], gyro_cali_dps->float_val[2],
        accl_cali_mps2->float_val[0], accl_cali_mps2->float_val[1], accl_cali_mps2->float_val[2]
    );
    uint32_t now_ms = millis();
    if (still_now) {
        if (!ctx->just_still) {
            ctx->just_still  = 1;
            ctx->still_ts_ms = now_ms;
            ctx->bias_alpha_base = ahrs_attitude.getGyroBiasAlpha();
            ahrs_attitude.setGyroBiasAlpha(BIAS_ALPHA_BOOST);
        } else {
            if ((uint32_t)(now_ms - ctx->still_ts_ms) > BIAS_ALPHA_BOOST_MS) {
                ahrs_attitude.setGyroBiasAlpha(ctx->bias_alpha_base);
                ctx->just_still = 0;
            }
        }
    } else {
        if (ctx->just_still) {
            ahrs_attitude.setGyroBiasAlpha(ctx->bias_alpha_base);
            ctx->just_still = 0;
        }
    }

    // 4) dt seconds (nominal Ts on first sample / large gaps)
    float Ts = 1.0f / (ctx->nominal_fs_hz > 1.0f ? ctx->nominal_fs_hz : 100.0f);

    uint32_t now_us = micros();
    float dt_curr;
    if (ctx->last_ts_us == 0) {
        dt_curr = Ts;
    } else {
        uint32_t du = now_us - ctx->last_ts_us;
        dt_curr = (float)du * 1e-6f;
    }
    ctx->last_ts_us = now_us;

    if (dt_curr < 1e-5f) dt_curr = 1e-5f;
    if (dt_curr > 0.05f) {
        dt_curr = Ts;
        ctx->have_prev = 0;
    }

    // 5) coning two-sample -> equivalent rate
    float w_eq_dps[3];
    if (ctx->have_prev) {
        coning_two_sample_dps(ctx->w_prev_dps, ctx->dt_prev_s,
                              gyro_att.float_val, dt_curr,
                              w_eq_dps);
    } else {
        w_eq_dps[0] = gyro_att.float_val[0];
        w_eq_dps[1] = gyro_att.float_val[1];
        w_eq_dps[2] = gyro_att.float_val[2];
        ctx->have_prev = 1;
    }

    // 6) attitude update
    ahrs_attitude.updateIMU_dualAccel(
        w_eq_dps[0], w_eq_dps[1], w_eq_dps[2],
        ctx->ax_lp, ctx->ay_lp, ctx->az_lp,
        acc_att.float_val[0], acc_att.float_val[1], acc_att.float_val[2]
    );

    // 7) pitch/roll
    att_out->float_val[0] = ahrs_attitude.getLocalPitch();
    att_out->float_val[1] = ahrs_attitude.getLocalRoll();

    // 8) gimbal-lock hysteresis + projected yaw integration
    float pitch_deg_now = att_out->float_val[0];
    if (!ctx->yaw_gl_locked && fabsf(pitch_deg_now) >= GL_ENTER_DEG) {
        ctx->yaw_gl_locked = 1;
        ctx->yaw_hold_deg = ahrs_attitude.getLocalYaw();
    }
    if (ctx->yaw_gl_locked && fabsf(pitch_deg_now) <= GL_EXIT_DEG) {
        ctx->yaw_gl_locked = 0;
    }

    if (ctx->yaw_gl_locked) {
        float q0,q1,q2,q3; ahrs_attitude.getQuatWS(q0,q1,q2,q3);
        float Rwb[9]; quat_to_Rwb(q0,q1,q2,q3, Rwb);

        float omega_w_dps[3] = {
            Rwb[0]*w_eq_dps[0] + Rwb[1]*w_eq_dps[1] + Rwb[2]*w_eq_dps[2],
            Rwb[3]*w_eq_dps[0] + Rwb[4]*w_eq_dps[1] + Rwb[5]*w_eq_dps[2],
            Rwb[6]*w_eq_dps[0] + Rwb[7]*w_eq_dps[1] + Rwb[8]*w_eq_dps[2]
        };
        ctx->yaw_hold_deg += omega_w_dps[2] * dt_curr;
        att_out->float_val[2] = ctx->yaw_hold_deg;
    } else {
        att_out->float_val[2] = ahrs_attitude.getLocalYaw();
        ctx->yaw_hold_deg = att_out->float_val[2];
    }

    // roll forward coning state
    ctx->w_prev_dps[0] = gyro_att.float_val[0];
    ctx->w_prev_dps[1] = gyro_att.float_val[1];
    ctx->w_prev_dps[2] = gyro_att.float_val[2];
    ctx->dt_prev_s = dt_curr;

    // 9) circular MA on angles (deg)
    float raw_pitch = att_out->float_val[0];
    float raw_roll  = att_out->float_val[1];
    float raw_yaw   = att_out->float_val[2];

    att_out->float_val[0] = circle_ma_filter_deg(raw_pitch,
                                pitch_sin_buf, pitch_cos_buf,
                                &ctx->pitch_sin_sum, &ctx->pitch_cos_sum,
                                &ctx->pitch_idx, &ctx->pitch_count,
                                ATT_MA_N);
    att_out->float_val[1] = circle_ma_filter_deg(raw_roll,
                                roll_sin_buf, roll_cos_buf,
                                &ctx->roll_sin_sum, &ctx->roll_cos_sum,
                                &ctx->roll_idx, &ctx->roll_count,
                                ATT_MA_N);
    att_out->float_val[2] = circle_ma_filter_deg(raw_yaw,
                                yaw_sin_buf, yaw_cos_buf,
                                &ctx->yaw_sin_sum, &ctx->yaw_cos_sum,
                                &ctx->yaw_idx, &ctx->yaw_count,
                                ATT_MA_N);
}
