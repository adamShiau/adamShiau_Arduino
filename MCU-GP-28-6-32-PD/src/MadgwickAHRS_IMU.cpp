#include "MadgwickAHRS_IMU.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef   512.0f          // default sample frequency in Hz
#define betaDef         0.1f            // 2 * proportional gain

//============================================================================================
// Constructor

Madgwick::Madgwick() {
    // beta = betaDef;
    // q0 = 1.0f;
    // q1 = 0.0f;
    // q2 = 0.0f;
    // q3 = 0.0f;
    // invSampleFreq = 1.0f / sampleFreqDef;
    // anglesComputed = 0;
    // roll = pitch = yaw = 0.0f;

	beta = betaDef;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    invSampleFreq = 1.0f / sampleFreqDef;
    anglesComputed = 0;
    roll = pitch = yaw = 0.0f;

    // 奇異點保護/unwrap 初始
    yaw_deg_prev  = 0.0f;
    yaw_deg_cont  = 0.0f;
    roll_deg_prev = 0.0f;
    has_prev_angles = false;
    gimbalLockEnable   = true;    // 預設啟用
    gimbalLockPitchDeg = 89.0f;   // 接近 90° 時鎖定

    // 陀螺偏置學習（降低飄）
    gyroBiasEnable = true;
    bgx_dps = bgy_dps = bgz_dps = 0.0f;
    biasAlpha = 0.002f;           // 偏置學習速率（小一點更穩）
    stillGyroThreshDps = 1.0f;    // 靜止判定（gyro）
    stillAccTolG       = 0.05f;   // 靜止判定（acc ±5%）
}

//=============================================================================================
// AHRS + Mag（若 mx my mz 為 0 會 fallback 到 IMU）

void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az,
                      float mx, float my, float mz) {
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // --- 陀螺：度/秒 → rad/s，並扣除偏置（降低飄）---
    if (gyroBiasEnable && isStill(gx, gy, gz, ax, ay, az)) {
        // 學習偏置（以當前讀值視為偏置，慢慢收斂）
        bgx_dps = (1.0f - biasAlpha) * bgx_dps + biasAlpha * gx;
        bgy_dps = (1.0f - biasAlpha) * bgy_dps + biasAlpha * gy;
        bgz_dps = (1.0f - biasAlpha) * bgz_dps + biasAlpha * gz;
    }
    gx -= bgx_dps; gy -= bgy_dps; gz -= bgz_dps;

    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

    // --- gyro 推進 ---
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // --- 加速度/磁力計有效才修正 ---
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // normalize acc/mag
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // 省算輔助量
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0   = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;

        q0q0 = q0*q0; q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3;
        q1q1 = q1*q1; q1q2 = q1*q2; q1q3 = q1*q3;
        q2q2 = q2*q2; q2q3 = q2*q3; q3q3 = q3*q3;

        // 地磁參考
        hx = mx * q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
        hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
        _2bx = sqrtf(hx*hx + hy*hy);
        _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // 梯度下降修正
        float s0 = -_2q2 * (2.0f*q1q3 - _2q0q2 - ax) + _2q1 * (2.0f*q0q1 + _2q2q3 - ay)
                 - _2bz * q2 * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                 + (-_2bx*q3 + _2bz*q1) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                 + _2bx * q2 * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        float s1 =  _2q3 * (2.0f*q1q3 - _2q0q2 - ax) + _2q0 * (2.0f*q0q1 + _2q2q3 - ay)
                 - 4.0f*q1*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
                 + _2bz*q3 * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                 + (_2bx*q2 + _2bz*q0) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                 + (_2bx*q3 - _4bz*q1) * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        float s2 = -_2q0 * (2.0f*q1q3 - _2q0q2 - ax) + _2q3 * (2.0f*q0q1 + _2q2q3 - ay)
                 - 4.0f*q2*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
                 + (-_4bx*q2 - _2bz*q0) * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                 + (_2bx*q1 + _2bz*q3) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                 + (_2bx*q0 - _4bz*q2) * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        float s3 =  _2q1 * (2.0f*q1q3 - _2q0q2 - ax) + _2q2 * (2.0f*q0q1 + _2q2q3 - ay)
                 + (-_4bx*q3 + _2bz*q1) * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                 + (-_2bx*q0 + _2bz*q2) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                 + _2bx * q1 * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        // 步長正規化
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;

        // 回授
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 積分
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // 正規化
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;

    anglesComputed = 0;
}

//=============================================================================================
// IMU only

void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // --- 靜止時學習偏置，然後扣除（降低 yaw 漂移）---
    if (gyroBiasEnable && isStill(gx, gy, gz, ax, ay, az)) {
        bgx_dps = (1.0f - biasAlpha) * bgx_dps + biasAlpha * gx;
        bgy_dps = (1.0f - biasAlpha) * bgy_dps + biasAlpha * gy;
        bgz_dps = (1.0f - biasAlpha) * bgz_dps + biasAlpha * gz;
    }
    gx -= bgx_dps; gy -= bgy_dps; gz -= bgz_dps;

    // 度/秒 → rad/s
    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

    // gyro 推進
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // 有效加速度才修正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // normalize acc
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // 輔助量
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        // 梯度下降修正（原作者 IMU 版）
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2*q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1*q1 + _8q2*q2*q2 + _4q2*az;
        s3 = 4.0f*q1*q1*q3 - _2q1*ax + 4.0f*q2*q2*q3 - _2q2*ay;

        // 步長正規化
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;

        // 回授
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 積分
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // 正規化
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;

    anglesComputed = 0;
}

//=============================================================================================
// inv sqrt

float Madgwick::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//=============================================================================================
// Euler 計算（含奇異點保護 + yaw 連續化）

void Madgwick::computeAngles() {
    // 先用標準 ZYX（yaw-pitch-roll）由四元數換角
    // 為了奇異點處理，採用 test = 2*(w*y - z*x)
    float w=q0, x=q1, y=q2, z=q3;
    float test = 2.0f * (w*y - z*x);

    float roll_rad, pitch_rad, yaw_rad;

    // ---- 奇異點：pitch ≈ +90° 或 -90° ----
    float lim = sinf((gimbalLockPitchDeg) * (float)M_PI/180.0f); // 對應閾值的正弦
    if (gimbalLockEnable && (test >  lim)) {
        // north pole（+90°）：鎖定 pitch=+90°，roll/yaw 使用上一幀值（避免亂跳）
        pitch_rad =  + (float)M_PI * 0.5f;
        // 若有上一幀角度，就沿用（維持 GUI 穩定），否則設為 0
        float roll_deg  = has_prev_angles ? roll_deg_prev  : 0.0f;
        float yaw_deg   = has_prev_angles ? yaw_deg_prev   : (2.0f * atan2f(x, w) * 180.0f/(float)M_PI);
        // 實際回存（弧度）
        roll_rad = roll_deg * (float)M_PI/180.0f;
        yaw_rad  = yaw_deg  * (float)M_PI/180.0f;
    }
    else if (gimbalLockEnable && (test < -lim)) {
        // south pole（-90°）
        pitch_rad =  - (float)M_PI * 0.5f;
        float roll_deg  = has_prev_angles ? roll_deg_prev  : 0.0f;
        float yaw_deg   = has_prev_angles ? yaw_deg_prev   : (-2.0f * atan2f(x, w) * 180.0f/(float)M_PI);
        roll_rad = roll_deg * (float)M_PI/180.0f;
        yaw_rad  = yaw_deg  * (float)M_PI/180.0f;
    }
    else {
        // 一般區域：標準公式
        roll_rad  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
        pitch_rad = asinf(test);
        yaw_rad   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    }

    // 存入成員
    roll  = roll_rad;
    pitch = pitch_rad;
    yaw   = yaw_rad;
    anglesComputed = 1;

    // --- 角度（度）做 unwrap 與鎖定回存（給下次使用）---
    float roll_deg_now = roll  * 180.0f/(float)M_PI;
    float yaw_deg_now  = yaw   * 180.0f/(float)M_PI;

    // unwrap（讓 yaw 連續）
    yaw_deg_cont = has_prev_angles ? unwrapDeg(yaw_deg_prev, yaw_deg_now) : yaw_deg_now;

    // 更新 prev
    yaw_deg_prev  = yaw_deg_cont;   // 用連續後的當作 prev
    roll_deg_prev = roll_deg_now;
    has_prev_angles = true;
}

//=============================================================================================
// STOP 用：重置姿態（保留濾波參數與 gyro 偏置）
// - 四元數 -> 單位四元數
// - R/P/Y 快取清零
// - 連續角 (unwrap) 狀態清零
// - 不動 invSampleFreq / beta / 偏置學習狀態（避免重新暖機）
void Madgwick::resetAttitude() {
    // 四元數回到無旋轉
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;

    // 歐拉角快取清零
    roll = 0.0f; pitch = 0.0f; yaw = 0.0f;
    anglesComputed = 0;

    // 連續角/鎖定相關狀態清零
    yaw_deg_prev   = 0.0f;
    yaw_deg_cont   = 0.0f;
    roll_deg_prev  = 0.0f;
    has_prev_angles = false;

    // 注意：不清 gyro 偏置（bgx/bgy/bgz），讓重新開始時 yaw 漂移較小
    // 若你確實想同時清偏置，可取消下面註解：
    // bgx_dps = bgy_dps = bgz_dps = 0.0f;
}
