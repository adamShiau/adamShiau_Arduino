//=============================================================================================
// MadgwickAHRS.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
// 19/02/2012   SOH Madgwick    Magnetometer measurement is normalised
//
//=============================================================================================

#include "MadgwickAHRS_IMU.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef   512.0f          // default sample frequency in Hz
#define betaDef         0.1f            // 2 * proportional gain

//============================================================================================
// Constructor

Madgwick::Madgwick() {
    beta = betaDef;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    invSampleFreq = 1.0f / sampleFreqDef;
    anglesComputed = 0;
    roll = pitch = yaw = 0.0f;
}

//============================================================================================
// AHRS algorithm update (with magnetometer)

void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az,
                      float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // 如果磁力計無效，改用 IMU 更新
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // 陀螺：度/秒 → 弧度/秒
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 由 gyro 得到四元數導數
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // 僅在加速度有效時才做回授修正（避免除以 0）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 正規化加速度與磁力計
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // 省重複計算的輔助量
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0*q0; q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3;
        q1q1 = q1*q1; q1q2 = q1*q2; q1q3 = q1*q3;
        q2q2 = q2*q2; q2q3 = q2*q3; q3q3 = q3*q3;

        // 地磁場參考方向
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx*hx + hy*hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // 梯度下降修正步（原作者公式）
        s0 = -_2q2 * (2.0f*q1q3 - _2q0q2 - ax) + _2q1 * (2.0f*q0q1 + _2q2q3 - ay)
            - _2bz * q2 * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (-_2bx*q3 + _2bz*q1) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + _2bx * q2 * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s1 =  _2q3 * (2.0f*q1q3 - _2q0q2 - ax) + _2q0 * (2.0f*q0q1 + _2q2q3 - ay)
            - 4.0f*q1*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
            + _2bz*q3 * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (_2bx*q2 + _2bz*q0) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + (_2bx*q3 - _4bz*q1) * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s2 = -_2q0 * (2.0f*q1q3 - _2q0q2 - ax) + _2q3 * (2.0f*q0q1 + _2q2q3 - ay)
            - 4.0f*q2*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
            + (-_4bx*q2 - _2bz*q0) * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (_2bx*q1 + _2bz*q3) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + (_2bx*q0 - _4bz*q2) * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s3 =  _2q1 * (2.0f*q1q3 - _2q0q2 - ax) + _2q2 * (2.0f*q0q1 + _2q2q3 - ay)
            + (-_4bx*q3 + _2bz*q1) * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (-_2bx*q0 + _2bz*q2) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + _2bx * q1 * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        // 步長正規化
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // 回授修正
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

    // 四元數正規化
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    anglesComputed = 0;
}

//============================================================================================
// IMU algorithm update (no magnetometer)

void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // 陀螺：度/秒 → 弧度/秒
    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

    // gyro 推進
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // 僅在加速度有效時才做回授修正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 正規化加速度
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // 輔助量
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        // 梯度下降修正步（原作者公式）
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        // 步長正規化
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // 回授修正
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

    // 四元數正規化
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    anglesComputed = 0;
}

//============================================================================================
// Fast inverse square-root

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

//============================================================================================
// 將四元數轉為 ZYX 歐拉角（弧度）並快取

void Madgwick::computeAngles() {
    roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    pitch = asinf(-2.0f * (q1*q3 - q0*q2));
    yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    anglesComputed = 1;
}

//============================================================================================
// === 新增：重置姿態角（給 STOP 用） ===

void Madgwick::resetAttitude() {
    // 四元數回到「無旋轉」
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;

    // 角度快取歸零
    roll = pitch = yaw = 0.0f;
    anglesComputed = 0;

    // 不變更 invSampleFreq / beta；begin() 與濾波參數保持原設定
}
