//=============================================================================================
// MadgwickAHRS_IMU.cpp
//=============================================================================================
#include "MadgwickAHRS_IMU.h"

// 預設
#define sampleFreqDef   512.0f   // 內部預設採樣率；實務請用 begin(fs) 覆蓋
#define betaDef         0.1f     // Madgwick 增益：大=收斂快但噪、大甩易被拉偏；小=平滑但回正慢

//=============================================================================================
// 建構

/* Tuning cheatsheet:
- 手持：beta=0.08~0.12, accTolG=0.12~0.18, gyroHigh=120~180, betaMin=0.25~0.3
- 載具：beta=0.06~0.10, accTolG=0.18~0.25, gyroHigh=180~240
- 激烈：beta=0.05~0.08,  accTolG=0.22~0.30, gyroHigh=240~300, betaMin=0.2~0.25
- 漂移重：開 gyroBiasEnable、biasAlpha=0.003~0.005、stillGyro=1.5、stillAccTolG=0.06
- Pitch±90° 跳：gimbalLockEnable=true, gimbalLockPitchDeg=89.5~89.9
*/

Madgwick::Madgwick() {
    beta = betaDef;                          // 主增益：一般 0.05~0.15
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f; // 初始姿態（world←sensor）
    invSampleFreq = 1.0f / sampleFreqDef;    // 由 begin(fs) 實際設定
    anglesComputed = 0;
    roll = pitch = yaw = 0.0f;

// ---- 奇異點保護 / unwrap ----
    yaw_deg_prev  = 0.0f;                    // unwrap 狀態
    yaw_deg_cont  = 0.0f;
    roll_deg_prev = 0.0f;
    has_prev_angles   = false;
    gimbalLockEnable  = true;                // Pitch 近 ±90° 鎖定 roll/yaw
    gimbalLockPitchDeg= 89.0f;               // 88~89.9°；越大越晚鎖

    // ---- 陀螺偏置學習（靜止時）----
    gyroBiasEnable = true;                   // 自動學偏置，降漂移
    bgx_dps = bgy_dps = bgz_dps = 0.0f;      // 當前學到的偏置（deg/s）
    biasAlpha = 0.002f;                      // 學習速率：小=穩但慢(0.001~0.005)
    stillGyroThreshDps = 1.0f;               // 靜止判定角速閾值(合計)：0.5~2 dps
    stillAccTolG       = 0.05f;              // 靜止判定加速度容忍(‖a‖ vs 1g)：0.03~0.07

    // ---- 動態加速度權重（大甩降權）----
    dynAccWeightEnable = true;               // 依 ‖a‖ 偏離與 |gyro| 動態調 β
    g0 = 9.80665f;                           // 1g（m/s^2）
    accTolG = 0.15f;                         // ‖a‖ 相對 1g 容忍：0.1~0.25（大甩取大）
    gyroHighThreshDps = 150.0f;              // |gx|+|gy|+|gz| 高門檻：100~300 dps
    betaMinRatio = 0.25f;                    // β 最低比例：0.2~0.4（太低回正慢）
    betaSmoothAlpha = 0.20f;                 // β 平滑：0.1~0.3（大=反應快）
    betaEffLP = beta;                        // 目前生效 β（內部狀態）

    // ---- Sensor→Case 固定旋轉（幾何對應）----
    qc0 = 1.0f; qc1 = 0.0f; qc2 = 0.0f; qc3 = 0.0f; // 預設 case==sensor；用 setSensorToCase* 設定
}

void Madgwick::init(float data_rate) {
    begin(data_rate); // 設定採樣率
    // const float Rcs[9] = { 1,0,0,  0,1,0,  0,0,1 }; // 原始 Sensor frame，對應 ENU
    const float Rcs[9] = { 0,-1,0,  -1,0,0,  0,0,-1 };   // case frame, 對應 NED 座標系
    setSensorToCaseMatrix(Rcs); // 設定 Sensor→Case 固定旋轉
    setLocalFrameNED(true);     // 設定 Local 世界座標為 NED
    setGyroBiasLearning(true);  // 啟用靜止時自動估測偏置
    setDynamicAccelWeight(true); // 啟用動態加權
    setGimbalLockGuard(true, 89.0f); // 啟用奇異點保護
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
// 動態 β 計算

float Madgwick::computeBetaEffective(float gx_dps, float gy_dps, float gz_dps,
                                     float ax, float ay, float az) {
    if (!dynAccWeightEnable) { betaEffLP = beta; return betaEffLP; }

    // |‖a‖-1g|
    float anorm = sqrtf(ax*ax + ay*ay + az*az);
    float devRel = fabsf(anorm - g0) / g0;
    float k_acc = 1.0f / (1.0f + (devRel / accTolG)); // 偏離越大→越小

    // |gyro|
    float gsum = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);
    float k_gyro;
    if (gsum >= gyroHighThreshDps)      k_gyro = 0.3f;
    else if (gsum <= 0.0f)              k_gyro = 1.0f;
    else {
        float r = gsum / gyroHighThreshDps; // 0..1
        k_gyro = 1.0f - 0.7f * r;           // 1 → 0.3 線性
    }

    float k = k_acc * k_gyro;
    if (k < betaMinRatio) k = betaMinRatio;
    if (k > 1.0f)         k = 1.0f;

    float betaTarget = beta * k;
    betaEffLP = (1.0f - betaSmoothAlpha)*betaEffLP + betaSmoothAlpha*betaTarget;
    return betaEffLP;
}

//=============================================================================================
// AHRS + Mag（若 mx=my=mz=0 則 fallback 至 IMU）

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

    // 偏置學習 + 扣除
    if (gyroBiasEnable && isStill(gx, gy, gz, ax, ay, az)) {
        bgx_dps = (1.0f - biasAlpha) * bgx_dps + biasAlpha * gx;
        bgy_dps = (1.0f - biasAlpha) * bgy_dps + biasAlpha * gy;
        bgz_dps = (1.0f - biasAlpha) * bgz_dps + biasAlpha * gz;
    }
    gx -= bgx_dps; gy -= bgy_dps; gz -= bgz_dps;

    // deg/s → rad/s
    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

    // gyro 推進
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // 有效量測才修正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // normalize acc/mag
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // 省算
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
        _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;

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

        // 正規化步長
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;

        // 動態 β
        float betaEff = computeBetaEffective(gx*57.29578f, gy*57.29578f, gz*57.29578f, ax*g0, ay*g0, az*g0);
        qDot1 -= betaEff * s0;
        qDot2 -= betaEff * s1;
        qDot3 -= betaEff * s2;
        qDot4 -= betaEff * s3;
    }

    // 積分 + 正規化
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

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
    float _2q0,_2q1,_2q2,_2q3,_4q0,_4q1,_4q2,_8q1,_8q2;
    float q0q0,q1q1,q2q2,q3q3;

    // 偏置學習 + 扣除
    if (gyroBiasEnable && isStill(gx, gy, gz, ax, ay, az)) {
        bgx_dps = (1.0f - biasAlpha) * bgx_dps + biasAlpha * gx;
        bgy_dps = (1.0f - biasAlpha) * bgy_dps + biasAlpha * gy;
        bgz_dps = (1.0f - biasAlpha) * bgz_dps + biasAlpha * gz;
    }
    gx -= bgx_dps; gy -= bgy_dps; gz -= bgz_dps;

    // deg/s → rad/s
    const float d2r = 0.0174533f;
    float gx_rad = gx*d2r, gy_rad = gy*d2r, gz_rad = gz*d2r;

    // gyro 推進
    qDot1 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
    qDot2 = 0.5f * ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
    qDot3 = 0.5f * ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
    qDot4 = 0.5f * ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

    // 有效加速度才修正
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // normalize acc
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        float axu = ax*recipNorm, ayu = ay*recipNorm, azu = az*recipNorm;

        // 輔助量
        _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
        _4q0 = 4.0f*q0; _4q1 = 4.0f*q1; _4q2 = 4.0f*q2;
        _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        // 梯度下降修正（IMU 版）
        s0 = _4q0*q2q2 + _2q2*axu + _4q0*q1q1 - _2q1*ayu;
        s1 = _4q1*q3q3 - _2q3*axu + 4.0f*q0q0*q1 - _2q0*ayu - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*azu;
        s2 = 4.0f*q0q0*q2 + _2q0*axu + _4q2*q3q3 - _2q3*ayu - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*azu;
        s3 = 4.0f*q1*q1*q3 - _2q1*axu + 4.0f*q2*q2*q3 - _2q2*ayu;

        // 正規化步長
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;

        // 動態 β（這裡 computeBetaEffective 接收 dps 與 m/s^2）
        float betaEff = computeBetaEffective(gx, gy, gz, ax, ay, az);
        qDot1 -= betaEff * s0;
        qDot2 -= betaEff * s1;
        qDot3 -= betaEff * s2;
        qDot4 -= betaEff * s3;
    }

    // 積分 + 正規化
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;

    anglesComputed = 0;
}

void Madgwick::updateIMU_dualAccel(float gx, float gy, float gz,
                                   float ax_lp, float ay_lp, float az_lp,
                                   float ax_raw, float ay_raw, float az_raw) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0,_2q1,_2q2,_2q3,_4q0,_4q1,_4q2,_8q1,_8q2;
    float q0q0,q1q1,q2q2,q3q3;

    // 偏置學習（用原始量測判斷 still/否；沿用原本 isStill()）
    if (gyroBiasEnable && isStill(gx, gy, gz, ax_raw, ay_raw, az_raw)) {
        bgx_dps = (1.0f - biasAlpha) * bgx_dps + biasAlpha * gx;
        bgy_dps = (1.0f - biasAlpha) * bgy_dps + biasAlpha * gy;
        bgz_dps = (1.0f - biasAlpha) * bgz_dps + biasAlpha * gz;
    }
    gx -= bgx_dps; gy -= bgy_dps; gz -= bgz_dps;

    const float d2r = 0.0174533f;
    float gx_rad = gx*d2r, gy_rad = gy*d2r, gz_rad = gz*d2r;

    // gyro 推進
    qDot1 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
    qDot2 = 0.5f * ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
    qDot3 = 0.5f * ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
    qDot4 = 0.5f * ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

    // 用低通後的加速度做 normalize 與梯度下降
    if (!((ax_lp == 0.0f) && (ay_lp == 0.0f) && (az_lp == 0.0f))) {
        recipNorm = invSqrt(ax_lp*ax_lp + ay_lp*ay_lp + az_lp*az_lp);
        float axu = ax_lp*recipNorm, ayu = ay_lp*recipNorm, azu = az_lp*recipNorm;

        _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
        _4q0 = 4.0f*q0; _4q1 = 4.0f*q1; _4q2 = 4.0f*q2;
        _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        s0 = _4q0*q2q2 + _2q2*axu + _4q0*q1q1 - _2q1*ayu;
        s1 = _4q1*q3q3 - _2q3*axu + 4.0f*q0q0*q1 - _2q0*ayu - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*azu;
        s2 = 4.0f*q0q0*q2 + _2q0*axu + _4q2*q3q3 - _2q3*ayu - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*azu;
        s3 = 4.0f*q1*q1*q3 - _2q1*axu + 4.0f*q2*q2*q3 - _2q2*ayu;

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;

        // 動態 β：用「原始（未低通）」加速度判斷
        float betaEff = computeBetaEffective(gx, gy, gz, ax_raw, ay_raw, az_raw);
        qDot1 -= betaEff * s0;
        qDot2 -= betaEff * s1;
        qDot3 -= betaEff * s2;
        qDot4 -= betaEff * s3;
    }

    // 積分 + 正規化
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;

    anglesComputed = 0;
}


//=============================================================================================
// Euler 計算（含奇異點保護 + yaw 連續化）

void Madgwick::computeAngles() {
    float w=q0, x=q1, y=q2, z=q3;
    float test = 2.0f * (w*y - z*x);

    float roll_rad, pitch_rad, yaw_rad;
    float lim = sinf((gimbalLockPitchDeg) * (float)M_PI/180.0f);

    if (gimbalLockEnable && (test >  lim)) {
        // +90°
        pitch_rad =  + (float)M_PI * 0.5f;
        float roll_deg = has_prev_angles ? roll_deg_prev : 0.0f;
        float yaw_deg  = has_prev_angles ? yaw_deg_prev  : (2.0f * atan2f(x, w) * 57.29578f);
        roll_rad = roll_deg * (float)M_PI/180.0f;
        yaw_rad  = yaw_deg  * (float)M_PI/180.0f;
    } else if (gimbalLockEnable && (test < -lim)) {
        // -90°
        pitch_rad =  - (float)M_PI * 0.5f;
        float roll_deg = has_prev_angles ? roll_deg_prev : 0.0f;
        float yaw_deg  = has_prev_angles ? yaw_deg_prev  : (-2.0f * atan2f(x, w) * 57.29578f);
        roll_rad = roll_deg * (float)M_PI/180.0f;
        yaw_rad  = yaw_deg  * (float)M_PI/180.0f;
    } else {
        // 一般區域
        roll_rad  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
        pitch_rad = asinf(2.0f*(w*y - z*x));
        yaw_rad   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    }

    // 存入快取
    roll  = roll_rad;
    pitch = pitch_rad;
    yaw   = yaw_rad;
    anglesComputed = 1;

    // unwrap
    float roll_deg_now = roll  * 57.29578f;
    float yaw_deg_now  = yaw   * 57.29578f;
    yaw_deg_cont = has_prev_angles ? unwrapDeg(yaw_deg_prev, yaw_deg_now) : yaw_deg_now;
    yaw_deg_prev  = yaw_deg_cont;
    roll_deg_prev = roll_deg_now;
    has_prev_angles = true;
}

//=============================================================================================
// STOP 用：重置姿態（可選清偏置）

void Madgwick::resetAttitude(bool resetBias) {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    roll = pitch = yaw = 0.0f;
    anglesComputed = 0;
    yaw_deg_prev = yaw_deg_cont = 0.0f;
    roll_deg_prev = 0.0f;
    has_prev_angles = false;
    betaEffLP = beta;
    if (resetBias) { bgx_dps = bgy_dps = bgz_dps = 0.0f; }
}

//=============================================================================================
// Sensor→Case 固定旋轉設定 & Case 姿態輸出

void Madgwick::setSensorToCaseRPYdeg(float r_deg,float p_deg,float y_deg) {
    const float D2R = 0.01745329252f;
    float cr = cosf(0.5f*r_deg*D2R), sr = sinf(0.5f*r_deg*D2R);
    float cp = cosf(0.5f*p_deg*D2R), sp = sinf(0.5f*p_deg*D2R);
    float cy = cosf(0.5f*y_deg*D2R), sy = sinf(0.5f*y_deg*D2R);
    // ZYX: q = Rz(yaw) ⊗ Ry(pitch) ⊗ Rx(roll)
    float w = cr*cp*cy + sr*sp*sy;
    float x = sr*cp*cy - cr*sp*sy;
    float y = cr*sp*cy + sr*cp*sy;
    float z = cr*cp*sy - sr*sp*cy;
    qc0=w; qc1=x; qc2=y; qc3=z; quatNormalize(qc0,qc1,qc2,qc3);
}

void Madgwick::setSensorToCaseMatrix(const float R[9]) {
    // row-major: [r00 r01 r02; r10 r11 r12; r20 r21 r22]
    float tr = R[0] + R[4] + R[8];
    float w,x,y,z;
    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (R[7] - R[5]) / s;
        y = (R[2] - R[6]) / s;
        z = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        float s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        w = (R[7] - R[5]) / s;
        x = 0.25f * s;
        y = (R[1] + R[3]) / s;
        z = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        float s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        w = (R[2] - R[6]) / s;
        x = (R[1] + R[3]) / s;
        y = 0.25f * s;
        z = (R[5] + R[7]) / s;
    } else {
        float s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        w = (R[3] - R[1]) / s;
        x = (R[2] + R[6]) / s;
        y = (R[5] + R[7]) / s;
        z = 0.25f * s;
    }
    qc0=w; qc1=x; qc2=y; qc3=z; quatNormalize(qc0,qc1,qc2,qc3);
}

float Madgwick::getCaseRoll()  {
    if (!anglesComputed) computeAngles();
    // q_WC = q_WS ⊗ conj(q_CS)
    float wc0,wc1,wc2,wc3;
    quatMul(q0,q1,q2,q3,   qc0,-qc1,-qc2,-qc3,  wc0,wc1,wc2,wc3);
    float r,p,y; eulerZYX_from_quat_deg(wc0,wc1,wc2,wc3, r,p,y);
    return r;
}
float Madgwick::getCasePitch() {
    if (!anglesComputed) computeAngles();
    float wc0,wc1,wc2,wc3;
    quatMul(q0,q1,q2,q3,   qc0,-qc1,-qc2,-qc3,  wc0,wc1,wc2,wc3);
    float r,p,y; eulerZYX_from_quat_deg(wc0,wc1,wc2,wc3, r,p,y);
    return p;
}
float Madgwick::getCaseYaw()   {
    if (!anglesComputed) computeAngles();
    float wc0,wc1,wc2,wc3;
    quatMul(q0,q1,q2,q3,   qc0,-qc1,-qc2,-qc3,  wc0,wc1,wc2,wc3);
    float r,p,y; eulerZYX_from_quat_deg(wc0,wc1,wc2,wc3, r,p,y);
    return y;
}

void Madgwick::setLocalFrameNED(bool enable) {
  if (!enable) {
    // Local = ENU → 身分四元數
    qwl0=1.0f; qwl1=qwl2=qwl3=0.0f;
    return;
  }
  // Local = NED：R_L←E = [[0,1,0],[1,0,0],[0,0,-1]]
  const float R[9] = { 0,1,0,  1,0,0,  0,0,-1 };
  quatFromR(R, qwl0,qwl1,qwl2,qwl3); // 會得到 (w,x,y,z) ≈ (0, √2/2, √2/2, 0)
}

void Madgwick::setLocalFromENUMatrix(const float R[9]) {
  quatFromR(R, qwl0,qwl1,qwl2,qwl3);
}

float Madgwick::getLocalCaseRoll() {
  if (!anglesComputed) computeAngles();
  // q_WC = q_WS ⊗ conj(q_CS)
  float cq0=qc0, cq1=-qc1, cq2=-qc2, cq3=-qc3; // conj(q_CS)
  float wc0,wc1,wc2,wc3;
  quatMul(q0,q1,q2,q3,  cq0,cq1,cq2,cq3,  wc0,wc1,wc2,wc3);
  // q_LC = q_LW ⊗ q_WC
  float lc0,lc1,lc2,lc3;
  quatMul(qwl0,qwl1,qwl2,qwl3,  wc0,wc1,wc2,wc3,  lc0,lc1,lc2,lc3);
  float r,p,y; eulerZYX_from_quat_deg(lc0,lc1,lc2,lc3, r,p,y);
  return r;
}
float Madgwick::getLocalCasePitch() {
  if (!anglesComputed) computeAngles();
  float cq0=qc0, cq1=-qc1, cq2=-qc2, cq3=-qc3;
  float wc0,wc1,wc2,wc3; quatMul(q0,q1,q2,q3,  cq0,cq1,cq2,cq3,  wc0,wc1,wc2,wc3);
  float lc0,lc1,lc2,lc3; quatMul(qwl0,qwl1,qwl2,qwl3, wc0,wc1,wc2,wc3, lc0,lc1,lc2,lc3);
  float r,p,y; eulerZYX_from_quat_deg(lc0,lc1,lc2,lc3, r,p,y);
  return p;
}
float Madgwick::getLocalCaseYaw() {
  if (!anglesComputed) computeAngles();
  float lc0,lc1,lc2,lc3;
  currentLocalCaseQuat(q0,q1,q2,q3, qc0,qc1,qc2,qc3, qwl0,qwl1,qwl2,qwl3, lc0,lc1,lc2,lc3);

  if (yawZeroValid) {
    // q_adj = Rz(-yaw0) ⊗ q_LC
    float aj0,aj1,aj2,aj3;
    quatMul(qz0w,qz0x,qz0y,qz0z, lc0,lc1,lc2,lc3, aj0,aj1,aj2,aj3);
    float r,p,y; eulerZYX_from_quat_deg(aj0,aj1,aj2,aj3, r,p,y);
    return y;
  } else {
    float r,p,y; eulerZYX_from_quat_deg(lc0,lc1,lc2,lc3, r,p,y);
    return y;
  }
}


void Madgwick::captureYawZeroLocalCase() {
  if (!anglesComputed) computeAngles();
  // 1) 取得當下 Local+Case 的四元數
  float lc0,lc1,lc2,lc3;
  currentLocalCaseQuat(q0,q1,q2,q3, qc0,qc1,qc2,qc3, qwl0,qwl1,qwl2,qwl3, lc0,lc1,lc2,lc3);
  // 2) 取出當下的 yaw（ZYX），只用來做 Rz(-yaw0)
  float r,p,y; eulerZYX_from_quat_deg(lc0,lc1,lc2,lc3, r,p,y);
  setYawZeroDeg(y); // 將當下 yaw 當作零位
}

void Madgwick::setYawZeroDeg(float yaw0_deg) {
  float a = -0.5f * yaw0_deg * 0.01745329252f; // -yaw0/2 (rad)
  qz0w = cosf(a); qz0x = 0.0f; qz0y = 0.0f; qz0z = sinf(a); // Rz(-yaw0)
  yawZeroValid = true;
}

void Madgwick::clearYawZero() {
  yawZeroValid = false;
  qz0w = 1.0f; qz0x = qz0y = qz0z = 0.0f;
}

void Madgwick::sensorVecToCase(const float vS[3], float vC[3]) const {
    // v' = q * (0,v) * q_conj，這裡 q = q_CS
    float w=qc0, x=qc1, y=qc2, z=qc3;
    float tx = 2.0f * (y*vS[2] - z*vS[1]);
    float ty = 2.0f * (z*vS[0] - x*vS[2]);
    float tz = 2.0f * (x*vS[1] - y*vS[0]);
    vC[0] = vS[0] + w*tx + (y*tz - z*ty);
    vC[1] = vS[1] + w*ty + (z*tx - x*tz);
    vC[2] = vS[2] + w*tz + (x*ty - y*tx);
}

void Madgwick::setGyroBiasAlpha(float a) {
    // 夾一個合理範圍，避免填入 0 或過大
    if (a < 1e-6f) a = 1e-6f;
    if (a > 0.1f)  a = 0.1f;
    biasAlpha = a;
}
float Madgwick::getGyroBiasAlpha() const {
    return biasAlpha;
}




