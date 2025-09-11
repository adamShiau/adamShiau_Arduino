//=============================================================================================
// MadgwickAHRS_IMU.h  (IMU only: gyro + accel)
// - 保留原 Madgwick IMU 演算法
// - resetAttitude(resetBias)
// - Pitch±90° 保護（可關閉）與 yaw unwrap
// - 靜止時估測 gyro 偏置（可關閉）
// - 動態加速度權重（大甩降權、平穩恢復）
// - ★ 新增：Sensor→Case 固定旋轉；提供 getCaseRoll/Pitch/Yaw()
//=============================================================================================
#ifndef MadgwickAHRS_IMU_h
#define MadgwickAHRS_IMU_h

#include <Arduino.h>
#include <math.h>

class Madgwick {
private:
    // --- 工具 ---
    static float invSqrt(float x);
    static inline void quatNormalize(float& w,float& x,float& y,float& z) {
        float n = sqrtf(w*w + x*x + y*y + z*z);
        if (n == 0.0f) { w=1.0f; x=y=z=0.0f; } else { w/=n; x/=n; y/=n; z/=n; }
    }
    static inline void quatMul( // r = a ⊗ b
        float aw,float ax,float ay,float az,
        float bw,float bx,float by,float bz,
        float& rw,float& rx,float& ry,float& rz) {
        rw = aw*bw - ax*bx - ay*by - az*bz;
        rx = aw*bx + ax*bw + ay*bz - az*by;
        ry = aw*by - ax*bz + ay*bw + az*bx;
        rz = aw*bz + ax*by - ay*bx + az*bw;
    }

    static inline void quatConj(float w,float x,float y,float z,
                            float& cw,float& cx,float& cy,float& cz){
        cw=w; cx=-x; cy=-y; cz=-z;
    }
    // q_LC = q_WL←WE ⊗ ( q_WS ⊗ conj(q_CS) )
    static inline void currentLocalCaseQuat(
        float q0,float q1,float q2,float q3,         // q_WS
        float qc0,float qc1,float qc2,float qc3,     // q_CS
        float qwl0,float qwl1,float qwl2,float qwl3, // q_WL←WE
        float& lc0,float& lc1,float& lc2,float& lc3)
    {
        float c0,c1,c2,c3;  quatConj(qc0,qc1,qc2,qc3, c0,c1,c2,c3);
        float wc0,wc1,wc2,wc3;  quatMul(q0,q1,q2,q3, c0,c1,c2,c3, wc0,wc1,wc2,wc3);
        quatMul(qwl0,qwl1,qwl2,qwl3, wc0,wc1,wc2,wc3, lc0,lc1,lc2,lc3);
    }
    static inline void eulerZYX_from_quat_deg(float w,float x,float y,float z,
                                              float& roll_deg,float& pitch_deg,float& yaw_deg) {
        // ZYX: yaw-pitch-roll
        float siny_cosp = 2.0f*(w*z + x*y);
        float cosy_cosp = 1.0f - 2.0f*(y*y + z*z);
        float yaw   = atan2f(siny_cosp, cosy_cosp);
        float sinp  = 2.0f*(w*y - z*x);
        float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(M_PI/2.0f, sinp) : asinf(sinp);
        float sinr_cosp = 2.0f*(w*x + y*z);
        float cosr_cosp = 1.0f - 2.0f*(x*x + y*y);
        float roll  = atan2f(sinr_cosp, cosr_cosp);
        const float R2D = 57.29578f;
        roll_deg  = roll*R2D; pitch_deg = pitch*R2D; yaw_deg = yaw*R2D;
    }

    static inline void quatFromR(const float R[9], float& w,float& x,float& y,float& z) {
        float tr = R[0] + R[4] + R[8];
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
        float n = sqrtf(w*w+x*x+y*y+z*z);
        if (n==0){ w=1;x=y=z=0; } else { w/=n; x/=n; y/=n; z/=n; }
    }


    // --- 濾波參數/狀態 ---
    float beta;                 // algorithm gain（基準）
    float q0,q1,q2,q3;          // q_WS: world←sensor
    float invSampleFreq;        // 1/fs
    float roll,pitch,yaw;       // 快取歐拉角（rad）
    char  anglesComputed;

    // --- 奇異點保護/解跳 & 連續角狀態 ---
    float yaw_deg_prev;         // 上一幀 yaw（deg）
    float yaw_deg_cont;         // unwrap 後連續 yaw（deg）
    float roll_deg_prev;        // 上一幀 roll（deg）
    bool  has_prev_angles;      // 是否已有上一幀
    bool  gimbalLockEnable;     // 啟用奇異點保護
    float gimbalLockPitchDeg;   // 幾度內視為奇異點（預設 89°）

    // --- 陀螺偏置估測（降低漂移）---
    bool  gyroBiasEnable;       // 啟用靜止時自動估測
    float bgx_dps,bgy_dps,bgz_dps; // 三軸偏置（deg/s）
    float biasAlpha;            // 偏置更新係數（小→慢、穩）
    float stillGyroThreshDps;   // 靜止判定：|gyro| 門檻（deg/s）
    float stillAccTolG;         // 靜止判定：|‖a‖-1g|/1g 容忍（如 0.05=±5%）

    // --- 動態檢測權重（加速度信任度）---
    bool  dynAccWeightEnable;   // 啟用動態加權
    float g0;                   // 1g = 9.80665
    float accTolG;              // |‖a‖-1g|/1g 容忍（>此值開始降權）
    float gyroHighThreshDps;    // 角速度高門檻（>降權）
    float betaMinRatio;         // β 的最小比例（0~1）
    float betaSmoothAlpha;      // β 平滑係數（0~1）
    float betaEffLP;            // 當前生效 β（低通後）

    // --- Sensor→Case 固定旋轉（新增）---
    // q_CS: case←sensor（固定，不隨時間變）
    float qc0, qc1, qc2, qc3;

    // 世界座標切換（Local←ENU）：預設=單位 → ENU
    float qwl0 = 1.0f, qwl1 = 0.0f, qwl2 = 0.0f, qwl3 = 0.0f; // q_WL←WE

    // --- Yaw 零位（只修正偏航）---
    bool  yawZeroValid = false;
    float qz0w = 1.0f, qz0x = 0.0f, qz0y = 0.0f, qz0z = 0.0f; // q_Z0 = Rz(-yaw0)


    void computeAngles();
    bool isStill(float gx_dps, float gy_dps, float gz_dps,
                 float ax, float ay, float az) const {
        float gmag = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);
        if (gmag > stillGyroThreshDps) return false;
        float anorm = sqrtf(ax*ax + ay*ay + az*az);
        float dev = fabsf(anorm - g0) / g0;
        return (dev <= stillAccTolG);
    }
    static float unwrapDeg(float prev_deg, float curr_deg) {
        float d = curr_deg - prev_deg;
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return prev_deg + d;
    }
    float computeBetaEffective(float gx_dps, float gy_dps, float gz_dps,
                               float ax, float ay, float az);

public:
    Madgwick();

    // 將 Local frame 設為 NED（true）或 ENU（false, 預設）
    void setLocalFrameNED(bool enable);

    // 也可以直接用 3x3 矩陣設定 Local←ENU（row-major）
    void setLocalFromENUMatrix(const float R[9]);

    // 直接回傳「Local(ENU/NED) + Case」的角（度）
    float getLocalCaseRoll();
    float getLocalCasePitch();
    float getLocalCaseYaw();

    // 從「當下 Local+Case 的 yaw」捕捉零位（之後 yaw 會以此為 0）
    void captureYawZeroLocalCase();

    // 直接指定一個 yaw0（度），等效於之後都加 Rz(-yaw0)
    void setYawZeroDeg(float yaw0_deg);

    // 取消 yaw 零位（回到絕對 yaw）
    void clearYawZero();


    // 基本設定
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }

    // 更新（含磁力計；若 mx=my=mz=0 自動 fallback 到 IMU）
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz);
    // 更新（IMU only）
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    // 取得「sensor frame」對世界的 RPY（deg / rad）
    float getRoll()        { if (!anglesComputed) computeAngles(); return roll  * 57.29578f; }
    float getPitch()       { if (!anglesComputed) computeAngles(); return pitch * 57.29578f; }
    float getYaw()         { if (!anglesComputed) computeAngles(); return yaw   * 57.29578f; }
    float getRollRadians() { if (!anglesComputed) computeAngles(); return roll;  }
    float getPitchRadians(){ if (!anglesComputed) computeAngles(); return pitch; }
    float getYawRadians()  { if (!anglesComputed) computeAngles(); return yaw;   }
    float getYawUnwrappedDeg() const { return yaw_deg_cont; }

    // STOP 用：重置姿態；resetBias=true 時也清三軸偏置
    void resetAttitude(bool resetBias = true);

    // 參數開關
    void setGimbalLockGuard(bool enable, float deg = 89.0f) {
        gimbalLockEnable = enable; gimbalLockPitchDeg = deg;
    }
    void setGyroBiasLearning(bool enable) { gyroBiasEnable = enable; }
    void setDynamicAccelWeight(bool enable) { dynAccWeightEnable = enable; }

    // ====== ★ Sensor→Case 固定旋轉設定 + Case 姿態輸出 ======
    // 以 RPY(度) 設定 q_CS（case←sensor），ZYX 組合（Rz*Ry*Rx）
    void setSensorToCaseRPYdeg(float roll_deg, float pitch_deg, float yaw_deg);
    // 以 3×3 矩陣（row-major）設定 q_CS（case←sensor）
    void setSensorToCaseMatrix(const float R[9]);
    // 回傳「case frame」對世界的 R/P/Y（deg）
    float getCaseRoll();
    float getCasePitch();
    float getCaseYaw();

    // 將 sensor 向量旋到 case：v_case = R_CS * v_sensor
    void sensorVecToCase(const float v_sensor[3], float v_case[3]) const;
};

#endif // MadgwickAHRS_IMU_h
