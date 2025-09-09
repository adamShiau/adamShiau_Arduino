//=============================================================================================
// MadgwickAHRS.h
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
//
//=============================================================================================
#ifndef MadgwickAHRS_IMU_h
#define MadgwickAHRS_IMU_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Class declaration
class Madgwick {
private:
    static float invSqrt(float x);

    // --- 濾波參數/狀態 ---
    float beta;                 // algorithm gain
    float q0, q1, q2, q3;       // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;        // 1 / sampleFreq
    float roll, pitch, yaw;     // 快取的歐拉角（radians）
    char  anglesComputed;

    // --- 奇異點保護/解跳 & 連續角狀態 ---
    float yaw_deg_prev;         // 上一次輸出的 yaw（度）
    float yaw_deg_cont;         // unwrap 後的連續 yaw（度）
    float roll_deg_prev;        // 上一次輸出的 roll（度）
    bool  has_prev_angles;      // 有沒有上一幀角度可用

    // Pitch 接近 ±90° 的鎖定設定
    bool  gimbalLockEnable;     // 啟用奇異點保護
    float gimbalLockPitchDeg;   // 幾度以內視為奇異點（預設 89°）

    // --- 陀螺偏置估測（降低 yaw 漂移）---
    bool  gyroBiasEnable;       // 啟用靜止時自動估測
    float bgx_dps, bgy_dps, bgz_dps; // 陀螺偏置（度/秒）
    float biasAlpha;            // 偏置更新係數（小 → 慢、穩）
    float stillGyroThreshDps;   // 靜止判定：|gyro| < 門檻
    float stillAccTolG;         // 靜止判定：|‖a‖-1g| < 容忍比例（例如 0.05 = ±5%）

    void computeAngles();

    // 工具：unwrap（將 curr_deg 連到 prev_deg 附近，避免 ±180° 跳）
    static float unwrapDeg(float prev_deg, float curr_deg) {
        float d = curr_deg - prev_deg;
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return prev_deg + d;
    }

    // 靜止偵測（用於偏置學習）
    bool isStill(float gx_dps, float gy_dps, float gz_dps,
                 float ax, float ay, float az) const {
        // gyro 大小
        float gmag = fabsf(gx_dps) + fabsf(gy_dps) + fabsf(gz_dps);
        if (gmag > stillGyroThreshDps) return false;
        // acc 大小相對 1g
        float anorm = sqrtf(ax*ax + ay*ay + az*az);
        float dev = fabsf(anorm - 9.80665f) / 9.80665f; // 相對誤差
        return (dev <= stillAccTolG);
    }

public:
    Madgwick(void);

    // 設定採樣頻率（Hz）
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }

    // 含磁力計（若 mx,my,mz=0 會自動 fallback 到 IMU）
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz);

    // IMU-only
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    // 取得角度（度 / 弧度）
    float getRoll()  { if (!anglesComputed) computeAngles(); return roll  * 57.29578f; }
    float getPitch() { if (!anglesComputed) computeAngles(); return pitch * 57.29578f; }
    float getYaw()   { if (!anglesComputed) computeAngles(); return yaw   * 57.29578f; }

    float getRollRadians()  { if (!anglesComputed) computeAngles(); return roll;  }
    float getPitchRadians() { if (!anglesComputed) computeAngles(); return pitch; }
    float getYawRadians()   { if (!anglesComputed) computeAngles(); return yaw;   }

    // 連續的 Yaw（unwrap 過，度）
    float getYawUnwrappedDeg() const { return yaw_deg_cont; }

    // STOP 時重置姿態
    void resetAttitude();

    // 設定奇異點保護：當 |pitch| >= deg 時鎖定 roll/yaw
    void setGimbalLockGuard(bool enable, float deg = 89.0f) {
        gimbalLockEnable   = enable;
        gimbalLockPitchDeg = deg;
    }

    // 啟用/關閉 靜止時陀螺偏置估測（降低漂移）
    void setGyroBiasLearning(bool enable) { gyroBiasEnable = enable; }
};

#endif
