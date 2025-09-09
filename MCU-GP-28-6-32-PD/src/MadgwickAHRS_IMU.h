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
    float beta;          // algorithm gain
    float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll, pitch, yaw; // cached Euler angles (radians)
    char anglesComputed;
    void computeAngles();

public:
    Madgwick(void);

    // 設定採樣頻率（Hz）
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }

    // 含磁力計版本
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz);

    // IMU 版本（無磁力計）
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    // === 取得角度（度 / 弧度） ===
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }

    // === 新增：重置姿態角 ===
    // 將四元數設為單位四元數、Roll/Pitch/Yaw 清零。
    // 可在 .ino 收到 STOP 時呼叫，清掉累積漂移。
    void resetAttitude();
};

#endif
