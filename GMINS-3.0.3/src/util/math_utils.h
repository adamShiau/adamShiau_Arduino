#pragma once
#include <stdint.h>
#include <math.h>

// 取消 Arduino 可能定義的衝突宏
#ifdef PI
#undef PI
#endif
#ifdef TWO_PI 
#undef TWO_PI
#endif
#ifdef HALF_PI
#undef HALF_PI
#endif
#ifdef DEG_TO_RAD
#undef DEG_TO_RAD
#endif
#ifdef RAD_TO_DEG
#undef RAD_TO_DEG
#endif

namespace mu {

    // ============================================================================
    // 1) 常數 - 使用命名空間避免與 Arduino 巨集衝突
    // ============================================================================
    constexpr float kPi       = 3.14159265358979323846f;
    constexpr float kTwoPi    = 6.28318530717958647692f;
    constexpr float kHalfPi   = 1.57079632679489661923f;
    constexpr float kEpsilon  = 1e-6f;
    
    // 常數版本（用於乘法）
    constexpr float kRadToDeg = 57.29577951308232f;
    constexpr float kDegToRad = 0.017453292519943295f;
    
    // 函式版本避免巨集衝突  
    inline float RadToDeg(float r) { return r * kRadToDeg; }
    inline float DegToRad(float d) { return d * kDegToRad; }

    // ============================================================================
    // 2) 基本數學工具
    // ============================================================================
    template<typename T>
    inline T clamp(T value, T min_val, T max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }

    template<typename T>
    inline T lerp(T a, T b, float t) { return a + t * (b - a); }

    template<typename T>
    inline T min(T a, T b) { return (a < b) ? a : b; }

    template<typename T>
    inline T max(T a, T b) { return (a > b) ? a : b; }

    inline float WrapAngleRad(float angle) {
        while (angle > kPi)  angle -= kTwoPi;
        while (angle < -kPi) angle += kTwoPi;
        return angle;
    }

    inline float wrapAngle2Pi(float angle) {
        while (angle >= kTwoPi) angle -= kTwoPi;
        while (angle < 0)       angle += kTwoPi;
        return angle;
    }

    inline bool isNearZero(float value, float epsilon = kEpsilon) {
        return fabsf(value) < epsilon;
    }

    inline bool isEqual(float a, float b, float epsilon = kEpsilon) {
        return fabsf(a - b) < epsilon;
    }

    // ============================================================================
    // 3) 三維向量
    // ============================================================================
    struct Vector3f {
        float x, y, z;
        Vector3f() : x(0), y(0), z(0) {}
        Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

        float magnitude()        const { return sqrtf(x*x + y*y + z*z); }
        float magnitudeSquared() const { return x*x + y*y + z*z; }

        void normalize() {
            float mag = magnitude();
            if (mag > kEpsilon) { x /= mag; y /= mag; z /= mag; }
        }
        Vector3f normalized() const { Vector3f r(*this); r.normalize(); return r; }

        Vector3f operator+(const Vector3f& o) const { return {x+o.x, y+o.y, z+o.z}; }
        Vector3f operator-(const Vector3f& o) const { return {x-o.x, y-o.y, z-o.z}; }
        Vector3f operator*(float s)           const { return {x*s, y*s, z*s}; }

        float     dot  (const Vector3f& o) const { return x*o.x + y*o.y + z*o.z; }
        Vector3f  cross(const Vector3f& o) const {
            return { y*o.z - z*o.y,  z*o.x - x*o.z,  x*o.y - y*o.x };
        }
    };

    // ============================================================================
    // 4) 矩陣運算 (3×3) —— 在 .cpp 實作
    // ============================================================================
    void  matrixMultiply3x3      (const float a[3][3], const float b[3][3], float r[3][3]);
    void  matrixVectorMultiply3x3(const float m[3][3], const float v[3],    float r[3]);
    void  matrixTranspose3x3     (const float m[3][3], float r[3][3]);
    void  matrixInverse3x3       (const float m[3][3], float r[3][3]);
    float matrixDeterminant3x3   (const float m[3][3]);
    void  matrixIdentity3x3      (float r[3][3]);

    // ============================================================================
    // 5) 統計運算 —— 在 .cpp 實作
    //   (variance 為母體變異數；需要樣本變異數時可自行 *count/(count-1))
    // ============================================================================
    float mean               (const float* data, int count);
    float variance           (const float* data, int count);
    float standardDeviation  (const float* data, int count);
    float median             (      float* data, int count); // 會修改輸入陣列

    // ============================================================================
    // 6) 濾波器 —— 在 .cpp 實作
    // ============================================================================
    class LowPassFilter {
    public:
        LowPassFilter(float cutoff_freq, float sample_rate);
        void  reset (float initial_value = 0.0f);
        float update(float input);
    private:
        float alpha_;
        float output_;
        bool  initialized_;
    };

    class HighPassFilter {
    public:
        HighPassFilter(float cutoff_freq, float sample_rate);
        void  reset();
        float update(float input);
    private:
        float alpha_;
        float prev_input_;
        float prev_output_;
        bool  initialized_;
    };

    template<int N>
    class MovingAverage {
    public:
        MovingAverage() : sum_(0), count_(0), index_(0) {
            for (int i = 0; i < N; ++i) buffer_[i] = 0;
        }
        float update(float value) {
            if (count_ < N) {
                buffer_[count_] = value; sum_ += value; ++count_;
                return sum_ / count_;
            } else {
                sum_ -= buffer_[index_];
                buffer_[index_] = value;
                sum_ += value;
                index_ = (index_ + 1) % N;
                return sum_ / N;
            }
        }
        void  reset()            { sum_ = 0; count_ = 0; index_ = 0; }
        float getValue() const   { return (count_ > 0) ? (sum_ / count_) : 0.0f; }
    private:
        float buffer_[N];
        float sum_;
        int   count_;
        int   index_;
    };

    // ============================================================================
    // 7) 快速數學近似 —— 在 .cpp 實作
    // ============================================================================
    float fastSqrt (float x);
    float fastInvSqrt(float x);
    float fastSin  (float x);
    float fastCos  (float x);
    float fastAtan2(float y, float x);

    // ============================================================================
    // 8) 四元數 / 歐拉角 (NED, ZYX)
    // ============================================================================
    inline bool quaternion_normalize(float& qw, float& qx, float& qy, float& qz) {
        float n = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
        if (n < kEpsilon) { qw = 1.0f; qx = qy = qz = 0.0f; return false; }
        float inv = 1.0f / n; qw*=inv; qx*=inv; qy*=inv; qz*=inv; return true;
    }

    inline void quaternion_multiply(
        float q1w,float q1x,float q1y,float q1z,
        float q2w,float q2x,float q2y,float q2z,
        float& qw,float& qx,float& qy,float& qz
    ) {
        qw = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z;
        qx = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y;
        qy = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x;
        qz = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w;
    }

    inline bool is_valid_quaternion(float qw, float qx, float qy, float qz) {
        if (!isfinite(qw) || !isfinite(qx) || !isfinite(qy) || !isfinite(qz)) return false;
        float norm2 = qw*qw + qx*qx + qy*qy + qz*qz;
        // 放寬容忍度以支持MTI數據 (原來 0.8-1.25，現在 0.1-4.0)
        return (norm2 > 0.1f && norm2 < 4.0f);
    }

    inline void euler_to_quaternion_ned(float r,float p,float y,
                                        float& qw,float& qx,float& qy,float& qz) {
        float cr=cosf(r*0.5f), sr=sinf(r*0.5f);
        float cp=cosf(p*0.5f), sp=sinf(p*0.5f);
        float cy=cosf(y*0.5f), sy=sinf(y*0.5f);
        qw = cr*cp*cy + sr*sp*sy;
        qx = sr*cp*cy - cr*sp*sy;
        qy = cr*sp*cy + sr*cp*sy;
        qz = cr*cp*sy - sr*sp*cy;
    }

    inline void quaternion_to_euler_ned(float qw,float qx,float qy,float qz,
                                        float& r,float& p,float& y) {
        if (!quaternion_normalize(qw,qx,qy,qz)) { r=p=y=0.0f; return; }
        r = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
        float sp = 2.0f*(qw*qy - qz*qx);
        p = (fabsf(sp) >= 1.0f) ? copysignf(kHalfPi, sp) : asinf(sp);
        y = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
        r = WrapAngleRad(r); p = WrapAngleRad(p); y = WrapAngleRad(y);
    }

    // ============================================================================
    // 9) Shift 校正 & 其他工具
    // ============================================================================
    inline float apply_shift_correction (float raw_yaw, float offset) { return WrapAngleRad(raw_yaw + offset + kHalfPi); }
    inline float calculate_shift_offset (float gnss, float imu)       { return WrapAngleRad(gnss - imu); }
    inline float angle_difference       (float a1, float a2)          { return WrapAngleRad(a1 - a2); }

    inline uint64_t millis_to_us(uint32_t ms) { return static_cast<uint64_t>(ms) * 1000ULL; }
    inline uint32_t us_to_millis(uint64_t us) { return static_cast<uint32_t>(us / 1000ULL); }

    inline float normalize_angle_deg_360(float a) { while (a<0) a+=360.0f; while (a>=360.0f) a-=360.0f; return a; }
    inline float normalize_angle_deg_180(float a) { while (a>180.0f) a-=360.0f; while (a<=-180.0f) a+=360.0f; return a; }

    inline void euler_to_rotation_matrix(float r,float p,float y, float m[9]) {
        float cr=cosf(r), sr=sinf(r);
        float cp=cosf(p), sp=sinf(p);
        float cy=cosf(y), sy=sinf(y);
        m[0]=cy*cp; m[1]=cy*sp*sr - sy*cr; m[2]=cy*sp*cr + sy*sr;
        m[3]=sy*cp; m[4]=sy*sp*sr + cy*cr; m[5]=sy*sp*cr - cy*sr;
        m[6]=-sp;  m[7]=cp*sr;            m[8]=cp*cr;
    }

    inline void apply_rotation_matrix(const float m[9], const float in[3], float out[3]) {
        out[0]=m[0]*in[0] + m[1]*in[1] + m[2]*in[2];
        out[1]=m[3]*in[0] + m[4]*in[1] + m[5]*in[2];
        out[2]=m[6]*in[0] + m[7]*in[1] + m[8]*in[2];
    }

} // namespace mu
