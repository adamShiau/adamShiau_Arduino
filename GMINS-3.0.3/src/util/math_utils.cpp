#include "math_utils.h"

namespace mu {

// ============================================================================
// 4) 矩陣運算 (3×3)
// ============================================================================
void matrixMultiply3x3(const float a[3][3], const float b[3][3], float r[3][3]) {
    for (int i=0;i<3;++i) {
        for (int j=0;j<3;++j) {
            r[i][j] = a[i][0]*b[0][j] + a[i][1]*b[1][j] + a[i][2]*b[2][j];
        }
    }
}

void matrixVectorMultiply3x3(const float m[3][3], const float v[3], float r[3]) {
    r[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
    r[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
    r[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
}

void matrixTranspose3x3(const float m[3][3], float r[3][3]) {
    r[0][0]=m[0][0]; r[0][1]=m[1][0]; r[0][2]=m[2][0];
    r[1][0]=m[0][1]; r[1][1]=m[1][1]; r[1][2]=m[2][1];
    r[2][0]=m[0][2]; r[2][1]=m[1][2]; r[2][2]=m[2][2];
}

float matrixDeterminant3x3(const float m[3][3]) {
    return
        m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1]) -
        m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0]) +
        m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
}

void matrixInverse3x3(const float m[3][3], float r[3][3]) {
    float det = matrixDeterminant3x3(m);
    if (fabsf(det) < kEpsilon) {
        // 奇異矩陣：回傳單位矩陣以避免數值崩潰
        matrixIdentity3x3(r);
        return;
    }
    float inv = 1.0f / det;

    r[0][0] =  (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * inv;
    r[0][1] = -(m[0][1]*m[2][2] - m[0][2]*m[2][1]) * inv;
    r[0][2] =  (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * inv;

    r[1][0] = -(m[1][0]*m[2][2] - m[1][2]*m[2][0]) * inv;
    r[1][1] =  (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * inv;
    r[1][2] = -(m[0][0]*m[1][2] - m[0][2]*m[1][0]) * inv;

    r[2][0] =  (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * inv;
    r[2][1] = -(m[0][0]*m[2][1] - m[0][1]*m[2][0]) * inv;
    r[2][2] =  (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * inv;
}

void matrixIdentity3x3(float r[3][3]) {
    r[0][0]=1.0f; r[0][1]=0.0f; r[0][2]=0.0f;
    r[1][0]=0.0f; r[1][1]=1.0f; r[1][2]=0.0f;
    r[2][0]=0.0f; r[2][1]=0.0f; r[2][2]=1.0f;
}

// ============================================================================
// 5) 統計運算 (mean / variance 為「母體」定義)
// ============================================================================
float mean(const float* data, int count) {
    if (!data || count <= 0) return 0.0f;
    float s = 0.0f;
    for (int i=0;i<count;++i) s += data[i];
    return s / (float)count;
}

float variance(const float* data, int count) {
    if (!data || count <= 0) return 0.0f;
    float m = mean(data, count);
    float s2 = 0.0f;
    for (int i=0;i<count;++i) {
        float d = data[i] - m;
        s2 += d*d;
    }
    return s2 / (float)count; // 母體變異數
}

float standardDeviation(const float* data, int count) {
    return sqrtf(variance(data, count));
}

static void insertionSort(float* a, int n) {
    for (int i=1;i<n;++i) {
        float key = a[i];
        int j = i-1;
        while (j>=0 && a[j] > key) {
            a[j+1] = a[j];
            --j;
        }
        a[j+1] = key;
    }
}

// 注意：此實作會排序輸入陣列
float median(float* data, int count) {
    if (!data || count <= 0) return 0.0f;
    insertionSort(data, count);
    if (count & 1) {
        return data[count/2];
    } else {
        int i = count/2;
        return 0.5f * (data[i-1] + data[i]);
    }
}

// ============================================================================
// 6) 濾波器
// ============================================================================
static inline float calc_alpha(float cutoff_freq, float sample_rate) {
    if (cutoff_freq <= 0.0f || sample_rate <= 0.0f) return 1.0f;
    float dt = 1.0f / sample_rate;
    float rc = 1.0f / (2.0f * kPi * cutoff_freq);
    return dt / (rc + dt);
}

// --- LowPassFilter ---
LowPassFilter::LowPassFilter(float cutoff_freq, float sample_rate)
: alpha_(calc_alpha(cutoff_freq, sample_rate)), output_(0.0f), initialized_(false) {}

void LowPassFilter::reset(float initial_value) {
    output_ = initial_value;
    initialized_ = true;
}

float LowPassFilter::update(float input) {
    if (!initialized_) { output_ = input; initialized_ = true; return output_; }
    output_ = alpha_ * input + (1.0f - alpha_) * output_;
    return output_;
}

// --- HighPassFilter ---
HighPassFilter::HighPassFilter(float cutoff_freq, float sample_rate)
: alpha_(calc_alpha(cutoff_freq, sample_rate)), prev_input_(0.0f), prev_output_(0.0f), initialized_(false) {}

void HighPassFilter::reset() {
    prev_input_ = 0.0f;
    prev_output_ = 0.0f;
    initialized_ = false;
}

float HighPassFilter::update(float input) {
    if (!initialized_) { prev_input_ = input; prev_output_ = 0.0f; initialized_ = true; return prev_output_; }
    // 一階高通差分形式：y[n] = (1 - alpha)*y[n-1] + (1 - alpha)*(x[n] - x[n-1])
    float y = (1.0f - alpha_) * (prev_output_ + (input - prev_input_));
    prev_input_  = input;
    prev_output_ = y;
    return y;
}

// ============================================================================
// 7) 快速數學近似
//   注意：皆為近似；若需高精度請改用 std:: 版本
// ============================================================================

float fastInvSqrt(float x) {
    // Quake III 近似
    union { float f; uint32_t i; } conv = { x };
    float xhalf = 0.5f * x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    float y = conv.f;
    // 一次牛頓修正
    y = y * (1.5f - xhalf * y * y);
    return y;
}

float fastSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    return x * fastInvSqrt(x);
}

static inline float wrap_pi_pi(float x) {
    while (x >  kPi) x -= kTwoPi;
    while (x < -kPi) x += kTwoPi;
    return x;
}

// 近似 sin：先映射到 [-pi, pi]，再用分段多項式近似 + 簡單修正
float fastSin(float x) {
    x = wrap_pi_pi(x);
    // Primary approximation
    float y = 1.27323954f * x - 0.405284735f * x * ((x<0)?-x:x);
    // Correction (improve near extrema)
    y = 0.775f * y + 0.225f * y * ((y<0)?-y:y);
    return y;
}

float fastCos(float x) {
    return fastSin(x + kHalfPi);
}

// Kahan/Mapa 型 atan2 近似
float fastAtan2(float y, float x) {
    if (x == 0.0f) {
        if (y > 0.0f) return kHalfPi;
        if (y < 0.0f) return -kHalfPi;
        return 0.0f;
    }
    float abs_y = (y<0)?-y:y;
    float angle, r;
    if (x >= 0.0f) {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.78539816339f; // pi/4
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = 2.35619449019f; // 3*pi/4
    }
    // 三次多項式校正
    angle += (0.1963f * r * r - 0.9817f) * r;
    return (y < 0.0f) ? -angle : angle;
}

} // namespace MathUtils
