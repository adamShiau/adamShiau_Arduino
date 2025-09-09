#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

// Arduino環境兼容性檢查
#ifdef ARDUINO
    // Arduino環境下可能缺少一些C++標準庫函數
    #ifndef std
        using std::isnan;
        using std::isfinite;
        using std::llround;
    #endif
#endif

/**
 * @brief 數值安全工具 - 防溢出和型別轉換
 * 用於 MAVLink 封包中的安全數值處理
 */

template<typename T, typename U>
inline T saturate_cast(U v) {
    if (std::isnan(static_cast<double>(v))) return T(0);
    if (v > static_cast<U>(std::numeric_limits<T>::max())) return std::numeric_limits<T>::max();
    if (v < static_cast<U>(std::numeric_limits<T>::min())) return std::numeric_limits<T>::min();
    return static_cast<T>(v);
}

template<typename T>
inline T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// 🛡️ NaN/極值清洗輔助函數
inline bool finite_nonneg(double x) { return std::isfinite(x) && x >= 0.0; }
inline bool is_finite_safe(double x) { return std::isfinite(x); }

inline double clamp_lat(double lat) {
    if (!std::isfinite(lat)) return 0.0;
    return (lat > 90.0) ? 90.0 : (lat < -90.0 ? -90.0 : lat);
}

inline double clamp_lon(double lon) {
    if (!std::isfinite(lon)) return 0.0;
    return (lon > 180.0) ? 180.0 : (lon < -180.0 ? -180.0 : lon);
}

// 常用的 MAVLink 數值轉換函數
inline uint16_t meters_to_cm_u16(double meters) {
    if (!std::isfinite(meters) || meters < 0) return uint16_t(0);
    return clamp<uint16_t>(static_cast<uint16_t>(std::llround(meters * 100.0)), 0, 65535);
}

inline uint16_t degrees_to_cdeg_u16(double degrees) {
    if (!std::isfinite(degrees)) return std::numeric_limits<uint16_t>::max();
    // 正規化到 0-360 範圍
    while (degrees < 0) degrees += 360.0;
    while (degrees >= 360.0) degrees -= 360.0;
    int32_t cdeg = static_cast<int32_t>(std::llround(degrees * 100.0));
    return clamp<uint16_t>(static_cast<uint16_t>(cdeg), 0, 35999);
}