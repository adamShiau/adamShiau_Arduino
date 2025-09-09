#pragma once

/**
 * Coordinate Transformation Adapter
 * 
 * 提供統一的座標系轉換介面，整合 mu 的數學工具
 * 職責：
 * - 各種座標系之間的轉換 (NED ↔ ENU ↔ Body)
 * - 四元數和歐拉角的轉換
 * - 地理座標與局部座標的轉換
 * - 旋轉矩陣操作的統一介面
 * 
 * 設計原則：
 * - Header-only, inline 函數
 * - 無狀態設計
 * - 直接封裝 mu 的功能
 * - 提供語義化的座標系轉換介面
 */

#include "../data/data_types.h"
#include "../util/math_utils.h"

namespace CoordAdapter {
    
    // ============================================================================
    // 座標系轉換常用定義
    // ============================================================================
    
    /**
     * 座標系轉換類型枚舉
     */
    enum class CoordinateFrame {
        BODY,       // 機體座標系 (X前Y右Z下)
        NED,        // 北東地座標系 (X北Y東Z下)
        ENU,        // 東北天座標系 (X東Y北Z上) 
        ECEF        // 地心地固座標系
    };
    
    // ============================================================================
    // 四元數與歐拉角轉換 (整合 mu)
    // ============================================================================
    
    /**
     * 歐拉角轉四元數 (NED 座標系)
     * @param roll, pitch, yaw 歐拉角（弧度）
     * @param qw, qx, qy, qz 輸出四元數分量
     */
    inline void eulerToQuaternion(float roll, float pitch, float yaw,
                                 float& qw, float& qx, float& qy, float& qz) {
        mu::euler_to_quaternion_ned(roll, pitch, yaw, qw, qx, qy, qz);
    }
    
    /**
     * 四元數轉歐拉角 (NED 座標系)
     * @param qw, qx, qy, qz 四元數分量
     * @param roll, pitch, yaw 輸出歐拉角（弧度）
     * @return true 如果轉換成功
     */
    inline bool quaternionToEuler(float qw, float qx, float qy, float qz,
                                 float& roll, float& pitch, float& yaw) {
        if (!mu::is_valid_quaternion(qw, qx, qy, qz)) {
            return false;
        }
        mu::quaternion_to_euler_ned(qw, qx, qy, qz, roll, pitch, yaw);
        return true;
    }
    
    /**
     * 四元數正規化
     * @param qw, qx, qy, qz 四元數分量（修改）
     * @return true 如果正規化成功
     */
    inline bool normalizeQuaternion(float& qw, float& qx, float& qy, float& qz) {
        return mu::quaternion_normalize(qw, qx, qy, qz);
    }
    
    /**
     * 四元數相乘
     * @param q1w, q1x, q1y, q1z 第一個四元數
     * @param q2w, q2x, q2y, q2z 第二個四元數
     * @param qw, qx, qy, qz 輸出結果四元數
     */
    inline void multiplyQuaternions(float q1w, float q1x, float q1y, float q1z,
                                   float q2w, float q2x, float q2y, float q2z,
                                   float& qw, float& qx, float& qy, float& qz) {
        mu::quaternion_multiply(q1w, q1x, q1y, q1z, q2w, q2x, q2y, q2z, qw, qx, qy, qz);
    }
    
    // ============================================================================
    // 旋轉矩陣操作 (整合 mu)
    // ============================================================================
    
    /**
     * 歐拉角轉旋轉矩陣
     * @param roll, pitch, yaw 歐拉角（弧度）
     * @param matrix 輸出旋轉矩陣 (3x3, row-major)
     */
    inline void eulerToRotationMatrix(float roll, float pitch, float yaw, float matrix[9]) {
        mu::euler_to_rotation_matrix(roll, pitch, yaw, matrix);
    }
    
    /**
     * 應用旋轉矩陣到向量
     * @param matrix 旋轉矩陣 (3x3)
     * @param input_vector 輸入向量
     * @param output_vector 輸出向量
     */
    inline void applyRotation(const float matrix[9], const float input_vector[3], float output_vector[3]) {
        mu::apply_rotation_matrix(matrix, input_vector, output_vector);
    }
    
    /**
     * 旋轉矩陣與向量相乘（使用 mu::Vector3f）
     * @param matrix 旋轉矩陣 (3x3)
     * @param input 輸入向量
     * @return 旋轉後的向量
     */
    inline mu::Vector3f rotateVector(const float matrix[9], const mu::Vector3f& input) {
        float in[3] = {input.x, input.y, input.z};
        float out[3];
        mu::apply_rotation_matrix(matrix, in, out);
        return mu::Vector3f(out[0], out[1], out[2]);
    }
    
    /**
     * 創建繞 Z 軸的旋轉矩陣（偏航角）
     * @param yaw_rad 偏航角（弧度）
     * @param matrix 輸出旋轉矩陣 (3x3)
     */
    inline void createYawRotationMatrix(float yaw_rad, float matrix[9]) {
        float cos_yaw = cosf(yaw_rad);
        float sin_yaw = sinf(yaw_rad);
        
        matrix[0] = cos_yaw;  matrix[1] = -sin_yaw; matrix[2] = 0.0f;
        matrix[3] = sin_yaw;  matrix[4] = cos_yaw;  matrix[5] = 0.0f;
        matrix[6] = 0.0f;     matrix[7] = 0.0f;     matrix[8] = 1.0f;
    }
    
    // ============================================================================
    // 座標系間轉換
    // ============================================================================
    
    /**
     * NED 座標轉 ENU 座標
     * @param ned_vector NED 座標向量
     * @return ENU 座標向量
     */
    inline mu::Vector3f nedToEnu(const mu::Vector3f& ned_vector) {
        // NED → ENU: (north, east, down) → (east, north, -down)
        return mu::Vector3f(ned_vector.y, ned_vector.x, -ned_vector.z);
    }
    
    /**
     * ENU 座標轉 NED 座標
     * @param enu_vector ENU 座標向量
     * @return NED 座標向量
     */
    inline mu::Vector3f enuToNed(const mu::Vector3f& enu_vector) {
        // ENU → NED: (east, north, up) → (north, east, -up)
        return mu::Vector3f(enu_vector.y, enu_vector.x, -enu_vector.z);
    }
    
    /**
     * Body 座標轉 NED 座標（使用四元數）
     * @param body_vector Body 座標向量
     * @param qw, qx, qy, qz Body-to-NED 四元數
     * @return NED 座標向量
     */
    inline mu::Vector3f bodyToNed(const mu::Vector3f& body_vector,
                                        float qw, float qx, float qy, float qz) {
        // 四元數旋轉：v' = q * v * q^(-1)
        // 簡化計算：使用旋轉矩陣
        float roll, pitch, yaw;
        mu::quaternion_to_euler_ned(qw, qx, qy, qz, roll, pitch, yaw);
        
        float rotation_matrix[9];
        eulerToRotationMatrix(roll, pitch, yaw, rotation_matrix);
        
        return rotateVector(rotation_matrix, body_vector);
    }
    
    /**
     * NED 座標轉 Body 座標（使用四元數）
     * @param ned_vector NED 座標向量
     * @param qw, qx, qy, qz Body-to-NED 四元數
     * @return Body 座標向量
     */
    inline mu::Vector3f nedToBody(const mu::Vector3f& ned_vector,
                                        float qw, float qx, float qy, float qz) {
        // 使用逆四元數：q^(-1) = (w, -x, -y, -z) / |q|²
        // 對於單位四元數：q^(-1) = (w, -x, -y, -z)
        return bodyToNed(ned_vector, qw, -qx, -qy, -qz);
    }
    
    // ============================================================================
    // 角度處理工具
    // ============================================================================
    
    /**
     * 角度正規化到 [-π, π] 範圍
     * @param angle 輸入角度（弧度）
     * @return 正規化後的角度
     */
    inline float wrapAngle(float angle) {
        return mu::wrapAngle(angle);
    }
    
    /**
     * 角度正規化到 [0, 2π] 範圍
     * @param angle 輸入角度（弧度）
     * @return 正規化後的角度
     */
    inline float wrapAngle2Pi(float angle) {
        return mu::wrapAngle2Pi(angle);
    }
    
    /**
     * 度數轉弧度
     * @param degrees 度數
     * @return 弧度
     */
    inline float degreesToRadians(float degrees) {
        return degrees * mu::DEG_TO_RAD;
    }
    
    /**
     * 弧度轉度數
     * @param radians 弧度
     * @return 度數
     */
    inline float radiansToDegrees(float radians) {
        return radians * mu::RAD_TO_DEG;
    }
    
    /**
     * 計算兩個角度之間的差異
     * @param angle1, angle2 兩個角度（弧度）
     * @return 角度差異（弧度，範圍 [-π, π]）
     */
    inline float angleDifference(float angle1, float angle2) {
        return mu::angle_difference(angle1, angle2);
    }
    
    // ============================================================================
    // Shift 校正專用工具
    // ============================================================================
    
    /**
     * 計算 GNSS 航向與 IMU 偏航角的 Shift 偏移量
     * @param gnss_heading GNSS 航向角（弧度）
     * @param imu_yaw IMU 偏航角（弧度）
     * @return Shift 偏移量（弧度）
     */
    inline float calculateShiftOffset(float gnss_heading, float imu_yaw) {
        return mu::calculate_shift_offset(gnss_heading, imu_yaw);
    }
    
    /**
     * 應用 Shift 校正到角度
     * @param raw_angle 原始角度（弧度）
     * @param shift_offset Shift 偏移量（弧度）
     * @return 校正後的角度（弧度）
     */
    inline float applyShiftCorrection(float raw_angle, float shift_offset) {
        return mu::apply_shift_correction(raw_angle, shift_offset);
    }
    
    // ============================================================================
    // 地理座標轉換工具
    // ============================================================================
    
    /**
     * WGS84 地理座標轉局部切平面座標 (簡化版)
     * 注意：這是簡化版本，適用於小範圍應用
     * @param lat_rad, lon_rad 緯度經度（弧度）
     * @param ref_lat_rad, ref_lon_rad 參考點緯度經度（弧度）
     * @param local_north, local_east 輸出局部座標（米）
     */
    inline void geodeticToLocal(double lat_rad, double lon_rad,
                               double ref_lat_rad, double ref_lon_rad,
                               float& local_north, float& local_east) {
        constexpr double EARTH_RADIUS = 6378137.0; // WGS84 地球半徑（米）
        
        double dlat = lat_rad - ref_lat_rad;
        double dlon = lon_rad - ref_lon_rad;
        
        local_north = static_cast<float>(dlat * EARTH_RADIUS);
        local_east = static_cast<float>(dlon * EARTH_RADIUS * cos(ref_lat_rad));
    }
    
    /**
     * 局部切平面座標轉 WGS84 地理座標 (簡化版)
     * @param local_north, local_east 局部座標（米）
     * @param ref_lat_rad, ref_lon_rad 參考點緯度經度（弧度）
     * @param lat_rad, lon_rad 輸出緯度經度（弧度）
     */
    inline void localToGeodetic(float local_north, float local_east,
                               double ref_lat_rad, double ref_lon_rad,
                               double& lat_rad, double& lon_rad) {
        constexpr double EARTH_RADIUS = 6378137.0; // WGS84 地球半徑（米）
        
        double dlat = local_north / EARTH_RADIUS;
        double dlon = local_east / (EARTH_RADIUS * cos(ref_lat_rad));
        
        lat_rad = ref_lat_rad + dlat;
        lon_rad = ref_lon_rad + dlon;
    }
    
    // ============================================================================
    // 向量操作工具
    // ============================================================================
    
    /**
     * 向量正規化
     * @param vector 輸入向量（修改）
     * @return true 如果正規化成功
     */
    inline bool normalizeVector(mu::Vector3f& vector) {
        float magnitude = vector.magnitude();
        if (magnitude < mu::EPSILON) {
            return false;
        }
        vector = vector * (1.0f / magnitude);
        return true;
    }
    
    /**
     * 向量點積
     * @param a, b 輸入向量
     * @return 點積結果
     */
    inline float dotProduct(const mu::Vector3f& a, const mu::Vector3f& b) {
        return a.dot(b);
    }
    
    /**
     * 向量叉積
     * @param a, b 輸入向量
     * @return 叉積結果向量
     */
    inline mu::Vector3f crossProduct(const mu::Vector3f& a, const mu::Vector3f& b) {
        return a.cross(b);
    }
    
    /**
     * 計算兩個向量之間的夾角
     * @param a, b 輸入向量
     * @return 夾角（弧度，範圍 [0, π]）
     */
    inline float angleBetweenVectors(const mu::Vector3f& a, const mu::Vector3f& b) {
        float dot = a.dot(b);
        float mag_product = a.magnitude() * b.magnitude();
        
        if (mag_product < mu::EPSILON) {
            return 0.0f;
        }
        
        float cos_angle = mu::clamp(dot / mag_product, -1.0f, 1.0f);
        return acosf(cos_angle);
    }
    
    // ============================================================================
    // 座標系驗證工具
    // ============================================================================
    
    /**
     * 檢查向量是否為有效的單位向量
     * @param vector 輸入向量
     * @param tolerance 容差（預設使用 EPSILON）
     * @return true 如果是單位向量
     */
    inline bool isUnitVector(const mu::Vector3f& vector, float tolerance = mu::EPSILON) {
        float magnitude_squared = vector.magnitudeSquared();
        return mu::isEqual(magnitude_squared, 1.0f, tolerance);
    }
    
    /**
     * 檢查旋轉矩陣是否為正交矩陣
     * @param matrix 3x3 旋轉矩陣
     * @return true 如果是正交矩陣
     */
    inline bool isOrthogonalMatrix(const float matrix[9]) {
        // 檢查行向量是否為單位向量並且互相正交
        mu::Vector3f col1(matrix[0], matrix[3], matrix[6]);
        mu::Vector3f col2(matrix[1], matrix[4], matrix[7]);
        mu::Vector3f col3(matrix[2], matrix[5], matrix[8]);
        
        // 檢查是否為單位向量
        if (!isUnitVector(col1) || !isUnitVector(col2) || !isUnitVector(col3)) {
            return false;
        }
        
        // 檢查是否互相正交
        const float tolerance = 0.001f;
        return mu::isNearZero(col1.dot(col2), tolerance) &&
               mu::isNearZero(col1.dot(col3), tolerance) &&
               mu::isNearZero(col2.dot(col3), tolerance);
    }
    
    // ============================================================================
    // 調試和診斷工具
    // ============================================================================
    
    /**
     * 格式化四元數為字符串
     * @param qw, qx, qy, qz 四元數分量
     * @param buffer 輸出緩衝區
     * @param buffer_size 緩衝區大小
     */
    inline void quaternionToString(float qw, float qx, float qy, float qz, 
                                  char* buffer, size_t buffer_size) {
        snprintf(buffer, buffer_size, "q[w=%.4f, x=%.4f, y=%.4f, z=%.4f]", qw, qx, qy, qz);
    }
    
    /**
     * 格式化歐拉角為字符串（度數）
     * @param roll, pitch, yaw 歐拉角（弧度）
     * @param buffer 輸出緩衝區
     * @param buffer_size 緩衝區大小
     */
    inline void eulerToString(float roll, float pitch, float yaw, 
                             char* buffer, size_t buffer_size) {
        snprintf(buffer, buffer_size, "RPY[%.2f°, %.2f°, %.2f°]", 
                radiansToDegrees(roll), radiansToDegrees(pitch), radiansToDegrees(yaw));
    }
    
    /**
     * 格式化向量為字符串
     * @param vector 輸入向量
     * @param buffer 輸出緩衝區
     * @param buffer_size 緩衝區大小
     */
    inline void vectorToString(const mu::Vector3f& vector, 
                              char* buffer, size_t buffer_size) {
        snprintf(buffer, buffer_size, "V[%.3f, %.3f, %.3f]", vector.x, vector.y, vector.z);
    }
    
} // namespace CoordAdapter