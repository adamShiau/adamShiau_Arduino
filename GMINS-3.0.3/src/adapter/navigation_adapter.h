#pragma once

/**
 * Navigation State Adapter
 * 
 * 提供 NavigationState 結構與 mu 之間的適配介面
 * 職責：
 * - 導航狀態數據的讀取和更新
 * - 四元數和歐拉角轉換的統一介面
 * - Shift 校正的應用和管理
 * - 融合結果的格式化輸出
 * 
 * 設計原則：
 * - Header-only, inline 函數
 * - 無狀態設計
 * - 支援融合引擎和 Shift 校正的數據操作
 * - 提供統一的座標系轉換介面
 */

#include "../data/data_types.h"
#include "../util/math_utils.h"
#include <cstdio>

namespace NavigationAdapter {
    
    // ============================================================================
    // 姿態數據存取和操作
    // ============================================================================
    
    // /**
    //  * 從導航狀態提取四元數 - 已停用，不再使用四元數
    //  */
    // inline bool extractQuaternion(const NavigationState& nav_state,
    //                               float& qw, float& qx, float& qy, float& qz) {
    //     // 四元數功能已停用
    //     return false;
    // }
    
    /**
     * 從導航狀態提取歐拉角 (NED 座標系)
     * @param nav_state 導航狀態結構
     * @param roll, pitch, yaw 輸出歐拉角（弧度）
     * @return true 如果提取成功
     */
    inline bool extractEulerAngles(const NavigationState& nav_state, 
                                   float& roll, float& pitch, float& yaw) {
        if (!(nav_state.flags & NAV_ATTITUDE_VALID)) {
            return false;
        }
        
        // 直接從歐拉角欄位提取，轉換度數為弧度
        roll = nav_state.euler_roll * mu::kDegToRad;
        pitch = nav_state.euler_pitch * mu::kDegToRad;
        yaw = nav_state.euler_yaw * mu::kDegToRad;
        
        return true;
    }
    
    /**
     * 更新導航狀態的四元數
     * @param nav_state 導航狀態結構（修改）
     * @param qw, qx, qy, qz 新的四元數分量
     * @return true 如果更新成功
     */
    // 四元數相關功能已停用
    // inline bool updateQuaternion(...) { return false; }
    
    /**
     * 使用歐拉角更新導航狀態的四元數
     * @param nav_state 導航狀態結構（修改）
     * @param roll, pitch, yaw 歐拉角（弧度）
     * @return true 如果更新成功
     */
    inline bool updateFromEulerAngles(NavigationState& nav_state,
                                     float roll, float pitch, float yaw) {
        // 直接存儲歐拉角（轉換弧度為度數）
        nav_state.euler_roll = roll * mu::kRadToDeg;
        nav_state.euler_pitch = pitch * mu::kRadToDeg;
        nav_state.euler_yaw = yaw * mu::kRadToDeg;
        nav_state.flags |= NAV_ATTITUDE_VALID;
        return true;
    }
    
    // ============================================================================
    // 位置和速度操作
    // ============================================================================
    
    /**
     * 獲取位置向量 (NED 座標系)
     * @param nav_state 導航狀態結構
     * @return 位置向量（米）
     */
    inline mu::Vector3f getPositionVector(const NavigationState& nav_state) {
        if (!(nav_state.flags & NAV_POSITION_VALID)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        return mu::Vector3f(nav_state.position_north, 
                                  nav_state.position_east, 
                                  nav_state.position_down);
    }
    
    /**
     * 獲取速度向量 (NED 座標系)
     * @param nav_state 導航狀態結構
     * @return 速度向量（m/s）
     */
    inline mu::Vector3f getVelocityVector(const NavigationState& nav_state) {
        if (!(nav_state.flags & NAV_VELOCITY_VALID)) {
            return mu::Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        return mu::Vector3f(nav_state.velocity_north, 
                                  nav_state.velocity_east, 
                                  nav_state.velocity_down);
    }
    
    /**
     * 獲取加速度向量 (NED 座標系)
     * @param nav_state 導航狀態結構
     * @return 加速度向量（m/s²）
     */
    inline mu::Vector3f getAccelerationVector(const NavigationState& nav_state) {
        return mu::Vector3f(nav_state.acceleration_north, 
                                  nav_state.acceleration_east, 
                                  nav_state.acceleration_down);
    }
    
    /**
     * 更新導航狀態的位置
     * @param nav_state 導航狀態結構（修改）
     * @param position_north, position_east, position_down 位置分量（米）
     */
    inline void updatePosition(NavigationState& nav_state,
                              float position_north, float position_east, float position_down) {
        nav_state.position_north = position_north;
        nav_state.position_east = position_east;
        nav_state.position_down = position_down;
        nav_state.flags |= NAV_POSITION_VALID;
    }
    
    /**
     * 更新導航狀態的速度
     * @param nav_state 導航狀態結構（修改）
     * @param velocity_north, velocity_east, velocity_down 速度分量（m/s）
     */
    inline void updateVelocity(NavigationState& nav_state,
                              float velocity_north, float velocity_east, float velocity_down) {
        nav_state.velocity_north = velocity_north;
        nav_state.velocity_east = velocity_east;
        nav_state.velocity_down = velocity_down;
        nav_state.flags |= NAV_VELOCITY_VALID;
    }
    
    /**
     * 更新導航狀態的加速度
     * @param nav_state 導航狀態結構（修改）
     * @param acceleration_north, acceleration_east, acceleration_down 加速度分量（m/s²）
     */
    inline void updateAcceleration(NavigationState& nav_state,
                                  float acceleration_north, float acceleration_east, float acceleration_down) {
        nav_state.acceleration_north = acceleration_north;
        nav_state.acceleration_east = acceleration_east;
        nav_state.acceleration_down = acceleration_down;
        // 注意：沒有專門的 NAV_ACCELERATION_VALID 標誌，通常與 IMU 數據一起驗證
    }
    
    /**
     * 更新導航狀態的角速度
     * @param nav_state 導航狀態結構（修改）
     * @param angular_velocity_x, angular_velocity_y, angular_velocity_z 角速度分量（rad/s）
     */
    inline void updateAngularVelocity(NavigationState& nav_state,
                                     float angular_velocity_x, float angular_velocity_y, float angular_velocity_z) {
        nav_state.angular_velocity_x = angular_velocity_x;
        nav_state.angular_velocity_y = angular_velocity_y;
        nav_state.angular_velocity_z = angular_velocity_z;
        nav_state.flags |= NAV_ANGULAR_VEL_VALID;
    }
    
    // ============================================================================
    // Shift 校正相關操作
    // ============================================================================
    
    /**
     * 應用 Shift 偏移到導航狀態的偏航角（四元數模式）
     * @param nav_state 導航狀態結構（修改）
     * @param shift_offset Shift 偏移量（弧度）
     * @return true 如果應用成功
     */
    inline bool applyYawShiftToQuaternion(NavigationState& nav_state, float shift_offset) {
        float qw, qx, qy, qz;
        // extractQuaternion 已停用，直接繼續
        
        // 創建 Z 軸旋轉四元數（偏航角修正）
        float half_shift = shift_offset * 0.5f;
        float cos_half = cosf(half_shift);
        float sin_half = sinf(half_shift);
        float shift_qw = cos_half;
        float shift_qx = 0.0f;
        float shift_qy = 0.0f;
        float shift_qz = sin_half;
        
        // 四元數相乘：q_corrected = q_shift * q_original
        float corrected_qw, corrected_qx, corrected_qy, corrected_qz;
        mu::quaternion_multiply(
            shift_qw, shift_qx, shift_qy, shift_qz,
            qw, qx, qy, qz,
            corrected_qw, corrected_qx, corrected_qy, corrected_qz
        );
        
        // 更新導航狀態
        // 簡化為直接操作歐拉角 YAW
        float shift_deg = shift_offset * mu::kRadToDeg;
        nav_state.euler_yaw += shift_deg;
        nav_state.applied_shift_offset = shift_offset;
        return true;
    }
    
    /**
     * 應用 Shift 偏移到導航狀態的偏航角（歐拉角模式）
     * @param nav_state 導航狀態結構（修改）
     * @param shift_offset Shift 偏移量（弧度）
     * @return true 如果應用成功
     */
    inline bool applyYawShiftToEuler(NavigationState& nav_state, float shift_offset) {
        float roll, pitch, yaw;
        if (!extractEulerAngles(nav_state, roll, pitch, yaw)) {
            return false;
        }
        
        // 應用 Shift 修正到偏航角
        float corrected_yaw = mu::apply_shift_correction(yaw, shift_offset);
        
        // 更新導航狀態
        bool success = updateFromEulerAngles(nav_state, roll, pitch, corrected_yaw);
        
        if (success) {
            nav_state.applied_shift_offset = shift_offset;
        }
        
        return success;
    }
    
    /**
     * 獲取當前應用的 Shift 偏移量
     * @param nav_state 導航狀態結構
     * @return 當前 Shift 偏移量（弧度）
     */
    inline float getCurrentShiftOffset(const NavigationState& nav_state) {
        return nav_state.applied_shift_offset;
    }
    
    /**
     * 更新 Shift 校正置信度
     * @param nav_state 導航狀態結構（修改）
     * @param confidence 置信度（0.0-1.0）
     */
    inline void updateShiftConfidence(NavigationState& nav_state, float confidence) {
        nav_state.shift_confidence = mu::clamp(confidence, 0.0f, 1.0f);
    }
    
    // ============================================================================
    // 融合引擎介面
    // ============================================================================
    
    /**
     * 從融合輸出更新導航狀態
     * @param nav_state 導航狀態結構（修改）
     * @param position 位置向量 (NED, 米)
     * @param velocity 速度向量 (NED, m/s)
     * @param quat_w, quat_x, quat_y, quat_z 四元數分量
     * @return true 如果更新成功
     */
    inline bool updateFromFusionOutput(NavigationState& nav_state,
                                      const mu::Vector3f& position,
                                      const mu::Vector3f& velocity,
                                      float quat_w, float quat_x, float quat_y, float quat_z) {
        // 更新位置
        updatePosition(nav_state, position.x, position.y, position.z);
        
        // 更新速度
        updateVelocity(nav_state, velocity.x, velocity.y, velocity.z);
        
        // 更新四元數
        // bool quat_success = updateQuaternion(...); // 四元數功能已停用
        bool quat_success = true; // 直接返回 true
        
        // if (quat_success) { // 四元數驗證已停用
        nav_state.flags |= NAV_FUSION_ACTIVE;
        // }
        
        // return quat_success; // 四元數功能已停用
        return true;
    }
    
    /**
     * 設置不確定性估計
     * @param nav_state 導航狀態結構（修改）
     * @param pos_std, vel_std, att_std 位置、速度、姿態標準差
     */
    inline void updateUncertainties(NavigationState& nav_state,
                                   const mu::Vector3f& pos_std,
                                   const mu::Vector3f& vel_std,
                                   const mu::Vector3f& att_std) {
        nav_state.position_std_north = pos_std.x;
        nav_state.position_std_east = pos_std.y;
        nav_state.position_std_down = pos_std.z;
        
        nav_state.velocity_std_north = vel_std.x;
        nav_state.velocity_std_east = vel_std.y;
        nav_state.velocity_std_down = vel_std.z;
        
        nav_state.attitude_std_roll = att_std.x;
        nav_state.attitude_std_pitch = att_std.y;
        nav_state.attitude_std_yaw = att_std.z;
    }
    
    /**
     * 檢查不確定性估計是否有效設置
     * @param nav_state 導航狀態結構
     * @return true 如果 COV 數據有效
     */
    inline bool hasValidUncertainties(const NavigationState& nav_state) {
        const float min_std = 0.001f;  // 最小有效標準差閾值
        return (nav_state.position_std_north > min_std) || 
               (nav_state.velocity_std_north > min_std) || 
               (nav_state.attitude_std_roll > min_std);
    }
    
    // ============================================================================
    // 數據驗證和品質檢查
    // ============================================================================
    
    /**
     * 檢查導航狀態數據是否有效
     * @param nav_state 導航狀態結構
     * @return true 如果數據有效
     */
    inline bool isDataValid(const NavigationState& nav_state) {
        // 檢查基本標誌
        if (!(nav_state.flags & (NAV_POSITION_VALID | NAV_ATTITUDE_VALID))) {
            return false;
        }
        
        // 檢查四元數有效性
        // 四元數驗證已停用
        if (false) {
            return false;
        }
        
        // 檢查位置合理性（避免極端值）
        mu::Vector3f pos = NavigationAdapter::getPositionVector(nav_state);
        if (pos.magnitude() > 100000.0f) { // 100km 範圍檢查
            return false;
        }
        
        // 檢查時間戳合理性
        return nav_state.timestamp_us > 0;
    }
    
    /**
     * 計算導航狀態品質分數 (0-100)
     * @param nav_state 導航狀態結構
     * @return 品質分數，100 為最佳
     */
    inline uint8_t calculateDataQuality(const NavigationState& nav_state) {
        uint8_t score = 0;
        
        // 基本有效性（40分）
        if (nav_state.flags & NAV_POSITION_VALID) score += 10;
        if (nav_state.flags & NAV_VELOCITY_VALID) score += 10;
        if (nav_state.flags & NAV_ATTITUDE_VALID) score += 20;
        
        // 融合狀態（30分）
        if (nav_state.flags & NAV_FUSION_ACTIVE) score += 20;
        if (nav_state.flags & NAV_SHIFT_CORRECTED) score += 10;
        
        // Shift 校正（20分）
        if (nav_state.shift_confidence > 0.8f) score += 20;
        else if (nav_state.shift_confidence > 0.5f) score += 15;
        else if (nav_state.shift_confidence > 0.2f) score += 10;
        
        // 數據準備就緒（10分）
        if (nav_state.flags & NAV_READY_FOR_OUTPUT) score += 10;
        
        return mu::min(score, static_cast<uint8_t>(100));
    }
    
    // ============================================================================
    // 時間和同步工具
    // ============================================================================
    
    /**
     * 檢查導航狀態數據是否過時
     * @param nav_state 導航狀態結構
     * @param current_time_us 當前時間戳（微秒）
     * @param max_age_us 最大允許年齡（微秒）
     * @return true 如果數據過時
     */
    inline bool isDataStale(const NavigationState& nav_state,
                           timestamp_us_t current_time_us,
                           timestamp_us_t max_age_us = 200000) { // 預設200ms
        return (current_time_us - nav_state.timestamp_us) > max_age_us;
    }
    
    /**
     * 更新導航狀態時間戳
     * @param nav_state 導航狀態結構（修改）
     * @param timestamp_us 新的時間戳（微秒）
     */
    inline void updateTimestamp(NavigationState& nav_state, timestamp_us_t timestamp_us) {
        nav_state.timestamp_us = timestamp_us;
        nav_state.flags |= NAV_READY_FOR_OUTPUT;
    }
    
    // ============================================================================
    // 調試和診斷工具
    // ============================================================================
    
    /**
     * 生成導航狀態的調試字符串
     * @param nav_state 導航狀態結構
     * @param buffer 輸出緩衝區
     * @param buffer_size 緩衝區大小
     */
    inline void debugString(const NavigationState& nav_state, char* buffer, size_t buffer_size) {
        float roll, pitch, yaw;
        bool euler_ok = NavigationAdapter::extractEulerAngles(nav_state, roll, pitch, yaw);
        
        mu::Vector3f pos = NavigationAdapter::getPositionVector(nav_state);
        mu::Vector3f vel = NavigationAdapter::getVelocityVector(nav_state);
        
        snprintf(buffer, buffer_size,
                "NAV[%llu] Pos:(%.2f,%.2f,%.2f) Vel:(%.2f,%.2f,%.2f) RPY:%s Shift:%.3f° Conf:%.2f Q:%d",
                nav_state.timestamp_us,
                pos.x, pos.y, pos.z,
                vel.x, vel.y, vel.z,
                euler_ok ? "OK" : "FAIL",
                nav_state.applied_shift_offset * mu::kRadToDeg,
                nav_state.shift_confidence,
                calculateDataQuality(nav_state));
    }
    
} // namespace NavigationAdapter