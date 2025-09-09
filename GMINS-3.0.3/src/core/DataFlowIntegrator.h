#pragma once

/**
 * Data Flow Integrator (DFI) - 極簡版
 * 
 * 職責：
 * - 收集 IMU/GNSS 數據（來自 Parser/Framers）
 * - 調用 Adapter 進行數據轉換和驗證
 * - 組合成 NavigationState
 * - 僅做最小品質門檻檢查（避免複雜統計）
 * - 單槽背壓設計（覆蓋最新值，無排隊）
 * 
 * 設計原則：
 * - 極簡設計，只做必要的品質門檻
 * - 使用現有 Adapter 的 inline 函數
 * - 無複雜統計和診斷功能
 * - Header-only 實作
 */

#include "../data/data_types.h"
#include "../adapter/imu_adapter.h"
#include "../adapter/gnss_adapter.h"
#include "../adapter/navigation_adapter.h"
#include "../util/time_aligner.h"
#include "../util/log.h"
#include "../util/data_flow_monitor.h"
#include "../util/tdd_flow_checker.h"
#include "../config/compile_flags.h"
#include <string>
#include <vector>
#include <Arduino.h>

class DataFlowIntegrator {
public:
    // 導航狀態回調類型
    using NavCallback = void(*)(const NavigationState&);
    
    // Push→Pull 解耦回調類型
    using IMUUpdateCallback = void(*)(const IMUData&, const NavigationState&);
    using GNSSUpdateCallback = void(*)(const GNSSData&, const NavigationState&);
    
    DataFlowIntegrator() 
        : have_imu_(false), have_gnss_(false), nav_callback_(nullptr), nav_monitor_(nullptr),
          imu_update_callback_(nullptr), gnss_update_callback_(nullptr),
          imu_sync_(1.0), gnss_sync_(1.0), time_offset_(0), last_sync_time_(0),
          plshd_valid_(false), plshd_heading_deg_(0.0f), plshd_last_time_(0),
          dfi_imu_callback_checker_("DFI:IMUCallback", "IMUCallback", 5000, true),
          dfi_gnss_callback_checker_("DFI:GNSSCallback", "GNSSCallback", 5000, true) {
        // 初始化 NavigationState
        memset(&nav_state_, 0, sizeof(nav_state_));
        nav_state_.schema = DATA_SCHEMA_VERSION;
        
        // 簡化後不需要初始化複雜的緩衝區
        
        // 創建 NavigationState 監控器
        nav_monitor_ = monitor::createSensorMonitor("NavigationState");
    }
    
    ~DataFlowIntegrator() {
        if (nav_monitor_) {
            delete nav_monitor_;
            nav_monitor_ = nullptr;
        }
    }
    
    // 禁止拷貝
    DataFlowIntegrator(const DataFlowIntegrator&) = delete;
    DataFlowIntegrator& operator=(const DataFlowIntegrator&) = delete;
    
    /**
     * @brief 設定導航狀態回調
     * @param callback 回調函數，當有新的 NavigationState 時調用
     */
    void setNavCallback(NavCallback callback) {
        nav_callback_ = callback;
    }
    
    /**
     * @brief 設定 IMU 更新回調（用於 Push→Pull 解耦）
     * @param callback IMU 數據更新回調
     */
    void setIMUUpdateCallback(IMUUpdateCallback callback) {
        imu_update_callback_ = callback;
    }
    
    /**
     * @brief 設定 GNSS 更新回調（用於 Push→Pull 解耦）
     * @param callback GNSS 數據更新回調  
     */
    void setGNSSUpdateCallback(GNSSUpdateCallback callback) {
        gnss_update_callback_ = callback;
    }
    
    /**
     * @brief 更新PLSHD狀態（直接來自PLSHD解析結果）
     * @param valid PLSHD valid_flag狀態
     * @param heading_deg heading值（度）
     */
    void updatePLSHDStatus(bool valid, float heading_deg) {
        plshd_valid_ = valid;
        if (valid) {
            plshd_heading_deg_ = heading_deg;
        }
        plshd_last_time_ = millis();
    }
    
    /**
     * @brief 獲取 NavigationState 監控器
     * @return DataFlowMonitor* 監控器指針，用於集成到 MultiChannelMonitor
     */
    monitor::DataFlowMonitor* getNavigationMonitor() const {
        return nav_monitor_;
    }
    
    /**
     * @brief 更新 IMU 數據
     * @param imu_data 新的 IMU 數據
     */
    void updateIMU(IMUData imu_data) {
        // TDD: 記錄 processIMU 流入 - 外部調用時由外部記錄
        // 這裡不重複記錄避免雙重計數
        
        // 頻率檢測統計
        static uint32_t last_freq_time = 0;
        static uint32_t updateIMU_count = 0;
        static bool first_call = true;
        uint32_t now = millis();
        
        updateIMU_count++;
        
        // 移除頻率統計LOG，避免系統死機
        if (first_call) {
            last_freq_time = now;
            first_call = false;
        }
        
        if (now - last_freq_time >= 5000) {
            updateIMU_count = 0;
            last_freq_time = now;
        }
        
        // 取得系統當下 μs
        uint64_t sys_now = static_cast<uint64_t>(micros());
        
        // 時間對齊：將 IMU 時間對齊到系統 micros() 基準
        uint64_t imu_aligned_us;
        if (imu_data.timestamp_us != 0) {
            // 已有時間戳，進行對齊
            imu_aligned_us = imu_sync_.toSystemMicros(imu_data.timestamp_us, sys_now);
            imu_sync_.refine(imu_data.timestamp_us, sys_now, 0.001, 0.005);
        } else {
            // 沒有時間戳，使用系統時間
            imu_aligned_us = sys_now;
        }
        
        // 更新 IMU 數據的時間戳為對齊後的值
        imu_data.timestamp_us = imu_aligned_us;
        imu_data_ = imu_data;
        have_imu_ = true;
        
        // current_time_ 只前推不回退
        current_time_ = (imu_aligned_us > current_time_) ? imu_aligned_us : current_time_;
        
        IF_DETAILED_STATS(processing_stats_.imu_updates++;)
        
        // 立即檢查並處理（避免延遲）
        processIfReady();
        
        // Push→Pull 解耦：通過回調觸發 IMU 相關協議快照更新
        if (imu_update_callback_) {
            dfi_imu_callback_checker_.recordIn();
            imu_update_callback_(imu_data_, nav_state_);
            dfi_imu_callback_checker_.recordOut();
            dfi_imu_callback_checker_.update();
        }
    }
    
    /**
     * @brief 更新 GNSS 數據
     * @param gnss_data 新的 GNSS 數據
     */
    void updateGNSS(GNSSData gnss_data) {
        // 取得系統當下 μs  
        uint64_t sys_now = static_cast<uint64_t>(micros());
        
        // 時間對齊：將 GNSS 時間對齊到系統 micros() 基準
        uint64_t gnss_aligned_us;
        if (gnss_data.timestamp_us != 0) {
            // 已有時間戳，進行對齊
            gnss_aligned_us = gnss_sync_.toSystemMicros(gnss_data.timestamp_us, sys_now);
            gnss_sync_.refine(gnss_data.timestamp_us, sys_now, 0.001, 0.01);
        } else {
            // 沒有時間戳，使用系統時間
            gnss_aligned_us = sys_now;
        }
        
        // 更新 GNSS 數據的時間戳為對齊後的值
        gnss_data.timestamp_us = gnss_aligned_us;
        gnss_data_ = gnss_data;
        have_gnss_ = true;
        
        
        // current_time_ 只前推不回退
        current_time_ = (gnss_aligned_us > current_time_) ? gnss_aligned_us : current_time_;
        
        IF_DETAILED_STATS(processing_stats_.gnss_updates++;)
        
        // 立即檢查並處理（避免延遲）
        processIfReady();
        
        // Push→Pull 解耦：通過回調觸發 GNSS 相關協議快照更新  
        if (gnss_update_callback_) {
            dfi_gnss_callback_checker_.recordIn();
            gnss_update_callback_(gnss_data_, nav_state_);
            dfi_gnss_callback_checker_.recordOut();
            dfi_gnss_callback_checker_.update();
        }
    }
    
    /**
     * @brief 主處理循環，定期調用以處理數據
     * @param current_time_us 當前時間戳（微秒）
     */
    void tick(uint64_t current_time_us) {
        current_time_ = current_time_us;
        processIfReady();
    }
    
    /**
     * @brief 重置狀態
     */
    void reset() {
        have_imu_ = false;
        have_gnss_ = false;
        memset(&nav_state_, 0, sizeof(nav_state_));
        nav_state_.schema = DATA_SCHEMA_VERSION;
        
        LOG_INFO("DFI", "狀態已重置");
    }
    
    /**
     * @brief 檢查是否有有效的導航數據
     */
    bool hasValidNavigation() const {
        return NavigationAdapter::isDataValid(nav_state_);
    }
    
    /**
     * @brief 獲取當前導航狀態（只讀）
     */
    const NavigationState& getNavigationState() const {
        return nav_state_;
    }
    
    IF_DETAILED_STATS(
    /**
     * @brief 獲取處理統計（僅在啟用詳細統計時可用）
     */
    struct ProcessingStats {
        uint32_t imu_updates = 0;
        uint32_t gnss_updates = 0;
        uint32_t nav_outputs = 0;
        uint32_t quality_failures = 0;
        uint32_t shift_applications = 0;
    };
    
    ProcessingStats getProcessingStats() const {
        return processing_stats_;
    }
    
    void resetProcessingStats() {
        processing_stats_ = {};
        LOG_INFO("DFI", "處理統計已重置");
    }
    )
    
    IF_DEBUG_STRINGS(
    /**
     * @brief 生成調試字串（僅在啟用調試字串時可用）
     */
    std::string getDebugString() const {
        char buffer[256];
        snprintf(buffer, sizeof(buffer),
                "DFI[IMU:%s GNSS:%s NAV:%s] Stats[I:%lu G:%lu N:%lu]",
                have_imu_ ? "OK" : "NO",
                have_gnss_ ? "OK" : "NO", 
                hasValidNavigation() ? "OK" : "NO",
                processing_stats_.imu_updates,
                processing_stats_.gnss_updates,
                processing_stats_.nav_outputs);
        return std::string(buffer);
    }
    )

private:
    // 數據狀態
    IMUData imu_data_;
    GNSSData gnss_data_;
    NavigationState nav_state_;
    bool have_imu_;
    bool have_gnss_;
    uint64_t current_time_;
    
    // PLSHD狀態追蹤
    bool plshd_valid_;          // PLSHD valid_flag狀態
    float plshd_heading_deg_;   // PLSHD heading值
    uint32_t plshd_last_time_;  // 最後收到PLSHD時間
    
    // 回調
    NavCallback nav_callback_;
    
    // NavigationState 監控器
    monitor::DataFlowMonitor* nav_monitor_;
    
    // Push→Pull 解耦回調
    IMUUpdateCallback imu_update_callback_;
    GNSSUpdateCallback gnss_update_callback_;
    
    // TDD 頻率監控
    TDDFlowChecker dfi_imu_callback_checker_;
    TDDFlowChecker dfi_gnss_callback_checker_;
    
    // 時間對齊器
    TimeAligner imu_sync_;
    TimeAligner gnss_sync_;
    
    // 統計數據（條件編譯）
    IF_DETAILED_STATS(ProcessingStats processing_stats_;)
    
    // 簡化時間戳處理 - 移除複雜的MTI時間對齊機制
    int64_t time_offset_;                    // 保留基本偏移量 (μs)
    uint64_t last_sync_time_;               // 上次同步時間 (ms)
    static constexpr uint64_t SYNC_INTERVAL_MS = 5000;              // 每5秒重新同步
    
    // 最小品質門檻常數（配合 MTI 對齊，調整為更寬鬆的值）
    static constexpr uint64_t IMU_MAX_AGE_US = 1000000;   // 1s (配合時間對齊的寬鬆設定)
    static constexpr uint64_t GNSS_MAX_AGE_US = 5000000;  // 5s (GNSS可以更長間隔)
    
    /**
     * @brief MTI 時間同步（參考 MINSPixhawk syncPX4Time）
     * 計算並更新 time_offset，使 MTI 時間軸與系統時間軸對齊
     */
    void syncMTITime() {
        uint32_t now_ms = millis();
        if (now_ms - last_sync_time_ < SYNC_INTERVAL_MS) {
            return;  // 未到同步時間
        }
        
        // 獲取當前系統時間（微秒）
        uint64_t system_time_us = static_cast<uint64_t>(micros());
        
        // 使用 IMU 時間戳作為 MTI 基準時間（模擬 XsensTime）
        if (have_imu_ && imu_data_.timestamp_us > 0) {
            // 計算新的時間偏移：system_time - mti_time
            int64_t new_offset = static_cast<int64_t>(system_time_us - imu_data_.timestamp_us);
            
            // 簡化：直接使用新的偏移量，不做複雜的滑動平均
            time_offset_ = new_offset;
            
            last_sync_time_ = now_ms;
            
            // 移除時間同步LOG，避免系統死機
        }
    }
    
    /**
     * @brief 生成統一時間戳（參考 MINSPixhawk getUnifiedTimestamp）
     * 將 MTI 時間戳轉換為統一的系統時間軸
     * @param mti_timestamp MTI 原始時間戳（微秒）
     * @return 對齊後的統一時間戳（微秒）
     */
    uint64_t getUnifiedTimestamp(uint64_t mti_timestamp) {
        if (mti_timestamp == 0) {
            return static_cast<uint64_t>(micros());  // 直接使用系統時間
        }
        
        // 檢查溢出風險
        if (time_offset_ > 0 && mti_timestamp > UINT64_MAX - time_offset_) {
            static uint32_t last_overflow_warn = 0;
            uint32_t now = millis();
            if (now - last_overflow_warn >= 5000) {
                LOG_WARN("DFI_TIME", "⚠️ 時間戳溢出風險，使用原始時間");
                last_overflow_warn = now;
            }
            return mti_timestamp;
        }
        
        if (time_offset_ < 0 && mti_timestamp < static_cast<uint64_t>(-time_offset_)) {
            static uint32_t last_underflow_warn = 0;
            uint32_t now = millis();
            if (now - last_underflow_warn >= 5000) {
                LOG_WARN("DFI_TIME", "⚠️ 時間戳下溢風險，使用原始時間");
                last_underflow_warn = now;
            }
            return mti_timestamp;
        }
        
        return mti_timestamp + time_offset_;
    }
    
    /**
     * @brief 處理數據融合（僅在條件滿足時）
     */
    void processIfReady() {
        // 最小條件：至少要有 IMU 數據
        if (!have_imu_) {
            return;
        }
        
        // 暫時註解重複處理檢查，確保數據完整流通  
        // TODO: 這個檢查可能導致時間戳相同的數據被過濾掉一半
        // static uint64_t last_processed_timestamp = 0;
        // if (imu_data_.timestamp_us != 0 && imu_data_.timestamp_us == last_processed_timestamp) {
        //     return;  // 同一個數據，避免重複處理
        // }
        // last_processed_timestamp = imu_data_.timestamp_us;
        
        // 1) 姿態：IMU → 歐拉角 (直接使用度數)
        // 直接從 IMU 數據複製歐拉角到 NavigationState
        nav_state_.euler_roll = imu_data_.euler_pitch*-1;
        nav_state_.euler_pitch = imu_data_.euler_roll;  
        nav_state_.euler_yaw = imu_data_.euler_yaw;
        nav_state_.flags |= NAV_ATTITUDE_VALID;
        
        // 更新加速度到 NavigationState (轉換為 m/s²)
        mu::Vector3f acceleration = IMUAdapter::getAccelerationVector(imu_data_);
        acceleration.x *= 9.8f;  // 轉換為 m/s²
        acceleration.y *= 9.8f;  // 轉換為 m/s²
        acceleration.z *= -9.8f;  // 轉換為 m/s²
        NavigationAdapter::updateAcceleration(nav_state_, acceleration.x, acceleration.y, acceleration.z);
        
        // 更新角速度到 NavigationState
        mu::Vector3f angular_velocity = IMUAdapter::getGyroVector(imu_data_);
        // === 軸轉換與偏移 ===
        float new_x = angular_velocity.y;                 // X ← 原 Y
        float new_y = angular_velocity.x; 
        float new_z = -angular_velocity.z;                // Z ← -Z

        // 更新 NavigationState
        NavigationAdapter::updateAngularVelocity(nav_state_, new_x, new_y, new_z);

         
        // 設置基本不確定性估計 (COV) - 給 PX4 使用的標準差
        mu::Vector3f pos_std(2.0f, 2.0f, 3.0f);        // GPS 位置精度 (米)
        mu::Vector3f vel_std(0.2f, 0.2f, 0.3f);        // GPS 速度精度 (m/s)  
        mu::Vector3f att_std(0.02f, 0.02f, 0.05f);     // IMU 姿態精度 (弧度)
        NavigationAdapter::updateUncertainties(nav_state_, pos_std, vel_std, att_std);
        
        // 2) GNSS 數據處理（可選）
        if (have_gnss_) {
            // 位置更新 - 直接從 NMEA 數據提取經緯度
            double lat, lon;
            float alt;
            if (GNSSAdapter::getPosition(gnss_data_, lat, lon, alt)) {
                // 簡化版：直接用經緯度作為 NED 坐標（近似處理）
                NavigationAdapter::updatePosition(nav_state_, (float)lat, (float)lon, alt);
            }
            
            // 速度更新
            mu::Vector3f velocity = GNSSAdapter::getVelocityVector(gnss_data_);
            NavigationAdapter::updateVelocity(nav_state_, velocity.x, velocity.y, velocity.z);
        }
        
        // Shift 校正（無論有沒有 GNSS 都要執行）
        applyOptionalShiftCorrection();
        
        // 3) MTI 時間軸對齊和智能時間戳更新
        // 首先進行 MTI 時間同步
        syncMTITime();
        
        // 簡化時間戳處理：直接使用系統時間，不做複雜驗證
        uint64_t simple_timestamp = static_cast<uint64_t>(micros());
        
        // 更新當前時間和導航狀態時間戳
        current_time_ = simple_timestamp;
        NavigationAdapter::updateTimestamp(nav_state_, simple_timestamp);
        
        // 暫時註解最小品質門檻檢查，確保數據完整流通
        // TODO: 確認數據流正常後，再考慮是否需要重新加入檢查
        // if (!passesMinimalQualityCheck()) {
        //     IF_DETAILED_STATS(processing_stats_.quality_failures++;)
        //     return;
        // }
        
        // 暫時註解最終有效性檢查，確保數據完整流通
        // TODO: 確認數據流正常後，再考慮是否需要重新加入檢查
        // if (!NavigationAdapter::isDataValid(nav_state_)) {
        //     return;
        // }
        
        // 6) 單槽背壓：直接回調（覆蓋模式）
        if (nav_callback_) {
            // 移除NavigationState頻率LOG，避免系統死機
            static uint32_t last_nav_freq_time = 0;
            static uint32_t nav_output_count = 0;
            uint32_t now = millis();
            
            nav_output_count++;
            
            if (now - last_nav_freq_time >= 5000) {
                nav_output_count = 0;
                last_nav_freq_time = now;
            }
            
            static uint32_t push_count = 0;
            push_count++;
            // 暫時禁用高頻日志
            // if (push_count % 50 == 1) {  // 每50次推送打印一次
            //     LOG_INFO("DFI", "🚀 推送 NavigationState #%lu 到 ProtocolManager", push_count);
            // }
            
            // DataFlowMonitor: 監控 NavigationState 輸出
            if (nav_monitor_) {
                nav_monitor_->recordBytes(sizeof(NavigationState));
                nav_monitor_->recordPackets(1);
                nav_monitor_->recordOperations(1);
                // LOG_DEBUG("DFI", "[MONITOR] 記錄 %zu bytes, 1 packet", sizeof(NavigationState));
            } else {
                static uint32_t last_warn_time = 0;
                uint32_t now = millis();
                if (now - last_warn_time >= 5000) {  // 5秒一次警告
                    LOG_WARN("[DFI-MONITOR] nav_monitor_ 為 nullptr!");
                    last_warn_time = now;
                }
            }
            
            nav_callback_(nav_state_);
            IF_DETAILED_STATS(processing_stats_.nav_outputs++;)
        }
    }
    
    /**
     * @brief MINSPixhawk 風格的最小品質門檻檢查
     * 採用信任時間對齊系統的方式，只檢查基本數據完整性
     * 時間相關驗證已在 smoothTimestamp() 中完成
     */
    bool passesMinimalQualityCheck() {
        static uint32_t rej_quat = 0; // 移除不再使用的 imu_stale, gnss_stale, rej_ts0 計數
        
        // 暫時註解姿態有效性檢查，確保數據完整流通
        // TODO: 確認數據流正常後，再考慮是否需要重新加入
        // if (!(nav_state_.flags & NAV_ATTITUDE_VALID)) {
        //     rej_quat++;
        //     LOG_DEBUG("DFI", "拒絕計數: quat=%lu", rej_quat);
        //     return false;
        // }
        
        // 2) 採用 MINSPixhawk 方式：信任時間對齊系統，不做原始數據新鮮度檢測
        // 時間相關驗證已在 smoothTimestamp() 中完成（跳躍檢測、異常修復）
        // 這裡只需記錄統計用途，不阻斷數據流
        
        // 自適應時間軸系統不需要檢查時間戳為0
        // micros() 系統時間不可能為0，各協議都有獨立時間軸
        
        // 定期輸出拒絕計數 (每100次檢查，簡化版)
        static uint32_t check_count = 0;
        check_count++;
        if (check_count % 100 == 0) {
            LOGI("[DFI-REJECT] quat=%lu ts0=%lu (MINSPixhawk風格：只檢查基本有效性)", 
                 rej_quat);
        }
        
        return true;
    }
    
    /**
     * @brief 可選的 Shift 校正應用
     * 有 GNSS heading 才應用，沒有就略過（不降級、不評分）
     */
    void applyOptionalShiftCorrection() {
        // 簡單的 shift 校正：MTI_yaw + heading_shift_offset
        float mti_yaw_deg = nav_state_.euler_yaw;  // 來自 MTI 的原始 YAW
        
        // 偏移量變數（有 GPS heading 時會更新，沒有時保持定值）
        static float heading_shift_offset = -90.0f;  // 預設 90° 偏移
        
        // 檢查是否有有效的 GPS heading 數據
        if (have_gnss_ && (gnss_data_.flags & GNSS_HEADING_VALID)) {
            // 將 GPS heading 從弧度轉換為度數
            float gps_heading_deg = gnss_data_.gnss_heading * mu::kRadToDeg;
            
            // Debug: 記錄 shift 變化
            float old_shift = heading_shift_offset;
            
            // 更新偏移量：heading_shift_offset = GPS_heading - MTI_yaw
            heading_shift_offset = gps_heading_deg - mti_yaw_deg;
            
            // 角度包裝到 [-180, 180] 範圍
            while (heading_shift_offset > 180.0f) heading_shift_offset -= 360.0f;
            while (heading_shift_offset < -180.0f) heading_shift_offset += 360.0f;
            
        }
        
        // 最終結果 = MTI_yaw + heading_shift_offset（無論有沒有 GPS）
        float corrected_yaw_deg = mti_yaw_deg + heading_shift_offset;
        
        // 包裝到 [0, 360] 範圍
        while (corrected_yaw_deg < 0.0f) corrected_yaw_deg += 360.0f;
        while (corrected_yaw_deg >= 360.0f) corrected_yaw_deg -= 360.0f;
        
        // YAW 平滑濾波：在 shift 處理完後應用，防止突然的大幅度變化
        static float previous_smoothed_yaw_deg = corrected_yaw_deg;
        static bool first_run = true;
        
        if (first_run) {
            // 第一次運行，直接使用當前值
            previous_smoothed_yaw_deg = corrected_yaw_deg;
            first_run = false;
        } else {
            // 處理角度跨越 0°/360° 邊界的情況
            float diff = corrected_yaw_deg - previous_smoothed_yaw_deg;
            float adjusted_current = corrected_yaw_deg;
            
            if (diff > 180.0f) {
                adjusted_current -= 360.0f;  // 調整新值
            } else if (diff < -180.0f) {
                adjusted_current += 360.0f;  // 調整新值
            }
            
            // 加權平均：當前值 * 0.7 + 上一次值 * 0.3
            float smoothed_yaw_deg = adjusted_current * 0.7f + previous_smoothed_yaw_deg * 0.3f;
            
            // 包裝到 [0, 360] 範圍
            while (smoothed_yaw_deg < 0.0f) smoothed_yaw_deg += 360.0f;
            while (smoothed_yaw_deg >= 360.0f) smoothed_yaw_deg -= 360.0f;
            
            corrected_yaw_deg = smoothed_yaw_deg;
            previous_smoothed_yaw_deg = corrected_yaw_deg;  // 更新上一次的值
        }
        
        // 更新校正後的 YAW
        nav_state_.euler_yaw = corrected_yaw_deg;
        
        // 每 5 秒打印一次：初始值 -> 校正後值
        static uint32_t last_print_time = 0;
        uint32_t current_time = millis();
        if (current_time - last_print_time >= 3000) {
            Serial.print("YAW: MTI:");
            Serial.print(mti_yaw_deg, 1);
            Serial.print("° + shift:");
            Serial.print(heading_shift_offset, 1);
            Serial.print("° = ");
            Serial.print(corrected_yaw_deg, 1);
            Serial.print("°");
            
            // 檢查PLSHD heading狀態 - 直接反映PLSHD valid_flag
            if (plshd_valid_) {
                Serial.print(" | GPS heading:");
                Serial.print(plshd_heading_deg_, 1);
                Serial.print("° (PLSHD有效)");
            } else {
                Serial.print(" | GPS heading:無 (PLSHD無效)");
            }
            Serial.println();
            
            last_print_time = current_time;
        }
    }
};

/**
 * @brief DFI 工廠函數
 * 提供標準配置的 DFI 實例
 */
inline DataFlowIntegrator* createStandardDFI() {
    return new DataFlowIntegrator();
}