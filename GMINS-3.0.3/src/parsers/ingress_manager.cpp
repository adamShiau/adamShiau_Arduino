/**
 * @file ingress_manager.cpp
 * @brief Ingress 數據管理器實現
 * 
 * 統一管理所有 Ingress Framer 的實現。
 */

#include "ingress_manager.h"
#include "../hal/board_support.h"
#include "../util/log.h"
#include "../util/tdd_flow_checker.h"
#include <Arduino.h>  // for millis()

// 函數現在在 hal 命名空間中，無需額外聲明

#define LOG_TAG "INGRESS_MGR"

// TDD Flow Checkers for IngressManager (已升級為頻率計算版本)
static TDDFlowChecker ingress_manager_process_checker("IngressManager:process", "process", 5000, true);        // 總處理頻率監控
static TDDFlowChecker ingress_manager_imu_read_checker("IngressManager:IMU_read", "IMU_read", 5000, true);      // IMU 讀取頻率監控  
static TDDFlowChecker ingress_manager_xbus_feed_checker("IngressManager:XBUS_feed", "XBUS_feed", 5000, true);   // XBUS 餵送頻率監控
static TDDFlowChecker ingress_manager_gnss_read_checker("IngressManager:GNSS_read", "GNSS_read", 5000, true);   // GNSS 讀取頻率監控
static TDDFlowChecker ingress_manager_nmea_feed_checker("IngressManager:NMEA_feed", "NMEA_feed", 5000, true);   // NMEA 餵送頻率監控

IngressManager::IngressManager() 
    : initialized_(false), imu_source_(nullptr), gnss_source_(nullptr) {
}

bool IngressManager::initialize() {
    if (initialized_) {
        LOGW("Ingress Manager 已經初始化");
        return true;
    }
    
    LOGI("🔄 正在初始化 Ingress Manager...");
    
    // 重置統計信息
    stats_ = {};
    
    // 重置所有 Framer
    xbus_framer_.reset();
    nmea_framer_.reset();
    
    // 設定 NMEA Framer 選項
    nmea_framer_.setChecksumValidation(true);
    nmea_framer_.setMaxSentenceLength(256);
    
    // 更新數據源
    updateDataSources();
    
    // 檢查數據源可用性
    if (!imu_source_ || !gnss_source_) {
        LOGE("❌ 無法獲取數據源 - HAL 系統可能未正確初始化");
        return false;
    }
    
    initialized_ = true;
    
    LOGI("✅ Ingress Manager 初始化完成");
    LOGI("  🧭 XBUS Framer: %s", xbus_framer_.getTypeName());
    LOGI("  🛰️ NMEA Framer: %s", nmea_framer_.getTypeName());
    
    return true;
}

void IngressManager::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOGI("🔄 正在關閉 Ingress Manager...");
    
    // 清除回調
    xbus_framer_.on_frame = nullptr;
    nmea_framer_.on_frame = nullptr;
    
    // 重置 Framer
    resetFramers();
    
    // 清空數據源指針
    imu_source_ = nullptr;
    gnss_source_ = nullptr;
    
    initialized_ = false;
    
    LOGI("✅ Ingress Manager 已關閉");
}

void IngressManager::process() {
    if (!initialized_) {
        return;
    }
    
    ingress_manager_process_checker.recordIn();  // TDD: 記錄 process 調用
    stats_.process_calls++;
    
    // 更新數據源（防止指針失效）
    updateDataSources();
    
    if (!imu_source_ || !gnss_source_) {
        static uint32_t null_source_count = 0;
        null_source_count++;
        if (null_source_count % 10000 == 1) {  // 每1萬次報告一次
            LOGI("⚠️ [DATA-SOURCE] 數據源檢查 #%lu: imu_source_=%p, gnss_source_=%p", 
                 null_source_count, imu_source_, gnss_source_);
        }
        return;
    }
    
    // 處理緩衝區 (優化為更小緩衝區以提高響應速度)
    uint8_t buffer[64];
    
    // 診斷：檢查數據可用性
    static uint32_t data_check_count = 0;
    static uint32_t last_data_check_time = 0;
    data_check_count++;
    uint32_t now = millis();
    
    size_t imu_available = imu_source_->available();
    size_t gnss_available = gnss_source_->available();
    
    // 處理 IMU 數據 (XBUS)
    while (imu_source_->available() > 0) {
        ingress_manager_imu_read_checker.recordIn();  // TDD: 記錄 IMU 讀取嘗試
        size_t bytes_read = imu_source_->read(buffer, sizeof(buffer));
        if (bytes_read > 0) {
            ingress_manager_imu_read_checker.recordOut();  // TDD: 記錄成功讀取
            stats_.total_imu_bytes += bytes_read;
            
            ingress_manager_xbus_feed_checker.recordIn();  // TDD: 記錄 XBUS feed 流入
            xbus_framer_.feed(buffer, bytes_read);
            ingress_manager_xbus_feed_checker.recordOut();  // TDD: 記錄 XBUS feed 完成
        }
    }
    
    // 處理 GNSS 數據 (NMEA)
    while (gnss_source_->available() > 0) {
        ingress_manager_gnss_read_checker.recordIn();  // TDD: 記錄 GNSS 讀取嘗試
        size_t bytes_read = gnss_source_->read(buffer, sizeof(buffer));
        if (bytes_read > 0) {
            ingress_manager_gnss_read_checker.recordOut();  // TDD: 記錄成功讀取
            stats_.total_gnss_bytes += bytes_read;
            
            ingress_manager_nmea_feed_checker.recordIn();  // TDD: 記錄 NMEA feed 流入
            nmea_framer_.feed(buffer, bytes_read);
            ingress_manager_nmea_feed_checker.recordOut();  // TDD: 記錄 NMEA feed 完成
        }
    }
    
    ingress_manager_process_checker.recordOut();  // TDD: 記錄 process 完成
    
    // TDD: 更新所有檢測器
    ingress_manager_process_checker.update();
    ingress_manager_imu_read_checker.update();
    ingress_manager_xbus_feed_checker.update();
    ingress_manager_gnss_read_checker.update();
    ingress_manager_nmea_feed_checker.update();
}

void IngressManager::setXbusFrameCallback(std::function<void(const uint8_t*, size_t)> callback) {
    xbus_framer_.on_frame = callback;
    LOGI("✅ XBUS 封包回調已設定");
}

void IngressManager::setNmeaFrameCallback(std::function<void(const uint8_t*, size_t)> callback) {
    nmea_framer_.on_frame = callback;
    LOGI("✅ NMEA 句子回調已設定");
}

void IngressManager::resetFramers() {
    xbus_framer_.reset();
    nmea_framer_.reset();
    LOGI("🔄 所有 Framer 已重置");
}

void IngressManager::reportStats() const {
    uint32_t now = millis();
    
    LOGI("📊 === Ingress Manager 統計報告 ===");
    
    // 管理器統計
    LOGI("  🔄 處理統計:");
    LOGI("    📞 process() 調用次數: %lu", stats_.process_calls);
    LOGI("    🧭 IMU 處理字節: %lu bytes", stats_.total_imu_bytes);
    LOGI("    🛰️ GNSS 處理字節: %lu bytes", stats_.total_gnss_bytes);
    
    // XBUS Framer 統計
    auto xbus_stats = xbus_framer_.getStats();
    LOGI("  🧭 XBUS Framer (%s):", xbus_framer_.getStateDescription());
    LOGI("    📥 處理字節: %lu", xbus_stats.total_bytes_processed);
    LOGI("    ✅ 完整封包: %lu", xbus_stats.frames_completed);
    LOGI("    ❌ 校驗錯誤: %lu", xbus_stats.checksum_errors);
    LOGI("    ⚠️ 同步錯誤: %lu", xbus_stats.sync_errors);
    LOGI("    📏 超大封包: %lu", xbus_stats.oversized_frames);
    
    // NMEA Framer 統計
    auto nmea_stats = nmea_framer_.getStats();
    LOGI("  🛰️ NMEA Framer (%s):", nmea_framer_.getStateDescription());
    LOGI("    📥 處理字節: %lu", nmea_stats.total_bytes_processed);
    LOGI("    ✅ 完整句子: %lu", nmea_stats.sentences_completed);
    LOGI("    ❌ 校驗錯誤: %lu", nmea_stats.checksum_errors);
    LOGI("    ⚠️ 格式錯誤: %lu", nmea_stats.malformed_sentences);
    LOGI("    📏 超長句子: %lu", nmea_stats.oversized_sentences);
    
    // 數據源狀態
    LOGI("  📡 數據源狀態:");
    LOGI("    🧭 IMU 可用: %s (%zu bytes)", 
         imu_source_ ? "YES" : "NO", 
         imu_source_ ? imu_source_->available() : 0);
    LOGI("    🛰️ GNSS 可用: %s (%zu bytes)", 
         gnss_source_ ? "YES" : "NO", 
         gnss_source_ ? gnss_source_->available() : 0);
    
    LOGI("=====================================");
}

void IngressManager::resetStats() {
    LOGI("🔄 重置 Ingress Manager 統計信息...");
    
    stats_ = {};
    xbus_framer_.resetStats();
    nmea_framer_.resetStats();
    
    LOGI("✅ 統計信息重置完成");
}

void IngressManager::updateDataSources() {
    // 從 HAL 獲取最新的數據源指針
    imu_source_ = hal::getIMUSource();
    gnss_source_ = hal::getGNSSSource();
}