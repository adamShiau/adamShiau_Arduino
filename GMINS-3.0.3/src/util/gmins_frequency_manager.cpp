#define LOG_TAG "FREQ_MGR"

// ===== 頻率管理器日誌控制 =====
#ifndef LOG_LEVEL_LOCAL
#define LOG_LEVEL_LOCAL LOG_INFO
#endif

#ifndef LOG_LEVEL_MASK_LOCAL
// 完全靜音（本檔）
#define LOG_LEVEL_MASK_LOCAL 0
#endif

#include "gmins_frequency_manager.h"
#include "log.h"
#include <Arduino.h>

namespace gmins {

    // 全域管理器實例
    static FrequencyManager* g_frequency_manager = nullptr;

    // ==================== FrequencyManager 實作 ====================
    
    FrequencyManager::FrequencyManager(uint32_t report_interval_ms) 
        : report_interval_ms_(report_interval_ms) {
        
        // 初始化陣列為nullptr
        for (int i = 0; i < INPUT_SENSOR_COUNT; i++) {
            input_monitors_[i] = nullptr;
        }
        for (int i = 0; i < OUTPUT_MAVLINK_COUNT; i++) {
            output_monitors_[i] = nullptr;
        }
        for (int i = 0; i < CUSTOM_PACKET_COUNT; i++) {
            custom_monitors_[i] = nullptr;
        }
        
        LOGI("FrequencyManager created (interval: %lu ms)", report_interval_ms_);
    }
    
    FrequencyManager::~FrequencyManager() {
        shutdown();
        LOGI("FrequencyManager destroyed");
    }
    
    bool FrequencyManager::initialize() {
        if (initialized_) {
            LOGW("FrequencyManager already initialized");
            return true;
        }
        
        LOGI("Initializing GMINS Frequency Manager...");
        
        createMonitors();
        initialized_ = true;
        
        LOGI("✅ GMINS Frequency Manager initialized successfully");
        LOGI("監控項目: MTI, GNSS(GNGGA/GNRMC/PLSHD), MAVLink(ODO/INPUT/RAW), Custom(Fusion/Status/Debug)");
        
        return true;
    }
    
    void FrequencyManager::shutdown() {
        if (!initialized_) return;
        
        LOGI("Shutting down GMINS Frequency Manager...");
        
        // 刪除所有監控器
        for (int i = 0; i < INPUT_SENSOR_COUNT; i++) {
            delete input_monitors_[i];
            input_monitors_[i] = nullptr;
        }
        for (int i = 0; i < OUTPUT_MAVLINK_COUNT; i++) {
            delete output_monitors_[i];
            output_monitors_[i] = nullptr;
        }
        for (int i = 0; i < CUSTOM_PACKET_COUNT; i++) {
            delete custom_monitors_[i];
            custom_monitors_[i] = nullptr;
        }
        
        initialized_ = false;
        LOGI("GMINS Frequency Manager shutdown complete");
    }
    
    void FrequencyManager::createMonitors() {
        // 創建感測器輸入監控器
        input_monitors_[MTI_IMU] = monitor::createSensorMonitor("MTI-IMU");
        input_monitors_[GNSS_GNGGA] = monitor::createSensorMonitor("GNSS-GNGGA");
        input_monitors_[GNSS_GNRMC] = monitor::createSensorMonitor("GNSS-GNRMC");
        input_monitors_[GNSS_PLSHD] = monitor::createSensorMonitor("GNSS-PLSHD");
        
        // 創建MAVLink輸出監控器
        output_monitors_[MAVLINK_ODO] = monitor::createProtocolMonitor("MAVLink-ODO");
        output_monitors_[MAVLINK_INPUT] = monitor::createProtocolMonitor("MAVLink-INPUT");
        output_monitors_[MAVLINK_RAW] = monitor::createProtocolMonitor("MAVLink-RAW");
        
        // 創建自定義封包監控器
        custom_monitors_[CUSTOM_FUSION] = monitor::createProtocolMonitor("Custom-Fusion");
        custom_monitors_[CUSTOM_STATUS] = monitor::createProtocolMonitor("Custom-Status");
        custom_monitors_[CUSTOM_DEBUG] = monitor::createProtocolMonitor("Custom-Debug");
        
        // 設定報告間隔
        for (int i = 0; i < INPUT_SENSOR_COUNT; i++) {
            if (input_monitors_[i]) {
                input_monitors_[i]->setReportInterval(report_interval_ms_);
                mcm_.registerMonitor(input_monitors_[i]);
            }
        }
        for (int i = 0; i < OUTPUT_MAVLINK_COUNT; i++) {
            if (output_monitors_[i]) {
                output_monitors_[i]->setReportInterval(report_interval_ms_);
                mcm_.registerMonitor(output_monitors_[i]);
            }
        }
        for (int i = 0; i < CUSTOM_PACKET_COUNT; i++) {
            if (custom_monitors_[i]) {
                custom_monitors_[i]->setReportInterval(report_interval_ms_);
                mcm_.registerMonitor(custom_monitors_[i]);
            }
        }
        
        LOGI("Created %d input monitors, %d output monitors, %d custom monitors", 
             INPUT_SENSOR_COUNT, OUTPUT_MAVLINK_COUNT, CUSTOM_PACKET_COUNT);
    }
    
    // ==================== 感測器輸入記錄 ====================
    
    void FrequencyManager::recordMTI(uint32_t bytes) {
        if (input_monitors_[MTI_IMU]) {
            input_monitors_[MTI_IMU]->recordBytes(bytes);
            input_monitors_[MTI_IMU]->recordPackets(1);
            input_monitors_[MTI_IMU]->recordOperations(1);
        }
    }
    
    void FrequencyManager::recordGNSS_GNGGA(uint32_t bytes) {
        if (input_monitors_[GNSS_GNGGA]) {
            input_monitors_[GNSS_GNGGA]->recordBytes(bytes);
            input_monitors_[GNSS_GNGGA]->recordPackets(1);
            input_monitors_[GNSS_GNGGA]->recordOperations(1);
        }
    }
    
    void FrequencyManager::recordGNSS_GNRMC(uint32_t bytes) {
        if (input_monitors_[GNSS_GNRMC]) {
            input_monitors_[GNSS_GNRMC]->recordBytes(bytes);
            input_monitors_[GNSS_GNRMC]->recordPackets(1);
            input_monitors_[GNSS_GNRMC]->recordOperations(1);
        }
    }
    
    void FrequencyManager::recordGNSS_PLSHD(uint32_t bytes) {
        if (input_monitors_[GNSS_PLSHD]) {
            input_monitors_[GNSS_PLSHD]->recordBytes(bytes);
            input_monitors_[GNSS_PLSHD]->recordPackets(1);
            input_monitors_[GNSS_PLSHD]->recordOperations(1);
        }
    }
    
    // ==================== MAVLink輸出記錄 ====================
    
    void FrequencyManager::recordMAVLink_ODO(uint32_t bytes) {
        if (output_monitors_[MAVLINK_ODO]) {
            output_monitors_[MAVLINK_ODO]->recordBytes(bytes);
            output_monitors_[MAVLINK_ODO]->recordPackets(1);
            output_monitors_[MAVLINK_ODO]->recordOperations(1);
        }
    }
    
    void FrequencyManager::recordMAVLink_INPUT(uint32_t bytes) {
        if (output_monitors_[MAVLINK_INPUT]) {
            output_monitors_[MAVLINK_INPUT]->recordBytes(bytes);
            output_monitors_[MAVLINK_INPUT]->recordPackets(1);
            output_monitors_[MAVLINK_INPUT]->recordOperations(1);
        }
    }
    
    void FrequencyManager::recordMAVLink_RAW(uint32_t bytes) {
        if (output_monitors_[MAVLINK_RAW]) {
            output_monitors_[MAVLINK_RAW]->recordBytes(bytes);
            output_monitors_[MAVLINK_RAW]->recordPackets(1);
            output_monitors_[MAVLINK_RAW]->recordOperations(1);
        }
    }
    
    // ==================== 自定義封包記錄 ====================
    
    void FrequencyManager::recordCustom_Fusion(uint32_t bytes) {
        if (custom_monitors_[CUSTOM_FUSION]) {
            custom_monitors_[CUSTOM_FUSION]->recordBytes(bytes);
            custom_monitors_[CUSTOM_FUSION]->recordPackets(1);
            custom_monitors_[CUSTOM_FUSION]->recordOperations(1);
        }
    }
    
    void FrequencyManager::recordCustom_Status(uint32_t bytes) {
        if (custom_monitors_[CUSTOM_STATUS]) {
            custom_monitors_[CUSTOM_STATUS]->recordBytes(bytes);
            custom_monitors_[CUSTOM_STATUS]->recordPackets(1);
            custom_monitors_[CUSTOM_STATUS]->recordOperations(1);
        }
    }
    
    void FrequencyManager::recordCustom_Debug(uint32_t bytes) {
        if (custom_monitors_[CUSTOM_DEBUG]) {
            custom_monitors_[CUSTOM_DEBUG]->recordBytes(bytes);
            custom_monitors_[CUSTOM_DEBUG]->recordPackets(1);
            custom_monitors_[CUSTOM_DEBUG]->recordOperations(1);
        }
    }
    
    // ==================== 頻率查詢 ====================
    
    float FrequencyManager::getMTI_Frequency() const {
        if (input_monitors_[MTI_IMU]) {
            monitor::DataFlowStats stats = input_monitors_[MTI_IMU]->getStats();
            return stats.packet_rate_hz;
        }
        return 0.0f;
    }
    
    float FrequencyManager::getGNSS_TotalFrequency() const {
        float total = 0.0f;
        for (int i = GNSS_GNGGA; i <= GNSS_PLSHD; i++) {
            if (input_monitors_[i]) {
                monitor::DataFlowStats stats = input_monitors_[i]->getStats();
                total += stats.packet_rate_hz;
            }
        }
        return total;
    }
    
    float FrequencyManager::getMAVLink_TotalFrequency() const {
        float total = 0.0f;
        for (int i = 0; i < OUTPUT_MAVLINK_COUNT; i++) {
            if (output_monitors_[i]) {
                monitor::DataFlowStats stats = output_monitors_[i]->getStats();
                total += stats.packet_rate_hz;
            }
        }
        return total;
    }
    
    float FrequencyManager::getMAVLink_ODO_Frequency() const {
        if (output_monitors_[MAVLINK_ODO]) {
            monitor::DataFlowStats stats = output_monitors_[MAVLINK_ODO]->getStats();
            return stats.packet_rate_hz;
        }
        return 0.0f;
    }
    
    float FrequencyManager::getMAVLink_INPUT_Frequency() const {
        if (output_monitors_[MAVLINK_INPUT]) {
            monitor::DataFlowStats stats = output_monitors_[MAVLINK_INPUT]->getStats();
            return stats.packet_rate_hz;
        }
        return 0.0f;
    }
    
    float FrequencyManager::getMAVLink_RAW_Frequency() const {
        if (output_monitors_[MAVLINK_RAW]) {
            monitor::DataFlowStats stats = output_monitors_[MAVLINK_RAW]->getStats();
            return stats.packet_rate_hz;
        }
        return 0.0f;
    }
    
    float FrequencyManager::getCustom_TotalFrequency() const {
        float total = 0.0f;
        for (int i = 0; i < CUSTOM_PACKET_COUNT; i++) {
            if (custom_monitors_[i]) {
                monitor::DataFlowStats stats = custom_monitors_[i]->getStats();
                total += stats.packet_rate_hz;
            }
        }
        return total;
    }
    
    // ==================== 報告生成 ====================
    
    void FrequencyManager::generateFrequencyReport() {
        if (!initialized_) return;
        
        LOGI("🎯 ===== GMINS 頻率分析報告 =====");
        
        // 感測器輸入頻率
        LOGI("📡 感測器輸入頻率:");
        LOGI("  🧭 MTI-IMU:     %.2f Hz (目標: %.1f Hz)", getMTI_Frequency(), targets_.mti_target_hz);
        
        if (input_monitors_[GNSS_GNGGA]) {
            LOGI("  📍 GNSS-GNGGA:  %.2f Hz", input_monitors_[GNSS_GNGGA]->getStats().packet_rate_hz);
        }
        if (input_monitors_[GNSS_GNRMC]) {
            LOGI("  📍 GNSS-GNRMC:  %.2f Hz", input_monitors_[GNSS_GNRMC]->getStats().packet_rate_hz);
        }
        if (input_monitors_[GNSS_PLSHD]) {
            LOGI("  📍 GNSS-PLSHD:  %.2f Hz", input_monitors_[GNSS_PLSHD]->getStats().packet_rate_hz);
        }
        LOGI("  📍 GNSS 總計:   %.2f Hz (目標: %.1f Hz)", getGNSS_TotalFrequency(), targets_.gnss_target_hz);
        
        // MAVLink輸出頻率
        LOGI("📤 MAVLink 輸出頻率:");
        if (output_monitors_[MAVLINK_ODO]) {
            LOGI("  🚀 ODO:         %.2f Hz", output_monitors_[MAVLINK_ODO]->getStats().packet_rate_hz);
        }
        if (output_monitors_[MAVLINK_INPUT]) {
            LOGI("  🚀 INPUT:       %.2f Hz", output_monitors_[MAVLINK_INPUT]->getStats().packet_rate_hz);
        }
        if (output_monitors_[MAVLINK_RAW]) {
            LOGI("  🚀 RAW:         %.2f Hz", output_monitors_[MAVLINK_RAW]->getStats().packet_rate_hz);
        }
        LOGI("  🚀 MAVLink 總計: %.2f Hz (目標: %.1f Hz)", getMAVLink_TotalFrequency(), targets_.mavlink_target_hz);
        
        // 自定義封包頻率
        LOGI("📦 自定義封包頻率:");
        if (custom_monitors_[CUSTOM_FUSION]) {
            LOGI("  🔄 Fusion:      %.2f Hz", custom_monitors_[CUSTOM_FUSION]->getStats().packet_rate_hz);
        }
        if (custom_monitors_[CUSTOM_STATUS]) {
            LOGI("  📊 Status:      %.2f Hz", custom_monitors_[CUSTOM_STATUS]->getStats().packet_rate_hz);
        }
        if (custom_monitors_[CUSTOM_DEBUG]) {
            LOGI("  🔧 Debug:       %.2f Hz", custom_monitors_[CUSTOM_DEBUG]->getStats().packet_rate_hz);
        }
        LOGI("  📦 Custom 總計:  %.2f Hz (目標: %.1f Hz)", getCustom_TotalFrequency(), targets_.custom_target_hz);
        
        // 性能評估
        bool targets_met = checkFrequencyTargets();
        LOGI("🎯 目標達成: %s", targets_met ? "✅ 達成" : "❌ 未達成");
        
        LOGI("===================================");
    }
    
    void FrequencyManager::generateQuickSummary() {
        if (!initialized_) return;
        
        LOGI("⚡ GMINS頻率速覽: MTI=%.1fHz | GNSS=%.1fHz | MAVLink=%.1fHz | Custom=%.1fHz",
             getMTI_Frequency(), getGNSS_TotalFrequency(), 
             getMAVLink_TotalFrequency(), getCustom_TotalFrequency());
    }
    
    void FrequencyManager::updateAll() {
        if (!initialized_) return;
        mcm_.updateAll();
    }
    
    // ==================== 目標檢查 ====================
    
    bool FrequencyManager::checkFrequencyTargets() {
        if (!initialized_) return false;
        
        bool mti_ok = checkSingleTarget(getMTI_Frequency(), targets_.mti_target_hz);
        bool gnss_ok = checkSingleTarget(getGNSS_TotalFrequency(), targets_.gnss_target_hz);
        bool mavlink_ok = checkSingleTarget(getMAVLink_TotalFrequency(), targets_.mavlink_target_hz);
        bool custom_ok = checkSingleTarget(getCustom_TotalFrequency(), targets_.custom_target_hz);
        
        return mti_ok && gnss_ok && mavlink_ok && custom_ok;
    }
    
    bool FrequencyManager::checkSingleTarget(float actual, float target, float tolerance) const {
        if (target <= 0.0f) return true;  // 無目標要求時視為達成
        
        float diff_percent = fabs(actual - target) / target;
        return diff_percent <= tolerance;
    }
    
    void FrequencyManager::setFrequencyTargets(float mti_hz, float gnss_hz, float mavlink_hz, float custom_hz) {
        targets_.mti_target_hz = mti_hz;
        targets_.gnss_target_hz = gnss_hz;
        targets_.mavlink_target_hz = mavlink_hz;
        targets_.custom_target_hz = custom_hz;
        
        LOGI("頻率目標已更新: MTI=%.1f, GNSS=%.1f, MAVLink=%.1f, Custom=%.1f Hz",
             mti_hz, gnss_hz, mavlink_hz, custom_hz);
    }
    
    void FrequencyManager::resetAllStatistics() {
        if (!initialized_) return;
        
        for (int i = 0; i < INPUT_SENSOR_COUNT; i++) {
            if (input_monitors_[i]) input_monitors_[i]->resetStats();
        }
        for (int i = 0; i < OUTPUT_MAVLINK_COUNT; i++) {
            if (output_monitors_[i]) output_monitors_[i]->resetStats();
        }
        for (int i = 0; i < CUSTOM_PACKET_COUNT; i++) {
            if (custom_monitors_[i]) custom_monitors_[i]->resetStats();
        }
        
        LOGI("所有頻率統計已重置");
    }
    
    void FrequencyManager::setReportInterval(uint32_t interval_ms) {
        report_interval_ms_ = interval_ms;
        
        if (!initialized_) return;
        
        for (int i = 0; i < INPUT_SENSOR_COUNT; i++) {
            if (input_monitors_[i]) input_monitors_[i]->setReportInterval(interval_ms);
        }
        for (int i = 0; i < OUTPUT_MAVLINK_COUNT; i++) {
            if (output_monitors_[i]) output_monitors_[i]->setReportInterval(interval_ms);
        }
        for (int i = 0; i < CUSTOM_PACKET_COUNT; i++) {
            if (custom_monitors_[i]) custom_monitors_[i]->setReportInterval(interval_ms);
        }
        
        LOGI("報告間隔已設為 %lu ms", interval_ms);
    }
    
    // ==================== 全域函數 ====================
    
    FrequencyManager* getGlobalFrequencyManager() {
        return g_frequency_manager;
    }
    
    bool initializeGlobalFrequencyManager(uint32_t report_interval_ms) {
        if (g_frequency_manager) {
            LOGW("Global FrequencyManager already exists");
            return true;
        }
        
        g_frequency_manager = new FrequencyManager(report_interval_ms);
        if (g_frequency_manager && g_frequency_manager->initialize()) {
            LOGI("✅ Global GMINS FrequencyManager initialized");
            return true;
        }
        
        delete g_frequency_manager;
        g_frequency_manager = nullptr;
        LOGE("❌ Failed to initialize Global FrequencyManager");
        return false;
    }
    
    void shutdownGlobalFrequencyManager() {
        if (g_frequency_manager) {
            g_frequency_manager->shutdown();
            delete g_frequency_manager;
            g_frequency_manager = nullptr;
            LOGI("Global FrequencyManager shutdown");
        }
    }

} // namespace gmins