#pragma once

#include "../data/data_types.h"
#include "system_health.h"
#include "DataFlowIntegrator.h"
#include "ProtocolManagerDualMode.h"
#include "../transport/TxMultiplexer.h"
#include "../transport/ITransport.h"
#include "../parsers/ingress_manager.h"
#include "../util/log.h"
#include "../util/data_flow_monitor.h"

enum class SystemState {
    INIT,
    CALIBRATING,
    RUNNING,
    ERROR,
    SAFE_MODE,
    SHUTDOWN
};

enum class OperationMode {
    NORMAL,
    GPS_ONLY,
    IMU_ONLY,
    EMERGENCY
};


class SystemController {
public:
    SystemController();
    ~SystemController();
    
    bool initialize();
    void run();
    void shutdown();
    
    SystemState getState() const { return current_state_; }
    OperationMode getMode() const { return operation_mode_; }
    SystemHealth getHealth() const { return system_health_; }
    
    void requestModeChange(OperationMode new_mode);
    void handleError(const char* error_msg);
    
    // 新增：協議管理介面
    bool setProtocolMode(const std::string& protocol_name);
    bool setProtocolMode(const char* protocol_name);
    const char* getCurrentProtocolName() const;
    
    // 新增：數據流控制
    void enableDataFlow(bool enable);
    bool isDataFlowEnabled() const { return data_flow_enabled_; }
    
    // 新增：統計信息
    struct SystemStats {
        uint32_t total_cycles;
        uint32_t dfi_process_count;
        uint32_t protocol_send_count;
        // error_recovery_count 已移除 - 錯誤恢復系統已禁用
        uint64_t last_nav_timestamp;
    };
    
    SystemStats getStats() const { return stats_; }
    void resetStats();
    
    // 新增：單次處理循環
    void tick();
    
    // 數據流監控報告已移除
    // void generateDataFlowReport();
    
    // 測試功能：合成資料注入
    void testSyntheticDataInjection();
    
    // CUBE/Pixhawk自動偵測 (MTI校正完成後執行)
    void waitForMTICalibrationAndDetectPixhawk();
    
    // Integration Layer 存取方法
    DataFlowIntegrator* getDataFlowIntegrator() { return dfi_; }
    ProtocolManagerDualMode* getProtocolManager() { return protocol_manager_; }
    TxMultiplexer* getTxMultiplexer() { return tx_multiplexer_; }
    
    // 高性能版本：移除複雜的頻率控制接口
    // ProtocolManager 現在使用固定 100Hz 頻率
    void setOutputFrequency(float hz) {
        // 高性能版本：不再支持動態頻率調整
        // 固定使用 100Hz 以獲得最佳性能
    }
    
    float getCurrentFrequency() const {
        return 100.0f; // 高性能版本固定頻率
    }
    
    // CUBE/Pixhawk自動偵測和模式切換
    bool detectPixhawkAndSetMode();
    
private:
    SystemState current_state_;
    OperationMode operation_mode_;
    SystemHealth system_health_;
    
    // 新增：核心組件
    DataFlowIntegrator* dfi_;
    ProtocolManagerDualMode* protocol_manager_;
    TxMultiplexer* tx_multiplexer_;
    IngressManager* ingress_manager_;
    ITransport* serial_transport_;  // 通過 HAL Interface 管理的傳輸層
    
    // 數據流監控系統
    monitor::MultiChannelMonitor mcm_;
    
    // 新增：運行時狀態
    bool data_flow_enabled_;
    uint64_t last_tick_time_;
    
    // CUBE/Pixhawk自動偵測結果
    bool pixhawk_detected_;
    SystemStats stats_;
    
    void processState();
    void updateHealth();
    bool checkTransitionConditions(SystemState target_state);
    
    // 數據流監控系統已移除
    // void setupDataFlowMonitoring();
    
    // 新增：核心處理循環
    void processingLoop();
    // 移除舊的 handleNavigationData - Push→Pull 解耦後不再需要
    // void handleNavigationData(const NavigationState& nav_state);
    
    // 新增：組件管理
    bool initializeComponents();
    void shutdownComponents();
    bool setupDataFlow();
    
    // 錯誤恢復系統已移除 - 簡化架構，避免不必要的數據重置
    
    // 新增：健康檢查
    void updateComponentHealth();
    bool checkDFIHealth();
    bool checkProtocolHealth();
    bool checkIngressHealth();
    
    // 新增：統計更新
    void updateStats();
    
    // Push→Pull 解耦回調函數
    static void imuUpdateCallback(const IMUData& imu_data, const NavigationState& nav_state);
    static void gnssUpdateCallback(const GNSSData& gnss_data, const NavigationState& nav_state);
    static SystemController* instance_; // 單例指標
    
    // 新增：時間管理
    uint64_t getCurrentTimeUs() const;
};