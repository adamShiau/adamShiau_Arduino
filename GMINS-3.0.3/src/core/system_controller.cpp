// 暫時啟用LOG_INFO用於調試數據流
#define LOG_LEVEL_LOCAL LOG_INFO  

#include "system_controller.h"
#include "../hal/board_support.h"
#include "../parsers/xbus_parser.h"
#include "../parsers/nmea_parser.h"
#include "../comm/uart_interface.h"
#include "../util/tdd_flow_checker.h"

#define LOG_TAG "SYS_CTRL"

// 單例指標初始化
SystemController* SystemController::instance_ = nullptr;

// Integration Layer TDD Flow Checkers (已升級為頻率計算版本)
static TDDFlowChecker xbus_parser_checker("XBUSParser:parseFrame", "parseFrame", 5000, true);           // IMU 解析頻率監控
static TDDFlowChecker data_flow_integrator_checker("DataFlowIntegrator:processIMU", "processIMU", 5000, true); // 數據整合頻率監控  
// 舊的 protocol_manager_checker 已移除 - Push→Pull 架構中不再使用統一的 sendNavigation 路徑

SystemController::SystemController()
    : current_state_(SystemState::INIT)
    , operation_mode_(OperationMode::NORMAL)
    , dfi_(nullptr)
    , protocol_manager_(nullptr)
    , tx_multiplexer_(nullptr)
    , ingress_manager_(nullptr)
    , serial_transport_(nullptr)
    , data_flow_enabled_(false)
    , last_tick_time_(0)
    , pixhawk_detected_(false)
{
    // 設置單例指標
    instance_ = this;
    
    // 初始化統計
    resetStats();
    
    // 初始化健康狀態
    system_health_.communication_ok = false;
    system_health_.fusion_ok = false;
    system_health_.safety_ok = true;
    system_health_.protocol_ok = false;
    system_health_.dfi_ok = false;
    system_health_.error_count = 0;
    system_health_.warning_count = 0;
    
    LOG_INFO("SystemController 建構完成");
}

SystemController::~SystemController() {
    shutdown();
    instance_ = nullptr;
}

bool SystemController::initialize() {
    LOG_INFO("開始系統初始化");
    
    current_state_ = SystemState::INIT;
    
    // 1) 初始化核心組件
    if (!initializeComponents()) {
        LOG_ERR("核心組件初始化失敗");
        current_state_ = SystemState::ERROR;
        return false;
    }
    
    // 2) 設置數據流
    if (!setupDataFlow()) {
        LOG_ERR("數據流設置失敗");
        current_state_ = SystemState::ERROR;
        return false;
    }
    
    // 3) 啟用數據流
    enableDataFlow(true);
    
    // 4) 轉換到運行狀態  
    current_state_ = SystemState::RUNNING;
    
    LOG_INFO("系統初始化完成");
    return true;
}

void SystemController::tick() {
    if (current_state_ == SystemState::RUNNING) {
        processingLoop();
        updateHealth();
        updateStats();
        
        // TDD: 更新 Integration Layer 流程檢測器
        xbus_parser_checker.update();
        data_flow_integrator_checker.update();
        // protocol_manager_checker.update(); // 已移除 - 改用各協議內部的 transport 監控
        
        // 🗑️ 已移除 TxMultiplexer 頻率統計 - 防止記憶體累積
        // 原本的 updateFrequencyStats() 方法已從 TxMultiplexer 中移除
    }
}

void SystemController::run() {
    LOG_INFO("開始主循環");
    
    last_tick_time_ = getCurrentTimeUs();
    
    while (current_state_ != SystemState::SHUTDOWN) {
        uint64_t current_time = getCurrentTimeUs();
        
        // 處理狀態機
        processState();
        
        // 主處理循環
        if (current_state_ == SystemState::RUNNING) {
            processingLoop();
        }
        
        // 更新健康狀態
        updateHealth();
        
        // 更新統計
        updateStats();
        
        // 錯誤恢復系統已移除 - 系統保持運行狀態，避免不必要的重置
        
        last_tick_time_ = current_time;
        
        // 簡單的循環控制（避免過度消耗 CPU）
        delayMicroseconds(1000); // 1ms
    }
    
    LOG_INFO("主循環結束");
}

void SystemController::shutdown() {
    LOG_INFO("開始系統關閉");
    
    current_state_ = SystemState::SHUTDOWN;
    
    // 停用數據流
    enableDataFlow(false);
    
    // 關閉組件
    shutdownComponents();
    
    LOG_INFO("系統關閉完成");
}

bool SystemController::setProtocolMode(const char* protocol_name) {
    if (!protocol_manager_) {
        LOG_ERR("ProtocolManager 未初始化");
        return false;
    }
    
    LOG_INFO("請求切換協議到：%s", protocol_name);
    
    bool success = protocol_manager_->setProtocolMode(protocol_name);
    if (success) {
        LOG_INFO("協議切換成功：%s", getCurrentProtocolName());
    } else {
        LOG_ERR("協議切換失敗");
        system_health_.error_count++;
    }
    
    return success;
}

const char* SystemController::getCurrentProtocolName() const {
    return protocol_manager_ ? protocol_manager_->getCurrentProtocolName() : "NONE";
}

void SystemController::enableDataFlow(bool enable) {
    data_flow_enabled_ = enable;
    LOG_INFO("數據流已%s", enable ? "啟用" : "停用");
}


void SystemController::resetStats() {
    stats_ = {
        .total_cycles = 0,
        .dfi_process_count = 0,
        .protocol_send_count = 0,
        // error_recovery_count 已移除
        .last_nav_timestamp = 0
    };
}

void SystemController::requestModeChange(OperationMode new_mode) {
    LOG_INFO("請求模式變更：%d → %d", 
             static_cast<int>(operation_mode_), static_cast<int>(new_mode));
    operation_mode_ = new_mode;
}

void SystemController::handleError(const char* error_msg) {
    LOG_ERR("系統錯誤：%s", error_msg);
    system_health_.error_count++;
    
    // 不再設置 ERROR 狀態 - 允許系統在錯誤時繼續運行
}

// ============================================================================
// 私有方法實作
// ============================================================================

bool SystemController::initializeComponents() {
    // 1) 初始化 HAL 外設（由 framer 處理數據發送）
    hal::initPeripherals();
    
    // 2) 建立 TxMultiplexer（通過 HAL Interface 而非直接 Serial1）
    HardwareSerial& mavlink_serial = hal::getUARTSerial(UARTChannel::MAVLINK);
    serial_transport_ = new UARTTransport(mavlink_serial, 230400);  // ✅ 預設AR1AFC波特率
    if (!serial_transport_->initialize()) {
        LOG_ERR("UARTTransport 初始化失敗");
        return false;
    }
    tx_multiplexer_ = new TxMultiplexer(serial_transport_);
    
    // 3) 建立 ProtocolManagerDualMode (簡化版本)
    protocol_manager_ = new ProtocolManagerDualMode(tx_multiplexer_);
    
    // 4) 建立 DataFlowIntegrator
    dfi_ = createStandardDFI();
    
    // 5) 建立 IngressManager
    ingress_manager_ = new IngressManager();
    if (!ingress_manager_->initialize()) {
        LOG_ERR("IngressManager 初始化失敗");
        return false;
    }
    
    
    LOG_INFO("核心組件初始化完成");
    return true;
}

void SystemController::shutdownComponents() {
    // 按相反順序關閉組件
    
    if (ingress_manager_) {
        ingress_manager_->shutdown();
        delete ingress_manager_;
        ingress_manager_ = nullptr;
    }
    
    if (dfi_) {
        delete dfi_;
        dfi_ = nullptr;
    }
    
    if (protocol_manager_) {
        delete protocol_manager_;
        protocol_manager_ = nullptr;
    }
    
    if (tx_multiplexer_) {
        delete tx_multiplexer_;
        tx_multiplexer_ = nullptr;
    }
    
    // 清理 serial_transport_（通過 HAL Interface 管理）
    if (serial_transport_) {
        serial_transport_->cleanup();
        delete serial_transport_;
        serial_transport_ = nullptr;
    }
    
    LOG_INFO("組件關閉完成");
}

bool SystemController::setupDataFlow() {
    if (!dfi_ || !ingress_manager_) {
        LOG_ERR("DFI 或 IngressManager 未初始化");
        return false;
    }
    
    // 設置新的 Push→Pull 解耦回調（使用靜態函數）
    dfi_->setIMUUpdateCallback(imuUpdateCallback);
    dfi_->setGNSSUpdateCallback(gnssUpdateCallback);
    
    // 移除舊的 NavCallback - 不再需要！
    // dfi_->setNavCallback(navigationDataCallback);
    
    // 設置 IngressManager 回調 (加入 MTI 診斷)
    ingress_manager_->setXbusFrameCallback([this](const uint8_t* data, size_t length) {
        if (!data_flow_enabled_) {
            static uint32_t blocked_count = 0;
            blocked_count++;
            if ((blocked_count % 100) == 1) {  // 每100次報告一次
                LOGI("🚫 XBUS callback BLOCKED: data_flow_enabled_=false (count=%lu)", blocked_count);
            }
            return;
        }
        
        // MTI 診斷統計
        static uint32_t mti_frame_count = 0;
        static uint32_t mti_parse_success = 0;
        static uint32_t mti_parse_failed = 0;
        static uint32_t mti_invalid_data = 0;
        static uint32_t last_report_time = 0;
        
        mti_frame_count++;
        
        // 診斷：定期記錄 (每5秒報告一次)
        uint32_t current_time = millis();
        if (current_time - last_report_time >= 5000 && mti_frame_count <= 10) {
            // LOGI("📞 XBUS CALLBACK #%lu: length=%u", mti_frame_count, (unsigned)length);
            // LOGI("[MTI-DIAG] 收到 frame #%lu: 長度=%zu bytes, 前4字節=0x%02X%02X%02X%02X", 
            //      mti_frame_count, length, 
            //      length >= 1 ? data[0] : 0, length >= 2 ? data[1] : 0,
            //      length >= 3 ? data[2] : 0, length >= 4 ? data[3] : 0);
            // LOGI("🔍 RAW DATA: length=%u, data[0-4]=0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", 
            //      (unsigned)length, data[0], data[1], data[2], data[3], length > 4 ? data[4] : 0);
            last_report_time = current_time;
        }
        
        // 解析 XBUS 數據為 IMUData
        xbus_parser_checker.recordIn();  // TDD: 記錄 XBUS 數據流入
        
        // 修正數據偏移 - 尋找正確的 XBUS frame 開始位置
        const uint8_t* frame_start = data;
        size_t frame_length = length;
        
        // 檢查是否需要偏移修正
        bool offset_detected = (length >= 4 && data[0] != 0xFA && data[1] == 0xFA && data[2] == 0xFF);
        
        // 只在前10個frame或每5秒報告一次詳細診斷
        if ((mti_frame_count <= 10) || (current_time - last_report_time >= 5000)) {
        }
        
        if (offset_detected) {
            if ((mti_frame_count <= 10) || (current_time - last_report_time >= 5000)) {
            }
            frame_start = data + 1;
            frame_length = length - 1;
        } else {
            if ((mti_frame_count <= 10) || (current_time - last_report_time >= 5000)) {
            }
        }
        
        if ((mti_frame_count <= 10) || (current_time - last_report_time >= 5000)) {
        }
        
        IMUData imu_data;
        
        // 臨時測試：直接檢查基本的 XBUS frame 格式 (只在前10個frame或每5秒報告一次)
        if ((mti_frame_count <= 10) || (current_time - last_report_time >= 5000)) {

            
            if (frame_start[0] == 0xFA && frame_start[1] == 0xFF && frame_start[2] == 0x36) {

            } else {
            }

        }
        
        // 調用真正的 parseXbusFrame (重用已存在的 imu_data 變數)
        bool parse_result = XbusParser::parseXbusFrame(frame_start, frame_length, imu_data);
        
        if ((mti_frame_count <= 10) || (current_time - last_report_time >= 5000)) {
            // LOGI("🔍 REAL parseXbusFrame RETURNED: %s", parse_result ? "SUCCESS" : "FAILED");
        }
        // TDD 關卡 1: XBUS 解析結果檢查
        static TDDFlowChecker parse_success_checker("CHECKPOINT:parseSuccess", "parseSuccess");
        static TDDFlowChecker validation_pass_checker("CHECKPOINT:validationPass", "validationPass");
        static TDDFlowChecker dfi_ready_checker("CHECKPOINT:dfiReady", "dfiReady");
        
        if (parse_result) {
            parse_success_checker.recordIn();
            parse_success_checker.recordOut();  // TDD 關卡 1: 解析成功
            
            xbus_parser_checker.recordOut();  // TDD: 記錄解析成功流出
            mti_parse_success++;
            
            // TDD 關卡 2: 數據有效性檢查（簡化版）
            bool has_accel = (imu_data.flags & IMU_ACCEL_VALID) != 0;
            bool has_quat = (imu_data.flags & IMU_QUATERNION_VALID) != 0;
            bool has_euler = (imu_data.flags & IMU_EULER_VALID) != 0;
            bool has_fresh = (imu_data.flags & IMU_DATA_FRESH) != 0;
            bool is_valid = has_fresh && (imu_data.flags != 0);  // 只要有 FRESH 標誌就算有效
            
            // 移除過多的IMU驗證LOG，避免系統死機
            
            if (is_valid) {
                validation_pass_checker.recordIn();
                validation_pass_checker.recordOut();  // TDD 關卡 2: 驗證通過
                
                // TDD 關卡 3: DFI 可用性檢查
                if (dfi_ != nullptr) {
                    dfi_ready_checker.recordIn();
                    dfi_ready_checker.recordOut();  // TDD 關卡 3: DFI 準備就緒
                    
                    static uint32_t imu_push = 0;
                    imu_push++;
                
                    
                    data_flow_integrator_checker.recordIn();  // TDD: 記錄 DataFlowIntegrator 流入
                    dfi_->updateIMU(imu_data);
                    data_flow_integrator_checker.recordOut();  // TDD: 記錄 DataFlowIntegrator 處理完成

                } else {
                    LOG_ERROR("SYS_CTRL", "❌ [關卡3] DFI 為 nullptr!");
                }
            } else {
                mti_invalid_data++;
                LOG_WARN("❌ [關卡2] XBUS 數據驗證失敗 (flags=0x%04X)", imu_data.flags);
                system_health_.warning_count++;
            }
        } else {
            LOG_WARN("❌ [關卡1] XBUS 解析失敗");
        }
        
        // TDD 關卡狀態更新
        parse_success_checker.update();
        validation_pass_checker.update();
        dfi_ready_checker.update();
        
        // 定期輸出 MTI 診斷報告 (每5秒)
        current_time = millis();
        if (current_time - last_report_time >= 5000) {

            last_report_time = current_time;
        }
    });
    
    ingress_manager_->setNmeaFrameCallback([this](const uint8_t* data, size_t length) {
        if (!data_flow_enabled_) return;
        
        // 檢查是否為PLSHD句子並直接解析valid_flag
        bool is_plshd = (length > 6 && strncmp((char*)data, "$PLSHD", 6) == 0);
        if (is_plshd) {
            // 直接解析PLSHD句子的valid_flag (第1個逗號後的值)
            char* sentence = (char*)data;
            char* first_comma = strchr(sentence, ',');
            if (first_comma && first_comma[1] != '\0') {
                int valid_flag = atoi(first_comma + 1);
                bool is_valid = (valid_flag == 1 || valid_flag == 9);
                
                // 如果valid_flag=1，還需要解析heading值
                float heading_deg = 0.0f;
                if (is_valid) {
                    // 找到第5個逗號後的heading值
                    char* comma = first_comma;
                    for (int i = 0; i < 4 && comma; i++) {
                        comma = strchr(comma + 1, ',');
                    }
                    if (comma && comma[1] != '\0') {
                        heading_deg = atof(comma + 1);
                    }
                }
                
                dfi_->updatePLSHDStatus(is_valid, heading_deg);
            }
        }
        
        // 解析 NMEA 數據為 GNSSData - 🔧 重要修復：清零初始化
        GNSSData gnss_data = {};  // 清零初始化，防止未初始化的 fix_type=255
        if (NmeaParser::parseNmeaSentence(data, length, gnss_data)) {
            if (NmeaParser::isValidNmeaData(gnss_data)) {
                static uint32_t gnss_push = 0;
                static uint32_t last_gnss_report = 0;
                gnss_push++;
                dfi_->updateGNSS(gnss_data);
                
                // 只在前10次或每5秒報告一次
                uint32_t current_time = millis();
                
            } else {
                // 🔍 死機診斷：詳細記錄無效數據的原因
                static uint32_t invalid_count = 0;
                invalid_count++;
                
                if (invalid_count % 50 == 1) {  // 每50次記錄一次詳細信息
                    LOG_WARN("NMEA 數據無效 #%lu: fix_type=%d, flags=0x%02X, sentence=%s", 
                             invalid_count, gnss_data.fix_type, gnss_data.flags, 
                             gnss_data.nmea_sentence);
                }
                
                // 🚨 防止 warning_count 無限累加導致系統問題
                if (system_health_.warning_count < 1000) {
                    system_health_.warning_count++;
                }
            }
        } else {
            // NMEA 解析失敗不一定是錯誤，可能只是不支援的句子類型
            LOG_DEBUG("NMEA 句子跳過或解析失敗");
        }
    });
    
    // 設置數據流監控系統已移除
    // setupDataFlowMonitoring();
    
    // 診斷：確認回調是否設置成功
    LOGI("✅ 診斷檢查:");
    LOGI("  - IngressManager已初始化: %s", ingress_manager_->isInitialized() ? "YES" : "NO");
    LOGI("  - DFI已創建: %s", dfi_ ? "YES" : "NO");
    LOGI("  - XBUS回調已設置: YES (lambda function)");
    LOGI("  - NMEA回調已設置: YES (lambda function)");
    LOGI("  - IMU更新回調已設置: YES (static function)");
    LOGI("  - GNSS更新回調已設置: YES (static function)");
    
    LOG_INFO("數據流設置完成");
    return true;
}

void SystemController::processingLoop() {
    uint64_t current_time = getCurrentTimeUs();
    
    // 1) 處理輸入數據
    if (ingress_manager_ && data_flow_enabled_) {
        static uint32_t process_count = 0;
        static uint32_t last_process_report = 0;
        process_count++;
        uint32_t now = millis();
        
        ingress_manager_->process();
    } else {
        static uint32_t blocked_count = 0;
        blocked_count++;
        if (blocked_count % 1000 == 1) {
            LOGI("🚫 [INGRESS-BLOCKED] #%lu - ingress_manager_=%p, data_flow_enabled_=%s", 
                 blocked_count, ingress_manager_, data_flow_enabled_ ? "true" : "false");
        }
    }
    
    // 2) 處理 DFI - 移除 tick 保險機制
    // DFI 由 updateIMU/updateGNSS 觸發，不需要額外的 tick 保險
    // if (dfi_ && data_flow_enabled_) {
    //     dfi_->tick(current_time);
    // }
    
    // 3) Phase 3: 切換到真實感測器數據
    // 移除測試數據注入，使用真實的 IMU(Serial2) + GNSS(Serial4) 數據
    // 真實數據透過以下路徑流動：
    // Serial2/4 → hal::processAllSensorData() → IngressManager → XbusParser/NmeaParser 
    // → DFI → NavigationCallback → ProtocolManager
    
    // 備註：protocol_manager_->continuousUpdate() 現在由 DFI 回調觸發
    // 不再需要手動注入測試數據
    
    if (protocol_manager_ && data_flow_enabled_) {
        // 🚫 停用 TDD 頻率監控，防止累加導致死機
        // TDD_FREQ("SystemController::processingLoop", 3000);
        
        // 真實數據處理已由上面的 ingress_manager_->process() 觸發
        // LOGD("✅ Phase 3: 使用真實感測器數據 - 已移除測試數據注入");
    }
    
    // 移除：MTI校正完成後的偵測邏輯已移至setup階段
    
    stats_.total_cycles++;
}

// 移除舊的 handleNavigationData - Push→Pull 解耦後不再需要
// 數據流現在通過 IMU/GNSS 回調直接觸發協議更新

void SystemController::processState() {
    switch (current_state_) {
        case SystemState::INIT:
            // 初始化狀態已在 initialize() 中處理
            break;
            
        case SystemState::CALIBRATING:
            // TODO: 實作校準邏輯
            current_state_ = SystemState::RUNNING;
            break;
            
        case SystemState::RUNNING:
            // 正常運行，無需特殊處理
            break;
            
        case SystemState::ERROR:
            // 錯誤狀態不再觸發恢復 - 系統繼續運行避免數據中斷
            break;
            
        case SystemState::SAFE_MODE:
            // TODO: 實作安全模式邏輯
            break;
            
        case SystemState::SHUTDOWN:
            // 關機狀態無需處理
            break;
    }
}

void SystemController::updateHealth() {
    updateComponentHealth();
    
    // 整體通訊狀態
    system_health_.communication_ok = system_health_.protocol_ok && 
                                     checkIngressHealth();
    
    // 融合狀態
    system_health_.fusion_ok = system_health_.dfi_ok;
}

void SystemController::updateComponentHealth() {
    system_health_.dfi_ok = checkDFIHealth();
    system_health_.protocol_ok = checkProtocolHealth();
}

bool SystemController::checkDFIHealth() {
    if (!dfi_) return false;
    
    return dfi_->hasValidNavigation();
}

bool SystemController::checkProtocolHealth() {
    if (!protocol_manager_) return false;
    
    return true; // 簡化後協議管理器總是運行正常
}

bool SystemController::checkIngressHealth() {
    if (!ingress_manager_) return false;
    
    return ingress_manager_->isInitialized();
}

// 錯誤恢復系統已移除 - 避免不必要的數據重置導致 Valley 問題
// Push→Pull 架構本身已經足夠穩定，不需要複雜的錯誤恢復機制

void SystemController::updateStats() {
    // 統計更新邏輯已在各個處理函數中實作
}

bool SystemController::checkTransitionConditions(SystemState target_state) {
    // TODO: 實作狀態轉換條件檢查
    return true;
}



// 靜態回調函數
// Push→Pull 解耦回調函數
void SystemController::imuUpdateCallback(const IMUData& imu_data, const NavigationState& nav_state) {
    if (instance_ && instance_->protocol_manager_) {
        // IMU更新時傳遞實際的IMU數據，GNSS使用空值
        static uint32_t callback_count = 0;
        static uint32_t last_report_time = 0;
        callback_count++;
        uint32_t current_time = millis();
        
        // 測試用：暫時禁用詳細IMU數據顯示以減少日誌噪音
        // 每5秒或前10次顯示詳細的真實IMU數據
        /*if (callback_count <= 10 || (current_time - last_report_time >= 5000)) {
            LOGI("🔗 [REAL-IMU-DATA] Callback #%lu", callback_count);
            LOGI("  📊 IMU: accel(%.3f,%.3f,%.3f) gyro(%.3f,%.3f,%.3f)", 
                 imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                 imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
            LOGI("  🧭 Quat: w=%.4f x=%.4f y=%.4f z=%.4f flags=0x%04X", 
                 imu_data.quat_w, imu_data.quat_x, imu_data.quat_y, imu_data.quat_z, imu_data.flags);
            LOGI("  📍 NavState: accel_NED(%.3f,%.3f,%.3f) angvel(%.3f,%.3f,%.3f)", 
                 nav_state.acceleration_north, nav_state.acceleration_east, nav_state.acceleration_down,
                 nav_state.angular_velocity_x, nav_state.angular_velocity_y, nav_state.angular_velocity_z);
            if (current_time - last_report_time >= 5000) {
                last_report_time = current_time;
            }
        }*/
        
        // 使用數據快照機制更新IMU數據
        instance_->protocol_manager_->updateIMUSnapshot(imu_data, nav_state);
    } else {
        static uint32_t error_count = 0;
        error_count++;
        if (error_count % 100 == 1) {  // 錯誤情況記錄更頻繁
            LOGE("❌ imuUpdateCallback failed: instance_=%p, protocol_manager_=%p", 
                 instance_, instance_ ? instance_->protocol_manager_ : nullptr);
        }
    }
}

void SystemController::gnssUpdateCallback(const GNSSData& gnss_data, const NavigationState& nav_state) {
    if (instance_ && instance_->protocol_manager_) {
        static uint32_t gnss_callback_count = 0;
        static uint32_t last_gnss_report_time = 0;
        gnss_callback_count++;
        uint32_t current_time = millis();
        
        
        // 使用數據快照機制更新GNSS數據（會自動觸發MAVLink發送）
        instance_->protocol_manager_->updateGNSSSnapshot(gnss_data, nav_state);
    }
}

uint64_t SystemController::getCurrentTimeUs() const {
    return micros(); // Arduino 函數
}

void SystemController::waitForMTICalibrationAndDetectPixhawk() {
    LOG_INFO("🎯 等待MTI校正完成後執行CUBE/Pixhawk自動偵測...");
    
    uint32_t start_time = millis();
    const uint32_t MAX_WAIT_TIME_MS = 10000; // 最多等待10秒
    const uint32_t MIN_STABLE_TIME_MS = 3000; // 至少等待3秒讓MTI穩定
    
    bool mti_ready = false;
    
    // 等待MTI校正完成或超時
    while (!mti_ready && (millis() - start_time) < MAX_WAIT_TIME_MS) {
        // 處理ingress manager來獲得MTI數據
        if (ingress_manager_ && data_flow_enabled_) {
            ingress_manager_->process();
        }
        
        // 檢查MTI是否已準備就緒
        if (dfi_ && (millis() - start_time) >= MIN_STABLE_TIME_MS) {
            const NavigationState& nav = dfi_->getNavigationState();
            mti_ready = dfi_->hasValidNavigation() && (nav.flags & NAV_ATTITUDE_VALID);
        }
        
        // 每秒報告一次等待狀態
        static uint32_t last_report = 0;
        uint32_t current_time = millis();
        if (current_time - last_report >= 1000) {
            uint32_t elapsed = (current_time - start_time) / 1000;
            LOG_INFO("⏳ 等待MTI校正... %lu秒 (MTI準備就緒: %s)", 
                     elapsed, mti_ready ? "是" : "否");
            last_report = current_time;
        }
        
        delay(100); // 避免過度占用CPU
    }
    
    // 執行CUBE/Pixhawk偵測
    if (mti_ready) {
        LOG_INFO("✅ MTI校正完成，開始執行CUBE/Pixhawk自動偵測");
    } else {
        LOG_INFO("⏰ 等待超時，開始執行CUBE/Pixhawk自動偵測 (MTI可能尚未完全校正)");
    }
    
    bool detection_result = detectPixhawkAndSetMode();
    pixhawk_detected_ = detection_result;
    
    if (detection_result) {
        LOG_INFO("🚀 CUBE/Pixhawk偵測成功，已自動切換至MAVLink模式");
    } else {
        LOG_INFO("🎯 未偵測到CUBE/Pixhawk，保持AR1AFC模式");
    }
}

bool SystemController::detectPixhawkAndSetMode() {
    LOG_INFO("🔍 開始偵測CUBE/Pixhawk...");
    LOG_INFO("🔧 檢查Serial1狀態...");
    LOG_INFO("📡 等待MAVLink HEARTBEAT訊號...");
    
    const uint32_t DETECTION_TIMEOUT_MS = 3000;  // 3秒超時 (簡化)
    const uint32_t CHECK_INTERVAL_MS = 50;       // 每50ms檢查一次 (更快)
    
    uint32_t start_time = millis();
    bool pixhawk_detected = false;
    int check_count = 0;
    
    // 監聽MAVLink HEARTBEAT消息
    while ((millis() - start_time) < DETECTION_TIMEOUT_MS && !pixhawk_detected) {
        // 處理接收的資料
        if (ingress_manager_) {
            ingress_manager_->process();
        }
        
        // 檢查是否收到MAVLink HEARTBEAT
        // 這裡需要檢查是否有MAVLink資料到達
        // 簡化版本：檢查Serial1是否有特定的MAVLink pattern
        
        // 檢查Serial1 (PX4/CUBE/Pixhawk MAVLink接口)
        int available_bytes = hal::Serial1.available();
        if (available_bytes > 0) {
            LOG_INFO("📡 Serial1有資料：%d bytes", available_bytes);
            uint8_t buffer[64];
            int bytes_read = 0;
            
            // 讀取可用的資料
            while (hal::Serial1.available() > 0 && bytes_read < sizeof(buffer)) {
                buffer[bytes_read++] = hal::Serial1.read();
            }
            
            // Debug: 顯示收到的原始資料
            Serial.print("📊 收到資料: ");
            for (int i = 0; i < bytes_read; i++) {
                Serial.print("0x");
                if (buffer[i] < 0x10) Serial.print("0");
                Serial.print(buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            
            // 檢查MAVLink magic byte (0xFE for MAVLink v1, 0xFD for MAVLink v2)
            for (int i = 0; i < bytes_read; i++) {
                if (buffer[i] == 0xFE) {
                    pixhawk_detected = true;
                    LOG_INFO("✅ 偵測到MAVLink v1封包 (magic byte: 0x%02X)", buffer[i]);
                    break;
                } else if (buffer[i] == 0xFD) {
                    pixhawk_detected = true;
                    LOG_INFO("✅ 偵測到MAVLink v2封包 (magic byte: 0x%02X)", buffer[i]);
                    break;
                }
            }
        }
        
        if (!pixhawk_detected) {
            check_count++;
            if (check_count % 20 == 1) {  // 每秒報告一次 (50ms * 20 = 1000ms)
                LOG_INFO("⏰ 偵測中... %d秒 (Serial1 available: %d bytes)", 
                        check_count / 20 + 1, hal::Serial1.available());
            }
            delay(CHECK_INTERVAL_MS);
        }
    }
    
    // 檢測完成，顯示結果
    if (!pixhawk_detected) {
        LOG_INFO("⏰ CUBE/Pixhawk檢測超時 (%dms)，未偵測到MAVLink信號", DETECTION_TIMEOUT_MS);
    }
    
    // 根據偵測結果設定協議模式
    if (pixhawk_detected) {
        if (protocol_manager_->switchToMAVLink()) {
            LOG_INFO("🚀 已切換至MAVLink模式");
            return true;
        } else {
            LOG_ERR("❌ MAVLink模式切換失敗，使用AR1AFC模式");
        }
    }
    
    // 預設使用AR1AFC模式
    if (protocol_manager_->switchToAR1AFC()) {
        LOG_INFO("🎯 已設定為AR1AFC模式");
    } else {
        LOG_ERR("❌ AR1AFC模式設定失敗");
    }
    
    return pixhawk_detected;
}