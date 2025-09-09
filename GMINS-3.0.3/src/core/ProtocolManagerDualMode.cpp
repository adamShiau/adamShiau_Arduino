#include "ProtocolManagerDualMode.h"
#include "../core/system_health.h"

ProtocolManagerDualMode::ProtocolManagerDualMode(TxMultiplexer* tx_mux) 
    : tx_mux_(tx_mux), 
      current_protocol_(Protocol::NONE),
      ar1afc_protocol_(nullptr),
      mavlink_protocol_(nullptr),
      active_protocol_(nullptr) {
    
    // 初始化數據快照
    memset(&latest_gnss_data_, 0, sizeof(latest_gnss_data_));
    memset(&latest_imu_data_, 0, sizeof(latest_imu_data_));
    memset(&latest_nav_state_, 0, sizeof(latest_nav_state_));
    latest_gnss_data_.flags = 0;
    
    // 預創建兩個協議實例（避免動態分配）
    ar1afc_protocol_ = new AR1AFCProtocol();
    mavlink_protocol_ = new MAVLinkProtocol();
    
    // 🔄 切換到AR1AFC模式 - 預設AR1AFC協議
    switchToProtocol(Protocol::AR1AFC);
}

ProtocolManagerDualMode::~ProtocolManagerDualMode() {
    if (ar1afc_protocol_) {
        ar1afc_protocol_->cleanup();
        delete ar1afc_protocol_;
    }
    if (mavlink_protocol_) {
        mavlink_protocol_->cleanup();
        delete mavlink_protocol_;
    }
}

void ProtocolManagerDualMode::continuousUpdate(const GNSSData& gnss_data, const NavigationState& nav_state, const IMUData& imu_data) {
    // 移除：調用頻率監控（與實際執行重複）
    
    // 🔍 頻率控制：限制在100Hz
    static uint32_t last_update = 0;
    uint32_t now = millis();
    if (now - last_update < 10) {
        return;
    }
    last_update = now;
    
    // 🚫 停用 TDD 頻率監控，防止累加導致死機
    // TDD_FREQ("ProtocolManager::continuousUpdate 實際執行", 3000);
    
    // 測試空的MAVLink case（Transport日誌已關閉）
    if (active_protocol_) {
        switch (current_protocol_) {
            case Protocol::AR1AFC:
                active_protocol_->sendNavigationData(latest_nav_state_);
                break;
                
            case Protocol::MAVLINK:
                active_protocol_->onIMUUpdate(latest_imu_data_, latest_nav_state_);
                
                // 始終發送MAVLink數據，確保YAW能輸出
                active_protocol_->sendGNSSData(latest_gnss_data_);
                break;
        }
    }
}

bool ProtocolManagerDualMode::switchToAR1AFC() {
    return switchToProtocol(Protocol::AR1AFC);
}

bool ProtocolManagerDualMode::switchToMAVLink() {
    return switchToProtocol(Protocol::MAVLINK);
}

bool ProtocolManagerDualMode::setProtocolMode(const char* protocol_name) {
    if (strcmp(protocol_name, "AR1AFC") == 0) {
        return switchToAR1AFC();
    } else if (strcmp(protocol_name, "MAVLINK") == 0) {
        return switchToMAVLink();
    }
    return false;
}

const char* ProtocolManagerDualMode::getCurrentProtocolName() const {
    switch (current_protocol_) {
        case Protocol::AR1AFC: return "AR1AFC";
        case Protocol::MAVLINK: return "MAVLINK";
        case Protocol::NONE: return "NONE";
        default: return "UNKNOWN";
    }
}

bool ProtocolManagerDualMode::isReady() const {
    return active_protocol_ && active_protocol_->isReady();
}

bool ProtocolManagerDualMode::switchToProtocol(Protocol target_protocol) {
    // 簡化檢查：已是目標協議就直接返回
    if (current_protocol_ == target_protocol && active_protocol_ != nullptr) {
        return true;
    }
    
    // 簡化協議切換
    IProtocol* target = nullptr;
    const char* protocol_name = nullptr;
    uint32_t target_baud_rate = 115200;
    
    if (target_protocol == Protocol::AR1AFC) {
        target = ar1afc_protocol_;
        protocol_name = "AR1AFC";
        target_baud_rate = 230400;
    } else if (target_protocol == Protocol::MAVLINK) {
        target = mavlink_protocol_;
        protocol_name = "MAVLINK";
        target_baud_rate = 460800;
    }
    
    if (!target || !tx_mux_) {
        return false;
    }
    
    // 切換傳輸控制權
    const char* actual_owner = tx_mux_->getCurrentOwner();
    const char* old_owner = actual_owner ? actual_owner : "NONE";
    ITransport* transport = tx_mux_->switchOwner(old_owner, protocol_name);
    
    if (!transport || !transport->setBaudRate(target_baud_rate)) {
        return false;
    }
    
    // 初始化協議
    if (target->initialize(transport)) {
        current_protocol_ = target_protocol;
        active_protocol_ = target;
        return true;
    }
    
    return false;
}

void ProtocolManagerDualMode::updateGNSSSnapshot(const GNSSData& gnss_data, const NavigationState& nav_state) {
    static bool processing_gnss = false;
    if (processing_gnss) {
        return;
    }
    processing_gnss = true;
    
    // 放寬檢查：只要有任何有用數據就更新
    bool has_useful_data = (gnss_data.latitude != 0.0 || gnss_data.longitude != 0.0) ||
                           (gnss_data.satellites_visible > 0);
    
    if (has_useful_data) {
        latest_gnss_data_ = gnss_data;
        latest_nav_state_ = nav_state;
    }
    
    processing_gnss = false;
}

void ProtocolManagerDualMode::updateIMUSnapshot(const IMUData& imu_data, const NavigationState& nav_state) {
    latest_imu_data_ = imu_data;
    latest_nav_state_ = nav_state;
    
    // IMU更新時不主動發送，等待GNSS觸發
}