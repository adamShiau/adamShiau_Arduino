// 關閉MAVLink協議的LOG_INFO輸出以提升性能 - 必須在所有include之前定義
#define LOG_LEVEL_LOCAL LOG_ERROR  // 只顯示ERROR及以上等級

#include "MAVLinkProtocol.h"
#include <cstring>  // for memset

/**
 * MAVLinkProtocol 構造函數實現 - 加入 TDD 輸出頻率監控
 * 
 * 加入TDD監控實際MAVLink封包輸出頻率
 */
MAVLinkProtocol::MAVLinkProtocol() 
    : sequence_number_(0), 
      odometry_first_time_ms_(0), 
      gps_first_time_ms_(0),
      imu_update_time_(0), 
      gnss_update_time_(0),
      // TDD輸出頻率監控初始化
      tdd_mavlink_output_count_(0),
      tdd_odometry_output_count_(0),
      tdd_gps_output_count_(0),
      tdd_last_output_log_time_(0) {
      
    // 初始化數據快照
    memset(&snap_nav_, 0, sizeof(snap_nav_));
    memset(&snap_imu_, 0, sizeof(snap_imu_));
    memset(&snap_gnss_, 0, sizeof(snap_gnss_));
}

/**
 * Push→Pull 解耦：實現新介面 - 暫時移除TDD避免死機
 */
void MAVLinkProtocol::onIMUUpdate(const IMUData& imu_data, const NavigationState& nav_state) {
    // ✅ 簡化版：不發送IMU相關訊息，符合只要GPS_INPUT的需求
    snap_imu_ = imu_data;
    snap_nav_ = nav_state;
   
}

/**
 * TDD輸出頻率監控函數 - 簡化版（移除millis依賴）
 */
void MAVLinkProtocol::tddTrackMAVLinkOutput(const char* packet_type) {
    // ✅ 簡化版：不進行頻率監控，減少Arduino函數依賴
    // 直接返回，避免millis()依賴問題
}

/**
 * 實現NavigationData發送 - 在.cpp中加入TDD監控
 */
bool MAVLinkProtocol::sendNavigationData(const NavigationState& nav_data) {
    // ✅ 簡化版：不發送Navigation數據，只保留GPS_INPUT
    return true;  // 直接返回成功，不發送
}

/**
 * 實現GNSS數據發送 - 在.cpp中加入TDD監控  
 */
bool MAVLinkProtocol::sendGNSSData(const GNSSData& gnss_data) {
    // TDD頻率監控：MAVLink sendGNSSData被調用頻率
    // 移除：過度細節的頻率監控
    
    bool result = sendGNSSBasedData(gnss_data);
    return result;
}

/**
 * sendGNSSBasedData 實現 - 從.h移動到.cpp
 */
bool MAVLinkProtocol::sendGNSSBasedData(const GNSSData& gnss_data) {
    // 🔧 測試階段第三步：測試 TDD 頻率監控是否有問題
    // 移除：重複的封包發送監控
    
    // 基本檢查
    if (!transport_ || !transport_->isReady()) {
        return false;
    }
    
    // 使用傳入的 gnss_data 參數
    const GNSSData& data = gnss_data;
    
    
    // MAVLink 封包編碼
    mavlink_message_t msg;
    mavlink_gps_input_t gps_input = {};
    
    // 基本資料設定
    gps_input.time_usec = (uint64_t)millis() * 1000;
    gps_input.gps_id = 0;
    gps_input.fix_type = 3;  // 強制設為3D定位 (GPS_FIX_TYPE_3D_FIX)
    
    // 位置資料
    gps_input.lat = (int32_t)(data.latitude * 1E7);
    gps_input.lon = (int32_t)(data.longitude * 1E7);
    gps_input.alt = data.altitude_msl;
    
    // 速度資料
    gps_input.vn = data.velocity_north;
    gps_input.ve = data.velocity_east;
    gps_input.vd = data.velocity_down;
    
    // 精度資料 - 使用合理預設值確保EKF信任GPS數據
    gps_input.hdop = (data.hdop > 0) ? data.hdop : 1.0f;                    // 1.0米HDOP精度
    gps_input.vdop = (data.vdop > 0) ? data.vdop : 1.5f;                    // 1.5米VDOP精度
    gps_input.satellites_visible = (data.satellites_visible > 0) ? data.satellites_visible : 8; // 8顆衛星
    gps_input.speed_accuracy = (data.speed_accuracy > 0) ? data.speed_accuracy : 0.5f;     // 0.5m/s速度精度
    gps_input.horiz_accuracy = (data.horizontal_accuracy > 0) ? data.horizontal_accuracy : 1.0f; // 1.0米水平精度
    gps_input.vert_accuracy = (data.vertical_accuracy > 0) ? data.vertical_accuracy : 1.5f;     // 1.5米垂直精度
    
    // 🧭 YAW 始終有效，不允許被忽略 - 使用與 AR1AFC 相同的數據源
    float yaw_deg = snap_nav_.euler_yaw;  // 使用 NavigationState (包含 90° 校正)
    while (yaw_deg < 0) yaw_deg += 360.0f;
    while (yaw_deg >= 360) yaw_deg -= 360.0f;
    gps_input.yaw = (uint16_t)(yaw_deg * 100);  // 度×100
    
    // 注意：GPS_INPUT結構體中沒有yaw_accuracy欄位，EKF會根據其他精度參數判斷信任度
    
    // 🔑 強制設定：YAW 始終有效，不忽略任何欄位
    gps_input.ignore_flags = 0;
    
    // 封裝並傳送（無LOG）
    mavlink_msg_gps_input_encode(SYSTEM_ID, COMPONENT_ID, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    if (len == 0) {
        return false;
    }
    
    size_t sent = transport_->write(buffer, len);
    return (sent == len);
    
    /* 原始功能代碼 - 測試階段暫時關閉
    // TDD頻率監控：實際GPS_INPUT封包發送頻率
    // 移除：重複的封包發送監控
    
    if (!transport_) {
        LOG_INFO("❌ transport_ 為 null");
        return false;
    }
    
    if (!transport_->isReady()) {
        LOG_INFO("❌ transport_ 未準備好");
        return false;
    }
    
    LOG_INFO("✅ 傳輸層檢查通過");
    
    // 使用傳入的 gnss_data 參數（由上層 ProtocolManager 提供測試資料）
    const GNSSData& data = gnss_data;
    
    // 完全採用舊版本實現 - 直接複製 ArduinoPixhawk 邏輯
    mavlink_message_t msg;
    mavlink_gps_input_t gps_input = {};
    
    // 基本資料設定 (使用固定測試資料)
    gps_input.time_usec = (uint64_t)millis() * 1000;
    gps_input.gps_id = 0;
    gps_input.fix_type = 3;  // 強制設為3D定位 (GPS_FIX_TYPE_3D_FIX)
    
    // 位置資料 (lat/lon: 1E7, alt: float)
    gps_input.lat = (int32_t)(data.latitude * 1E7);
    gps_input.lon = (int32_t)(data.longitude * 1E7);
    gps_input.alt = data.altitude_msl;
    
    // 速度資料 (使用測試資料的速度)
    gps_input.vn = data.velocity_north;
    gps_input.ve = data.velocity_east;
    gps_input.vd = data.velocity_down;
    
    // 精度資料 - 使用合理預設值確保EKF信任GPS數據
    gps_input.hdop = (data.hdop > 0) ? data.hdop : 1.0f;                    // 1.0米HDOP精度
    gps_input.vdop = (data.vdop > 0) ? data.vdop : 1.5f;                    // 1.5米VDOP精度
    gps_input.satellites_visible = (data.satellites_visible > 0) ? data.satellites_visible : 8; // 8顆衛星
    
    // Accuracy 精度設定 - 使用合理預設值
    gps_input.speed_accuracy = (data.speed_accuracy > 0) ? data.speed_accuracy : 0.5f;     // 0.5m/s速度精度
    gps_input.horiz_accuracy = (data.horizontal_accuracy > 0) ? data.horizontal_accuracy : 1.0f; // 1.0米水平精度
    gps_input.vert_accuracy = (data.vertical_accuracy > 0) ? data.vertical_accuracy : 1.5f;     // 1.5米垂直精度
    
    // 注意：GPS_INPUT結構體中沒有yaw_accuracy欄位
    gps_input.ignore_flags = 0;
    
    gps_input.yaw = 0;  // 暫時不使用 YAW
    
    // 封裝並傳送 (完全照抄舊版本)
    LOG_INFO("📦 開始MAVLink封包編碼");
    mavlink_msg_gps_input_encode(SYSTEM_ID, COMPONENT_ID, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    if (len == 0) {
        LOG_INFO("❌ MAVLink編碼失敗");
        return false;  // 編碼失敗
    }
    
    LOG_INFO("📡 發送GPS_INPUT封包: %d bytes", len);
    
    // 顯示要發送的原始hex數據 - 使用TDD頻率控制
    static uint32_t hex_last_print = 0;
    uint32_t hex_now = millis();
    if (hex_now - hex_last_print >= 3000) {  // 3秒輸出一次
        hex_last_print = hex_now;
        char hex_buffer[100] = "🔍 發送HEX: ";
        char temp[4];
        for (int i = 0; i < len && i < 32; i++) {
            sprintf(temp, "%02X ", buffer[i]);
            strcat(hex_buffer, temp);
        }
        TDD_PRINT("%s", hex_buffer);
    }
    
    size_t sent = transport_->write(buffer, len);
    LOG_INFO("📊 實際發送: %u bytes", (unsigned int)sent);
    
    // 解析並顯示發送的封包內容
    LOG_INFO("🔍 解析條件: sent=%u, len=%d, 條件=%s", 
             (unsigned int)sent, len, (sent == len && len > 0) ? "通過" : "失敗");
    
    if (sent == len && len > 0) {
        LOG_INFO("📋 調用 parseMAVLinkPacket");
        parseMAVLinkPacket(buffer, len);
    } else {
        LOG_INFO("❌ 解析條件失敗，跳過解析");
    }
    
    return (sent == len);
    */
}

/**
 * 測試數據發送函數 - 從.h移動到.cpp
 */
bool MAVLinkProtocol::sendTestGPSInput() {
    // ❌ 移除固定測試數據 - 統一使用 ProtocolManagerDualMode 提供的數據
    // 此函數現在直接使用 sendGNSSBasedData() 和最新的 snap_gnss_ 快照
    return sendGNSSBasedData(snap_gnss_);
}

/**
 * onGNSSUpdate 實現 - 從.h移動到.cpp
 */
void MAVLinkProtocol::onGNSSUpdate(const GNSSData& gnss_data, const NavigationState& nav_state) {
    // 🔧 固定頻率輸出：移除條件檢查，確保穩定 100Hz
    snap_gnss_ = gnss_data;
    
    // ✅ 無條件發送 GPS_INPUT，實現固定頻率輸出
    if (transport_ && transport_->isReady()) {
        sendGNSSBasedData(gnss_data);
    }
}

/**
 * MAVLink封包解析函數實現
 */
void MAVLinkProtocol::parseMAVLinkPacket(const uint8_t* buffer, uint16_t len) {
    // 🔧 測試階段：暫時關閉所有MAVLink封包解析功能
    // 直接返回，不進行任何解析處理
    return;
    
    /* 原始解析功能代碼 - 測試階段暫時關閉
    LOG_INFO("🔍 parseMAVLinkPacket 開始: len=%d", len);
    
    if (len < 8) {
        LOG_INFO("❌ 封包太短: %d < 8", len);
        return;
    }
    
    LOG_INFO("✅ 封包長度檢查通過");
    
    uint8_t stx = buffer[0];
    uint8_t payload_len = buffer[1];
    uint8_t seq, sysid, compid;
    uint32_t msgid;
    
    LOG_INFO("🔍 STX檢查: 0x%02X", stx);
    
    if (stx == 0xFE) {
        LOG_INFO("✅ MAVLink v1 封包");
        seq = buffer[2];
        sysid = buffer[3];
        compid = buffer[4];
        msgid = buffer[5];
    } else if (stx == 0xFD) {
        LOG_INFO("✅ MAVLink v2 封包");
        seq = buffer[4];
        sysid = buffer[5];
        compid = buffer[6];
        msgid = buffer[7] | (buffer[8] << 8) | (buffer[9] << 16);
    } else {
        LOG_INFO("❌ 無效STX: 0x%02X", stx);
        return;
    }
    
    LOG_INFO("📋 封包信息: seq=%d, sysid=%d, compid=%d, msgid=%lu", seq, sysid, compid, (unsigned long)msgid);
    
    if (msgid == 232) {  // GPS_INPUT
        LOG_INFO("🎯 進入GPS_INPUT解析分支");
        const uint8_t* payload = (stx == 0xFE) ? &buffer[6] : &buffer[10];
        
        LOG_INFO("🔍 payload_len檢查: %d >= 62?", payload_len);
        
        if (payload_len >= 62) {
            LOG_INFO("✅ payload長度足夠，開始解析座標");
            int32_t lat, lon;
            memcpy(&lat, payload + 12, sizeof(lat));
            memcpy(&lon, payload + 16, sizeof(lon));
            
            LOG_INFO("📍 解析結果: lat=%ld, lon=%ld", (long)lat, (long)lon);
            
            LOG_INFO("🎉 GPS_INPUT解析完成 - 洛杉磯座標輸出");
        } else {
            LOG_INFO("❌ payload長度不足: %d < 62", payload_len);
        }
    } else {
        LOG_INFO("❌ 非GPS_INPUT訊息: msgid=%lu", (unsigned long)msgid);
    }
    */
}