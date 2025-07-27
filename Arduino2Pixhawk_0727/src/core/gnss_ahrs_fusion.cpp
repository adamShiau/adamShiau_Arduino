#include "gnss_ahrs_fusion.h"

// 全局輸出配置
OutputConfig fusionConfig;

// 全局擴展 GNSS 資料
GNSSExtendedData gnss_extended_data;

// GPS 訊息頻率控制變數
unsigned long last_gps_raw_time = 0;
unsigned long last_gps_status_time = 0;
const unsigned long GPS_RAW_INTERVAL = 100;    // 10Hz
const unsigned long GPS_STATUS_INTERVAL = 500; // 2Hz

// Fix Type 轉換函數
uint8_t convertFixType(int gga_quality) {
    switch(gga_quality) {
        case 0: return 0; // NO_GPS
        case 1: return 3; // 3D_FIX
        case 2: return 4; // DGPS
        case 3: return 2; // 2D_FIX
        case 4: return 5; // RTK_FIXED
        case 5: return 6; // RTK_FLOAT
        case 6: return 1; // ESTIMATED/DR
        default: return 1; // NO_FIX
    }
}

// 建構函數
GNSSAHRSFusion::GNSSAHRSFusion() {
    status.gnss_valid = false;
    status.mtdata2_valid = false;
    status.fusion_active = false;
    status.mavlink_sent = false;
    status.last_gnss_update = 0;
    status.last_mtdata2_update = 0;
    status.last_fusion_send = 0;
    status.gnss_quality = 0;
    status.fusion_count = 0;
    status.mavlink_send_count = 0;
}

// 設定除錯模式
void GNSSAHRSFusion::setDebugMode(bool enabled) {
    debug_enabled = enabled;
    if (debug_enabled) {
        debugPrint("[FUSION] Debug mode enabled");
    }
}

// 設定時間監控模式
void GNSSAHRSFusion::setTimeCheckMode(bool enabled) {
    time_check_enabled = enabled;
    if (time_check_enabled) {
        debugPrint("[FUSION] Time check mode enabled - Monitoring timing performance");
    }
}

// 除錯輸出函數
void GNSSAHRSFusion::debugPrint(const String& message) {
    if (debug_enabled) {
        Serial.println(message);
    }
}

// 更新 GNSS 資料
void GNSSAHRSFusion::updateGNSSData(const String& nmea_sentence) {
    if (nmea_sentence.length() < 10) return;
    
    // 時間監控 - NMEA 解析開始
    unsigned long t_nmea_start = 0;
    unsigned long t_nmea_parse = 0;
    unsigned long t_nmea_end = 0;
    
    if (time_check_enabled) {
        t_nmea_start = micros();
    }
    
    // 解析 GGA 句子
    if (nmea_sentence.startsWith("$GNGGA") || nmea_sentence.startsWith("$GPGGA")) {
        debugPrint("[FUSION] Processing GGA: " + nmea_sentence.substring(0, 20) + "...");
        
        // 簡單的 NMEA 解析
        int comma_positions[15];
        int comma_count = 0;
        
        // 找出所有逗號位置
        for (int i = 0; i < nmea_sentence.length() && comma_count < 15; i++) {
            if (nmea_sentence[i] == ',') {
                comma_positions[comma_count] = i;
                comma_count++;
            }
        }
        
        if (comma_count >= 10) {
            // 提取緯度 (欄位 2,3)
            String lat_str = nmea_sentence.substring(comma_positions[1] + 1, comma_positions[2]);
            String lat_dir = nmea_sentence.substring(comma_positions[2] + 1, comma_positions[3]);
            
            // 提取經度 (欄位 4,5)
            String lon_str = nmea_sentence.substring(comma_positions[3] + 1, comma_positions[4]);
            String lon_dir = nmea_sentence.substring(comma_positions[4] + 1, comma_positions[5]);
            
            // 提取品質 (欄位 6)
            String quality_str = nmea_sentence.substring(comma_positions[5] + 1, comma_positions[6]);
            
            // 提取衛星數量 (欄位 7)
            String sat_str = nmea_sentence.substring(comma_positions[6] + 1, comma_positions[7]);
            
            // 提取高度 (欄位 9)
            String alt_str = nmea_sentence.substring(comma_positions[8] + 1, comma_positions[9]);
            
            if (lat_str.length() > 4 && lon_str.length() > 5) {
                // 轉換緯度 (ddmm.mmmm)
                double lat_deg = lat_str.substring(0, 2).toDouble();
                double lat_min = lat_str.substring(2).toDouble();
                gnss_data.latitude = lat_deg + lat_min / 60.0;
                if (lat_dir == "S") gnss_data.latitude = -gnss_data.latitude;
                
                // 轉換經度 (dddmm.mmmm)
                double lon_deg = lon_str.substring(0, 3).toDouble();
                double lon_min = lon_str.substring(3).toDouble();
                gnss_data.longitude = lon_deg + lon_min / 60.0;
                if (lon_dir == "W") gnss_data.longitude = -gnss_data.longitude;
                
                // 高度、品質和衛星數量
                gnss_data.altitude = alt_str.toDouble();
                gnss_data.quality = quality_str.toInt();
                gnss_data.satellites = sat_str.toInt();
                
                // 同步到擴展 GNSS 資料
                gnss_extended_data.gga_quality = gnss_data.quality;
                gnss_extended_data.satellites_used = gnss_data.satellites;
                
                gnss_data.valid = (gnss_data.quality > 0);
                gnss_data.timestamp = millis();
                status.last_gnss_update = millis();
                
                debugPrint("[FUSION] GNSS GGA Updated: Lat=" + String(gnss_data.latitude, 6) + 
                          " Lon=" + String(gnss_data.longitude, 6) + 
                          " Alt=" + String(gnss_data.altitude, 1) + 
                          " Q=" + String(gnss_data.quality) + 
                          " Sats=" + String(gnss_data.satellites) + 
                          " Valid=" + String(gnss_data.valid ? "YES" : "NO"));
            } else {
                debugPrint("[FUSION] GNSS GGA: Invalid lat/lon data - Lat len=" + String(lat_str.length()) + 
                          " Lon len=" + String(lon_str.length()));
            }
        }
    }
    
    // 解析 RMC 句子
    else if (nmea_sentence.startsWith("$GNRMC") || nmea_sentence.startsWith("$GPRMC")) {
        debugPrint("[FUSION] Processing RMC: " + nmea_sentence.substring(0, 20) + "...");
        
        int comma_positions[12];
        int comma_count = 0;
        
        for (int i = 0; i < nmea_sentence.length() && comma_count < 12; i++) {
            if (nmea_sentence[i] == ',') {
                comma_positions[comma_count] = i;
                comma_count++;
            }
        }
        
        if (comma_count >= 8) {
            // 提取速度 (欄位 7) 和航向 (欄位 8)
            String speed_str = nmea_sentence.substring(comma_positions[6] + 1, comma_positions[7]);
            String course_str = nmea_sentence.substring(comma_positions[7] + 1, comma_positions[8]);
            
            if (speed_str.length() > 0 && course_str.length() > 0) {
                gnss_data.speed = speed_str.toDouble() * 0.514444; // knots to m/s
                gnss_data.course = course_str.toDouble();
                
                // 計算速度分量
                double course_rad = gnss_data.course * M_PI / 180.0;
                gnss_data.velocity_north = gnss_data.speed * cos(course_rad);
                gnss_data.velocity_east = gnss_data.speed * sin(course_rad);
                gnss_data.velocity_down = 0.0;
                
                debugPrint("[FUSION] GNSS RMC Updated: Speed=" + String(gnss_data.speed, 3) + 
                          " Course=" + String(gnss_data.course, 1) + 
                          " VN=" + String(gnss_data.velocity_north, 3) + 
                          " VE=" + String(gnss_data.velocity_east, 3));
            } else {
                debugPrint("[FUSION] GNSS RMC: Invalid speed/course data - Speed len=" + String(speed_str.length()) + 
                          " Course len=" + String(course_str.length()));
            }
        }
    }
    
    // 解析 GSV 句子 (衛星詳細資訊)
    else if (nmea_sentence.startsWith("$GPGSV") || nmea_sentence.startsWith("$GLGSV") || 
             nmea_sentence.startsWith("$GAGSV") || nmea_sentence.startsWith("$GBGSV") || 
             nmea_sentence.startsWith("$GNGSV")) {
        debugPrint("[FUSION] Processing GSV: " + nmea_sentence.substring(0, 25) + "...");
        parseGSVSentence(nmea_sentence);
    }
    
    if (time_check_enabled) {
        t_nmea_parse = micros();
    }
    
    updateFusionStatus();
    
    if (time_check_enabled) {
        t_nmea_end = micros();
        
        // 時間監控輸出 (每5秒一次)
        static unsigned long last_nmea_timing = 0;
        if (millis() - last_nmea_timing > 5000) {
            Serial.print("[FUSION_TIME] updateGNSSData(us): ");
            Serial.print("Parse="); Serial.print(t_nmea_parse - t_nmea_start); Serial.print(" | ");
            Serial.print("UpdateStatus="); Serial.print(t_nmea_end - t_nmea_parse); Serial.print(" | ");
            Serial.print("Total="); Serial.println(t_nmea_end - t_nmea_start);
            last_nmea_timing = millis();
        }
    }
}

// 更新 MTDATA2 資料
void GNSSAHRSFusion::updateMTData2(const my_data_4f& quat, const my_data_3f& omg, 
                                   const my_data_3f& acc, const my_data_3f& ori, uint32_t timestamp) {
    // 時間監控 - IMU 處理開始
    unsigned long t_imu_start = 0;
    unsigned long t_imu_validate = 0;
    unsigned long t_imu_end = 0;
    
    if (time_check_enabled) {
        t_imu_start = micros();
    }
    
    // 備援機制：保留上一筆有效四元數
    static float last_valid_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 單位四元數
    static bool has_valid_backup = false;
    
    // 當前四元數拷貝 (準備驗證和可能的修正)
    float current_quat[4];
    for (int i = 0; i < 4; i++) {
        current_quat[i] = quat.float_val[i];
    }
    
    // 檢查四元數有效性
    bool quat_valid = true;
    for (int i = 0; i < 4; i++) {
        if (isnan(current_quat[i]) || isinf(current_quat[i])) {
            quat_valid = false;
            break;
        }
    }
    
    if (!quat_valid) {
        static unsigned long last_nan_report = 0;
        if (millis() - last_nan_report > 500) {  // 每 500ms 報告一次 (2Hz)
            debugPrint("[FUSION] MTDATA2 Invalid quaternion (NaN/Inf) - skipping update");
            last_nan_report = millis();
        }
        return;
    }
    
    // 計算四元數模長 - 改進：提高驗證門檻
    float quat_norm = sqrt(current_quat[0]*current_quat[0] + 
                          current_quat[1]*current_quat[1] + 
                          current_quat[2]*current_quat[2] + 
                          current_quat[3]*current_quat[3]);
    
    // 改進：更嚴格的有效性驗證
    if (quat_norm < 0.5f || quat_norm > 1.5f || isnan(quat_norm)) {
        static unsigned long last_invalid_report = 0;
        if (millis() - last_invalid_report > 1000) {  // 每秒最多報告一次
            debugPrint("[FUSION] Invalid quaternion norm: " + String(quat_norm, 3) + 
                      " - using backup");
            last_invalid_report = millis();
        }
        
        // 使用備援四元數
        if (has_valid_backup) {
            memcpy(current_quat, last_valid_quaternion, sizeof(float) * 4);
            quat_norm = 1.0f;  // 備援四元數已正規化
        } else {
            // 沒有備援，跳過此次更新
            return;
        }
    } else {
        // 當前四元數有效，更新備援
        memcpy(last_valid_quaternion, current_quat, sizeof(float) * 4);
        has_valid_backup = true;
    }
    
    // 改進：使用驗證過的四元數並正規化
    for (int i = 0; i < 4; i++) {
        mtdata2_data.quaternion[i] = current_quat[i] / quat_norm;  // 正規化
    }
    
    // 複製角速度
    for (int i = 0; i < 3; i++) {
        mtdata2_data.angular_velocity[i] = omg.float_val[i];
        mtdata2_data.acceleration[i] = acc.float_val[i];
        mtdata2_data.orientation[i] = ori.float_val[i];
    }
    
    mtdata2_data.timestamp = timestamp;
    mtdata2_data.valid = true;
    status.last_mtdata2_update = millis();
    
    // 降低除錯輸出頻率
    static unsigned long last_debug_output = 0;
    if (millis() - last_debug_output > 500) { // 每 500ms 輸出一次
        debugPrint("[FUSION] MTDATA2 Updated: Q0=" + String(mtdata2_data.quaternion[0], 3) + 
                  " Q1=" + String(mtdata2_data.quaternion[1], 3) + 
                  " Q2=" + String(mtdata2_data.quaternion[2], 3) + 
                  " Q3=" + String(mtdata2_data.quaternion[3], 3) + 
                  " Norm=" + String(quat_norm, 3));
        last_debug_output = millis();
    }
    
    if (time_check_enabled) {
        t_imu_validate = micros();
    }
    
    updateFusionStatus();
    
    if (time_check_enabled) {
        t_imu_end = micros();
        
        // 時間監控輸出 (每5秒一次)
        static unsigned long last_imu_timing = 0;
        if (millis() - last_imu_timing > 5000) {
            Serial.print("[FUSION_TIME] updateMTData2(us): ");
            Serial.print("Validate="); Serial.print(t_imu_validate - t_imu_start); Serial.print(" | ");
            Serial.print("UpdateStatus="); Serial.print(t_imu_end - t_imu_validate); Serial.print(" | ");
            Serial.print("Total="); Serial.println(t_imu_end - t_imu_start);
            last_imu_timing = millis();
        }
    }
}

// 更新融合狀態
void GNSSAHRSFusion::updateFusionStatus() {
    unsigned long current_time = millis();
    
    // 改進：依據實際更新頻率調整超時閾值
    const unsigned long GNSS_TIMEOUT_MS = 750;   // 適合 2Hz GNSS (500ms + 250ms buffer)
    const unsigned long XSENS_TIMEOUT_MS = 50;   // 適合 100Hz XSENS (10ms + 40ms buffer)
    
    // 檢查資料有效性 (依據頻率設定超時)
    status.gnss_valid = gnss_data.valid && (current_time - status.last_gnss_update < GNSS_TIMEOUT_MS);
    status.mtdata2_valid = mtdata2_data.valid && (current_time - status.last_mtdata2_update < XSENS_TIMEOUT_MS);
    
    // 融合狀態
    status.fusion_active = status.gnss_valid && status.mtdata2_valid;
    status.gnss_quality = gnss_data.quality;
    
    // 檢測融合初次啟動
    static bool fusion_ever_active = false;
    if (!fusion_ever_active && status.fusion_active) {
        debugPrint("[FUSION] 初次啟動成功，開始融合");
        fusion_ever_active = true;
    }
    
    // 降低狀態輸出頻率
    static unsigned long last_status_output = 0;
    if (status.fusion_active && millis() - last_status_output > 1000) { // 每 1 秒輸出一次
        debugPrint("[FUSION] Status: ACTIVE - GNSS:" + String(status.gnss_valid ? "OK" : "FAIL") + 
                  " MTDATA2:" + String(status.mtdata2_valid ? "OK" : "FAIL"));
        last_status_output = millis();
    }
}

// 檢查資料是否準備好
bool GNSSAHRSFusion::isDataReady() {
    return status.fusion_active;
}

// 處理融合
bool GNSSAHRSFusion::processFusion() {
    if (!isDataReady()) {
        static unsigned long last_error_output = 0;
        if (millis() - last_error_output > 1000) {
            debugPrint("[FUSION] Data not ready - GNSS:" + String(status.gnss_valid ? "OK" : "FAIL") + 
                      " MTDATA2:" + String(status.mtdata2_valid ? "OK" : "FAIL"));
            last_error_output = millis();
        }
        return false;
    }
    
    status.fusion_count++;
    
    // 降低處理輸出頻率
    static unsigned long last_process_output = 0;
    if (millis() - last_process_output > 1000) {
        debugPrint("[FUSION] Processing fusion #" + String(status.fusion_count));
        last_process_output = millis();
    }
    
    return true;
}

// 發送 MAVLink ODO 封包
bool GNSSAHRSFusion::sendMAVLinkOdometry(HardwareSerial& serial_port, uint64_t timestamp_usec) {
    // 時間監控 - 開始計時 // 備用傳送函式：目前已被 smartSendOdometry() 取代
    unsigned long t_start = 0;
    unsigned long t_process = 0;
    unsigned long t_mavlink_prep = 0;
    unsigned long t_covariance = 0;
    unsigned long t_send = 0;
    unsigned long t_end = 0;
    
    if (time_check_enabled) {
        t_start = micros();
    }
    
    if (!processFusion()) {
        return false;
    }
    
    if (time_check_enabled) {
        t_process = micros();
    }
    
    // 改進：關鍵錯誤即時輸出，正常資訊維持低頻率
    static unsigned long last_data_check = 0;
    static bool last_gnss_valid = true;
    static bool last_mtdata2_valid = true;
    
    // 檢查關鍵錯誤狀態變化 - 立即輸出
    bool current_gnss_valid = gnss_data.valid;
    bool current_mtdata2_valid = mtdata2_data.valid;
    
    if (current_gnss_valid != last_gnss_valid) {
        debugPrint("[FUSION ERROR] GNSS 狀態變化: " + 
                  String(current_gnss_valid ? "有效" : "無效"));
        last_gnss_valid = current_gnss_valid;
    }
    
    if (current_mtdata2_valid != last_mtdata2_valid) {
        debugPrint("[FUSION ERROR] MTDATA2 狀態變化: " + 
                  String(current_mtdata2_valid ? "有效" : "無效"));
        last_mtdata2_valid = current_mtdata2_valid;
    }
    
    // 無效數據時立即輸出詳細資訊
    if (!current_gnss_valid || !current_mtdata2_valid) {
        debugPrint("[FUSION ERROR] 資料來源失效:");
        if (!current_gnss_valid) {
            debugPrint("  GNSS: 無效 (品質=" + String(gnss_data.quality) + ")");
        }
        if (!current_mtdata2_valid) {
            debugPrint("  MTDATA2: 無效 (最後更新距今 " + 
                      String(millis() - status.last_mtdata2_update) + "ms)");
        }
    }
    
    // 正常狀態資訊 - 維持低頻率輸出
    if (millis() - last_data_check > 2000 && current_gnss_valid && current_mtdata2_valid) {
        debugPrint("[FUSION] Data Source Check:");
        debugPrint("  GNSS Lat: " + String(gnss_data.latitude, 6) + 
                  " Lon: " + String(gnss_data.longitude, 6) + 
                  " Alt: " + String(gnss_data.altitude, 2));
        debugPrint("  GNSS Vel: N=" + String(gnss_data.velocity_north, 3) + 
                  " E=" + String(gnss_data.velocity_east, 3) + 
                  " D=" + String(gnss_data.velocity_down, 3));
        debugPrint("  GNSS Quality: " + String(gnss_data.quality));
        debugPrint("  MTDATA2 Q: [" + String(mtdata2_data.quaternion[0], 3) + 
                  "," + String(mtdata2_data.quaternion[1], 3) + 
                  "," + String(mtdata2_data.quaternion[2], 3) + 
                  "," + String(mtdata2_data.quaternion[3], 3) + "]");
        last_data_check = millis();
    }
    
    // 創建 MAVLink odometry 封包
    mavlink_odometry_t odom = {};
    
    // 改進：修正時間戳計算 - 使用微秒級時間戳
    odom.time_usec = timestamp_usec;  // 使用傳入的統一時間戳
    
    // 座標系設定 - 使用 GLOBAL 座標系以便 Pixhawk 理解
    odom.frame_id = MAV_FRAME_GLOBAL;
    odom.child_frame_id = MAV_FRAME_LOCAL_NED;
    
    // 位置 (從 GNSS) - 確保資料有效，統一使用與標準模式相同的座標系
    if (gnss_data.valid && gnss_data.latitude != 0.0 && gnss_data.longitude != 0.0) {
        odom.x = gnss_data.latitude;
        odom.y = gnss_data.longitude;
        odom.z = gnss_data.altitude; // 統一座標系：使用正值高度，與標準模式一致
    } else {
        debugPrint("[FUSION] WARNING: Invalid GNSS position data");
        odom.x = 0.0;
        odom.y = 0.0;
        odom.z = 0.0;
    }
    
    // 速度 (從 GNSS)
    odom.vx = gnss_data.velocity_north;
    odom.vy = gnss_data.velocity_east;
    odom.vz = gnss_data.velocity_down;
    
    // 姿態 (從 MTDATA2)
    odom.q[0] = mtdata2_data.quaternion[0];
    odom.q[1] = mtdata2_data.quaternion[1];
    odom.q[2] = mtdata2_data.quaternion[2];
    odom.q[3] = mtdata2_data.quaternion[3];
    
    // 角速度 (從 MTDATA2)
    odom.rollspeed = mtdata2_data.angular_velocity[0];
    odom.pitchspeed = mtdata2_data.angular_velocity[1];
    odom.yawspeed = mtdata2_data.angular_velocity[2];
    
    if (time_check_enabled) {
        t_mavlink_prep = micros();
    }
    
    // 使用動態協方差調整函數
    adaptiveFusionCovariance(odom);
    
    if (time_check_enabled) {
        t_covariance = micros();
    }
    
    // 設定估計器類型統一為 VIO，與標準模式一致
    odom.estimator_type = MAV_ESTIMATOR_TYPE_VIO;
    odom.quality = 100; // 固定為最高品質100
    odom.reset_counter = 0;
    
    // 發送 MAVLink 封包
    mavlink_message_t msg;
    mavlink_msg_odometry_encode(1, 200, &msg, &odom);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    static unsigned long last_debug_fusion_odom = 0;
    if (millis() - last_debug_fusion_odom >= 5000) {
      Serial.print("[MAVLINK] FUSION_ODOMETRY Packet len = ");
      Serial.println(len);
      last_debug_fusion_odom = millis();
    }
    
    serial_port.write(buffer, len);
    
    if (time_check_enabled) {
        t_send = micros();
        t_end = micros();
    }
    
    // 更新狀態
    status.mavlink_sent = true;
    status.mavlink_send_count++;
    status.last_fusion_send = millis();
    
    // 降低 MAVLink 輸出頻率 (每5秒一次)
    static unsigned long last_mavlink_output = 0;
    if (millis() - last_mavlink_output > 5000) {
        debugPrint("[FUSION] =============== MAVLink ODO Packet ===============");
        debugPrint("[FUSION] Mode: FUSION, Packet #" + String(status.mavlink_send_count) + 
                  " Length:" + String(len) + " bytes");
        debugPrint("[FUSION] Frame: " + String(odom.frame_id) + 
                  " Child: " + String(odom.child_frame_id));
        debugPrint("[FUSION] Position (GNSS): X=" + String(odom.x, 6) + 
                  " Y=" + String(odom.y, 6) + 
                  " Z=" + String(odom.z, 2));
        debugPrint("[FUSION] Velocity (GNSS): VX=" + String(odom.vx, 3) + 
                  " VY=" + String(odom.vy, 3) + 
                  " VZ=" + String(odom.vz, 3));
        debugPrint("[FUSION] Quaternion (XSENS): Q0=" + String(odom.q[0], 3) + 
                  " Q1=" + String(odom.q[1], 3) + 
                  " Q2=" + String(odom.q[2], 3) + 
                  " Q3=" + String(odom.q[3], 3));
        debugPrint("[FUSION] Angular Vel (XSENS): RX=" + String(odom.rollspeed, 3) + 
                  " RY=" + String(odom.pitchspeed, 3) + 
                  " RZ=" + String(odom.yawspeed, 3));
        debugPrint("[FUSION] Covariance: Pos=" + String(odom.pose_covariance[0], 6) + 
                  " Vel=" + String(odom.velocity_covariance[0], 6) + 
                  " Att=" + String(odom.pose_covariance[21], 6));
        debugPrint("[FUSION] GNSS: Q=" + String(gnss_data.quality) + 
                  " Sats=" + String(gnss_data.satellites));
        debugPrint("[FUSION] Estimator Type: " + String(odom.estimator_type) + 
                  " Reset Counter: " + String(odom.reset_counter));
        debugPrint("[FUSION] ================================================");
        last_mavlink_output = millis();
    }
    
    // 時間監控輸出
    if (time_check_enabled) {
        static unsigned long last_timing_output = 0;
        if (millis() - last_timing_output > 5000) {
            Serial.print("[FUSION_TIME] sendMAVLinkOdometry(us): ");
            Serial.print("ProcessFusion="); Serial.print(t_process - t_start); Serial.print(" | ");
            Serial.print("MAVLinkPrep="); Serial.print(t_mavlink_prep - t_process); Serial.print(" | ");
            Serial.print("Covariance="); Serial.print(t_covariance - t_mavlink_prep); Serial.print(" | ");
            Serial.print("Send="); Serial.print(t_send - t_covariance); Serial.print(" | ");
            Serial.print("Total="); Serial.println(t_end - t_start);
            last_timing_output = millis();
        }
    }
    
    return true;
}

// 更新融合數據到latest_odom_data
void GNSSAHRSFusion::updateOdomData() {    
    // 時間分析 - 開始追蹤
    static unsigned long last_timing_report = 0;
    bool should_report_timing = time_check_enabled && (millis() - last_timing_report > 5000);
    
    if (should_report_timing) {
        Serial.println("=== [FUSION_TIMING] updateOdomData ===");
    }
    
    unsigned long t0 = micros();
    if (!processFusion()) {
        return;  // 如果融合失敗，不更新數據
    }
    unsigned long t1 = micros();
    
    // 更新位置 (來自GNSS)
    if (gnss_data.valid && gnss_data.latitude != 0.0 && gnss_data.longitude != 0.0) {
        latest_odom_data.pos_n = gnss_data.latitude;
        latest_odom_data.pos_e = gnss_data.longitude;
        latest_odom_data.pos_d = gnss_data.altitude;
    }
    unsigned long t2 = micros();
    
    // 更新速度 (來自GNSS)
    latest_odom_data.vel_n = gnss_data.velocity_north;
    latest_odom_data.vel_e = gnss_data.velocity_east;
    latest_odom_data.vel_d = gnss_data.velocity_down;
    unsigned long t3 = micros();
    
    // 更新姿態和角速度 (來自IMU)
    for (int i = 0; i < 4; i++) {
        latest_odom_data.quat[i] = mtdata2_data.quaternion[i];
    }
    for (int i = 0; i < 3; i++) {
        latest_odom_data.angular_vel[i] = mtdata2_data.angular_velocity[i];
    }
    unsigned long t4 = micros();
    
    // 計算協方差矩陣 (最花時間的步驟)
    mavlink_odometry_t temp_odom = {};
    adaptiveFusionCovariance(temp_odom);
    memcpy(latest_odom_data.pose_covariance, temp_odom.pose_covariance, sizeof(latest_odom_data.pose_covariance));
    memcpy(latest_odom_data.velocity_covariance, temp_odom.velocity_covariance, sizeof(latest_odom_data.velocity_covariance));
    unsigned long t5 = micros();
    
    // 更新元數據
    latest_odom_data.frame_id = MAV_FRAME_GLOBAL;
    latest_odom_data.child_frame_id = MAV_FRAME_LOCAL_NED;
    latest_odom_data.estimator_type = MAV_ESTIMATOR_TYPE_VIO;
    latest_odom_data.quality = 100;
    latest_odom_data.reset_counter = 0;
    latest_odom_data.valid = true;
    unsigned long t6 = micros();
    
    // 時間報告 (每5秒一次)
    if (should_report_timing) {
        Serial.print("[FUSION_PERF] Fusion="); Serial.print(t1 - t0);
        Serial.print("μs | Pos="); Serial.print(t2 - t1);
        Serial.print("μs | Vel="); Serial.print(t3 - t2);
        Serial.print("μs | IMU="); Serial.print(t4 - t3);
        Serial.print("μs | Cov="); Serial.print(t5 - t4);
        Serial.print("μs | Meta="); Serial.print(t6 - t5);
        Serial.print("μs | Total="); Serial.print(t6 - t0);
        Serial.println("μs");
        last_timing_report = millis();
    }
}

// 獲取最新可用數據
const OdomData& GNSSAHRSFusion::getLatestOdom() {
    return latest_odom_data;
}

// 非阻塞智慧傳送ODOMETRY
bool GNSSAHRSFusion::smartSendOdometry(HardwareSerial& serial_port, uint64_t timestamp_usec) {
    // 時間分析 - 詳細步驟追蹤
    static unsigned long last_send_timing_report = 0;
    bool should_report_send_timing = time_check_enabled && (millis() - last_send_timing_report > 5000);
    
    if (should_report_send_timing) {
        Serial.println("=== [FUSION_TIMING] smartSendOdometry ===");
    }
    
    unsigned long t0 = micros();
    
    // 使用最新可用數據
    const OdomData& odom_data = getLatestOdom();
    if (!odom_data.valid) {
        tx_stats.tx_drop_count++;
        return false;
    }
    unsigned long t1 = micros();
    
    // 準備MAVLink封包 - 基本資料設定
    mavlink_odometry_t odom;
    odom.time_usec = timestamp_usec;
    odom.frame_id = odom_data.frame_id;
    odom.child_frame_id = odom_data.child_frame_id;
    unsigned long t2 = micros();
    
    // 位置和速度資料複製
    odom.x = odom_data.pos_n;
    odom.y = odom_data.pos_e;
    odom.z = odom_data.pos_d;
    odom.vx = odom_data.vel_n;
    odom.vy = odom_data.vel_e;
    odom.vz = odom_data.vel_d;
    unsigned long t3 = micros();
    
    // 姿態和角速度資料複製
    memcpy(odom.q, odom_data.quat, sizeof(odom.q));
    odom.rollspeed = odom_data.angular_vel[0];
    odom.pitchspeed = odom_data.angular_vel[1];
    odom.yawspeed = odom_data.angular_vel[2];
    unsigned long t4 = micros();
    
    // 協方差矩陣複製 (可能是時間消耗大戶)
    memcpy(odom.pose_covariance, odom_data.pose_covariance, sizeof(odom.pose_covariance));
    memcpy(odom.velocity_covariance, odom_data.velocity_covariance, sizeof(odom.velocity_covariance));
    unsigned long t5 = micros();
    
    // 其他屬性設定
    odom.estimator_type = odom_data.estimator_type;
    odom.quality = odom_data.quality;
    odom.reset_counter = odom_data.reset_counter;
    unsigned long t6 = micros();
    
    // MAVLink 編碼
    mavlink_message_t msg;
    mavlink_msg_odometry_encode(1, 200, &msg, &odom);
    unsigned long t7 = micros();
    
    // Buffer 準備
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    unsigned long t8 = micros();
    
    // 檢查Serial緩衝區空間 - 非阻塞傳送
    if (serial_port.availableForWrite() >= len) {
        static unsigned long last_debug_smart_odom = 0;
        if (millis() - last_debug_smart_odom >= 5000) {
          Serial.print("[MAVLINK] SMART_ODOMETRY Packet len = ");
          Serial.println(len);
          last_debug_smart_odom = millis();
        }
        
        // 實際傳送
        serial_port.write(buffer, len);
        tx_stats.tx_ok_count++;
        unsigned long t9 = micros();
        
        // 詳細時間報告 (每5秒一次)
        if (should_report_send_timing) {
            Serial.print("[SEND_PERF] GetData="); Serial.print(t1 - t0);
            Serial.print("μs | Basic="); Serial.print(t2 - t1);
            Serial.print("μs | PosVel="); Serial.print(t3 - t2);
            Serial.print("μs | AttAng="); Serial.print(t4 - t3);
            Serial.print("μs | Cov="); Serial.print(t5 - t4);
            Serial.print("μs | Attr="); Serial.print(t6 - t5);
            Serial.println("μs");
            Serial.print("[SEND_PERF] Encode="); Serial.print(t7 - t6);
            Serial.print("μs | Buffer="); Serial.print(t8 - t7);
            Serial.print("μs | Write="); Serial.print(t9 - t8);
            Serial.print("μs | Total="); Serial.print(t9 - t0);
            Serial.println("μs");
            last_send_timing_report = millis();
        }
        
        return true;
    } else {
        // 緩衝區滿，丟棄此次傳送
        tx_stats.tx_drop_count++;
        
        if (time_check_enabled) {
            static unsigned long last_drop_warning = 0;
            if (millis() - last_drop_warning > 1000) {
                Serial.println("[FUSION_WARN] Serial buffer full, packet dropped");
                last_drop_warning = millis();
            }
        }
        
        return false;
    }
}

// 顯示傳送統計
void GNSSAHRSFusion::printTransmissionStats() {
    unsigned long current_time = millis();
    
    // 每秒更新統計
    if (current_time - tx_stats.last_stats_time >= 1000) {
        uint32_t total_attempts = tx_stats.tx_ok_count + tx_stats.tx_drop_count;
        if (total_attempts > 0) {
            tx_stats.success_rate = 100.0f * tx_stats.tx_ok_count / total_attempts;
        }
        
        Serial.print("[TX_STATS] OK: ");
        Serial.print(tx_stats.tx_ok_count);
        Serial.print(" DROP: ");
        Serial.print(tx_stats.tx_drop_count);
        Serial.print(" RATE: ");
        Serial.print(tx_stats.success_rate, 1);
        Serial.println("%");
        
        // 重置計數器
        tx_stats.tx_ok_count = 0;
        tx_stats.tx_drop_count = 0;
        tx_stats.last_stats_time = current_time;
        
        // 低成功率警告
        if (tx_stats.success_rate < 95.0f && total_attempts > 10) {
            Serial.println("[TX_WARN] Low transmission success rate!");
        }
    }
}

// 取得融合狀態
FusionStatus GNSSAHRSFusion::getFusionStatus() {
    updateFusionStatus();
    return status;
}

// 取得 GNSS 詳細數據
GNSSData GNSSAHRSFusion::getGNSSData() {
    return gnss_data;
}

// 列印詳細狀態
void GNSSAHRSFusion::printDetailedStatus() {
    updateFusionStatus();
    unsigned long current_time = millis();
    
    debugPrint("=============== FUSION STATUS ===============");
    debugPrint("[FUSION] Active: " + String(status.fusion_active ? "YES" : "NO"));
    debugPrint("[FUSION] Count: " + String(status.fusion_count));
    debugPrint("[FUSION] MAVLink Sent: " + String(status.mavlink_send_count));
    
    debugPrint("[GNSS] Valid: " + String(status.gnss_valid ? "YES" : "NO"));
    debugPrint("[GNSS] Quality: " + String(status.gnss_quality));
    debugPrint("[GNSS] Age: " + String(current_time - status.last_gnss_update) + "ms");
    debugPrint("[GNSS] Lat: " + String(gnss_data.latitude, 6));
    debugPrint("[GNSS] Lon: " + String(gnss_data.longitude, 6));
    debugPrint("[GNSS] Alt: " + String(gnss_data.altitude, 1));
    debugPrint("[GNSS] Speed: " + String(gnss_data.speed, 3) + " m/s");
    
    debugPrint("[MTDATA2] Valid: " + String(status.mtdata2_valid ? "YES" : "NO"));
    debugPrint("[MTDATA2] Age: " + String(current_time - status.last_mtdata2_update) + "ms");
    debugPrint("[MTDATA2] Q0: " + String(mtdata2_data.quaternion[0], 3));
    debugPrint("[MTDATA2] Q1: " + String(mtdata2_data.quaternion[1], 3));
    debugPrint("[MTDATA2] Q2: " + String(mtdata2_data.quaternion[2], 3));
    debugPrint("[MTDATA2] Q3: " + String(mtdata2_data.quaternion[3], 3));
    
    debugPrint("=============================================");
}

// 診斷函數
void GNSSAHRSFusion::runDiagnostics() {
    debugPrint("[FUSION] Running diagnostics...");
    printDetailedStatus();
    
    if (!status.gnss_valid) {
        debugPrint("[FUSION] DIAGNOSTIC: GNSS data is invalid or too old");
    }
    
    if (!status.mtdata2_valid) {
        debugPrint("[FUSION] DIAGNOSTIC: MTDATA2 data is invalid or too old");
    }
    
    if (status.fusion_active) {
        debugPrint("[FUSION] DIAGNOSTIC: All systems operational");
    } else {
        debugPrint("[FUSION] DIAGNOSTIC: Fusion inactive - check data sources");
    }
}

// 動態協方差調整函數
void GNSSAHRSFusion::adaptiveFusionCovariance(mavlink_odometry_t &odom) {
    // 1. GNSS 位置協方差 - 根據衛星數量和品質動態調整
    float gnss_pos_cov, gnss_vel_cov;
    
    // 根據 GNSS 品質和衛星數量計算位置協方差
    if (gnss_data.quality == 2) {
        // RTK 固定解
        gnss_pos_cov = 1e-5f;  // 高精度
        gnss_vel_cov = 1e-6f;
    } else if (gnss_data.quality == 1) {
        // DGPS
        gnss_pos_cov = 1e-3f;
        gnss_vel_cov = 1e-4f;
    } else {
        // 標準 GPS - 根據衛星數量調整
        if (gnss_data.satellites >= 8) {
            gnss_pos_cov = 5e-2f;  // 8+ 顆衛星，良好
            gnss_vel_cov = 1e-3f;
        } else if (gnss_data.satellites >= 5) {
            gnss_pos_cov = 1e-1f;  // 5-7 顆衛星，中等
            gnss_vel_cov = 5e-3f;
        } else {
            gnss_pos_cov = 5e-1f;  // <5 顆衛星，較差
            gnss_vel_cov = 1e-2f;
        }
    }
    
    // 2. MTData2 姿態協方差 - 使用與標準模式相同的動態計算
    // 假設中等運動狀態的 trust factor (0.5)，實際應從主程式傳入
    float trust_factor = 0.5f; // 暫時固定，未來可改為參數
    
    float max_att_cov = 1e-1f;
    float min_att_cov = 1e-6f;
    float att_cov = max_att_cov * (1.0f - trust_factor) + min_att_cov * trust_factor;
    
    // YAW 軸協方差 - 暫時使用標準模式的設定
    float yaw_trust_factor = 0.5f; // 暫時固定
    float yaw_att_cov = max_att_cov * (1.0f - yaw_trust_factor) + min_att_cov * yaw_trust_factor;
    
    // 設定位置協方差（GNSS 來源）
    odom.pose_covariance[0]  = gnss_pos_cov;           // x position
    odom.pose_covariance[7]  = gnss_pos_cov;           // y position  
    odom.pose_covariance[14] = gnss_pos_cov * 1.5f;    // z position (高度稍微不確定些)
    odom.pose_covariance[21] = att_cov;                 // roll (MTData2)
    odom.pose_covariance[28] = att_cov;                 // pitch (MTData2)
    odom.pose_covariance[35] = yaw_att_cov;             // yaw (MTData2 + 未來 GNSS)
    
    // 設定速度協方差
    odom.velocity_covariance[0]  = gnss_vel_cov;           // vx (GNSS)
    odom.velocity_covariance[7]  = gnss_vel_cov;           // vy (GNSS)
    odom.velocity_covariance[14] = gnss_vel_cov * 1.2f;    // vz (GNSS)
    odom.velocity_covariance[21] = att_cov * 5.0f;         // roll rate (MTData2)
    odom.velocity_covariance[28] = att_cov * 5.0f;         // pitch rate (MTData2)
    odom.velocity_covariance[35] = yaw_att_cov * 3.0f;     // yaw rate (MTData2)
    
    // 調試輸出
    static unsigned long last_cov_debug = 0;
    if (millis() - last_cov_debug > 3000) {
        debugPrint("[FUSION COVARIANCE] GNSS: Q=" + String(gnss_data.quality) + 
                  " Sats=" + String(gnss_data.satellites) + 
                  " PosCov=" + String(gnss_pos_cov, 6));
        debugPrint("[FUSION COVARIANCE] MTData2: AttCov=" + String(att_cov, 6) + 
                  " YawCov=" + String(yaw_att_cov, 6) + 
                  " Trust=" + String(trust_factor, 3));
        last_cov_debug = millis();
    }
}

// 計算平均 SNR 輔助函數
float GNSSAHRSFusion::calculateAverageSNR() {
    if (gnss_extended_data.satellites_in_view == 0) return 0.0;
    
    float total_snr = 0.0;
    int valid_count = 0;
    
    for (int i = 0; i < gnss_extended_data.satellites_in_view && i < 20; i++) {
        if (gnss_extended_data.satellites[i].snr > 0) {
            total_snr += gnss_extended_data.satellites[i].snr;
            valid_count++;
        }
    }
    
    return (valid_count > 0) ? (total_snr / valid_count) : 0.0;
}

// GSV 句子解析函數
void GNSSAHRSFusion::parseGSVSentence(const String& nmea_sentence) {
    // GSV 格式：$GPGSV,3,1,12,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
    // 欄位: 頭,總句數,句號,總衛星數,PRN1,仰角1,方位角1,SNR1,PRN2,仰角2,方位角2,SNR2,...
    
    int comma_positions[25]; // 最多支援 4 顆衛星 * 4 欄位 + 4 個標頭欄位 + 一些餘裕
    int comma_count = 0;
    
    // 找出所有逗號位置
    for (int i = 0; i < nmea_sentence.length() && comma_count < 25; i++) {
        if (nmea_sentence[i] == ',') {
            comma_positions[comma_count] = i;
            comma_count++;
        }
    }
    
    if (comma_count < 3) {
        debugPrint("[FUSION] GSV: Invalid format - insufficient commas");
        return;
    }
    
    // 提取標頭資訊
    String total_sentences_str = nmea_sentence.substring(comma_positions[0] + 1, comma_positions[1]);
    String sentence_num_str = nmea_sentence.substring(comma_positions[1] + 1, comma_positions[2]);
    String total_sats_str = nmea_sentence.substring(comma_positions[2] + 1, comma_positions[3]);
    
    int total_sentences = total_sentences_str.toInt();
    int sentence_num = sentence_num_str.toInt();
    int total_sats = total_sats_str.toInt();
    
    if (total_sats > 20) total_sats = 20; // 限制最多 20 顆衛星
    
    // 更新總衛星數 (只在第一句更新)
    if (sentence_num == 1) {
        gnss_extended_data.satellites_in_view = total_sats;
        debugPrint("[FUSION] GSV: Total satellites in view: " + String(total_sats));
    }
    
    // 解析此句子中的衛星資訊 (每句最多4顆)
    int sat_start_idx = (sentence_num - 1) * 4;
    int satellites_in_this_sentence = min(4, total_sats - sat_start_idx);
    
    for (int i = 0; i < satellites_in_this_sentence; i++) {
        int base_idx = 3 + i * 4;  // 從第4個欄位開始，每4個欄位一顆衛星
        
        if (base_idx + 3 < comma_count) {
            int sat_idx = sat_start_idx + i;
            if (sat_idx >= 20) break; // 超出陣列範圍保護
            
            // 提取衛星資訊
            String prn_str = nmea_sentence.substring(comma_positions[base_idx] + 1, comma_positions[base_idx + 1]);
            String elevation_str = nmea_sentence.substring(comma_positions[base_idx + 1] + 1, comma_positions[base_idx + 2]);
            String azimuth_str = nmea_sentence.substring(comma_positions[base_idx + 2] + 1, comma_positions[base_idx + 3]);
            
            // SNR 處理 (可能包含 checksum)
            String snr_str;
            if (base_idx + 4 < comma_count) {
                snr_str = nmea_sentence.substring(comma_positions[base_idx + 3] + 1, comma_positions[base_idx + 4]);
            } else {
                // 最後一個欄位，需要處理 checksum
                String remaining = nmea_sentence.substring(comma_positions[base_idx + 3] + 1);
                int star_pos = remaining.indexOf('*');
                if (star_pos != -1) {
                    snr_str = remaining.substring(0, star_pos);
                } else {
                    snr_str = remaining;
                }
            }
            
            // 填入衛星資料
            gnss_extended_data.satellites[sat_idx].prn = prn_str.toInt();
            gnss_extended_data.satellites[sat_idx].elevation = elevation_str.toInt();
            gnss_extended_data.satellites[sat_idx].azimuth = azimuth_str.toInt();
            gnss_extended_data.satellites[sat_idx].snr = (snr_str.length() > 0) ? snr_str.toInt() : 0;
            gnss_extended_data.satellites[sat_idx].used = false; // 暫時設為未用於定位
            
            debugPrint("[FUSION] GSV Sat " + String(sat_idx) + 
                      ": PRN=" + String(gnss_extended_data.satellites[sat_idx].prn) +
                      " El=" + String(gnss_extended_data.satellites[sat_idx].elevation) +
                      " Az=" + String(gnss_extended_data.satellites[sat_idx].azimuth) +
                      " SNR=" + String(gnss_extended_data.satellites[sat_idx].snr));
        }
    }
}

// GPS_RAW_INT MAVLink 訊息實作
void GNSSAHRSFusion::sendMAVLink_GPS_RAW_INT(HardwareSerial& serial_port, uint64_t timestamp_usec) {
    if (!fusionConfig.enable_gps_raw_int || 
        millis() - last_gps_raw_time < GPS_RAW_INTERVAL) {
        return;
    }
    
    mavlink_message_t msg;
    mavlink_gps_raw_int_t gps_raw = {};
    
    gps_raw.time_usec = timestamp_usec;
    gps_raw.fix_type = convertFixType(gnss_extended_data.gga_quality);
    
    // 位置資訊 (度 → 1e7 度)
    if (gnss_data.valid && gnss_data.latitude != 0.0 && gnss_data.longitude != 0.0) {
        gps_raw.lat = (int32_t)(gnss_data.latitude * 1e7);
        gps_raw.lon = (int32_t)(gnss_data.longitude * 1e7);
        gps_raw.alt = (int32_t)(gnss_data.altitude * 1000); // m → mm
    } else {
        gps_raw.lat = 0;
        gps_raw.lon = 0;
        gps_raw.alt = 0;
    }
    
    gps_raw.satellites_visible = gnss_extended_data.satellites_in_view > 0 ? 
                                gnss_extended_data.satellites_in_view : gnss_data.satellites;
    
    // 速度和航向
    float ground_speed = sqrt(gnss_data.velocity_north * gnss_data.velocity_north + 
                             gnss_data.velocity_east * gnss_data.velocity_east);
    gps_raw.vel = (uint16_t)(ground_speed * 100);   // m/s → cm/s
    
    if (gnss_data.velocity_north != 0.0 || gnss_data.velocity_east != 0.0) {
        float course_rad = atan2(gnss_data.velocity_east, gnss_data.velocity_north);
        if (course_rad < 0) course_rad += 2 * M_PI;  // 確保為正值
        gps_raw.cog = (uint16_t)(course_rad * 180.0 / M_PI * 100); // 弧度 → 度*100
    } else {
        gps_raw.cog = 65535; // 無效值
    }
    
    // HDOP/VDOP 根據品質設定
    switch (gnss_extended_data.gga_quality) {
        case 4: case 5: // RTK
            gps_raw.eph = 50;   // 0.5m
            gps_raw.epv = 100;  // 1.0m
            break;
        case 2: // DGPS
            gps_raw.eph = 150;  // 1.5m
            gps_raw.epv = 200;  // 2.0m
            break;
        case 1: // GPS
            gps_raw.eph = 300;  // 3.0m
            gps_raw.epv = 400;  // 4.0m
            break;
        default:
            gps_raw.eph = 65535; // 無效
            gps_raw.epv = 65535; // 無效
            break;
    }
    
    mavlink_msg_gps_raw_int_encode(1, 200, &msg, &gps_raw);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    static unsigned long last_debug_fusion_gps_raw = 0;
    if (millis() - last_debug_fusion_gps_raw >= 5000) {
      Serial.print("[MAVLINK] FUSION_GPS_RAW_INT Packet len = ");
      Serial.println(len);
      last_debug_fusion_gps_raw = millis();
    }
    
    serial_port.write(buffer, len);
    
    last_gps_raw_time = millis();
    
    // Debug 輸出
    if (debug_enabled) {
        debugPrint("[GPS_RAW_INT] Sent - Fix:" + String(gps_raw.fix_type) + 
                  " Sats:" + String(gps_raw.satellites_visible) + 
                  " Vel:" + String(ground_speed, 2) + "m/s");
    }
}

// GPS_STATUS MAVLink 訊息實作
void GNSSAHRSFusion::sendMAVLink_GPS_STATUS(HardwareSerial& serial_port) {
    if (!fusionConfig.enable_gps_status || 
        millis() - last_gps_status_time < GPS_STATUS_INTERVAL) {
        return;
    }
    
    mavlink_message_t msg;
    mavlink_gps_status_t gps_status = {};
    
    gps_status.satellites_visible = gnss_extended_data.satellites_in_view;
    
    // 填入衛星資訊陣列 (最多20個)
    for (int i = 0; i < 20; i++) {
        if (i < gnss_extended_data.satellites_in_view) {
            gps_status.satellite_prn[i] = gnss_extended_data.satellites[i].prn;
            gps_status.satellite_elevation[i] = gnss_extended_data.satellites[i].elevation;
            gps_status.satellite_azimuth[i] = gnss_extended_data.satellites[i].azimuth;
            gps_status.satellite_snr[i] = gnss_extended_data.satellites[i].snr;
            gps_status.satellite_used[i] = gnss_extended_data.satellites[i].used ? 1 : 0;
        } else {
            // 填充空位
            gps_status.satellite_prn[i] = 0;
            gps_status.satellite_elevation[i] = 0;
            gps_status.satellite_azimuth[i] = 0;
            gps_status.satellite_snr[i] = 0;
            gps_status.satellite_used[i] = 0;
        }
    }
    
    mavlink_msg_gps_status_encode(1, 200, &msg, &gps_status);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    static unsigned long last_debug_fusion_gps_status = 0;
    if (millis() - last_debug_fusion_gps_status >= 5000) {
      Serial.print("[MAVLINK] FUSION_GPS_STATUS Packet len = ");
      Serial.println(len);
      last_debug_fusion_gps_status = millis();
    }
    
    serial_port.write(buffer, len);
    
    last_gps_status_time = millis();
    
    // Debug 輸出
    if (debug_enabled) {
        debugPrint("[GPS_STATUS] Sent - Sats:" + String(gps_status.satellites_visible) + 
                  " Total_SNR_avg:" + String(calculateAverageSNR(), 1) + "dB");
    }
}

// GPS_INPUT (#232) MAVLink 訊息實作 - 支援雙天線 GNSS
void GNSSAHRSFusion::sendMAVLink_GPS_INPUT(HardwareSerial& serial_port, uint64_t timestamp_usec) {
    if (!fusionConfig.enable_gps_input || !gnss_data.valid) {
        return; // GPS_INPUT 未啟用或沒有有效的 GNSS 資料
    }
    
    mavlink_message_t msg;
    mavlink_gps_input_t gps_input = {};
    
    // 基本資料設定
    gps_input.time_usec = timestamp_usec;
    gps_input.gps_id = 0;
    gps_input.fix_type = convertFixType(gnss_data.quality);
    
    // 位置資料 (lat/lon: 1E7, alt: mm)
    gps_input.lat = (int32_t)(gnss_data.latitude * 1E7);
    gps_input.lon = (int32_t)(gnss_data.longitude * 1E7);
    gps_input.alt = (float)(gnss_data.altitude);
    
    // 速度資料 (m/s)
    gps_input.vn = (float)gnss_data.velocity_north;
    gps_input.ve = (float)gnss_data.velocity_east;
    gps_input.vd = (float)gnss_data.velocity_down;
    
    // 精度資料 - 基於品質動態計算
    if (gnss_data.quality >= 4) { // RTK Fixed/Float
        gps_input.hdop = 0.5f;
        gps_input.vdop = 0.8f;
    } else if (gnss_data.quality >= 2) { // DGPS
        gps_input.hdop = 1.0f;
        gps_input.vdop = 1.5f;
    } else { // Standard GPS
        gps_input.hdop = 2.0f;
        gps_input.vdop = 3.0f;
    }
    
    // 初始化 ignore_flags
    gps_input.ignore_flags = 0;  // 預設不忽略任何資料
    
    // 雙天線航向角支援
    if (gnss_data.heading_valid && gnss_extended_data.plshd_valid) {
        gps_input.yaw = gnss_data.yaw_rad;
        // 動態 yaw_accuracy 計算
        if (gnss_extended_data.baseline_length_m >= 1.0f) {
            gnss_data.yaw_accuracy = 0.5f * (M_PI / 180.0f); // 0.5度轉弧度
        } else if (gnss_extended_data.baseline_length_m >= 0.5f) {
            gnss_data.yaw_accuracy = 1.0f * (M_PI / 180.0f); // 1.0度轉弧度
        } else {
            gnss_data.yaw_accuracy = 2.0f * (M_PI / 180.0f); // 2.0度轉弧度
        }
        
        // 衛星數修正
        int total_satellites = gnss_extended_data.ant1_satellites + gnss_extended_data.ant2_satellites;
        if (total_satellites < 16) {
            gnss_data.yaw_accuracy *= 1.5f;
        }
        
        // YAW 有效：ignore_flags = 0 (不忽略 YAW，啟用 yaw fusion)
        gps_input.ignore_flags = 0;
    } else {
        gps_input.yaw = 0.0f;
        gnss_data.yaw_accuracy = 0.0f;
        // YAW 無效：設定 bit 7 忽略 YAW (停用 yaw fusion)
        gps_input.ignore_flags |= (1 << 7);
    }
    
    gps_input.satellites_visible = gnss_data.satellites;
    
    // 封裝並傳送
    mavlink_msg_gps_input_encode(1, 200, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    serial_port.write(buffer, len);
    
    // Debug 輸出
    if (debug_enabled) {
        String yaw_status = (gps_input.ignore_flags == 0) ? "ENABLED" : "DISABLED";
        debugPrint("[GPS_INPUT] Sent - Fix:" + String(gps_input.fix_type) + 
                  " Sats:" + String(gps_input.satellites_visible) + 
                  " HDOP:" + String(gps_input.hdop, 2) + 
                  " Yaw:" + String(gnss_data.gnss_heading_deg, 1) + "° " +
                  "YawAcc:" + String(gnss_data.yaw_accuracy * 180.0 / M_PI, 1) + "° " +
                  "YawFusion:" + yaw_status + " (flags:" + String(gps_input.ignore_flags, HEX) + ")");
    }
}