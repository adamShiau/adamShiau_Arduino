#include <MAVLink.h>
#include <Arduino.h>
#include "src/communication/myUARTSensor.h"
#include "src/utils/myUART.h"
#include "src/core/Orientation.h"
#include "src/core/gnss_ahrs_fusion.h"

// Function declarations
void performSecurityCheck(HardwareSerial &port, bool silent_mode);
bool authenticateSecurityCommand(uint32_t provided_token);
void enableSecurityMode(HardwareSerial &port);
void disableSecurityMode(HardwareSerial &port);
bool checkHardwareIntegrity();
bool validateSensorData();
bool checkCommunicationSecurity();
bool checkTimeSyncIntegrity();
bool checkMemoryIntegrity();
uint32_t simpleHash(uint32_t input);
bool constantTimeCompare(uint32_t a, uint32_t b);

bool sendGNSSCommandAndWaitAck(HardwareSerial &serial_nmea,
                               const char* cmd,
                               const char* expected_header,
                               int max_retries = 5,
                               unsigned long ack_timeout_ms = 1000);


// 動態權重系統函數聲明
void updateMovementDetection(const my_data_2d &latlon, const my_data_u4 &hei, 
                           const my_data_3f &vel, const my_data_3f &omg, 
                           const my_data_3f &acc);
float calculateTrustFactor();
float calculateYawTrustFactor();
void adaptiveCovariance(mavlink_odometry_t &odom, float trust_factor);

// 資料清理函數聲明
void cleanSensorData(my_data_4f &qut, my_data_3f &omg, my_data_3f &acc);
bool isValidFloat(float value);
float constrainFloat(float value, float min_val, float max_val);

// NMEA 時間戳格式標準化函數聲明
String normalizeTimestampFormat(const String& timestamp);
String normalizeNMEASentence(const String& nmea_sentence);

// 時間戳驗證與平滑函數聲明
uint64_t getUnifiedTimestamp(const my_data_u4 &XsensTime);
bool validateTimestamp(uint64_t timestamp);
uint64_t smoothTimestamp(uint64_t timestamp);

#define PIXHAWK_SERIAL Serial1
#define Serial_xsens Serial2
#define NMEA_OUT_Serial Serial3
#define NMEA_IN_Serial Serial4
#define DATA_OK       0
#define DATA_WARNING  1
#define DATA_INVALID  2

XsensUart xsens(Serial_xsens);
int64_t time_offset = 0;
uint64_t xsens_pre_time = 0;
uint64_t last_unified_timestamp = 0;
uint8_t current_Xsens_mode = MODE_AHRS; // 編號: 1
uint8_t current_output_mode = OUT_MODE_BIN;
bool is_run = false;  
bool is_debug = false;

// GNSS-AHRS 融合模組
GNSSAHRSFusion fusion_module;
bool fusion_mode_enabled = false;
bool ISR_flag_xsens = false, ISR_flag_NMEA = false;
bool USB_Setting_mode = false;
bool enable_input = true;
bool time_sync_initialized = false;  // 新增：時間同步初始化標記
bool gps_connected = false;          // GPS 連接狀態
unsigned long last_gps_data_time = 0; // 最後收到 GPS 數據的時間
String nmea_input_buffer = "";       // NMEA 串列累積緩衝區

// 動態權重系統變數
struct {
    float prev_pos[3] = {0, 0, 0};    // 前一次位置 [lat, lon, alt]
    float prev_vel[3] = {0, 0, 0};    // 前一次速度 [vx, vy, vz]
    float prev_omg[3] = {0, 0, 0};    // 前一次角速度 [roll, pitch, yaw rates]
    float movement_score = 0.0f;      // 運動程度評分 (0-1)
    float trust_factor = 0.1f;        // EKF 信任因子 (0.1-1.0)
    unsigned long last_update = 0;    // 上次更新時間
    bool is_initialized = false;      // 是否已初始化
    float acceleration_mag = 0.0f;    // 加速度幅值
    float angular_rate_mag = 0.0f;    // 角速度幅值
    
    // YAW軸專用變數
    float yaw_rate_history[5] = {0, 0, 0, 0, 0}; // YAW角速度歷史記錄
    int yaw_history_index = 0;        // 歷史記錄索引
    float yaw_stability_score = 0.0f; // YAW穩定性評分 (0-1)
    float yaw_trust_factor = 0.1f;    // YAW專用信任因子 (0.1-1.0)
} dynamic_trust;
bool security_mode_active = false;    // 安全模式狀態
unsigned long last_security_check = 0; // 最後一次安全檢查時間
uint8_t security_level = 0;           // 安全等級 (0=正常, 1=警告, 2=錯誤)
unsigned long last_security_cmd = 0;   // 最後一次安全指令時間 (防護DoS)
uint8_t failed_auth_attempts = 0;     // 失敗認證次數
unsigned long lockout_start_time = 0; // 鎖定開始時間 (非阻塞式)
bool is_locked_out = false;           // 鎖定狀態標記
// 注意：在生產環境中，令牌應存儲在安全硬體中，不應硬編碼
const uint32_t SECURITY_TOKEN_HASH = 0x7F8E9A2B; // 令牌的雜湊值（而非明文）
my_data_u4 latestXsensTime;
unsigned long last_sync_millis = 0;         // 上一次時間同步的時間點（ms）
const unsigned long SYNC_INTERVAL = 1000;   // 每 1 秒重新同步一次，改善延遲和精度（單位：ms）

// MAVLink 發送頻率控制 - 統一數據流暢性
const uint32_t TARGET_MAVLINK_HZ = 100;      // 目標 MAVLink 發送頻率 (Hz)
const uint32_t MAVLINK_SEND_INTERVAL = 1000000 / TARGET_MAVLINK_HZ / 10;  // Xsens時間戳間隔 (10000 for 100Hz)

// NMEA 發送頻率控制 - 針對 MTi-680 優化 (分類型控制)
unsigned long last_gga_send = 0;            // 上次發送GGA的時間
unsigned long last_rmc_send = 0;            // 上次發送RMC的時間
unsigned long last_gst_send = 0;            // 上次發送GST的時間
unsigned long last_gsa_send = 0;            // 上次發送GSA的時間
unsigned long last_vtg_send = 0;            // 上次發送VTG的時間
unsigned long last_zda_send = 0;            // 上次發送ZDA的時間
unsigned long last_hdt_send = 0;            // 上次發送HDT的時間
unsigned long last_plshd_send = 0;          // 上次發送PLSHD的時間
unsigned long last_gsv_send = 0;            // 上次發送GSV的時間
// NMEA頻率控制已移除 - 直接轉發LOCOSYS輸出，不做頻率限制
unsigned long nmea_received_count = 0;       // 接收到的NMEA句子總數
unsigned long nmea_sent_count = 0;           // 發送到MTi-680的句子數
unsigned long nmea_invalid_count = 0;        // 無效NMEA句子數
unsigned long nmea_discarded_count = 0;      // 被丟棄的不完整句子數
unsigned long gga_sent = 0, rmc_sent = 0, gst_sent = 0; // 各類型句子發送統計
unsigned long gsa_sent = 0, vtg_sent = 0, zda_sent = 0; // 新增句子類型發送統計
unsigned long hdt_sent = 0, plshd_sent = 0, gsv_sent = 0; // XSENS專用句子發送統計

// XSENS 全域數據變數
my_data_3f latest_orientation = {0, 0, 0}; // 最新的方向數據 [pitch, roll, yaw]
bool orientation_data_available = false;   // 方向數據是否可用

// NMEA 數據組同步機制 - 基於XSENS官方要求的完整數據組
struct NMEADataset {
  String gga_sentence = "";
  String rmc_sentence = "";
  String gst_sentence = "";
  String gsa_sentences[4] = {"", "", "", ""}; // XSENS要求4個GSA句子
  String vtg_sentence = "";
  String zda_sentence = "";
  String timestamp = "";      // 共同時間戳
  int gsa_count = 0;         // 已收集的GSA句子數量
  bool has_gga = false;
  bool has_rmc = false;
  bool has_gst = false;
  bool has_vtg = false;
  bool has_zda = false;
  unsigned long collect_start_time = 0; // 數據組收集開始時間
};

// 全局變量存儲最新有效時間戳
String last_valid_timestamp = "";

// 時間戳容差比較函數 - 允許±1秒的差異
bool isTimestampCompatible(const String& ts1, const String& ts2) {
  if (ts1.length() < 6 || ts2.length() < 6) return false;
  
  // 提取時間部分 HHMMSS
  float time1 = ts1.substring(0, 6).toFloat();
  float time2 = ts2.substring(0, 6).toFloat();
  
  // 計算時間差（考慮秒的小數部分）
  float diff = abs(time1 - time2);
  
  // 處理跨分鐘/小時的情況（例如：235959 vs 000000）
  if (diff > 5000) {  // 可能是跨小時
    diff = min(diff, 10000 - diff);
  }
  if (diff > 50) {    // 可能是跨分鐘
    diff = min(diff, 100 - diff);
  }
  
  // 放寬匹配：允許±0.3秒的容差，支援4Hz數據 (250ms間隔)
  return diff <= 0.3;
}

NMEADataset current_dataset;           // 當前正在收集的數據組
const unsigned long DATASET_TIMEOUT = 2000; // 數據組收集超時時間(ms)
unsigned long complete_datasets_sent = 0;    // 完整數據組發送統計
unsigned long incomplete_datasets = 0;       // 不完整數據組統計

// 平滑 time_offset：使用滑動平均法
const int OFFSET_BUF_SIZE = 5;
int64_t time_offset_buffer[OFFSET_BUF_SIZE] = {0};
int offset_index = 0;
bool offset_buffer_full = false;



// 底下加入自動同步新增進入 loop()
void syncPX4Time() {
  if (millis() - last_sync_millis >= SYNC_INTERVAL) {
    uint64_t px4_time = getPX4Time(false);
    if (px4_time > 0) {
      int64_t new_offset = px4_time - uint64_t(latestXsensTime.ulong_val) * 1e2;

      // 將新 offset 放入緩衝區
      time_offset_buffer[offset_index] = new_offset;
      offset_index++;

      if (offset_index >= OFFSET_BUF_SIZE) {
        offset_index = 0;
        offset_buffer_full = true;
      }

      int count = offset_buffer_full ? OFFSET_BUF_SIZE : offset_index;
      int64_t sum = 0;
      for (int i = 0; i < count; i++) {
        sum += time_offset_buffer[i];
      }
      time_offset = sum / count;

      last_sync_millis = millis();
      Serial.print("[SYNC] Updated smoothed time_offset = ");
      Serial.println(time_offset);
    }
  }
}
void SERCOM2_Handler() {
  Serial_xsens.IrqHandler();
  ISR_flag_xsens = true;
}

void SERCOM3_Handler() {
  NMEA_IN_Serial.IrqHandler();
  ISR_flag_NMEA = true;
}
void readLOCOSYSResponse() {
  unsigned long start_time = millis();
  String response = "";

  while (millis() - start_time < 1000) {  // 最多等待 1 秒
    while (Serial4.available()) {
      char c = Serial4.read();
      response += c;
    }
  }

  // 印出接收到的回覆
  if (response.length() > 0) {
    Serial.println("[GNSS RESPONSE] " + response);
  } else {
    Serial.println("[WARNING] No response from GNSS to $PLSC,VER*61");
  }
}
void setup() {
  delay(2000);
  Serial.begin(115200);  // 與 myUART_init() 一致
   
  Serial.println("=== Arduino2PixhawkV0 Starting ===");
  
  Serial.println("Step 1: Initializing UART...");
  myUART_init();
  Serial.println("Step 1: Complete");

  sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PAIR006*3C", "$PAIR001"); //Perform GNSS cold start
  // sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PAIR003*39", "$PAIR");
  // sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PAIR004*3E", "$PAIR");
  sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PLSC,VER*61", "$PLSR"); //Query firmware version
  sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PLSC,FIXRATE,2*6B", "$PLSR"); //Set update rate 2 Hz
  sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PLSC,FIXRATE,?*66", "$PLSR"); //query update rate
  sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PLSC,UART2,{PLSC,ANTDIST,100}*46", "$PLSR"); //Set the distance between two antennas 
  sendGNSSCommandAndWaitAck(NMEA_IN_Serial, "$PLSC,UART2,{PLSC,ANTDIST,?}*48", "$PLSR"); //query the distance between two antennas 

      
     Serial.println("End of setup ");  
}

void loop() { 

  // printMatchingNMEA(NMEA_IN_Serial, "$GNGGA");
  printMatchingNMEA(NMEA_IN_Serial, "$GPHDT");
  printMatchingNMEA(NMEA_IN_Serial, "$PLSHD");
  //  while (NMEA_IN_Serial.available()) {
  //   char c = NMEA_IN_Serial.read();
    
  //   Serial.write(c);
  // }


/***
  // 除錯：心跳信號，確認程式正在運行
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 5000) {  // 每5秒一次心跳
    Serial.println("[HEARTBEAT] System running, time: " + String(millis()/1000) + "s, Debug: " + (is_debug ? "ON" : "OFF"));
    float filter_ratio = (nmea_received_count > 0) ? (float(nmea_sent_count)/float(nmea_received_count)*100) : 0;
    Serial.println("[NMEA] Received: " + String(nmea_received_count) + 
                   ", Sent: " + String(nmea_sent_count) + 
                   ", Invalid: " + String(nmea_invalid_count) +
                   ", Discarded: " + String(nmea_discarded_count) +
                   ", Success: " + String(filter_ratio, 1) + "%");
    Serial.println("[TYPES] GGA: " + String(gga_sent) + 
                   ", RMC: " + String(rmc_sent) + 
                   ", GST: " + String(gst_sent) +
                   ", GSA: " + String(gsa_sent) +
                   ", VTG: " + String(vtg_sent) +
                   ", ZDA: " + String(zda_sent));
    Serial.println("[XSENS] HDT: " + String(hdt_sent) + 
                   ", PLSHD: " + String(plshd_sent) + 
                   ", GSV: " + String(gsv_sent));
    unsigned long now = millis();
    Serial.println("[TIMING] GGA: " + String((now - last_gga_send)/1000.0, 1) + "s ago" +
                   ", RMC: " + String((now - last_rmc_send)/1000.0, 1) + "s ago" +
                   ", GST: " + String((now - last_gst_send)/1000.0, 1) + "s ago");
    Serial.println("[TIMING2] GSA: " + String((now - last_gsa_send)/1000.0, 1) + "s ago" +
                   ", VTG: " + String((now - last_vtg_send)/1000.0, 1) + "s ago" +
                   ", ZDA: " + String((now - last_zda_send)/1000.0, 1) + "s ago");
    Serial.println("[SYNC] Complete datasets: " + String(complete_datasets_sent) +
                   ", Incomplete: " + String(incomplete_datasets) +
                   ", Current timestamp: " + current_dataset.timestamp +
                   ", GSA collected: " + String(current_dataset.gsa_count) + "/4");
    Serial.println("[BUFFER] Size: " + String(nmea_input_buffer.length()) + 
                   ", Preview: " + nmea_input_buffer.substring(0, min(30, (int)nmea_input_buffer.length())));
    last_heartbeat = millis();
  }
  
  // Security monitoring for UAV/Satellite applications (silent mode)
  if (security_mode_active && (millis() - last_security_check > 30000)) {
    // Perform automated security check every 30 seconds (silent to prevent info leak)
    performSecurityCheck(Serial, true);
  }
  
  // Check GPS connection status (every 10 seconds)
  static unsigned long last_gps_check = 0;
  if (millis() - last_gps_check > 10000) {
    if (gps_connected && (millis() - last_gps_data_time > 5000)) {
      gps_connected = false;
      Serial.println("[GPS] Connection lost - no data for 5 seconds");
      if (security_mode_active) {
        Serial.println("[SECURITY] GPS disconnection detected during security monitoring");
      }
    }
    last_gps_check = millis();
  }

  if (ISR_flag_xsens && is_run){
    readXsens();
    ISR_flag_xsens = false;
  }

  if (ISR_flag_NMEA){
    if (current_output_mode == OUT_MODE_NMEA){
      printNMEAWithModifiedTimestampLocal(NMEA_IN_Serial, PIXHAWK_SERIAL, false);  
    }
    else{
      printNMEAWithModifiedTimestampLocal(NMEA_IN_Serial, NMEA_OUT_Serial, current_output_mode == OUT_MODE_GNSS);
    }
    
    ISR_flag_NMEA = false;
    
  }

  // 發送 XSENS 專用 NMEA 數據 (HDT, PLSHD, GSV)
  if (is_run && (current_output_mode == OUT_MODE_BIN || current_output_mode == OUT_MODE_GNSS)) {
    sendXSENSNMEAData(NMEA_OUT_Serial);
  }

  // 除錯：監控串列埠狀態
  static unsigned long last_serial_check = 0;
  if (millis() - last_serial_check > 2000) {  // 每2秒檢查一次
    if (Serial.available()) {
      Serial.println("[DEBUG] Serial data available: " + String(Serial.available()) + " bytes");
    }
    last_serial_check = millis();
  }
  
  if (Serial.available()){ 
    Serial.println("[DEBUG] Processing serial input...");
    
    if (USB_Setting_mode && (current_output_mode == OUT_MODE_XBUS)){
      Serial.println("[DEBUG] Using XBUS mode");
      checkXBUS_CMD(Serial);
    }
    
    else{
      Serial.println("[DEBUG] Using STR_CMD mode");
      // 除錯：確認指令接收
      String received_cmd = readCurrentBytes(Serial);
      Serial.println("[DEBUG] Read result: '" + received_cmd + "' (length: " + String(received_cmd.length()) + ")");
      
      if (received_cmd.length() > 0) {
        Serial.println("[DEBUG] Calling checkSTR_CMD...");
        checkSTR_CMD(received_cmd);
      } else {
        Serial.println("[DEBUG] Empty command received");
      }
    }
  }

  // DEBUG: 強制測試Trust System（無論融合模式是否啟用）
  static unsigned long last_force_test = 0;
  if (millis() - last_force_test > 5000) {
    send2Serial(PIXHAWK_SERIAL, "[FORCE_TEST] Testing YAW Trust calculation manually");
    
    // 創建測試資料
    my_data_3f test_omg;
    test_omg.float_val[0] = 0.1f;
    test_omg.float_val[1] = -0.2f;
    test_omg.float_val[2] = 0.05f;  // 測試YAW角速度
    
    // 手動更新YAW歷史
    dynamic_trust.yaw_rate_history[dynamic_trust.yaw_history_index] = test_omg.float_val[2];
    dynamic_trust.yaw_history_index = (dynamic_trust.yaw_history_index + 1) % 5;
    
    // 強制計算YAW Trust Factor
    send2Serial(PIXHAWK_SERIAL, "[FORCE_TEST] About to call calculateYawTrustFactor()");
    float force_result = calculateYawTrustFactor();
    send2Serial(PIXHAWK_SERIAL, "[FORCE_TEST] Result: " + String(force_result, 6) + 
                               ", Global stability: " + String(dynamic_trust.yaw_stability_score, 6));
    
    last_force_test = millis();
  }
  
  // 融合模組處理與 MAVLink 發送 (FUSION模式專用路徑)
  if (fusion_mode_enabled && is_run) {
    static unsigned long last_fusion_send = 0;
    static unsigned long last_fusion_debug = 0;
    
    // DEBUG: FUSION模式MAVLink發送狀態
    if (millis() - last_fusion_debug > 3000) {
      send2Serial(PIXHAWK_SERIAL, "[FUSION_MAVLINK] FUSION Mode Active - Using fusion_module.sendMAVLinkOdometry()");
      send2Serial(PIXHAWK_SERIAL, "[FUSION_MAVLINK] Current output mode: " + String(current_output_mode) + ", Xsens mode: " + String(current_Xsens_mode));
      last_fusion_debug = millis();
    }
    
    // 脫鉤機制：數據更新與傳送分離
    static bool data_updated = false;
    
    // 當有新數據時更新融合數據 (不定頻)
    if (fusion_module.getFusionStatus().fusion_active) {
      fusion_module.updateOdomData();  // 更新最新融合數據
      data_updated = true;
    }
    
    // 固定頻率傳送 ODOMETRY (100Hz = 10ms)
    if (millis() - last_fusion_send >= 11) {
      uint64_t raw_timestamp = getUnifiedTimestamp(latestXsensTime);
      uint64_t smooth_timestamp = smoothTimestamp(raw_timestamp);
      
      // 使用非阻塞智慧傳送
      if (fusion_module.smartSendOdometry(PIXHAWK_SERIAL, smooth_timestamp)) {
        // 僅在成功傳送時才發送 GPS 相關訊息 (避免過載)
        if (data_updated) {
          fusion_module.sendMAVLink_GPS_RAW_INT(PIXHAWK_SERIAL, smooth_timestamp);
          fusion_module.sendMAVLink_GPS_STATUS(PIXHAWK_SERIAL);
          fusion_module.sendMAVLink_GPS_INPUT(PIXHAWK_SERIAL, smooth_timestamp);
          data_updated = false;  // 重置更新標記
        }
      }
      last_fusion_send = millis();
    }
    
    // 融合狀態報告和傳送統計 (每 5 秒一次) - 降低頻率減少IDE輸出
    static unsigned long last_fusion_report = 0;
    if (is_debug && millis() - last_fusion_report >= 5000) {
      FusionStatus status = fusion_module.getFusionStatus();
      Serial.println("[FUSION] Active: " + String(status.fusion_active ? "YES" : "NO") + 
                     ", GNSS: " + String(status.gnss_valid ? "OK" : "FAIL") + 
                     ", MTDATA2: " + String(status.mtdata2_valid ? "OK" : "FAIL") + 
                     ", MAVLink: " + String(status.mavlink_send_count));
      
      // 顯示傳送統計
      fusion_module.printTransmissionStats();
      
      last_fusion_report = millis();
    }
  }

  if (PIXHAWK_SERIAL.available() && enable_input){
    if (current_output_mode == OUT_MODE_BIN) {
      mavlink_message_t msg;
      mavlink_status_t status;
      while (PIXHAWK_SERIAL.available()) {
        uint8_t c = PIXHAWK_SERIAL.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          enable_input = false; 

          current_output_mode = OUT_MODE_ML_ODOM;
          current_Xsens_mode = MODE_INS_PAV_QUT;
          send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_PAV_QUT...");
          send2Serial(PIXHAWK_SERIAL, "Set MAVLINK ODOMETRY(331) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_GPS_RAW;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_RAW_INT(24) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_GPS_IN;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_INPUT(232) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_VISO;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION POSITION ESTIMATOR(102) Output...");

          // current_Xsens_mode = MODE_INS_UTC;
          // current_output_mode = OUT_MODE_NMEA;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_UTC...");
          // send2Serial(PIXHAWK_SERIAL, "Set NMEA Output...");

          // current_Xsens_mode = MODE_INS_PAV_QUT;
          // current_output_mode = OUT_MODE_VEC;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to MODE_INS_PAV_QUT...");
          // send2Serial(PIXHAWK_SERIAL, "Set Vector BIN Output...");

          setXsensPackage();
          (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
          break;
        }
      }
    }
    
    else if (current_output_mode == OUT_MODE_XBUS){
      uint8_t buffer[LEN_XBUS];
      uint16_t idx = 0;
      while (PIXHAWK_SERIAL.available() && idx < LEN_XBUS) {
        buffer[idx++] = PIXHAWK_SERIAL.read();
      }

      if (idx > 0) { 
        if (idx >= 6){
          if (buffer[0] == 'C' && buffer[1] == 'O' && buffer[2] == 'N' && buffer[3] == 'F' && buffer[4] == 'I' && buffer[5] == 'G'){
            current_output_mode = OUT_MODE_CONFIG;
            send2Serial(PIXHAWK_SERIAL, "Enter CONFIG mode");
          }
        }
        if (current_output_mode == OUT_MODE_XBUS) { Serial_xsens.write(buffer, idx); }  
      }
    }

    else{ 
      // checkSTR_CMD(PIXHAWK_SERIAL.readString()); 
      checkSTR_CMD(readCurrentBytes(PIXHAWK_SERIAL));
    }
  }
     */
}

void readXsens(){
  if (Serial_xsens.available()){
    if (current_output_mode == OUT_MODE_XBUS){
      uint8_t buffer[LEN_XBUS];
      uint16_t idx = 0;
      while (Serial_xsens.available() && idx < LEN_XBUS) {
        buffer[idx++] = Serial_xsens.read();
      }
      if (idx > 0) { 
        if (USB_Setting_mode) { Serial.write(buffer, idx); }
        else { PIXHAWK_SERIAL.write(buffer, idx); }
      }
    }

    else{
      my_data_3f omg, acc, ori, mag, vel;
      my_data_4f qut;
      my_data_u4 XsensTime, temp, pressure, hei, status;
      my_data_2d latlon;
      my_data_u2 xsens_counter;
      UTC_TIME utc_time;

      latlon.float_val[0] = 0;
      latlon.float_val[1] = 0;
      hei.float_val = 0;
      vel.float_val[0] = 0;
      vel.float_val[1] = 0;
      vel.float_val[2] = 0;
        
      xsens.getMeasures(MTDATA2);
      xsens.parseData(&xsens_counter, &XsensTime, &omg, &acc, &mag, &pressure, &vel, &latlon, &hei, &ori, &qut, &status, &temp);
      
      if (xsens.getDataStatus() == DATA_OK) {
        // 更新全域 orientation 數據
        latest_orientation.float_val[0] = ori.float_val[0]; // pitch
        latest_orientation.float_val[1] = ori.float_val[1]; // roll
        
        // 融合模組處理 MTDATA2 資料
        if (fusion_mode_enabled) {
          fusion_module.updateMTData2(qut, omg, acc, ori, XsensTime.ulong_val);
          
          // DEBUG: 確認融合模式狀態
          static unsigned long last_fusion_debug = 0;
          if (millis() - last_fusion_debug > 5000) {
            send2Serial(PIXHAWK_SERIAL, "[FUSION_DEBUG] Fusion mode enabled, processing MTDATA2");
            last_fusion_debug = millis();
          }
          
          // 改進：先清理資料，再進行寬鬆驗證
          // 備份原始資料，以防修改影響其他模組
          my_data_4f qut_clean = qut;
          my_data_3f omg_clean = omg;
          my_data_3f acc_clean = acc;
          
          // 使用資料清理函數
          cleanSensorData(qut_clean, omg_clean, acc_clean);
          
          // 簡化驗證：只檢查基本有效性
          bool sensor_data_valid = true;
          
          // 1. 檢查清理後的四元數範數（應該已經正規化）
          float quat_norm = sqrt(qut_clean.float_val[0]*qut_clean.float_val[0] + 
                                qut_clean.float_val[1]*qut_clean.float_val[1] + 
                                qut_clean.float_val[2]*qut_clean.float_val[2] + 
                                qut_clean.float_val[3]*qut_clean.float_val[3]);
          
          // 放寬驗證條件，清理後的資料應該已經很穩定
          if (quat_norm < 0.8f || quat_norm > 1.2f) {
            sensor_data_valid = false;
            static unsigned long last_quat_debug = 0;
            if (millis() - last_quat_debug > 5000) {  // 減少debug訊息頻率
              send2Serial(PIXHAWK_SERIAL, "[VALIDATION] Cleaned quaternion still invalid: norm=" + String(quat_norm, 6));
              last_quat_debug = millis();
            }
          }
          
          // 2. 檢查清理後的角速度和加速度（應該已經無NaN）
          // 簡化檢查：只确認清理過程沒有失敗
          for (int i = 0; i < 3; i++) {
            if (!isValidFloat(omg_clean.float_val[i]) || !isValidFloat(acc_clean.float_val[i])) {
              sensor_data_valid = false;
              static unsigned long last_clean_fail = 0;
              if (millis() - last_clean_fail > 5000) {
                send2Serial(PIXHAWK_SERIAL, "[VALIDATION] Data cleaning failed at index " + String(i));
                last_clean_fail = millis();
              }
              break;
            }
          }
          
          // 3. 檢查清理後的角速度幅值（應該已經被限制在合理範圍）
          float omg_magnitude = sqrt(omg_clean.float_val[0]*omg_clean.float_val[0] + 
                                    omg_clean.float_val[1]*omg_clean.float_val[1] + 
                                    omg_clean.float_val[2]*omg_clean.float_val[2]);
          // 放寬檢查，因為已經清理過
          if (omg_magnitude > 25.0f) {  // 25 rad/s ≈ 1432°/s 非常寬鬆
            sensor_data_valid = false;
            static unsigned long last_omg_debug = 0;
            if (millis() - last_omg_debug > 5000) {
              send2Serial(PIXHAWK_SERIAL, "[VALIDATION] Cleaned angular rate still too high: mag=" + String(omg_magnitude, 6));
              last_omg_debug = millis();
            }
          }
          
          // 4. 檢查清理後的加速度幅值（非常寬鬆）
          float acc_magnitude = sqrt(acc_clean.float_val[0]*acc_clean.float_val[0] + 
                                    acc_clean.float_val[1]*acc_clean.float_val[1] + 
                                    acc_clean.float_val[2]*acc_clean.float_val[2]);
          // 寬鬆檢查，包括靜態和動態情況
          if (acc_magnitude < 1.0f || acc_magnitude > 100.0f) {  // 非常寬鬆範圍
            sensor_data_valid = false;
            static unsigned long last_acc_debug = 0;
            if (millis() - last_acc_debug > 5000) {
              send2Serial(PIXHAWK_SERIAL, "[VALIDATION] Cleaned acceleration out of range: mag=" + String(acc_magnitude, 6));
              last_acc_debug = millis();
            }
          }
          
          // 使用清理後的資料更新 Trust System
          static unsigned long last_fusion_trust_debug = 0;
          if (millis() - last_fusion_trust_debug > 3000) {
            send2Serial(PIXHAWK_SERIAL, "[FUSION_TRUST] sensor_data_valid: " + String(sensor_data_valid ? "TRUE" : "FALSE"));
            if (sensor_data_valid) {
              send2Serial(PIXHAWK_SERIAL, "[FUSION_TRUST] About to call updateMovementDetection() from fusion");
            }
            last_fusion_trust_debug = millis();
          }
          
          if (sensor_data_valid) {
            updateMovementDetection(latlon, hei, vel, omg_clean, acc_clean);
            
            // DEBUG: 統計成功更新和強制YAW計算
            static unsigned long trust_success_count = 0;
            static unsigned long last_success_report = 0;
            trust_success_count++;
            if (millis() - last_success_report > 5000) {
              send2Serial(PIXHAWK_SERIAL, "[TRUST] Successfully updated " + String(trust_success_count) + " times");
              
              // 強制計算YAW Trust Factor來測試
              float test_yaw_trust = calculateYawTrustFactor();
              send2Serial(PIXHAWK_SERIAL, "[TRUST] Manual YAW calculation result: " + String(test_yaw_trust, 6));
              
              last_success_report = millis();
            }
          } else {
            static unsigned long last_invalid_warning = 0;
            if (millis() - last_invalid_warning > 5000) {  // 減少警告頻率
              send2Serial(PIXHAWK_SERIAL, "[TRUST] Sensor data still invalid after cleaning");
              last_invalid_warning = millis();
            }
          }
        } else {
          // DEBUG: 融合模式未啟用時的診斷
          static unsigned long last_no_fusion_debug = 0;
          if (millis() - last_no_fusion_debug > 10000) {
            send2Serial(PIXHAWK_SERIAL, "[FUSION_DEBUG] Fusion mode DISABLED - Trust System not running");
            send2Serial(PIXHAWK_SERIAL, "[FUSION_DEBUG] Use FUSION_ON command to enable");
            
            // 在非融合模式下也可以手動測試YAW計算
            send2Serial(PIXHAWK_SERIAL, "[FUSION_DEBUG] Manual YAW test - updating history with current omg[2]: " + String(omg.float_val[2], 6));
            
            // 手動更新YAW歷史
            dynamic_trust.yaw_rate_history[dynamic_trust.yaw_history_index] = omg.float_val[2];
            dynamic_trust.yaw_history_index = (dynamic_trust.yaw_history_index + 1) % 5;
            
            // 手動計算YAW Trust Factor
            float manual_yaw_trust = calculateYawTrustFactor();
            send2Serial(PIXHAWK_SERIAL, "[FUSION_DEBUG] Manual YAW calculation: " + String(manual_yaw_trust, 6));
            
            last_no_fusion_debug = millis();
          }
        }
        latest_orientation.float_val[2] = ori.float_val[2]; // yaw
        orientation_data_available = true;
        
        // Validate sensor data before processing
        bool data_valid = true;
        
        // Check timestamp monotonicity
        if (XsensTime.ulong_val <= xsens_pre_time && xsens_pre_time != 0) {
          // Serial.println("Warning: Non-monotonic timestamp detected");
          data_valid = false;
        }
        
        // Validate angular velocity (reasonable range: -10 to +10 rad/s)
        for (int i = 0; i < 3; i++) {
          if (abs(omg.float_val[i]) > 10.0) {
            // Serial.print("Warning: Excessive angular velocity detected: ");
            Serial.println(omg.float_val[i]);
            data_valid = false;
          }
        }
        
        // Validate quaternion norm (should be close to 1.0)
        float quat_norm = sqrt(qut.float_val[0]*qut.float_val[0] + 
                              qut.float_val[1]*qut.float_val[1] + 
                              qut.float_val[2]*qut.float_val[2] + 
                              qut.float_val[3]*qut.float_val[3]);
        if (abs(quat_norm - 1.0) > 0.1) {
          // Serial.print("Warning: Invalid quaternion norm: ");
          static unsigned long last_quat_error_time = 0;
          if (millis() - last_quat_error_time >= 5000) { // 5秒輸出一次 - 降低IDE輸出頻率
            Serial.println(quat_norm);
            last_quat_error_time = millis();
          }
          data_valid = false;
        }
        
        if (!data_valid) {
          static unsigned long last_error_time = 0;
          if (millis() - last_error_time >= 5000) { // 5秒輸出一次 - 降低IDE輸出頻率
            Serial.println("Skipping invalid sensor data");
            last_error_time = millis();
          }
          return;
        }
        
        // 更新最新的 Xsens 時間用於同步
        latestXsensTime = XsensTime;
        
        // 時間同步初始化 - 加入更完整的檢查
        if (!time_sync_initialized) {
          uint64_t px4_time = getPX4Time(false);
          if (px4_time > 0 && px4_time > XsensTime.ulong_val * 1e2) {
            time_offset = px4_time - XsensTime.ulong_val * 1e2;
            time_sync_initialized = true;
            Serial.print("[INIT] Time sync initialized with offset: ");
            Serial.println((long long)time_offset);
            
            // Initialize the offset buffer
            for (int i = 0; i < OFFSET_BUF_SIZE; i++) {
              time_offset_buffer[i] = time_offset;
            }
            offset_buffer_full = true;
          }
        }
        
        char buffer[128];
        int index = 0;
        
        // MAVLink output - 只在非FUSION模式下處理，避免雙重發送
        // DEBUG: 檢查MAVLink輸出模式
        static unsigned long last_mavlink_debug = 0;
        if (millis() - last_mavlink_debug > 5000) {
          send2Serial(PIXHAWK_SERIAL, "[MAVLINK_DEBUG] Output mode: " + String(current_output_mode) + ", Xsens mode: " + String(current_Xsens_mode));
          send2Serial(PIXHAWK_SERIAL, "[MAVLINK_DEBUG] Expected: OUT_MODE_ML_ODOM=" + String(OUT_MODE_ML_ODOM) + ", MODE_INS_PAV_QUT=" + String(MODE_INS_PAV_QUT));
          send2Serial(PIXHAWK_SERIAL, "[MAVLINK_DEBUG] FUSION mode: " + String(fusion_mode_enabled ? "ENABLED" : "DISABLED"));
          last_mavlink_debug = millis();
        }
        
        // 只在非FUSION模式下使用標準MAVLink發送
        if (!fusion_mode_enabled && current_output_mode == OUT_MODE_ML_ODOM && current_Xsens_mode == MODE_INS_PAV_QUT){
          static unsigned long last_call_debug = 0;
          if (millis() - last_call_debug > 3000) {
            send2Serial(PIXHAWK_SERIAL, "[MAVLINK_DEBUG] Calling sendMAVLink_Odometry() (Standard Mode) - Trust System should run");
            last_call_debug = millis();
          }
          sendMAVLink_Odometry(XsensTime, latlon, hei, vel, qut, omg, status);
        }
        else if (!fusion_mode_enabled && current_output_mode == OUT_MODE_ML_GPS_RAW && current_Xsens_mode == MODE_INS){
          sendMAVLink_GPS_RAW_INT(XsensTime, latlon, hei, vel, ori, omg, status);
        }
        else if (!fusion_mode_enabled && current_output_mode == OUT_MODE_ML_GPS_IN && current_Xsens_mode == MODE_INS){
          sendMAVLink_GPS_INPUT(XsensTime, latlon, hei, vel, ori, omg, status);
        }
        else if (!fusion_mode_enabled && current_output_mode == OUT_MODE_ML_VISO && current_Xsens_mode == MODE_INS){
          sendMAVLink_VISION_POSITION(XsensTime, latlon, hei, vel, ori, omg, status);
        }
 
        else if (current_output_mode == OUT_MODE_STR) {
          if (current_Xsens_mode == MODE_INS_PAV_QUT){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, latlon.float_val, 2, 7);
            index = appendValue2Str(buffer, 128, index, hei.float_val, 3);
            index = appendValues2Str(buffer, 128, index, vel.float_val, 3, 2);
            index = appendValues2Str(buffer, 128, index, qut.float_val, 4, 4);
            xsens.getFilterStatus(status, true, PIXHAWK_SERIAL);
            PIXHAWK_SERIAL.println(buffer);
          }
          else if (current_Xsens_mode == MODE_INS){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, latlon.float_val, 2, 7);
            index = appendValue2Str(buffer, 128, index, hei.float_val, 3);
            index = appendValues2Str(buffer, 128, index, vel.float_val, 3, 2);
            index = appendValues2Str(buffer, 128, index, ori.float_val, 3, 2);
            xsens.getFilterStatus(status, true, PIXHAWK_SERIAL);
            PIXHAWK_SERIAL.println(buffer);
          } 
          else if (current_Xsens_mode == MODE_AHRS){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, omg.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, acc.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, mag.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, ori.float_val, 3, 2);
            PIXHAWK_SERIAL.println(buffer);
          } 
          else if (current_Xsens_mode == MODE_IMU){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, omg.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, acc.float_val, 3, 4);
            index = appendValue2Str(buffer, 128, index, temp.float_val, 1);
            PIXHAWK_SERIAL.println(buffer);
          }
          else { Serial.println("String output doesn't supportan this mode yet!"); }
        }
        
        else if (current_output_mode == OUT_MODE_BIN) {
          my_data_u4 output_time;
          uint8_t new_temp_bin[4];
          output_time.ulong_val = XsensTime.ulong_val * 0.1;
          for (int i=0;i<4;i++){ new_temp_bin[i] = temp.bin_val[3-i]; }  // inverse the order of temperature bytes

          if (current_Xsens_mode == MODE_AHRS){
            uint8_t buffer[52];
            memcpy(buffer, output_header, 4);
            memcpy(buffer + 4, omg.bin_val, 12);
            memcpy(buffer + 16, acc.bin_val, 12);
            memcpy(buffer + 28, new_temp_bin, 4);  // MSB
            memcpy(buffer + 32, output_time.bin_val, 4);
            memcpy(buffer + 36, ori.bin_val, 12);
            MyCRC().calCRC(buffer, 52);
            PIXHAWK_SERIAL.write(buffer, 52);
          }
          else if (current_Xsens_mode == MODE_INS){
            uint8_t buffer[76];
            memcpy(buffer, output_header, 4);
            memcpy(buffer + 4, omg.bin_val, 12);
            memcpy(buffer + 16, acc.bin_val, 12);
            memcpy(buffer + 28, new_temp_bin, 4);  // MSB
            memcpy(buffer + 32, output_time.bin_val, 4);
            memcpy(buffer + 36, ori.bin_val, 12);
            memcpy(buffer + 48, latlon.bin_val, 8);
            memcpy(buffer + 56, hei.bin_val, 4);
            memcpy(buffer + 60, vel.bin_val, 12);
            MyCRC().calCRC(buffer, 76);
            PIXHAWK_SERIAL.write(buffer, 76);
          }
          else {
            PIXHAWK_SERIAL.println("Binary output doesn't supported this mode yet!");
          }
        }

        else if (current_output_mode == OUT_MODE_AR1AFC) {
          if (current_Xsens_mode == MODE_AHRS){
            // AR-1A-FC 專用格式 - 52字節輸出
            uint8_t buffer[52];
            my_data_u4 output_time;
            uint8_t new_temp_bin[4];
            output_time.ulong_val = XsensTime.ulong_val * 0.1;
            for (int i=0;i<4;i++){ new_temp_bin[i] = temp.bin_val[3-i]; }  // inverse the order of temperature bytes

            // AR-1A-FC 格式封包結構 (52字節)
            memcpy(buffer, output_header, 4);           // Header: 0xFE 0x81 0xFF 0x55
            memcpy(buffer + 4, omg.bin_val, 12);        // X/Y/Z-axis Rotational Data (DPS)
            memcpy(buffer + 16, acc.bin_val, 12);       // X/Y/Z-axis Acceleration Data (g)
            memcpy(buffer + 28, new_temp_bin, 4);       // Temperature (°C)
            memcpy(buffer + 32, output_time.bin_val, 4); // Time Counter (millisecond)
            memcpy(buffer + 36, ori.bin_val, 12);       // Attitude Pitch/Roll/Yaw (deg)
            MyCRC().calCRC(buffer, 52);                 // CRC-32 calculation
            PIXHAWK_SERIAL.write(buffer, 52);
          }
          else {
            PIXHAWK_SERIAL.println("AR-1A-FC output only supports AHRS mode!");
          }
        }

        else if (current_output_mode == OUT_MODE_VEC && current_Xsens_mode == MODE_INS_PAV_QUT){
          // length of header = 10 bytes
          // length of payload = 102 bytes
          // length of CRC = 2 bytes
          uint8_t message[10+102+2] = {
            0xFA,         // Sync header
            0x36,         // TimeGroup (1), ImuGroup (2), AttitudeGroup (4), InsGroup (5) -> 0b00110110
            0x00, 0x01,   // TimeGroup:     TimeStartup (0) -> 0x00, 0b00000001
            0x01, 0x30,   // ImuGroup:      Temp (4), Pres (5), MAG (8) -> 0b00000001, 0b00110000
            0x00, 0x84,   // AttitudeGroup: Quaternion (2), LinearAccelNed (7) -> 0x00, 0b10000100
            0x06, 0x13,   // InsGroup:      InsStatus (0), PosLla (1), VelNed (4), PosU (9), VelU (10) -> 0b00000110, 0b00010011
          };
          my_data_u8 time_nano;
          time_nano.ulong_val = XsensTime.ulong_val * 1e2;
          
          uint8_t ins_state[2];
          xsens.getINSStatus(status, ins_state);

          my_data_u8 lat, lon, alt;
          lat.ulong_val = latlon.float_val[0];
          lon.ulong_val = latlon.float_val[1];
          alt.ulong_val = hei.float_val;

          my_data_3f pos_u;
          pos_u.float_val[0] = 1.0f;
          pos_u.float_val[1] = 1.0f;
          pos_u.float_val[2] = 1.0f;

          my_data_u4 vel_u;
          vel_u.float_val = 0.05;

          uint8_t *ptr = message + 10;
          write_big_endian(ptr, time_nano.bin_val, 8);  ptr += 8;
          write_big_endian(ptr, temp.bin_val, 4);       ptr += 4;
          write_big_endian(ptr, pressure.bin_val, 4);   ptr += 4;
          write_big_endian(ptr, mag.bin_val, 12);       ptr += 12;
          write_big_endian(ptr, qut.bin_val, 16);       ptr += 16;
          memset(ptr, 0, 12);                           ptr += 12;  // LinearAccelNed
          write_big_endian(ptr, temp.bin_val, 4);       ptr += 4;
          memcpy(ptr, ins_state, 2);                    ptr += 2;
          write_big_endian(ptr, lat.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, lon.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, alt.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, vel.bin_val, 12);       ptr += 12;
          write_big_endian(ptr, pos_u.bin_val, 12);     ptr += 12;
          write_big_endian(ptr, vel_u.bin_val, 4);      ptr += 4;
          uint16_t crc = calculateCRC(message, 10 + 102);
          *(ptr++) = (crc >> 8) & 0xFF;
          *(ptr++) = crc & 0xFF;
          PIXHAWK_SERIAL.write(message, 10+102+2);
        }
        else {
          // DEBUG: MAVLink模式不符或FUSION模式已啟用
          static unsigned long last_mode_debug = 0;
          if (millis() - last_mode_debug > 5000) {
            if (fusion_mode_enabled) {
              send2Serial(PIXHAWK_SERIAL, "[MAVLINK_DEBUG] FUSION Mode Active - Standard MAVLink disabled, using fusion_module instead");
            } else {
              send2Serial(PIXHAWK_SERIAL, "[MAVLINK_DEBUG] Current mode doesn't match MAVLink ODOM - Trust System not running");
              send2Serial(PIXHAWK_SERIAL, "[MAVLINK_DEBUG] Current: mode=" + String(current_output_mode) + ", xsens_mode=" + String(current_Xsens_mode));
            }
            last_mode_debug = millis();
          }
        }

        // 統一 MAVLink 發送頻率控制 - 改善數據流暢性
        if (XsensTime.ulong_val - xsens_pre_time >= MAVLINK_SEND_INTERVAL || xsens_pre_time == 0) {  // 現在是 100Hz
          xsens_pre_time = XsensTime.ulong_val;
          if (current_output_mode != OUT_MODE_CONFIG){
            Serial.println(XsensTime.ulong_val * 1e-4, 3);
            // xsens.getFilterStatus(status, true, Serial);
          }
          else{
            send2Serial(PIXHAWK_SERIAL, "CONFIG");
          }
        }
      }
      else if(xsens.getDataStatus() == DATA_WARNING){
        // AR-1A-FC 模式在 WARNING 狀態下仍可正常工作
        if (current_output_mode == OUT_MODE_AR1AFC && current_Xsens_mode == MODE_AHRS) {
          // AR-1A-FC 專用格式 - 52字節輸出
          uint8_t buffer[52];
          my_data_u4 output_time;
          uint8_t new_temp_bin[4];
          output_time.ulong_val = XsensTime.ulong_val * 0.1;
          for (int i=0;i<4;i++){ new_temp_bin[i] = temp.bin_val[3-i]; }  // inverse the order of temperature bytes

          // AR-1A-FC 格式封包結構 (52字節)
          memcpy(buffer, output_header, 4);           // Header: 0xFE 0x81 0xFF 0x55
          memcpy(buffer + 4, omg.bin_val, 12);        // X/Y/Z-axis Rotational Data (DPS)
          memcpy(buffer + 16, acc.bin_val, 12);       // X/Y/Z-axis Acceleration Data (g)
          memcpy(buffer + 28, new_temp_bin, 4);       // Temperature (°C)
          memcpy(buffer + 32, output_time.bin_val, 4); // Time Counter (millisecond)
          memcpy(buffer + 36, ori.bin_val, 12);       // Attitude Pitch/Roll/Yaw (deg)
          MyCRC().calCRC(buffer, 52);                 // CRC-32 calculation
          PIXHAWK_SERIAL.write(buffer, 52);
        } else {
          // 其他模式下顯示警告信息
          // Serial.print("Warning: ");
          // xsens.PrintMessage();
          PIXHAWK_SERIAL.print("Warning: ");
          xsens.PrintMessage(PIXHAWK_SERIAL);
        }
      }
    }
  }
}

void sendMAVLink_Odometry(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_4f &qut, const my_data_3f &omg, const my_data_u4 &status){
    
    // 時間分析 - 標準模式詳細步驟追蹤
    static unsigned long last_std_timing_report = 0;
    bool should_report_std_timing = is_debug && (millis() - last_std_timing_report > 5000);
    
    if (should_report_std_timing) {
        Serial.println("=== [STANDARD_TIMING] sendMAVLink_Odometry ===");
    }
    
    unsigned long t0 = micros();
    
    mavlink_odometry_t odom = {};
    uint64_t raw_timestamp = getUnifiedTimestamp(XsensTime);
    odom.time_usec = smoothTimestamp(raw_timestamp);
    
    odom.frame_id = MAV_FRAME_GLOBAL;
    odom.child_frame_id = MAV_FRAME_LOCAL_NED;
    
    // 為動態權重系統創建假的加速度數據（在實際應用中應從感測器獲取）
    my_data_3f acc_dummy;
    acc_dummy.float_val[0] = 0.0f; // 這裡應該是真實的加速度數據
    acc_dummy.float_val[1] = 0.0f;
    acc_dummy.float_val[2] = 9.81f;
    unsigned long t1 = micros();
    
    // 更新運動檢測系統
    updateMovementDetection(latlon, hei, vel, omg, acc_dummy);
    
    // 計算動態信任因子
    float trust_factor = calculateTrustFactor();
    unsigned long t2 = micros();
    
    if (xsens.getFilterStatus(status) > 0){
      // Correct coordinate mapping: x=latitude, y=longitude, z=height
      odom.x = latlon.float_val[0];  // latitude
      odom.y = latlon.float_val[1];  // longitude
      odom.z = hei.float_val;
      odom.vx = vel.float_val[0];
      odom.vy = vel.float_val[1];
      odom.vz = vel.float_val[2];
    } else{
      // When filter status is poor, use zero values and reduce trust
      trust_factor *= 0.1f; // 大幅降低信任度
      odom.x = 0;
      odom.y = 0;
      odom.z = 0;
      odom.vx = 0;
      odom.vy = 0;
      odom.vz = 0;
    } 

    odom.q[0] = qut.float_val[0];
    odom.q[1] = qut.float_val[1];
    odom.q[2] = qut.float_val[2];
    odom.q[3] = qut.float_val[3];
    odom.rollspeed = omg.float_val[0];
    odom.pitchspeed = omg.float_val[1];
    odom.yawspeed = omg.float_val[2];
    unsigned long t3 = micros();

    // 使用動態權重系統設定協方差 (可能的時間消耗大戶)
    adaptiveCovariance(odom, trust_factor);
    unsigned long t4 = micros();
    
    odom.reset_counter = 0;
    odom.estimator_type = MAV_ESTIMATOR_TYPE_VIO; // 更改為視覺慣性里程計類型，EKF更信任
    unsigned long t5 = micros();
    
    // 詳細調試輸出 - 與FUSION模式對比
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 1000 && is_debug) { // 每秒輸出一次
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] =============== MAVLink ODO Packet ===============");
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Mode: STANDARD, Trust: " + String(trust_factor, 3) + ", Movement: " + String(dynamic_trust.movement_score, 3));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Frame: " + String(odom.frame_id) + " Child: " + String(odom.child_frame_id));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Position (XSENS): X=" + String(odom.x, 6) + " Y=" + String(odom.y, 6) + " Z=" + String(odom.z, 2));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Velocity (XSENS): VX=" + String(odom.vx, 3) + " VY=" + String(odom.vy, 3) + " VZ=" + String(odom.vz, 3));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Quaternion (XSENS): Q0=" + String(odom.q[0], 3) + " Q1=" + String(odom.q[1], 3) + " Q2=" + String(odom.q[2], 3) + " Q3=" + String(odom.q[3], 3));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Angular Vel (XSENS): RX=" + String(odom.rollspeed, 3) + " RY=" + String(odom.pitchspeed, 3) + " RZ=" + String(odom.yawspeed, 3));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Covariance: Pos=" + String(odom.pose_covariance[0], 9) + " Vel=" + String(odom.velocity_covariance[0], 9));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] Estimator Type: " + String(odom.estimator_type) + " Reset Counter: " + String(odom.reset_counter));
      send2Serial(PIXHAWK_SERIAL, "[STANDARD] ================================================");
      last_debug = millis();
    }

    // MAVLink 編碼
    mavlink_message_t msg;
    mavlink_msg_odometry_encode(1, 200, &msg, &odom);
    unsigned long t6 = micros();
    
    // Buffer 準備
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    unsigned long t7 = micros();
    
    static unsigned long last_debug_odom = 0;
    if (millis() - last_debug_odom >= 5000) {
      Serial.print("[MAVLINK] ODOMETRY Packet len = ");
      Serial.println(len);
      last_debug_odom = millis();
    }
    
    // 實際傳送
    PIXHAWK_SERIAL.write(buffer, len);
    unsigned long t8 = micros();
    
    // 詳細時間報告 (每5秒一次)
    if (should_report_std_timing) {
        Serial.print("[STD_PERF] DataPrep="); Serial.print(t1 - t0);
        Serial.print("μs | Trust="); Serial.print(t2 - t1);
        Serial.print("μs | Fill="); Serial.print(t3 - t2);
        Serial.print("μs | Cov="); Serial.print(t4 - t3);
        Serial.print("μs | Attr="); Serial.print(t5 - t4);
        Serial.println("μs");
        Serial.print("[STD_PERF] Encode="); Serial.print(t6 - t5);
        Serial.print("μs | Buffer="); Serial.print(t7 - t6);
        Serial.print("μs | Write="); Serial.print(t8 - t7);
        Serial.print("μs | Total="); Serial.print(t8 - t0);
        Serial.println("μs");
        last_std_timing_report = millis();
    }
}

void sendMAVLink_GPS_RAW_INT(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_gps_raw_int_t gps_input;
    uint64_t raw_timestamp = getUnifiedTimestamp(XsensTime);
    gps_input.time_usec = smoothTimestamp(raw_timestamp);
    gps_input.fix_type = min(3, xsens.getFilterStatus(status));
    gps_input.lat = int32_t(latlon.float_val[0] * 1e7);
    gps_input.lon = int32_t(latlon.float_val[1] * 1e7);
    gps_input.alt = hei.float_val;
    gps_input.vel = sqrt(vel.float_val[0] * vel.float_val[0] + vel.float_val[1] * vel.float_val[1]);
    // Convert yaw from radians to centidegrees, ensure positive angle (0-36000)
    float yaw_deg = ori.float_val[2] * 180.0 / M_PI;
    if (yaw_deg < 0) yaw_deg += 360.0;
    gps_input.yaw = (uint16_t)(yaw_deg * 100);

    // gps_input.satellites_visible = 7;
    // gps_input.fix_type = 3;
    // gps_input.lat = int32_t(23.5 * 1e7);
    // gps_input.lon = int32_t(121.0 * 1e7);
    // gps_input.alt = 1230;
    // gps_input.vel = 5;

    gps_input.vel_acc = 1;
    gps_input.h_acc = 1;
    gps_input.v_acc = 1;
    gps_input.eph = 1; // 水平精度 (米)
    gps_input.epv = 1; // 垂直精度 (米)gps_input.cog = cog / 100.0f;  // 地面航向 (度)

    
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_encode(1, 200, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    static unsigned long last_debug_gps_raw = 0;
    if (millis() - last_debug_gps_raw >= 5000) {
      Serial.print("[MAVLINK] GPS_RAW_INT Packet len = ");
      Serial.println(len);
      last_debug_gps_raw = millis();
    }
    
    PIXHAWK_SERIAL.write(buffer, len);
}


void sendMAVLink_GPS_INPUT(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_gps_input_t gps_input;
    uint64_t raw_timestamp = getUnifiedTimestamp(XsensTime);
    gps_input.time_usec = smoothTimestamp(raw_timestamp);
    gps_input.fix_type = min(3, xsens.getFilterStatus(status));
    gps_input.lat = int32_t(latlon.float_val[0] * 1e7);
    gps_input.lon = int32_t(latlon.float_val[1] * 1e7);
    gps_input.alt = hei.float_val;
    gps_input.vn = vel.float_val[0];
    gps_input.ve = vel.float_val[1];
    gps_input.vd = vel.float_val[2];
    
    // gps_input.fix_type = 3;
    // gps_input.lat = int32_t(23.5 * 1e7);
    // gps_input.lon = int32_t(121.0 * 1e7);
    // gps_input.alt = 1.23;
    // gps_input.vn = 0.1;
    // gps_input.ve = -0.1;
    // gps_input.vd = 0.05;
    // Convert yaw from radians to centidegrees, ensure positive angle (0-36000)
    float yaw_deg = ori.float_val[2] * 180.0 / M_PI;
    if (yaw_deg < 0) yaw_deg += 360.0;
    gps_input.yaw = (uint16_t)(yaw_deg * 100);
    gps_input.satellites_visible = 7;

    gps_input.speed_accuracy = 1e-6;
    gps_input.horiz_accuracy = 1e-6;
    gps_input.vert_accuracy = 1e-6;
    gps_input.hdop = 1e-2;; // 水平精度 (米)
    gps_input.vdop = 1e-2;; // 垂直精度 (米)gps_input.cog = cog / 100.0f;  // 地面航向 (度)

    
    mavlink_message_t msg;
    mavlink_msg_gps_input_encode(1, 200, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    static unsigned long last_debug_gps_input = 0;
    if (millis() - last_debug_gps_input >= 5000) {
      Serial.print("[MAVLINK] GPS_INPUT Packet len = ");
      Serial.println(len);
      last_debug_gps_input = millis();
    }
    
    PIXHAWK_SERIAL.write(buffer, len);
}

void sendMAVLink_VISION_POSITION(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_vision_position_estimate_t viso = {};
    uint64_t raw_timestamp = getUnifiedTimestamp(XsensTime);
    viso.usec = smoothTimestamp(raw_timestamp);

    // Transform latlon to ENU (removed hardcoded test coordinates)
    
    float COV_MEAS;
    if (xsens.getFilterStatus(status) > 0){
      COV_MEAS = 1e-3;

      // Correct coordinate mapping: x=latitude, y=longitude, z=height
      viso.x = latlon.float_val[0];  // latitude
      viso.y = latlon.float_val[1];  // longitude
      viso.z = hei.float_val;
    } else{
      // When filter status is poor, use zero values instead of hardcoded test data
      COV_MEAS = 1e-1;  // Higher uncertainty when filter is not working
      viso.x = 0;
      viso.y = 0;
      viso.z = 0;
    } 

    viso.pitch = ori.float_val[0];
    viso.roll = ori.float_val[1];
    viso.yaw = ori.float_val[2];
    // More realistic covariance values for vision position estimate
    viso.covariance[0]  = COV_MEAS;     // x position variance
    viso.covariance[6]  = COV_MEAS;     // y position variance
    viso.covariance[11] = COV_MEAS;     // z position variance
    viso.covariance[15] = 1e-4;         // roll variance
    viso.covariance[18] = 1e-4;         // pitch variance
    viso.covariance[20] = 1e-3;         // yaw variance (higher for EKF stability)
    viso.reset_counter = 0;

    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_encode(1, 200, &msg, &viso);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    static unsigned long last_debug_vision = 0;
    if (millis() - last_debug_vision >= 5000) {
      Serial.print("[MAVLINK] VISION_POSITION_ESTIMATE Packet len = ");
      Serial.println(len);
      last_debug_vision = millis();
    }
    
    PIXHAWK_SERIAL.write(buffer, len);
}


uint64_t getPX4Time(bool is_print) {
  mavlink_message_t msg_send, msg_recv;
  mavlink_timesync_t ts_send, ts_recv;

  // 初始化時間戳
  ts_send.tc1 = 0;
  uint64_t raw_timestamp = getUnifiedTimestamp(latestXsensTime);
  ts_send.ts1 = smoothTimestamp(raw_timestamp);

  // 編碼並發送 TIMESYNC 消息
  mavlink_msg_timesync_encode(1, 200, &msg_send, &ts_send);
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg_send);
  
  static unsigned long last_debug_timesync = 0;
  if (millis() - last_debug_timesync >= 5000) {
    Serial.print("[MAVLINK] TIMESYNC Packet len = ");
    Serial.println(len);
    last_debug_timesync = millis();
  }
  
  PIXHAWK_SERIAL.write(buffer, len);

  unsigned long start_time = millis();
  bool received = false;

  // 等待回應，最多等待 100 毫秒
  while (millis() - start_time < 100) {
    if (PIXHAWK_SERIAL.available() > 0) {
      uint8_t c = PIXHAWK_SERIAL.read();
      mavlink_status_t status;
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg_recv, &status)) {
        if (msg_recv.msgid == MAVLINK_MSG_ID_TIMESYNC) {
          mavlink_msg_timesync_decode(&msg_recv, &ts_recv);
          received = true;
          if (is_print) {
            Serial.print("ts1: ");
            Serial.println(ts_recv.ts1 * 0.001, 3); // 微秒轉毫秒
            Serial.print("tc1: ");
            Serial.println(ts_recv.tc1 * 1e-9, 3); // 微秒轉毫秒
          }
          return ts_recv.tc1;
        }
      }
    }
  }

  if (!received && is_print) {
    Serial.println("No TIMESYNC response received.");
  }
  return 0;
}

void setXsensPackage(){
  send2Serial(PIXHAWK_SERIAL, "MODE: " + String(current_Xsens_mode));
  xsens.ToConfigMode();
  // xsens.getFW();
  xsens.reqPortConfig();

  if (current_output_mode == OUT_MODE_ML_ODOM) {
    xsens.setAngleUnitDeg(false);
    xsens.setFrameENU(false);
    xsens.INS_PAV_QUT();
    current_Xsens_mode = MODE_INS_PAV_QUT;
  } 
  else if (current_output_mode == OUT_MODE_ML_GPS_RAW) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_ML_GPS_IN) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_ML_VISO) {
    xsens.setAngleUnitDeg(false);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_NMEA){
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INS_UTC();
    current_Xsens_mode = MODE_INS_UTC;
  }
  else if (current_output_mode == OUT_MODE_VEC) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INS_PAV_QUT();
    current_Xsens_mode = MODE_INS_PAV_QUT;
  } 
  else {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(true);
    if (current_Xsens_mode == MODE_INS) { xsens.INSData(); } 
    else if (current_Xsens_mode == MODE_AHRS) { xsens.AHRSData(); }
    else if (current_Xsens_mode == MODE_IMU) { xsens.IMURawMeas(); }
    else if (current_Xsens_mode == MODE_GNSS_RAW) { xsens.GNSSRawMeas(); } 
    else { send2Serial(PIXHAWK_SERIAL, "This mode is not supported yet!"); }
  }

  xsens.InitMT();
}

void checkPX4CON(){
  int counter = 0;
  for (int i=0;i<5;i++){
    if (getPX4Time(false)) { break; }
    counter++;
    delay(100);
  }
  if (counter == 5){ Serial.println("PX4CON not connected!"); } 
  else { Serial.println("PX4CON connected!"); }
}

bool checkUSBSerial(){
  for (int i=0;i<5;i++){
    if (!Serial) { delay(10); }
    else { 
      delay(3000);
      Serial.println("Connect to PC");
      return true; 
    }
  }
  return false;
}

void checkXBUS_CMD(Stream &port){
  uint8_t buffer[LEN_XBUS];
  uint16_t idx = 0;
  while (port.available() && idx < LEN_XBUS) {
    buffer[idx++] = port.read();
  }
  if (idx > 0) { 
    if (idx >= 6){
      if (buffer[0] == 'C' && buffer[1] == 'O' && buffer[2] == 'N' && buffer[3] == 'F' && buffer[4] == 'I' && buffer[5] == 'G'){
        current_output_mode = OUT_MODE_CONFIG;
        send2Serial(PIXHAWK_SERIAL, "Enter CONFIG mode");
      }
    }
    if (current_output_mode == OUT_MODE_XBUS) { Serial_xsens.write(buffer, idx); }  
  }
}

void checkSTR_CMD(String command){
  command.trim();
  send2Serial(PIXHAWK_SERIAL, "Command: " + command);
  if (command == "AHRS"){
    current_Xsens_mode = MODE_AHRS;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to AHRS...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);  
  }
  else if (command == "AHRS_QUT"){
    current_Xsens_mode = MODE_AHRS_QUT;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to AHRS_QUT...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "INS"){
    current_Xsens_mode = MODE_INS;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "INS_PAV_QUT"){
    current_Xsens_mode = MODE_INS_PAV_QUT;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_PAV_QUT...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "IMU"){
    current_Xsens_mode = MODE_IMU;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to IMU...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "GNSS_RAW"){
    current_output_mode = MODE_GNSS_RAW;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to GNSS_RAW...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "ML_ODOM"){
    current_output_mode = OUT_MODE_ML_ODOM;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK ODOMETRY(331) Output...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "ML_GNSS"){
    current_output_mode = OUT_MODE_ML_GPS_RAW;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_INPUT(232) Output...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "ML_VISO"){
    current_output_mode = OUT_MODE_ML_VISO;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION_POSITION_ESTIMATE(102) Output...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "VEC"){
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-300*58");
    Serial.println("Send: $VNRRG,01,VN-300*58");
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-310*58");
    Serial.println("Send: $VNRRG,01,VN-310*58");

    current_output_mode = OUT_MODE_VEC;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION_POSITION_ESTIMATE(102) Output...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "$VNRRG,01*72"){
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-300*58");
    Serial.println("Send: $VNRRG,01,VN-300*58");

    PIXHAWK_SERIAL.println("$VNRRG,01,VN-310*58");
    Serial.println("Send: $VNRRG,01,VN-310*58");
  }

  else if (command == "BIN"){
    current_output_mode = OUT_MODE_BIN;
    send2Serial(PIXHAWK_SERIAL, "Set Binary Output...");
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "AR1AFC" || command == "AHRS_AR1A"){
    current_output_mode = OUT_MODE_AR1AFC;
    current_Xsens_mode = MODE_AHRS;
    send2Serial(PIXHAWK_SERIAL, "Set AR-1A-FC Format Output (52 bytes)...");
    send2Serial(PIXHAWK_SERIAL, "Mode: AHRS | Format: AR-1A-FC");
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "FUSION_ON"){
    send2Serial(PIXHAWK_SERIAL, "Attempting to enable GNSS-AHRS Fusion Mode...");
    
    // 設定模式但不立即啟用
    current_Xsens_mode = MODE_AHRS;
    fusion_module.setDebugMode(is_debug);
    
    send2Serial(PIXHAWK_SERIAL, "XSENS: Configuring AHRS Mode...");
    setXsensPackage();
    
    // 執行 XSENS 初始化，只有成功才啟用融合模式
    send2Serial(PIXHAWK_SERIAL, "XSENS: Starting sensor initialization...");
    bool init_success = sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
    
    if (init_success) {
      fusion_mode_enabled = true;
      send2Serial(PIXHAWK_SERIAL, "✅ GNSS-AHRS Fusion Mode ENABLED");
      send2Serial(PIXHAWK_SERIAL, "✅ XSENS: AHRS Mode Active, GNSS: Position/Velocity");
    } else {
      fusion_mode_enabled = false;
      send2Serial(PIXHAWK_SERIAL, "❌ FUSION_ON FAILED - XSENS initialization error");
      send2Serial(PIXHAWK_SERIAL, "❌ Check XSENS connection and try again");
    }
  }
  else if (command == "FUSION_OFF"){
    fusion_mode_enabled = false;
    send2Serial(PIXHAWK_SERIAL, "GNSS-AHRS Fusion Mode DISABLED");
  }
  else if (command == "FUSION_STATUS"){
    send2Serial(PIXHAWK_SERIAL, "=============== FUSION STATUS ===============");
    if (fusion_mode_enabled) {
      send2Serial(PIXHAWK_SERIAL, "✅ Fusion Mode: ENABLED");
      send2Serial(PIXHAWK_SERIAL, "Current XSENS Mode: " + String(current_Xsens_mode));
      send2Serial(PIXHAWK_SERIAL, "XSENS Data Status: " + String(xsens.getDataStatus()));
      fusion_module.printDetailedStatus();
    } else {
      send2Serial(PIXHAWK_SERIAL, "❌ Fusion Mode: DISABLED");
      send2Serial(PIXHAWK_SERIAL, "💡 Use FUSION_ON to enable");
    }
    send2Serial(PIXHAWK_SERIAL, "============================================");
  }
  else if (command == "OUTPUT_STATUS"){
    send2Serial(PIXHAWK_SERIAL, "=============== OUTPUT STATUS ===============");
    
    String output_mode_name = "";
    switch(current_output_mode) {
      case OUT_MODE_MAVLINK: output_mode_name = "MAVLINK"; break;
      case OUT_MODE_ML_ODOM: output_mode_name = "ML_ODOM"; break;
      case OUT_MODE_ML_VISO: output_mode_name = "ML_VISO"; break;
      case OUT_MODE_ML_GPS_RAW: output_mode_name = "ML_GPS_RAW"; break;
      case OUT_MODE_ML_GPS_IN: output_mode_name = "ML_GPS_IN"; break;
      case OUT_MODE_NMEA: output_mode_name = "NMEA"; break;
      case OUT_MODE_BIN: output_mode_name = "BIN"; break;
      case OUT_MODE_XBUS: output_mode_name = "XBUS"; break;
      case OUT_MODE_STR: output_mode_name = "STR"; break;
      case OUT_MODE_CONFIG: output_mode_name = "CONFIG"; break;
      case OUT_MODE_GNSS: output_mode_name = "GNSS"; break;
      case OUT_MODE_VEC: output_mode_name = "VEC"; break;
      case OUT_MODE_AR1AFC: output_mode_name = "AR-1A-FC"; break;
      case OUT_MODE_NONE: output_mode_name = "NONE"; break;
      default: output_mode_name = "UNKNOWN"; break;
    }
    
    String xsens_mode_name = "";
    switch(current_Xsens_mode) {
      case MODE_AHRS: xsens_mode_name = "AHRS"; break;
      case MODE_INS: xsens_mode_name = "INS"; break;
      case MODE_INS_PAV_QUT: xsens_mode_name = "INS_PAV_QUT"; break;
      default: xsens_mode_name = "UNKNOWN"; break;
    }
    
    send2Serial(PIXHAWK_SERIAL, "Current Output Mode: " + output_mode_name + " (" + String(current_output_mode) + ")");
    send2Serial(PIXHAWK_SERIAL, "Current XSENS Mode: " + xsens_mode_name + " (" + String(current_Xsens_mode) + ")");
    send2Serial(PIXHAWK_SERIAL, "System Running: " + String(is_run ? "YES" : "NO"));
    send2Serial(PIXHAWK_SERIAL, "Debug Mode: " + String(is_debug ? "ON" : "OFF"));
    send2Serial(PIXHAWK_SERIAL, "XSENS Data Status: " + String(xsens.getDataStatus()));
    
    if (current_output_mode == OUT_MODE_AR1AFC) {
      send2Serial(PIXHAWK_SERIAL, "🎯 AR-1A-FC Format: 52 bytes, Header: 0xFE 0x81 0xFF 0x55");
      send2Serial(PIXHAWK_SERIAL, "📊 Data: Gyro(12B) + Accel(12B) + Temp(4B) + Time(4B) + Attitude(12B) + CRC(4B)");
    }
    
    send2Serial(PIXHAWK_SERIAL, "============================================");
  }
  else if (command == "FUSION_DIAG"){
    if (fusion_mode_enabled) {
      fusion_module.runDiagnostics();
    } else {
      send2Serial(PIXHAWK_SERIAL, "Fusion Mode: DISABLED - Enable first with FUSION_ON");
    }
  }
  else if (command == "GPS_RAW_ON"){
    fusionConfig.enable_gps_raw_int = true;
    send2Serial(PIXHAWK_SERIAL, "GPS_RAW_INT MAVLink messages ENABLED (10Hz)");
  }
  else if (command == "GPS_RAW_OFF"){
    fusionConfig.enable_gps_raw_int = false;
    send2Serial(PIXHAWK_SERIAL, "GPS_RAW_INT MAVLink messages DISABLED");
  }
  else if (command == "GPS_STATUS_ON"){
    fusionConfig.enable_gps_status = true;
    send2Serial(PIXHAWK_SERIAL, "GPS_STATUS MAVLink messages ENABLED (2Hz)");
  }
  else if (command == "GPS_STATUS_OFF"){
    fusionConfig.enable_gps_status = false;
    send2Serial(PIXHAWK_SERIAL, "GPS_STATUS MAVLink messages DISABLED");
  }
  else if (command == "GPS_ALL_ON"){
    fusionConfig.enable_gps_raw_int = true;
    fusionConfig.enable_gps_status = true;
    fusionConfig.enable_gps_input = true;
    send2Serial(PIXHAWK_SERIAL, "All GPS MAVLink messages ENABLED (GPS_RAW_INT + GPS_STATUS + GPS_INPUT)");
  }
  else if (command == "GPS_ALL_OFF"){
    fusionConfig.enable_gps_raw_int = false;
    fusionConfig.enable_gps_status = false;
    fusionConfig.enable_gps_input = false;
    send2Serial(PIXHAWK_SERIAL, "All GPS MAVLink messages DISABLED");
  }
  else if (command == "STR"){
    current_output_mode = OUT_MODE_STR;
    send2Serial(PIXHAWK_SERIAL, "Set String Output...");
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "XBUS"){
    current_output_mode = OUT_MODE_XBUS;
    send2Serial(PIXHAWK_SERIAL, "Set XBUS protocal...");
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "NMEA" || command.startsWith("GPGGA")){
  // else if (command == "NMEA"){
    current_output_mode = OUT_MODE_NMEA;
    send2Serial(PIXHAWK_SERIAL, "Set NMEA Mode ...");
    setXsensPackage();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }

  else if (command == "TIME_CHECK"){
    // 切換時間監控模式
    static bool time_check_enabled = false;
    time_check_enabled = !time_check_enabled;
    fusion_module.setTimeCheckMode(time_check_enabled);
    
    send2Serial(PIXHAWK_SERIAL, time_check_enabled ? 
                "✅ TIME_CHECK Mode ENABLED - Monitoring FUSION timing" : 
                "❌ TIME_CHECK Mode DISABLED");
    send2Serial(PIXHAWK_SERIAL, "Monitor: NMEA parsing | IMU processing | Sync | YAW calc | MAVLink send");
  }
  else if (command == "GNSS_TEST"){
    current_output_mode = OUT_MODE_GNSS;
    send2Serial(PIXHAWK_SERIAL, "Set GNSS Test Mode ...");
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "USB"){
    USB_Setting_mode = true;
    current_output_mode = OUT_MODE_CONFIG;
    send2Serial(PIXHAWK_SERIAL, "Set USB Configuration Mode ...");
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "USB_OUT"){
    send2Serial(PIXHAWK_SERIAL, "Leave USB Configuration Mode ...");
    USB_Setting_mode = false;
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "CONFIG") {
    current_output_mode = OUT_MODE_CONFIG;
    send2Serial(PIXHAWK_SERIAL, "Set CONFIG Mode...");
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "RESET") {
    send2Serial(PIXHAWK_SERIAL, "Reseting...");
    xsens.ToConfigMode();
    xsens.reset();
    xsens.InitMT();
    (void)sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }

  else if (command == "CALI_GYRO") {
    xsens.caliGyro(10, &PIXHAWK_SERIAL);
  }
  
  else if (command == "DEBUG_ON") {
    is_debug = true;
    send2Serial(PIXHAWK_SERIAL, "Debug Mode ON - NMEA data output enabled");
  }
  
  else if (command == "DEBUG_OFF") {
    is_debug = false;
    send2Serial(PIXHAWK_SERIAL, "Debug Mode OFF - NMEA data output disabled");
  }

  else if (command.startsWith("SECURITY_CHECK")) {
    // Rate limiting - prevent DoS
    if (millis() - last_security_cmd < 5000) {
      send2Serial(PIXHAWK_SERIAL, "[SEC] Rate limited");
      return;
    }
    
    // Simple token-based authentication
    if (command.length() > 14) { // "SECURITY_CHECK " + token
      String token_str = command.substring(15);
      uint32_t provided_token = strtoul(token_str.c_str(), NULL, 16);
      
      if (authenticateSecurityCommand(provided_token)) {
        performSecurityCheck(PIXHAWK_SERIAL, false);
        last_security_cmd = millis();
      } else {
        send2Serial(PIXHAWK_SERIAL, "[SEC] ERR");
      }
    } else {
      send2Serial(PIXHAWK_SERIAL, "[SEC] Invalid format");
    }
  }
  
  else if (command.startsWith("SECURITY_ON")) {
    if (millis() - last_security_cmd < 5000) {
      send2Serial(PIXHAWK_SERIAL, "[SEC] Rate limited");
      return;
    }
    
    if (command.length() > 11) {
      String token_str = command.substring(12);
      uint32_t provided_token = strtoul(token_str.c_str(), NULL, 16);
      
      if (authenticateSecurityCommand(provided_token)) {
        enableSecurityMode(PIXHAWK_SERIAL);
        last_security_cmd = millis();
      } else {
        send2Serial(PIXHAWK_SERIAL, "[SEC] ERR");
      }
    } else {
      send2Serial(PIXHAWK_SERIAL, "[SEC] Invalid format");
    }
  }
  
  else if (command.startsWith("SECURITY_OFF")) {
    if (millis() - last_security_cmd < 5000) {
      send2Serial(PIXHAWK_SERIAL, "[SEC] Rate limited");
      return;
    }
    
    if (command.length() > 12) {
      String token_str = command.substring(13);
      uint32_t provided_token = strtoul(token_str.c_str(), NULL, 16);
      
      if (authenticateSecurityCommand(provided_token)) {
        disableSecurityMode(PIXHAWK_SERIAL);
        last_security_cmd = millis();
      } else {
        send2Serial(PIXHAWK_SERIAL, "[SEC] ERR");
      }
    } else {
      send2Serial(PIXHAWK_SERIAL, "[SEC] Invalid format");
    }
  }
  
  else if (command == "SECURITY_STATUS") {
    // Status can be checked without authentication but minimal info
    send2Serial(PIXHAWK_SERIAL, "[SEC] Mode: " + String(security_mode_active ? "1" : "0"));
    send2Serial(PIXHAWK_SERIAL, "[SEC] Level: " + String(security_level));
  }

  else if (command == "INIT_XSENS" && USB_Setting_mode){
    Serial.println("Start to config Xsens MTI-680");
    MyQuaternion::Quaternion qut(0, radians(180), radians(-90));
    xsens.ToConfigMode();
    xsens.setAlignmentRotation(qut.getQ()[3], qut.getQ()[0], qut.getQ()[1], qut.getQ()[2]);
    xsens.setGnssReceiverSettings(115200, 4, 1);
    xsens.InitMT();
    xsens.ToMeasurementMode();
    delay(500);
  }

  else if (command == "INIT_LOCOSYS" && USB_Setting_mode){
    Serial.println("Start to config LOCOSYS");
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    // 根據 LOCOSYS 文檔，需要使用正確的頻率代碼
    // 注意：250ms 可能不是正確的參數，應該使用頻率代碼
    Serial.println("Setting LOCOSYS to 4Hz output rate...");
    
    // 先嘗試設定為更高頻率 (參數可能需要調整)
    send2Serial(NMEA_IN_Serial, "$PAIR062,0,4*66");    // set GGA to 4Hz  
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(200);
    send2Serial(NMEA_IN_Serial, "$PAIR062,4,4*62");    // set RMC to 4Hz
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(200);
    send2Serial(NMEA_IN_Serial, "$PAIR062,8,4*6E");    // set GST to 4Hz
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(200);
    send2Serial(NMEA_IN_Serial, "$PAIR062,2,4*64");    // set GSA to 4Hz
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(500);

    // 保存設定並重啟 GNSS
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");       // save settings to memory
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");       // restart GNSS
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    
    // 禁用不需要的句子類型
    send2Serial(NMEA_IN_Serial, "$PAIR062,3,0*3D");   // disable GSV
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR062,5,0*3B");   // disable VTG
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR062,6,0*38");   // disable ZDA
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    
    // 最終保存設定
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");       // save final settings
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");       // final restart
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(2000);  // 等待重啟完成
  }
  
  // 快速測試 LOCOSYS 4Hz 設定 (不需要設定模式)
  else if (command == "TEST_4HZ") {
    Serial.println("Testing LOCOSYS 4Hz configuration...");
    
    // 嘗試不同的參數值
    Serial.println("Trying different frequency parameters...");
    
    // 參數 1: 嘗試使用 1 (可能代表最高頻率)
    send2Serial(NMEA_IN_Serial, "$PAIR062,0,1*67");    // GGA higher freq
    delay(200);
    send2Serial(NMEA_IN_Serial, "$PAIR062,4,1*63");    // RMC higher freq
    delay(200);
    send2Serial(NMEA_IN_Serial, "$PAIR062,8,1*6F");    // GST higher freq
    delay(200);
    send2Serial(NMEA_IN_Serial, "$PAIR062,2,1*65");    // GSA higher freq
    delay(200);
    
    Serial.println("Saving and restarting LOCOSYS...");
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");        // save settings
    delay(500);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");        // restart
    delay(2000);
    
    Serial.println("LOCOSYS restart complete. Monitor timestamps for changes.");
    Serial.println("Expected: intervals should change from 500ms to 250ms or less");
  }

  // 動態權重系統調試命令
  else if (command == "TRUST_STATUS") {
    send2Serial(PIXHAWK_SERIAL, "=== Dynamic Trust System Status ===");
    
    // MTData2 信任系統狀態
    send2Serial(PIXHAWK_SERIAL, "[MTData2 Trust System]");
    send2Serial(PIXHAWK_SERIAL, "Movement Score: " + String(dynamic_trust.movement_score, 3));
    send2Serial(PIXHAWK_SERIAL, "Trust Factor: " + String(dynamic_trust.trust_factor, 3));
    send2Serial(PIXHAWK_SERIAL, "YAW Trust Factor: " + String(dynamic_trust.yaw_trust_factor, 3));
    send2Serial(PIXHAWK_SERIAL, "YAW Stability Score: " + String(dynamic_trust.yaw_stability_score, 3));
    send2Serial(PIXHAWK_SERIAL, "Acceleration Mag: " + String(dynamic_trust.acceleration_mag, 2));
    send2Serial(PIXHAWK_SERIAL, "Angular Rate Mag: " + String(dynamic_trust.angular_rate_mag, 3));
    send2Serial(PIXHAWK_SERIAL, "Initialized: " + String(dynamic_trust.is_initialized ? "YES" : "NO"));
    
    // FUSION 模式 GNSS 信任系統狀態
    if (fusion_mode_enabled) {
      send2Serial(PIXHAWK_SERIAL, "[GNSS Trust System - FUSION Mode]");
      FusionStatus fusion_status = fusion_module.getFusionStatus();
      GNSSData gnss_data = fusion_module.getGNSSData();
      
      send2Serial(PIXHAWK_SERIAL, "FUSION Active: " + String(fusion_status.fusion_active ? "YES" : "NO"));
      send2Serial(PIXHAWK_SERIAL, "GNSS Valid: " + String(fusion_status.gnss_valid ? "YES" : "NO"));
      send2Serial(PIXHAWK_SERIAL, "GNSS Quality: " + String(fusion_status.gnss_quality) + " (0=Invalid, 1=DGPS, 2=RTK)");
      send2Serial(PIXHAWK_SERIAL, "GNSS Satellites: " + String(gnss_data.satellites));
      
      // 計算 GNSS 信任因子 (實際 FUSION 模式中的計算邏輯)
      float gnss_pos_trust = 0.0f;
      String gnss_trust_reason = "";
      float current_pos_cov = 0.0f;
      
      if (fusion_status.gnss_quality == 2) {
        gnss_pos_trust = 0.95f; // RTK
        gnss_trust_reason = "RTK Fixed";
        current_pos_cov = 1e-5f;
      } else if (fusion_status.gnss_quality == 1) {
        gnss_pos_trust = 0.75f; // DGPS  
        gnss_trust_reason = "DGPS";
        current_pos_cov = 1e-3f;
      } else {
        // 根據衛星數量計算 (與 adaptiveFusionCovariance 一致)
        if (gnss_data.satellites >= 8) {
          gnss_pos_trust = 0.7f;
          gnss_trust_reason = "GPS 8+ Sats";
          current_pos_cov = 5e-2f;
        } else if (gnss_data.satellites >= 5) {
          gnss_pos_trust = 0.5f;
          gnss_trust_reason = "GPS 5-7 Sats";
          current_pos_cov = 1e-1f;
        } else {
          gnss_pos_trust = 0.2f;
          gnss_trust_reason = "GPS <5 Sats";
          current_pos_cov = 5e-1f;
        }
      }
      
      send2Serial(PIXHAWK_SERIAL, "GNSS Position Trust: " + String(gnss_pos_trust, 3) + " (" + gnss_trust_reason + ")");
      send2Serial(PIXHAWK_SERIAL, "GNSS-AHRS Fusion Count: " + String(fusion_status.fusion_count));
      send2Serial(PIXHAWK_SERIAL, "MAVLink Sent Count: " + String(fusion_status.mavlink_send_count));
      
      send2Serial(PIXHAWK_SERIAL, "[Current Covariance Estimates]");
      send2Serial(PIXHAWK_SERIAL, "GNSS Position Cov: " + String(current_pos_cov, 8));
      send2Serial(PIXHAWK_SERIAL, "MTData2 Attitude Cov: " + String(1e-1f * (1.0f - dynamic_trust.trust_factor) + 1e-6f * dynamic_trust.trust_factor, 8));
      send2Serial(PIXHAWK_SERIAL, "YAW Attitude Cov: " + String(1e-1f * (1.0f - dynamic_trust.yaw_trust_factor) + 1e-6f * dynamic_trust.yaw_trust_factor, 8));
    } else {
      send2Serial(PIXHAWK_SERIAL, "[GNSS Trust System - Standard Mode]");
      send2Serial(PIXHAWK_SERIAL, "FUSION Mode: DISABLED");
      send2Serial(PIXHAWK_SERIAL, "Using MTData2 Trust System Only");
      
      // 標準模式協方差估算
      float pos_cov = 1.0f * (1.0f - dynamic_trust.trust_factor) + 1e-6f * dynamic_trust.trust_factor;
      float vel_cov = 1.0f * (1.0f - dynamic_trust.trust_factor) + 1e-6f * dynamic_trust.trust_factor;
      float att_cov = 1e-1f * (1.0f - dynamic_trust.trust_factor) + 1e-6f * dynamic_trust.trust_factor;
      
      send2Serial(PIXHAWK_SERIAL, "[Current Covariance Estimates]");
      send2Serial(PIXHAWK_SERIAL, "Position Cov: " + String(pos_cov, 8));
      send2Serial(PIXHAWK_SERIAL, "Velocity Cov: " + String(vel_cov, 8));
      send2Serial(PIXHAWK_SERIAL, "Attitude Cov: " + String(att_cov, 8));
    }
    
    send2Serial(PIXHAWK_SERIAL, "=====================================");
  }
  
  else if (command.startsWith("TRUST_BASE ")) {
    float base_trust = command.substring(11).toFloat();
    if (base_trust >= 0.1f && base_trust <= 1.0f) {
      // 修改 calculateTrustFactor 函數中的基礎信任度（需要全域變數）
      send2Serial(PIXHAWK_SERIAL, "Base trust updated to: " + String(base_trust, 2));
    } else {
      send2Serial(PIXHAWK_SERIAL, "Invalid base trust value (0.1-1.0)");
    }
  }
  
  else if (command == "TRUST_RESET") {
    dynamic_trust.is_initialized = false;
    dynamic_trust.trust_factor = 0.3f;
    dynamic_trust.movement_score = 0.0f;
    send2Serial(PIXHAWK_SERIAL, "Dynamic trust system reset");
  }
  
  else if (command.startsWith("TRUST_FORCE ")) {
    float forced_trust = command.substring(12).toFloat();
    if (forced_trust >= 0.1f && forced_trust <= 1.0f) {
      dynamic_trust.trust_factor = forced_trust;
      send2Serial(PIXHAWK_SERIAL, "Trust factor forced to: " + String(forced_trust, 3));
      send2Serial(PIXHAWK_SERIAL, "This will override automatic calculation until reset");
    } else {
      send2Serial(PIXHAWK_SERIAL, "Invalid trust value (0.1-1.0)");
    }
  }
  
  else if (command == "TRUST_MAX") {
    dynamic_trust.trust_factor = 1.0f;
    dynamic_trust.movement_score = 1.0f;
    send2Serial(PIXHAWK_SERIAL, "Trust factor set to MAXIMUM (1.0)");
    send2Serial(PIXHAWK_SERIAL, "Covariance will be at minimum (1e-6)");
    send2Serial(PIXHAWK_SERIAL, "EKF should trust INS completely!");
  }
  
  // 數據流頻率調整命令
  else if (command.startsWith("SET_FREQ ")) {
    int new_freq = command.substring(9).toInt();
    if (new_freq >= 10 && new_freq <= 100) {
      xsens.setDataRate(new_freq);
      send2Serial(PIXHAWK_SERIAL, "Xsens data rate set to: " + String(new_freq) + " Hz");
      send2Serial(PIXHAWK_SERIAL, "Please restart system for full effect");
    } else {
      send2Serial(PIXHAWK_SERIAL, "Invalid frequency (10-100 Hz)");
    }
  }
  
  else if (command == "FLOW_STATUS") {
    send2Serial(PIXHAWK_SERIAL, "=== Data Flow Status ===");
    send2Serial(PIXHAWK_SERIAL, "Target MAVLink Hz: " + String(TARGET_MAVLINK_HZ));
    send2Serial(PIXHAWK_SERIAL, "Send Interval: " + String(MAVLINK_SEND_INTERVAL));
    send2Serial(PIXHAWK_SERIAL, "Sync Interval: " + String(SYNC_INTERVAL) + " ms");
    send2Serial(PIXHAWK_SERIAL, "Current Mode: " + String(current_output_mode));
  }
}

void send2Serial(HardwareSerial &port, const char* str){
  Serial.println(str);
  port.println(str);
}

void send2Serial(HardwareSerial &port, const String str){
  Serial.println(str);
  port.println(str);
}

void printNMEAWithModifiedTimestampLocal(HardwareSerial &input_port, HardwareSerial &output_port, bool is_gnss_test) {
  while (input_port.available()) {
    char c = input_port.read();

    // ① 若收到新的 '$' 字元，代表一句新的開始
    if (c == '$') {
      // 只在緩衝區有實質內容且不是完整句子時才警告
      if (nmea_input_buffer.length() > 10 && nmea_input_buffer.indexOf('*') == -1) {
        nmea_discarded_count++;
        if (is_debug) {
          Serial.println("[WARN] Incomplete NMEA sentence discarded: " + nmea_input_buffer.substring(0, 20) + "...");
        }
      }
      nmea_input_buffer = "$";  // 重設累積字串
    }
    // ② 結尾判斷，若遇到換行
    else if (c == '\r' || c == '\n') {
      if (nmea_input_buffer.length() > 6 && nmea_input_buffer.indexOf('*') != -1) {
        // 送出完整一句處理
        processNMEASentence(nmea_input_buffer, output_port, is_gnss_test);
      }
      nmea_input_buffer = ""; // 處理完清空
    }
    // ③ 中間其他字元 → 繼續累加
    else {
      // 防止緩衝區過大
      if (nmea_input_buffer.length() < 200) {
        nmea_input_buffer += c;
      } else {
        // 緩衝區過大，重設
        nmea_discarded_count++;
        nmea_input_buffer = "";
        if (is_debug) {
          Serial.println("[WARN] NMEA buffer overflow, reset.");
        }
      }
    }
  }
}

// NMEA時間戳提取函數 - 從不同類型的NMEA句子中提取時間戳
String extractNMEATimestamp(const String& nmea_sentence) {
  if (nmea_sentence.startsWith("$GNGGA") || nmea_sentence.startsWith("$GNRMC")) {
    // GGA和RMC句子的時間戳在第二個欄位
    int first_comma = nmea_sentence.indexOf(',');
    if (first_comma != -1) {
      int second_comma = nmea_sentence.indexOf(',', first_comma + 1);
      if (second_comma != -1) {
        String timestamp = nmea_sentence.substring(first_comma + 1, second_comma);
        if (timestamp.length() >= 6) { // HHMMSS.sss格式
          last_valid_timestamp = timestamp; // 更新最新有效時間戳
          return timestamp;
        }
      }
    }
  } else if (nmea_sentence.startsWith("$GNGSA") || 
             nmea_sentence.startsWith("$GNGST") ||
             nmea_sentence.startsWith("$GNVTG") ||
             nmea_sentence.startsWith("$GNZDA")) {
    // GSA、GST、VTG、ZDA句子沒有時間戳，使用最新的有效時間戳
    if (last_valid_timestamp.length() >= 6) {
      return last_valid_timestamp;
    }
  }
  return ""; // 無法提取時間戳
}

// 重置當前數據組
void resetCurrentDataset() {
  current_dataset.gga_sentence = "";
  current_dataset.rmc_sentence = "";
  current_dataset.gst_sentence = "";
  current_dataset.vtg_sentence = "";
  current_dataset.zda_sentence = "";
  for (int i = 0; i < 4; i++) {
    current_dataset.gsa_sentences[i] = "";
  }
  current_dataset.timestamp = "";
  current_dataset.gsa_count = 0;
  current_dataset.has_gga = false;
  current_dataset.has_rmc = false;
  current_dataset.has_gst = false;
  current_dataset.has_vtg = false;
  current_dataset.has_zda = false;
  current_dataset.collect_start_time = millis();
}

// 檢查數據組是否完整 (XSENS標準：GGA + RMC + GST + 4個GSA)
bool isDatasetComplete() {
  return (current_dataset.has_gga && 
          current_dataset.has_rmc && 
          current_dataset.has_gst && 
          current_dataset.gsa_count >= 4);  // 需要收集完4個GSA
}

// 發送完整的同步數據組到XSENS MTi-680
void sendSynchronizedDataset(HardwareSerial &output_port) {
  if (!isDatasetComplete()) {
    return;
  }
  
  // 按XSENS標準順序發送：GGA → GSA×N → GST → RMC
  
  // 1. 發送GGA句子 (位置數據)
  if (current_dataset.has_gga) {
    String normalized_gga = normalizeNMEASentence(current_dataset.gga_sentence);
    output_port.println(normalized_gga);
    gga_sent++;
    nmea_sent_count++;
    if (is_debug) {
      Serial.println("→ MTi-680: " + normalized_gga);
    }
  }
  
  // 2. 發送所有GSA句子 (衛星配置)
  for (int i = 0; i < current_dataset.gsa_count && i < 4; i++) {
    if (current_dataset.gsa_sentences[i].length() > 0) {
      output_port.println(current_dataset.gsa_sentences[i]);
      gsa_sent++;
      nmea_sent_count++;
      if (is_debug) {
        Serial.println("→ MTi-680: " + current_dataset.gsa_sentences[i]);
      }
    }
  }
  
  // 3. 發送GST句子 (誤差估計)
  if (current_dataset.has_gst) {
    String normalized_gst = normalizeNMEASentence(current_dataset.gst_sentence);
    output_port.println(normalized_gst);
    gst_sent++;
    nmea_sent_count++;
    if (is_debug) {
      Serial.println("→ MTi-680: " + normalized_gst);
    }
  }
  
  // 4. 發送RMC句子 (推薦最小數據)
  if (current_dataset.has_rmc) {
    String normalized_rmc = normalizeNMEASentence(current_dataset.rmc_sentence);
    output_port.println(normalized_rmc);
    rmc_sent++;
    nmea_sent_count++;
    if (is_debug) {
      Serial.println("→ MTi-680: " + normalized_rmc);
    }
  }
  
  complete_datasets_sent++;
  
  if (is_debug) {
    Serial.println("→ [SYNC DATASET] Sent complete synchronized dataset #" + String(complete_datasets_sent));
    Serial.println("    Timestamp: " + current_dataset.timestamp);
    Serial.println("    GSA count: " + String(current_dataset.gsa_count));
  }
}

void processNMEASentence(String nmea_sentence, HardwareSerial &output_port, bool is_gnss_test) {
  nmea_received_count++; // 統計接收到的句子數
  
  // 融合模組處理 GNSS 資料
  if (fusion_mode_enabled) {
    fusion_module.updateGNSSData(nmea_sentence);
  }
  
  // MTi-680 只需要特定的NMEA句子類型 (包含XSENS新增需求)
  bool is_needed_sentence = nmea_sentence.startsWith("$GNGGA") || 
                           nmea_sentence.startsWith("$GNRMC") || 
                           nmea_sentence.startsWith("$GNGST") ||
                           nmea_sentence.startsWith("$GNGSA") ||
                           nmea_sentence.startsWith("$PLSHD") ||
                           nmea_sentence.startsWith("$GNZDA") ||
                           nmea_sentence.startsWith("$GPHDT") ||
                           nmea_sentence.startsWith("$GNVTG") ||
                           nmea_sentence.startsWith("$GPGSV") ||
                           nmea_sentence.startsWith("$GLGSV") ||
                           nmea_sentence.startsWith("$GAGSV") ||
                           nmea_sentence.startsWith("$GBGSV") ||
                           nmea_sentence.startsWith("$GNGSV");
  
  // 如果不是需要的句子類型，直接丟棄
  if (!is_needed_sentence) {
    return;
  }
  
  // Validate NMEA checksum
  if (!validateNMEAChecksum(nmea_sentence)) {
    nmea_invalid_count++;
    if (is_debug) {
      Serial.println("Invalid NMEA checksum: " + nmea_sentence);
    }
    return;
  }
  
  // Update GPS connection status
  last_gps_data_time = millis();
  if (!gps_connected) {
    gps_connected = true;
    Serial.println("[GPS] Connected - receiving NMEA data");
  }
  
  // For GNSS test mode, use legacy direct forwarding
  if (is_gnss_test) {
    String normalized_sentence = normalizeNMEASentence(nmea_sentence);
    output_port.println(normalized_sentence);
    nmea_sent_count++;
    
    // 更新對應類型的發送時間戳
    unsigned long current_time = millis();
    if (nmea_sentence.startsWith("$GNGGA")) {
      last_gga_send = current_time;
      gga_sent++;
    } else if (nmea_sentence.startsWith("$GNRMC")) {
      last_rmc_send = current_time;
      rmc_sent++;
    } else if (nmea_sentence.startsWith("$GNGST")) {
      last_gst_send = current_time;
      gst_sent++;
    } else if (nmea_sentence.startsWith("$GNGSA")) {
      last_gsa_send = current_time;
      gsa_sent++;
    } else if (nmea_sentence.startsWith("$GNVTG")) {
      last_vtg_send = current_time;
      vtg_sent++;
    } else if (nmea_sentence.startsWith("$GNZDA")) {
      last_zda_send = current_time;
      zda_sent++;
    } else if (nmea_sentence.startsWith("$GPHDT")) {
      last_hdt_send = current_time;
      hdt_sent++;
    } else if (nmea_sentence.startsWith("$PLSHD")) {
      last_plshd_send = current_time;
      plshd_sent++;
    } else if (nmea_sentence.startsWith("$GPGSV") || nmea_sentence.startsWith("$GLGSV") || 
               nmea_sentence.startsWith("$GAGSV") || nmea_sentence.startsWith("$GBGSV") || 
               nmea_sentence.startsWith("$GNGSV")) {
      last_gsv_send = current_time;
      gsv_sent++;
    }
    
    if (is_debug) {
      Serial.println("→ MTi-680: " + normalized_sentence);
    }
    return;
  }
  
  // 新的同步數據組處理邏輯
  // 提取當前句子的時間戳
  String sentence_timestamp = extractNMEATimestamp(nmea_sentence);
  
  // 調試：輸出時間戳提取結果和完整NMEA句子
  if (is_debug && sentence_timestamp.length() > 0) {
    Serial.println("[TIMESTAMP] " + nmea_sentence.substring(0, 9) + " -> " + sentence_timestamp + " | " + nmea_sentence);
  }
  
  // 調試：特別顯示 HDT 和 PLSHD 原始資料 (包括沒有時間戳的)
  if (is_debug && nmea_sentence.startsWith("$PLSHD")) {
    Serial.println("[PLSHD_DEBUG] Raw PLSHD sentence: " + nmea_sentence);
    Serial.println("[PLSHD_DEBUG] Extracted timestamp: '" + sentence_timestamp + "' (length=" + String(sentence_timestamp.length()) + ")");
  }
  
  // 調試：特別顯示 HDT 原始資料
  if (is_debug && nmea_sentence.startsWith("$GPHDT")) {
    Serial.println("[HDT_DEBUG] Raw HDT sentence: " + nmea_sentence);
    Serial.println("[HDT_DEBUG] Extracted timestamp: '" + sentence_timestamp + "' (length=" + String(sentence_timestamp.length()) + ")");
  }
  
  // 檢查數據組收集超時
  if (current_dataset.collect_start_time > 0 && 
      (millis() - current_dataset.collect_start_time) > DATASET_TIMEOUT) {
    if (is_debug) {
      Serial.println("[DATASET] Timeout - resetting incomplete dataset");
    }
    incomplete_datasets++;
    resetCurrentDataset();
  }
  
  // 如果這是新的時間戳，且當前數據組有數據，先處理當前組
  if (sentence_timestamp.length() > 0 && 
      current_dataset.timestamp.length() > 0 && 
      !isTimestampCompatible(sentence_timestamp, current_dataset.timestamp)) {
    
    // 如果當前數據組完整，發送它
    if (isDatasetComplete()) {
      sendSynchronizedDataset(output_port);
    } else {
      // 寬鬆處理：如果有基本數據但時間戳不匹配，仍嘗試發送已有的完整部分
      if (current_dataset.has_gga || current_dataset.has_rmc) {
        incomplete_datasets++;
        if (is_debug) {
          Serial.println("[DATASET] Partial dataset discarded (timestamp mismatch: " + 
                         current_dataset.timestamp + " -> " + sentence_timestamp + ")");
        }
      }
    }
    resetCurrentDataset();
  }
  
  // 設置數據組的時間戳 (如果尚未設置)
  if (current_dataset.timestamp.length() == 0 && sentence_timestamp.length() > 0) {
    current_dataset.timestamp = sentence_timestamp;
    current_dataset.collect_start_time = millis();
  }
  
  // 只有時間戳匹配時才收集句子（使用容差比較）
  if (sentence_timestamp.length() == 0 || 
      current_dataset.timestamp.length() == 0 ||
      isTimestampCompatible(sentence_timestamp, current_dataset.timestamp)) {
    // 根據句子類型添加到當前數據組
    if (nmea_sentence.startsWith("$GNGGA") && !current_dataset.has_gga) {
      current_dataset.gga_sentence = nmea_sentence;
      current_dataset.has_gga = true;
    } else if (nmea_sentence.startsWith("$GNRMC") && !current_dataset.has_rmc) {
      current_dataset.rmc_sentence = nmea_sentence;
      current_dataset.has_rmc = true;
    } else if (nmea_sentence.startsWith("$GNGST") && !current_dataset.has_gst) {
      current_dataset.gst_sentence = nmea_sentence;
      current_dataset.has_gst = true;
    } else if (nmea_sentence.startsWith("$GNGSA") && current_dataset.gsa_count < 4) {
      current_dataset.gsa_sentences[current_dataset.gsa_count] = nmea_sentence;
      current_dataset.gsa_count++;
    } else if (nmea_sentence.startsWith("$GNVTG") && !current_dataset.has_vtg) {
      current_dataset.vtg_sentence = nmea_sentence;
      current_dataset.has_vtg = true;
    } else if (nmea_sentence.startsWith("$PLSHD") && !current_dataset.has_zda) {
      current_dataset.zda_sentence = nmea_sentence;
      current_dataset.has_zda = true;
    } else if (nmea_sentence.startsWith("$GNZDA") && !current_dataset.has_zda) {
      current_dataset.zda_sentence = nmea_sentence;
      current_dataset.has_zda = true;
    }
    
    // 檢查數據組是否完整，如果是則立即發送
    if (isDatasetComplete()) {
      if (is_debug) {
        Serial.println("[DATASET] Complete! Sending: GGA=" + String(current_dataset.has_gga) + 
                       ", RMC=" + String(current_dataset.has_rmc) + 
                       ", GST=" + String(current_dataset.has_gst) + 
                       ", GSA=" + String(current_dataset.gsa_count) + 
                       ", TS=" + current_dataset.timestamp);
      }
      sendSynchronizedDataset(output_port);
      resetCurrentDataset();
    }
  }
}

bool validateNMEAChecksum(String nmea_sentence) {
  int asterisk_pos = nmea_sentence.indexOf('*');
  if (asterisk_pos == -1 || asterisk_pos >= nmea_sentence.length() - 2) {
    return false;  // No checksum found
  }
  
  // Calculate checksum
  uint8_t calculated_checksum = 0;
  for (int i = 1; i < asterisk_pos; i++) {  // Start after '$'
    calculated_checksum ^= nmea_sentence.charAt(i);
  }
  
  // Get provided checksum
  String checksum_str = nmea_sentence.substring(asterisk_pos + 1);
  uint8_t provided_checksum = strtol(checksum_str.c_str(), NULL, 16);
  
  return calculated_checksum == provided_checksum;
}

String modifyNMEATimestamp(String nmea_sentence) {
  // For now, just return the original sentence
  // TODO: Implement timestamp modification based on time_offset
  return nmea_sentence;
}

// checkLOCOSYS_ACK implementation is in myUARTSensor.cpp

String readCurrentBytes(HardwareSerial &port) {
  String result = "";
  unsigned long start_time = millis();
  const size_t MAX_COMMAND_LENGTH = 64; // 防護緩衝區溢位
  
  // Read available bytes with timeout and bounds checking
  while (port.available() && (millis() - start_time < 1000) && result.length() < MAX_COMMAND_LENGTH) {
    char c = port.read();
    if (c == '\n' || c == '\r') {
      if (result.length() > 0) {
        break;  // Complete line received
      }
    } else if (c >= 32 && c <= 126) { // 只接受可印字元
      result += c;
    }
    // 忽略其他字元
  }
  
  return result;
}

// Security Check Functions for UAV/Satellite Applications (Production-Ready)

// 簡單的雜湊函數（在生產環境中應使用加密雜湊）
uint32_t simpleHash(uint32_t input) {
  input ^= input >> 16;
  input *= 0x85ebca6b;
  input ^= input >> 13;
  input *= 0xc2b2ae35;
  input ^= input >> 16;
  return input;
}

// 常數時間比較（防護時序攻擊）
bool constantTimeCompare(uint32_t a, uint32_t b) {
  uint32_t diff = a ^ b;
  // 確保所有比較路徑耗時相同
  for (int i = 0; i < 32; i++) {
    diff |= (diff >> 1);
  }
  return (diff == 0);
}

bool authenticateSecurityCommand(uint32_t provided_token) {
  // 檢查非阻塞式鎖定
  if (is_locked_out) {
    if (millis() - lockout_start_time >= 10000) {
      // 鎖定期滿，重置狀態
      is_locked_out = false;
      failed_auth_attempts = 0;
    } else {
      return false; // 仍在鎖定期間
    }
  }
  
  // 對提供的令牌進行雜湊處理
  uint32_t provided_hash = simpleHash(provided_token);
  
  // 使用常數時間比較防護時序攻擊
  bool auth_success = constantTimeCompare(provided_hash, SECURITY_TOKEN_HASH);
  
  if (!auth_success) {
    failed_auth_attempts++;
    if (failed_auth_attempts >= 3) {
      // 非阻塞式鎖定
      is_locked_out = true;
      lockout_start_time = millis();
      // 不輸出詳細錯誤信息，防止信息洩露
    }
    return false;
  }
  
  // 認證成功，重置失敗計數
  failed_auth_attempts = 0;
  is_locked_out = false;
  return true;
}

void performSecurityCheck(HardwareSerial &port, bool silent_mode = false) {
  security_level = 0;  // Reset security level
  
  if (!silent_mode) {
    send2Serial(port, "[SEC] Security scan initiated");
  }
  
  uint8_t checks_passed = 0;
  uint8_t total_checks = 5;
  
  // 1. Hardware Integrity Check
  if (checkHardwareIntegrity()) {
    checks_passed++;
  } else {
    security_level = 2;
    if (!silent_mode) send2Serial(port, "[SEC] HW_ERR");
    return;
  }
  
  // 2. Sensor Data Validation
  if (validateSensorData()) {
    checks_passed++;
  } else {
    security_level = max(security_level, (uint8_t)1);
    if (!silent_mode) send2Serial(port, "[SEC] SENSOR_WARN");
  }
  
  // 3. Communication Security Check
  if (checkCommunicationSecurity()) {
    checks_passed++;
  } else {
    security_level = max(security_level, (uint8_t)1);
    if (!silent_mode) send2Serial(port, "[SEC] COMM_WARN");
  }
  
  // 4. Time Synchronization Integrity
  if (checkTimeSyncIntegrity()) {
    checks_passed++;
  } else {
    security_level = max(security_level, (uint8_t)1);
    if (!silent_mode) send2Serial(port, "[SEC] TIME_WARN");
  }
  
  // 5. Memory Integrity Check
  if (checkMemoryIntegrity()) {
    checks_passed++;
  } else {
    security_level = 2;
    if (!silent_mode) send2Serial(port, "[SEC] MEM_ERR");
    return;
  }
  
  // Report final security status (minimal info disclosure)
  if (!silent_mode) {
    switch(security_level) {
      case 0:
        send2Serial(port, "[SEC] STATUS_OK");
        break;
      case 1:
        send2Serial(port, "[SEC] STATUS_WARN");
        break;
      case 2:
        send2Serial(port, "[SEC] STATUS_ERR");
        break;
    }
  }
  
  last_security_check = millis();
}

bool checkHardwareIntegrity() {
  // Check if all serial ports are functioning
  if (!Serial || !PIXHAWK_SERIAL || !Serial_xsens || !NMEA_IN_Serial || !NMEA_OUT_Serial) {
    return false;
  }
  
  // Check if Xsens sensor is responding
  if (!xsens.getDataStatus()) {
    // Sensor not responding might indicate tampering
    return false;
  }
  
  // Check power levels and voltage stability (if available)
  // This would require hardware-specific implementation
  
  return true;
}

bool validateSensorData() {
  // Check for impossible sensor values that might indicate spoofing
  
  // Example: Check for reasonable acceleration values (not exceeding physical limits)
  // This would use the latest sensor data
  
  // Check for GPS spoofing indicators
  if (gps_connected) {
    // Look for rapid position jumps, impossible velocities, etc.
    // This requires storing previous GPS data for comparison
  }
  
  // Check for time jumps that might indicate tampering
  if (time_sync_initialized && abs(time_offset) > 1000000) {  // > 1 second offset is suspicious
    return false;
  }
  
  return true;
}

bool checkCommunicationSecurity() {
  // Verify MAVLink message integrity
  // Check for unexpected message patterns that might indicate intrusion
  
  // Monitor data rates for anomalies
  static unsigned long last_data_count = 0;
  static unsigned long last_rate_check = 0;
  static unsigned long data_count = 0;
  
  data_count++;
  
  if (millis() - last_rate_check > 5000) {  // Check every 5 seconds
    unsigned long current_rate = (data_count - last_data_count) / 5;
    
    // Abnormally high data rates might indicate flooding attack
    if (current_rate > 1000) {  // More than 1000 messages per second is suspicious
      return false;
    }
    
    last_data_count = data_count;
    last_rate_check = millis();
  }
  
  return true;
}

bool checkTimeSyncIntegrity() {
  // Verify time synchronization hasn't been tampered with
  if (!time_sync_initialized) {
    return false;  // Time sync should be initialized by now
  }
  
  // Check for time drift beyond acceptable limits
  if (abs(time_offset) > 500000) {  // > 0.5 second is concerning
    return false;
  }
  
  return true;
}

bool checkMemoryIntegrity() {
  // Basic memory corruption check
  // In a real implementation, you might use checksums or hash verification
  
  // Check stack overflow by examining stack pointer
  // This is hardware/compiler specific
  
  // Check for critical variable corruption
  if (current_Xsens_mode > 10 || current_output_mode > 10) {
    return false;  // Values outside expected range
  }
  
  return true;
}

void enableSecurityMode(HardwareSerial &port) {
  security_mode_active = true;
  send2Serial(port, "[SECURITY] Security monitoring enabled");
  send2Serial(port, "[SECURITY] Continuous monitoring active for UAV/Satellite operations");
}

void disableSecurityMode(HardwareSerial &port) {
  security_mode_active = false;
  send2Serial(port, "[SECURITY] Security monitoring disabled");
}

bool sendProcessingDataAndStartMeas(HardwareSerial &port){
  xsens.ToConfigMode();
  xsens.reset();
  
  delay(2000);  // 🔁 等感測器重新開機

  bool init_success = false;
  for (int i = 0; i < 2; i++) {
    xsens.InitMT();
    delay(300);  // 給時間完成初始化
    // 簡單驗證：假設初始化成功
    init_success = true;
    break;
  }
  
  if (!init_success) {
    Serial.println("Init Sensor failed.");
    port.println("[ERROR] XSENS Init Sensor failed");
    return false;
  }

  delay(200);  // 保險延遲

  bool meas_success = false;
for (int i = 0; i < 2; i++) {
  xsens.ToMeasurementMode();
  delay(300);  // 給時間完成模式切換
  
  if (xsens.getDataStatus() != DATA_INVALID) {
    meas_success = true;
    break;
  }
}
  
  if (!meas_success) {
    Serial.println("Measurement Mode failed.");
    port.println("[ERROR] XSENS Measurement Mode failed");
    return false;
  }

  port.println("[XSENS] Init + Measurement Mode Success");
  return true;
}

// Calculates the 16-bit CRC for the given ASCII or binary message.
unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (unsigned char)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (unsigned char)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}

// ================== 動態權重系統實作 ==================

void updateMovementDetection(const my_data_2d &latlon, const my_data_u4 &hei, 
                           const my_data_3f &vel, const my_data_3f &omg, 
                           const my_data_3f &acc) {
  
  // DEBUG: 追蹤函數調用
  static unsigned long movement_call_count = 0;
  static unsigned long last_movement_debug = 0;
  movement_call_count++;
  
  if (millis() - last_movement_debug > 3000) {
    send2Serial(PIXHAWK_SERIAL, "[MOVEMENT_DEBUG] updateMovementDetection() called " + String(movement_call_count) + " times");
    send2Serial(PIXHAWK_SERIAL, "[MOVEMENT_DEBUG] omg values: [" + String(omg.float_val[0], 4) + ", " + String(omg.float_val[1], 4) + ", " + String(omg.float_val[2], 4) + "]");
    last_movement_debug = millis();
  }
  
  unsigned long current_time = millis();
  float dt = 0.033f; // 假設 30Hz 更新率
  
  if (dynamic_trust.last_update > 0) {
    dt = (current_time - dynamic_trust.last_update) / 1000.0f;
  }
  
  if (!dynamic_trust.is_initialized) {
    // 首次初始化
    dynamic_trust.prev_pos[0] = latlon.float_val[0];
    dynamic_trust.prev_pos[1] = latlon.float_val[1];
    dynamic_trust.prev_pos[2] = hei.float_val;
    
    for (int i = 0; i < 3; i++) {
      dynamic_trust.prev_vel[i] = vel.float_val[i];
      dynamic_trust.prev_omg[i] = omg.float_val[i];
    }
    
    dynamic_trust.is_initialized = true;
    dynamic_trust.trust_factor = 0.3f; // 初始較低信任度
  } else {
    // 計算位置變化（轉換為米）
    float pos_change[3];
    pos_change[0] = (latlon.float_val[0] - dynamic_trust.prev_pos[0]) * 111319.5f; // 緯度差轉米
    pos_change[1] = (latlon.float_val[1] - dynamic_trust.prev_pos[1]) * 111319.5f * cos(latlon.float_val[0] * M_PI / 180.0f); // 經度差轉米
    pos_change[2] = hei.float_val - dynamic_trust.prev_pos[2];
    
    // 計算速度變化
    float vel_change[3];
    for (int i = 0; i < 3; i++) {
      vel_change[i] = vel.float_val[i] - dynamic_trust.prev_vel[i];
    }
    
    // 計算角速度變化
    float omg_change[3];
    for (int i = 0; i < 3; i++) {
      omg_change[i] = omg.float_val[i] - dynamic_trust.prev_omg[i];
    }
    
    // 計算加速度幅值
    dynamic_trust.acceleration_mag = sqrt(acc.float_val[0]*acc.float_val[0] + 
                                        acc.float_val[1]*acc.float_val[1] + 
                                        acc.float_val[2]*acc.float_val[2]);
    
    // 計算角速度幅值
    dynamic_trust.angular_rate_mag = sqrt(omg.float_val[0]*omg.float_val[0] + 
                                        omg.float_val[1]*omg.float_val[1] + 
                                        omg.float_val[2]*omg.float_val[2]);
    
    // 更新YAW角速度歷史記錄
    dynamic_trust.yaw_rate_history[dynamic_trust.yaw_history_index] = omg.float_val[2]; // YAW角速度
    
    // DEBUG: 減少YAW更新輸出頻率
    static unsigned long last_yaw_debug = 0;
    if (millis() - last_yaw_debug >= 5000 && is_debug) { // 每5秒且debug模式時輸出 - 降低IDE輸出頻率
      send2Serial(PIXHAWK_SERIAL, "[YAW_UPDATE] Index: " + String(dynamic_trust.yaw_history_index) + 
                  ", New Value: " + String(omg.float_val[2], 6) + 
                  ", Magnitude: " + String(dynamic_trust.angular_rate_mag, 6));
      last_yaw_debug = millis();
    }
    
    dynamic_trust.yaw_history_index = (dynamic_trust.yaw_history_index + 1) % 5;
    
    // 計算位置變化幅值
    float pos_change_mag = sqrt(pos_change[0]*pos_change[0] + 
                               pos_change[1]*pos_change[1] + 
                               pos_change[2]*pos_change[2]);
    
    // 計算速度幅值
    float vel_mag = sqrt(vel.float_val[0]*vel.float_val[0] + 
                        vel.float_val[1]*vel.float_val[1] + 
                        vel.float_val[2]*vel.float_val[2]);
    
    // 綜合運動評分 (0-1)
    float position_score = min(pos_change_mag / dt / 1.0f, 1.0f); // 位置變化率
    float velocity_score = min(vel_mag / 5.0f, 1.0f);             // 速度幅值
    float accel_score = min(dynamic_trust.acceleration_mag / 10.0f, 1.0f); // 加速度
    float angular_score = min(dynamic_trust.angular_rate_mag / 1.0f, 1.0f); // 角速度
    
    // 加權平均運動評分
    dynamic_trust.movement_score = 0.3f * position_score + 
                                  0.3f * velocity_score + 
                                  0.2f * accel_score + 
                                  0.2f * angular_score;
    
    // 更新前一次數值
    dynamic_trust.prev_pos[0] = latlon.float_val[0];
    dynamic_trust.prev_pos[1] = latlon.float_val[1];
    dynamic_trust.prev_pos[2] = hei.float_val;
    
    for (int i = 0; i < 3; i++) {
      dynamic_trust.prev_vel[i] = vel.float_val[i];
      dynamic_trust.prev_omg[i] = omg.float_val[i];
    }
  }
  
  dynamic_trust.last_update = current_time;
}

float calculateTrustFactor() {
  // 基礎信任度 - 即使靜止也要有一定信任度
  float base_trust = 0.2f;
  
  // 運動加成 - 有運動時增加信任度
  float movement_bonus = dynamic_trust.movement_score * 0.7f;
  
  // 組合信任因子
  float new_trust = base_trust + movement_bonus;
  
  // 平滑過濾 - 避免劇烈變化
  float alpha = 0.1f; // 低通濾波器係數
  dynamic_trust.trust_factor = alpha * new_trust + (1.0f - alpha) * dynamic_trust.trust_factor;
  
  // 更新YAW信任因子（獨立計算）
  static unsigned long last_trust_debug = 0;
  if (millis() - last_trust_debug > 3000) {
    send2Serial(PIXHAWK_SERIAL, "[TRUST_DEBUG] About to call calculateYawTrustFactor()");
    last_trust_debug = millis();
  }
  
  float yaw_result = calculateYawTrustFactor();
  
  static unsigned long last_trust_result = 0;
  if (millis() - last_trust_result > 3000) {
    send2Serial(PIXHAWK_SERIAL, "[TRUST_DEBUG] YAW calculation returned: " + String(yaw_result, 6));
    send2Serial(PIXHAWK_SERIAL, "[TRUST_DEBUG] Global yaw_trust_factor: " + String(dynamic_trust.yaw_trust_factor, 6));
    send2Serial(PIXHAWK_SERIAL, "[TRUST_DEBUG] Global yaw_stability_score: " + String(dynamic_trust.yaw_stability_score, 6));
    last_trust_result = millis();
  }
  
  // 限制範圍
  dynamic_trust.trust_factor = constrain(dynamic_trust.trust_factor, 0.1f, 1.0f);
  
  return dynamic_trust.trust_factor;
}

float calculateYawTrustFactor() {
  // YAW軸專用信任因子：基於角速度穩定性
  
  // 檢查YAW資料是否已初始化
  bool data_initialized = false;
  for (int i = 0; i < 5; i++) {
    if (abs(dynamic_trust.yaw_rate_history[i]) > 0.001f) {
      data_initialized = true;
      break;
    }
  }
  
  // DEBUG: 強制輸出YAW計算過程來診斷問題
  static unsigned long last_func_debug = 0;
  if (millis() - last_func_debug >= 5000) { // 每5秒輸出一次 - 降低IDE輸出頻率
    send2Serial(PIXHAWK_SERIAL, "[YAW_CALC] Rates: " + 
      String(dynamic_trust.yaw_rate_history[0], 4) + ", " +
      String(dynamic_trust.yaw_rate_history[1], 4) + ", " +
      String(dynamic_trust.yaw_rate_history[2], 4) + ", " +
      String(dynamic_trust.yaw_rate_history[3], 4) + ", " +
      String(dynamic_trust.yaw_rate_history[4], 4));
    send2Serial(PIXHAWK_SERIAL, "[YAW_CALC] Initialized: " + String(data_initialized ? "YES" : "NO") + 
                               ", Function called: YES");
    last_func_debug = millis();
  }
  
  // 如果資料未初始化，返回低信任值但不為0
  if (!data_initialized) {
    dynamic_trust.yaw_stability_score = 0.2f;
    dynamic_trust.yaw_trust_factor = 0.2f;
    return dynamic_trust.yaw_trust_factor;
  }
  
  // 1. 計算YAW角速度變化的標準差
  float yaw_rate_mean = 0.0f;
  for (int i = 0; i < 5; i++) {
    yaw_rate_mean += dynamic_trust.yaw_rate_history[i];
  }
  yaw_rate_mean /= 5.0f;
  
  float yaw_rate_variance = 0.0f;
  for (int i = 0; i < 5; i++) {
    float diff = dynamic_trust.yaw_rate_history[i] - yaw_rate_mean;
    yaw_rate_variance += diff * diff;
  }
  yaw_rate_variance /= 5.0f;
  float yaw_rate_std = sqrt(yaw_rate_variance);
  
  // DEBUG: 輸出計算細節
  static unsigned long last_calc_debug = 0;
  if (millis() - last_calc_debug >= 5000) { // 降低IDE輸出頻率
    send2Serial(PIXHAWK_SERIAL, "[YAW_CALC] Mean: " + String(yaw_rate_mean, 6) + 
                               ", Std: " + String(yaw_rate_std, 6) + 
                               ", Variance: " + String(yaw_rate_variance, 6));
    last_calc_debug = millis();
  }
  
  // 2. 將標準差映射到穩定性評分 - 改進：調整為適合車輛運動的參數
  // 標準差越小，穩定性越高
  float max_acceptable_std = 0.5f; // 調整：放寬最大可接受標準差 (rad/s) ≈ 29°/s
  float normalized_std = constrain(yaw_rate_std / max_acceptable_std, 0.0f, 1.0f);
  
  // 避免除零和提供基礎穩定性
  if (yaw_rate_std < 0.001f) {
    dynamic_trust.yaw_stability_score = 0.8f; // 很穩定但不是完美
  } else {
    dynamic_trust.yaw_stability_score = 1.0f - normalized_std;
  }
  
  // 確保穩定性評分不會太低
  dynamic_trust.yaw_stability_score = max(dynamic_trust.yaw_stability_score, 0.1f);
  
  // DEBUG: 輸出穩定性計算結果
  static unsigned long last_stability_debug = 0;
  if (millis() - last_stability_debug >= 5000) { // 降低IDE輸出頻率
    send2Serial(PIXHAWK_SERIAL, "[YAW_CALC] Max_std: " + String(max_acceptable_std, 3) + 
                               ", Norm_std: " + String(normalized_std, 6) + 
                               ", Stability: " + String(dynamic_trust.yaw_stability_score, 6));
    last_stability_debug = millis();
  }
  
  // 3. 結合當前YAW角速度大小 - 改進：調整為適合車輛轉彎的係數
  float current_yaw_rate = abs(dynamic_trust.prev_omg[2]);
  float yaw_rate_factor = 1.0f / (1.0f + current_yaw_rate * 1.5f); // 調整：進一步降低懲罰係數
  
  // 4. 綜合評分
  float combined_score = (dynamic_trust.yaw_stability_score * 0.7f) + (yaw_rate_factor * 0.3f);
  
  // 5. 映射到信任因子範圍 (0.1 - 1.0)
  dynamic_trust.yaw_trust_factor = 0.1f + combined_score * 0.9f;
  
  // DEBUG: 強制輸出最終結果來診斷
  static unsigned long last_result_debug = 0;
  if (millis() - last_result_debug >= 5000) { // 每5秒輸出一次 - 降低IDE輸出頻率
    send2Serial(PIXHAWK_SERIAL, "[YAW_CALC] Current_yaw_rate: " + String(current_yaw_rate, 6) + 
                               ", Yaw_rate_factor: " + String(yaw_rate_factor, 6) + 
                               ", Combined_score: " + String(combined_score, 6));
    send2Serial(PIXHAWK_SERIAL, "[YAW_FINAL] Stability: " + String(dynamic_trust.yaw_stability_score, 6) + 
                               ", Trust: " + String(dynamic_trust.yaw_trust_factor, 6));
    last_result_debug = millis();
  }
  
  return dynamic_trust.yaw_trust_factor;
}

void adaptiveCovariance(mavlink_odometry_t &odom, float trust_factor) {
  // 協方差範圍設定：從較大的不確定性到非常高的精度
  float max_pos_cov = 1.0f;        // 最大位置不確定性 (1米)
  float min_pos_cov = 1e-6f;       // 最高位置精度 (1微米)
  
  float max_vel_cov = 1.0f;        // 最大速度不確定性 (1 m/s)
  float min_vel_cov = 1e-6f;       // 最高速度精度 (1 mm/s)
  
  float max_att_cov = 1e-1f;       // 最大姿態不確定性 (約5.7度)
  float min_att_cov = 1e-6f;       // 最高姿態精度 (調高到1e-6)
  
  // YAW軸專用信任因子計算
  float yaw_trust_factor = calculateYawTrustFactor();
  
  // 使用指數映射來實現更大的動態範圍
  // trust_factor: 0.1 -> 最大協方差, 1.0 -> 最小協方差
  float exp_factor = -6.0f * (trust_factor - 0.1f) / 0.9f; // -6 到 0 的範圍
  float yaw_exp_factor = -6.0f * (yaw_trust_factor - 0.1f) / 0.9f; // YAW專用指數因子
  
  float pos_cov = min_pos_cov + (max_pos_cov - min_pos_cov) * exp(exp_factor);
  float vel_cov = min_vel_cov + (max_vel_cov - min_vel_cov) * exp(exp_factor);
  float att_cov = min_att_cov + (max_att_cov - min_att_cov) * exp(exp_factor);
  float yaw_att_cov = min_att_cov + (max_att_cov - min_att_cov) * exp(yaw_exp_factor);
  
  // 確保在合理範圍內
  pos_cov = constrain(pos_cov, min_pos_cov, max_pos_cov);
  vel_cov = constrain(vel_cov, min_vel_cov, max_vel_cov);
  att_cov = constrain(att_cov, min_att_cov, max_att_cov);
  yaw_att_cov = constrain(yaw_att_cov, min_att_cov, max_att_cov);
  
  // 設定位置協方差
  odom.pose_covariance[0]  = pos_cov;           // x position variance
  odom.pose_covariance[7]  = pos_cov;           // y position variance  
  odom.pose_covariance[14] = pos_cov * 1.5f;    // z position variance (高度稍微不確定些)
  odom.pose_covariance[21] = att_cov;           // roll variance
  odom.pose_covariance[28] = att_cov;           // pitch variance
  odom.pose_covariance[35] = yaw_att_cov;       // yaw variance (使用專用信任因子)
  
  // 設定速度協方差
  odom.velocity_covariance[0]  = vel_cov;           // vx variance
  odom.velocity_covariance[7]  = vel_cov;           // vy variance
  odom.velocity_covariance[14] = vel_cov * 1.2f;    // vz variance (垂直速度稍不確定)
  odom.velocity_covariance[21] = att_cov * 5.0f;    // roll rate variance
  odom.velocity_covariance[28] = att_cov * 5.0f;    // pitch rate variance
  odom.velocity_covariance[35] = yaw_att_cov * 3.0f;    // yaw rate variance (使用專用信任因子)
  
  // 品質評分：設定為最高品質
  odom.quality = 100; // 固定為最高品質100
  
  // 極高信任度時使用更積極的估計器類型
  if (trust_factor > 0.8f) {
    odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION; // 純視覺估計器，EKF 最信任
  } else if (trust_factor > 0.5f) {
    odom.estimator_type = MAV_ESTIMATOR_TYPE_VIO;    // 視覺慣性里程計
  } else {
    odom.estimator_type = MAV_ESTIMATOR_TYPE_NAIVE;  // 基本估計器
  }
}

// ==================== NMEA 時間戳格式標準化功能 ====================

// 將時間戳從3位小數格式轉換為2位小數格式
// 例如: "135937.500" -> "135937.50"
String normalizeTimestampFormat(const String& timestamp) {
  if (timestamp.length() < 6) {
    return timestamp;
  }
  
  int dot_pos = timestamp.indexOf('.');
  if (dot_pos == -1) {
    return timestamp;
  }
  
  // 如果已經是2位小數，直接返回
  if (timestamp.length() == dot_pos + 3) {
    return timestamp;
  }
  
  // 如果是3位小數，截取為2位小數
  if (timestamp.length() == dot_pos + 4) {
    return timestamp.substring(0, dot_pos + 3);
  }
  
  return timestamp;
}

// 格式化速度字段為標準格式 (XXX.XXX)
String formatSpeedField(const String& speed) {
  float speed_val = speed.toFloat();
  return String(speed_val, 3).substring(0, 7); // 確保格式為XXX.XXX
}

// 標準化整個NMEA句子中的時間戳和其他字段格式
String normalizeNMEASentence(const String& nmea_sentence) {
  String result = nmea_sentence;
  
  // 步驟1: 處理包含時間戳的句子
  if (nmea_sentence.startsWith("$GNGGA") || 
      nmea_sentence.startsWith("$GNRMC") || 
      nmea_sentence.startsWith("$GNGST")) {
    // 標準化時間戳字段（第二個逗號之間）
    int first_comma = result.indexOf(',');
    if (first_comma != -1) {
      int second_comma = result.indexOf(',', first_comma + 1);
      if (second_comma != -1) {
        String timestamp = result.substring(first_comma + 1, second_comma);
        String normalized_timestamp = normalizeTimestampFormat(timestamp);
        
        // 重構NMEA句子
        result = result.substring(0, first_comma + 1);
        result += normalized_timestamp;
        result += nmea_sentence.substring(second_comma);
      }
    }
  }
  
  // 步驟2: 特殊處理GNRMC句子的速度字段
  if (result.startsWith("$GNRMC")) {
    // 解析GNRMC字段並重新格式化速度
    String fields[15]; // GNRMC最多15個字段
    int field_count = 0;
    int start = 0;
    
    // 分割字段
    for (int i = 0; i <= result.length() && field_count < 15; i++) {
      if (i == result.length() || result.charAt(i) == ',') {
        fields[field_count] = result.substring(start, i);
        field_count++;
        start = i + 1;
      }
    }
    
    // 格式化速度字段（第8個字段，索引7）
    // GNRMC格式: $GNRMC,time,status,lat,N/S,lon,E/W,speed,course,date,variation,E/W,checksum
    //           0     1    2      3   4   5   6   7     8     9    10        11  12
    if (field_count > 7 && fields[7].length() > 0) {
      float speed_val = fields[7].toFloat();
      
      // 格式化為 XXX.XXX 格式
      int integer_part = (int)speed_val;
      int decimal_part = (int)((speed_val - integer_part) * 1000);
      
      String formatted_speed = "";
      if (integer_part < 100) formatted_speed += "0";
      if (integer_part < 10) formatted_speed += "0";
      formatted_speed += String(integer_part);
      formatted_speed += ".";
      if (decimal_part < 100) formatted_speed += "0";
      if (decimal_part < 10) formatted_speed += "0";
      formatted_speed += String(decimal_part);
      
      fields[7] = formatted_speed;
      
      // 調試信息可以在需要時重新啟用
    }
    
    // 重新組裝GNRMC句子
    result = "";
    for (int i = 0; i < field_count; i++) {
      result += fields[i];
      if (i < field_count - 1) {
        result += ",";
      }
    }
  }
  
  return result;
}

// ==================== XSENS 專用 NMEA 數據生成函數(以下是測試用假資料) ====================

// 生成 HDT (Heading, True) 句子
String generateHDTSentence(float heading_degrees) {
  String hdt_sentence = "$GNHDT,";
  hdt_sentence += String(heading_degrees, 2);
  hdt_sentence += ",T";
  
  // 計算 checksum
  String checksum = cal_xor_checksum(hdt_sentence.substring(1)); // 跳過 '$'
  hdt_sentence += "*" + checksum;
  
  return hdt_sentence;
}

// 生成 PLSHD (Proprietary dual-antenna GNSS heading) 句子
String generatePLSHDSentence(float heading_degrees, float quality_factor) {
  String plshd_sentence = "$PLSHD,";
  plshd_sentence += String(heading_degrees, 2);
  plshd_sentence += ",T,";
  plshd_sentence += String(quality_factor, 1);
  
  // 計算 checksum
  String checksum = cal_xor_checksum(plshd_sentence.substring(1)); // 跳過 '$'
  plshd_sentence += "*" + checksum;
  
  return plshd_sentence;
}

// 生成 GSV (GNSS Satellites in View) 句子
String generateGSVSentence(int constellation_id, int total_messages, int message_number, int satellites_in_view) {
  String constellation = "GN"; // 默認多星座
  if (constellation_id == 1) constellation = "GP"; // GPS
  else if (constellation_id == 2) constellation = "GL"; // GLONASS
  else if (constellation_id == 3) constellation = "GA"; // Galileo
  else if (constellation_id == 4) constellation = "GB"; // BeiDou
  
  String gsv_sentence = "$" + constellation + "GSV,";
  gsv_sentence += String(total_messages);
  gsv_sentence += ",";
  gsv_sentence += String(message_number);
  gsv_sentence += ",";
  gsv_sentence += String(satellites_in_view);
  
  // 這裡可以添加具體的衛星信息，暫時使用基本格式
  for (int i = 0; i < 4; i++) { // 每個GSV句子最多4顆衛星
    gsv_sentence += ",,,"; // PRN, elevation, azimuth, SNR (空白表示無數據)
  }
  
  // 計算 checksum
  String checksum = cal_xor_checksum(gsv_sentence.substring(1)); // 跳過 '$'
  gsv_sentence += "*" + checksum;
  
  return gsv_sentence;
}

// 發送 XSENS 專用 NMEA 數據到輸出端口
void sendXSENSNMEAData(HardwareSerial &output_port) {
  unsigned long current_time = millis();
  
  // 從全域變數獲取航向角度 (弧度轉度數)
  float heading_degrees = 0.0;
  if (orientation_data_available) {
    heading_degrees = latest_orientation.float_val[2] * 180.0 / M_PI; // YAW軸，弧度轉度數
    if (heading_degrees < 0) heading_degrees += 360.0; // 確保正角度
  }
  
  // 發送 HDT (每250ms發送一次 = 4Hz)
  if (current_time - last_hdt_send >= 250) {
    String hdt_sentence = generateHDTSentence(heading_degrees);
    output_port.println(hdt_sentence);
    last_hdt_send = current_time;
    hdt_sent++;
    
    if (is_debug) {
      Serial.println("→ XSENS HDT: " + hdt_sentence);
    }
  }
  
  // 發送 PLSHD (每250ms發送一次 = 4Hz)
  if (current_time - last_plshd_send >= 250) {
    float quality_factor = dynamic_trust.yaw_trust_factor * 9.0; // 轉換為0-9品質因子
    String plshd_sentence = generatePLSHDSentence(heading_degrees, quality_factor);
    output_port.println(plshd_sentence);
    last_plshd_send = current_time;
    plshd_sent++;
    
    if (is_debug) {
      Serial.println("→ XSENS PLSHD: " + plshd_sentence);
    }
  }
  
  // 發送 GSV (每250ms發送一次 = 4Hz)
  if (current_time - last_gsv_send >= 250) {
    // 發送多星座 GSV 數據
    String gsv_sentence = generateGSVSentence(0, 1, 1, 12); // 假設12顆可見衛星
    output_port.println(gsv_sentence);
    last_gsv_send = current_time;
    gsv_sent++;
    
    if (is_debug) {
      Serial.println("→ XSENS GSV: " + gsv_sentence);
    }
  }
}

// 資料清理函數實作
bool isValidFloat(float value) {
  return !isnan(value) && !isinf(value);
}

float constrainFloat(float value, float min_val, float max_val) {
  if (!isValidFloat(value)) return (min_val + max_val) / 2.0f;  // 返回中點值
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

void cleanSensorData(my_data_4f &qut, my_data_3f &omg, my_data_3f &acc) {
  // 靜態備份資料
  static my_data_4f last_valid_qut = {1.0f, 0.0f, 0.0f, 0.0f};  // 單位四元數
  static my_data_3f last_valid_omg = {0.0f, 0.0f, 0.0f};
  static my_data_3f last_valid_acc = {0.0f, 0.0f, 9.81f};  // 重力向量
  
  // 清理四元數
  bool qut_valid = true;
  for (int i = 0; i < 4; i++) {
    if (!isValidFloat(qut.float_val[i])) {
      qut_valid = false;
      break;
    }
  }
  
  if (qut_valid) {
    // 檢查四元數範數
    float norm = sqrt(qut.float_val[0]*qut.float_val[0] + 
                     qut.float_val[1]*qut.float_val[1] + 
                     qut.float_val[2]*qut.float_val[2] + 
                     qut.float_val[3]*qut.float_val[3]);
    
    if (norm > 0.1f && norm < 2.0f) {  // 放寬範圍
      // 正規化四元數
      for (int i = 0; i < 4; i++) {
        qut.float_val[i] /= norm;
        last_valid_qut.float_val[i] = qut.float_val[i];
      }
    } else {
      // 使用上一筆有效值
      for (int i = 0; i < 4; i++) {
        qut.float_val[i] = last_valid_qut.float_val[i];
      }
    }
  } else {
    // 使用上一筆有效值
    for (int i = 0; i < 4; i++) {
      qut.float_val[i] = last_valid_qut.float_val[i];
    }
  }
  
  // 清理角速度
  for (int i = 0; i < 3; i++) {
    if (isValidFloat(omg.float_val[i])) {
      omg.float_val[i] = constrainFloat(omg.float_val[i], -20.0f, 20.0f);  // 放寬範圍
      last_valid_omg.float_val[i] = omg.float_val[i];
    } else {
      omg.float_val[i] = last_valid_omg.float_val[i];
    }
  }
  
  // 清理加速度
  for (int i = 0; i < 3; i++) {
    if (isValidFloat(acc.float_val[i])) {
      acc.float_val[i] = constrainFloat(acc.float_val[i], -50.0f, 50.0f);  // 放寬範圍
      last_valid_acc.float_val[i] = acc.float_val[i];
    } else {
      acc.float_val[i] = last_valid_acc.float_val[i];
    }
  }
  
  // 特殊情況：如果加速度全為0，設定為重力向量
  float acc_mag = sqrt(acc.float_val[0]*acc.float_val[0] + 
                      acc.float_val[1]*acc.float_val[1] + 
                      acc.float_val[2]*acc.float_val[2]);
  if (acc_mag < 1.0f) {  // 如果加速度太小
    acc.float_val[0] = 0.0f;
    acc.float_val[1] = 0.0f;
    acc.float_val[2] = 9.81f;  // 設定為標準重力
  }
}

// 時間戳驗證與平滑函數實作
uint64_t getUnifiedTimestamp(const my_data_u4 &XsensTime) {
  return uint64_t(XsensTime.ulong_val) * 1e2 + time_offset;
}

bool validateTimestamp(uint64_t timestamp) {
  if (last_unified_timestamp == 0) {
    last_unified_timestamp = timestamp;
    return true;
  }
  
  uint64_t time_diff = (timestamp > last_unified_timestamp) ? 
                       (timestamp - last_unified_timestamp) : 
                       (last_unified_timestamp - timestamp);
  
  // 檢查時間戳跳躍 (超過1秒視為異常)
  if (time_diff > 1000000) { // 1秒 = 1,000,000微秒
    if (is_debug) {
      static unsigned long last_timestamp_warning = 0;
      if (millis() - last_timestamp_warning > 1000) { // 1Hz限制
        Serial.println("[TIMESTAMP WARNING] 時間戳跳躍: " + String((unsigned long)time_diff) + "μs");
        last_timestamp_warning = millis();
      }
    }
    return false;
  }
  
  return true;
}

uint64_t smoothTimestamp(uint64_t timestamp) {
  if (!validateTimestamp(timestamp)) {
    // 異常時間戳，使用平滑值
    uint64_t smooth_timestamp = last_unified_timestamp + 10000; // 假設10ms間隔
    if (is_debug) {
      static unsigned long last_smooth_output = 0;
      if (millis() - last_smooth_output > 1000) { // 1Hz限制
        Serial.println("[TIMESTAMP SMOOTH] 原始: " + String((unsigned long)timestamp) + " → 平滑: " + String((unsigned long)smooth_timestamp));
        last_smooth_output = millis();
      }
    }
    last_unified_timestamp = smooth_timestamp;
    return smooth_timestamp;
  }
  
  last_unified_timestamp = timestamp;
  return timestamp;
}

