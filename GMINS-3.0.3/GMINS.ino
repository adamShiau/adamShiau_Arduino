
// 🔧 確保LOG輸出開啟
#define LOG_LEVEL_GLOBAL LOG_DEBUG    // 全域LOG級別設為DEBUG
#define LOG_LEVEL_MASK_GLOBAL 0xFFFF  // 開啟所有LOG遮罩

// Phase 5 includes - TDD 與頻率管理
#include "src/util/log.h"
#include "src/hal/board_support.h"
#include "src/core/system_controller.h"
#include "src/core/DataFlowIntegrator.h"
#include "src/core/ProtocolManagerDualMode.h"  // 測試模式需要
#include "src/util/command_handler.h"
#include "src/drivers/xsens_mti_driver.h"
#include "src/util/tdd_flow_checker.h"  // TDD 輕量級流程檢測
#include "src/util/gmins_frequency_manager.h"  // 封包頻率測量
#define MAVLINK_USE_MESSAGE_INFO  // 使用訊息資訊
#define MAVLINK_MAX_DIALECT_PAYLOAD_SIZE 255
#ifndef MAVLINK_WMM_GEOMAGNETICMODEL_H
#define MAVLINK_STX 0xFE
#endif
#include <MAVLink.h>  // 最小化測試需要
#include <cstring>    // memset


// Phase 5 全域變數 - TDD 與頻率管理
SystemController* system_controller = nullptr;
CommandHandler* command_handler = nullptr;

// TDD Flow Checkers - 監控關鍵數據流 (已升級為頻率計算版本)
TDDFlowChecker gmins_main_loop_checker("GMINS:mainLoop", "mainLoop", 5000, false);              // 主循環狀態監控 (不需頻率)
TDDFlowChecker hal_sensor_checker("HAL:processAllSensorData", "processAllSensorData", 5000, true);    // 感測器數據處理頻率監控

// 系統運行統計
struct SystemStats {
  uint32_t total_cycles = 0;
  uint32_t nav_callbacks = 0;
  uint32_t protocol_switches = 0;
} sys_stats;

// 全域變數用於記錄 MTI 原始資料
static uint8_t mti_raw_data_count = 0;
static bool logging_mti_data = true;
static uint32_t mti_log_start_time = 0;

/*
// 其餘變數暫時保留
*/

// 🔍 MAVLink 封包解析函數
void parseMAVLinkPacket(const uint8_t* buffer, uint16_t len) {
  if (len < 8) {
    LOGI("📦 封包太短，無法解析");
    return;
  }
  
  uint8_t stx = buffer[0];
  uint8_t payload_len = buffer[1];
  uint8_t seq, sysid, compid;
  uint32_t msgid;
  
  if (stx == 0xFE) {
    // MAVLink v1: STX(1) + LEN(1) + SEQ(1) + SYSID(1) + COMPID(1) + MSGID(1) + PAYLOAD(...) + CRC(2)
    seq = buffer[2];
    sysid = buffer[3];
    compid = buffer[4];
    msgid = buffer[5];
  } else if (stx == 0xFD) {
    // MAVLink v2: STX(1) + LEN(1) + INC_FLAGS(1) + CMP_FLAGS(1) + SEQ(1) + SYSID(1) + COMPID(1) + MSGID(3) + PAYLOAD + CRC(2)
    seq = buffer[4];
    sysid = buffer[5];
    compid = buffer[6];
    msgid = buffer[7] | (buffer[8] << 8) | (buffer[9] << 16);  // 3 bytes msgid
  } else {
    LOGI("❌ 無效的 MAVLink 起始標誌: 0x%02X", stx);
    return;
  }
  
  LOGI("=== MAVLink 封包解析 ===");
  if (stx == 0xFE) {
    LOGI("📋 STX: 0x%02X (MAVLink v1)", stx);
  } else if (stx == 0xFD) {
    LOGI("📋 STX: 0x%02X (MAVLink v2)", stx);
  } else {
    LOGI("📋 STX: 0x%02X (無效)", stx);
  }
  LOGI("📏 長度: %d bytes", payload_len);
  LOGI("🔢 序號: %d", seq);
  LOGI("🆔 系統ID: %d", sysid);
  LOGI("🔧 組件ID: %d", compid);
  
  // 檢查是否為 GPS_INPUT (ID=232, 0xE8)
  if (msgid == 232) {
    LOGI("📨 訊息ID: %lu (GPS_INPUT)", (unsigned long)msgid);
    
    // 解析 GPS_INPUT 封包內容
    const uint8_t* payload;
    size_t header_size;
    
    if (stx == 0xFE) {
      header_size = 6;  // MAVLink v1 header
      payload = &buffer[6];
    } else {
      header_size = 10; // MAVLink v2 header
      payload = &buffer[10];
    }
    
    LOGI("🔍 封包檢查: 總長度=%d, header_size=%d, payload_len=%d", len, header_size, payload_len);
    
    // GPS_INPUT 的 payload 大小是可變的，只要有基本字段即可
    size_t min_gps_input_payload = 59;  // GPS_INPUT 最小 payload 大小
    
    if (len >= header_size + min_gps_input_payload + 2) {  // header + min_payload + crc(2)
      
      // GPS_INPUT 結構 (注意字節序)
      uint64_t time_usec;
      uint8_t gps_id;
      uint8_t fix_type;
      int32_t lat, lon;
      float alt, hdop, vdop;
      float vn, ve, vd;
      float speed_accuracy, horiz_accuracy, vert_accuracy;
      uint16_t ignore_flags, yaw;
      uint8_t satellites_visible;
      
      // 🚨 記憶體安全版本 - 直接從 payload 手動解析關鍵字段
      // 避免大結構在堆疊上分配
      
      // 只解析關鍵字段，避免完整結構解析
      if (payload_len >= 62) {  // GPS_INPUT 需要至少62字節包含 gps_id 和 fix_type
        // 直接從 payload 複製關鍵字段 (小心字節對齊)
        memcpy(&time_usec, payload + 0, sizeof(time_usec));
        gps_id = payload[60];
        fix_type = payload[61]; 
        memcpy(&lat, payload + 12, sizeof(lat));
        memcpy(&lon, payload + 16, sizeof(lon));
        
        // 只解析最重要的字段，其他設為預設值
        alt = 100.0f;  // 固定值避免解析
        hdop = 1.0f;
        vdop = 1.5f;
        satellites_visible = 8;
        
        // 其他字段設為安全值
        vn = ve = vd = 0.0f;
        speed_accuracy = 0.05f;
        horiz_accuracy = 0.02f; 
        vert_accuracy = 0.03f;
        ignore_flags = 0;
        yaw = 0;
      } else {
        LOGI("📦 封包長度不足，跳過解析");
        return;
      }
      
      LOGI("📍 GPS_INPUT 內容:");
      LOGI("⏰ 時間: %lu 微秒", (unsigned long)time_usec);
      LOGI("🆔 GPS ID: %d", gps_id);
      LOGI("📡 定位類型: %d (0=無, 1=2D, 2=3D, 3=DGPS, 4=RTK Float, 5=RTK Fixed)", fix_type);
      LOGI("📍 緯度: %ld (1E7) = %.7f°", (long)lat, (double)lat / 1E7);
      LOGI("📍 經度: %ld (1E7) = %.7f°", (long)lon, (double)lon / 1E7);
      LOGI("🏔️ 高度: %.2f 米", alt);
      LOGI("📏 精度: HDOP=%.2f, VDOP=%.2f", hdop, vdop);
      LOGI("🏃 速度: N=%.2f, E=%.2f, D=%.2f m/s", vn, ve, vd);
      LOGI("📊 精確度: 速度±%.3f, 水平±%.3f, 垂直±%.3f 米", speed_accuracy, horiz_accuracy, vert_accuracy);
      LOGI("🛰️ 可見衛星: %d 顆", satellites_visible);
      LOGI("🚫 忽略標誌: 0x%04X", ignore_flags);
      LOGI("🧭 航向: %d (度×100)", yaw);
    } else {
      LOGI("⚠️ GPS_INPUT 封包長度不足");
    }
  } else if (msgid == 0) {
    LOGI("📨 訊息ID: %lu (HEARTBEAT)", (unsigned long)msgid);
  } else if (msgid == 1) {
    LOGI("📨 訊息ID: %lu (SYS_STATUS)", (unsigned long)msgid);
  } else {
    LOGI("📨 訊息ID: %lu (未知訊息類型)", (unsigned long)msgid);
  }
  
  // 顯示原始 hex (只顯示前16字節)
  char hex_str[64] = "";
  char temp[8];
  for (int i = 0; i < len && i < 16; i++) {
    sprintf(temp, "%02X ", buffer[i]);
    strcat(hex_str, temp);
  }
  if (len > 16) strcat(hex_str, "...");
  LOGI("📦 原始封包: %s", hex_str);
  LOGI("========================");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // 🔧 確認Serial和LOG系統正常
  Serial.println("=== SERIAL TEST: Arduino setup() started ===");
  LOGI("=== LOG TEST: LOGI macro working ===");
  
  // 初始化硬體抽象層
  hal::initPeripherals();
  
  // 🔧 重要補充：初始化感測器測試系統
  LOGI("🔧 初始化XSENS測試系統...");
  hal::initializeXSENSTest();
  
  // 創建並初始化系統控制器
  system_controller = new SystemController();
  
  if (system_controller->initialize()) {
    LOGI("✅ SystemController 初始化成功");
    
    // 🔧 MTI 設備快速初始化
    LOGI("🔧 開始 MTI 設備初始化...");
    setupMTIDevice();
    
    // 🎯 立即執行 CUBE/Pixhawk 自動偵測 (無等待時間)
    LOGI("🎯 開始 CUBE/Pixhawk 自動偵測...");
    bool cube_detected = system_controller->detectPixhawkAndSetMode();
    if (cube_detected) {
      LOGI("✅ CUBE/Pixhawk 檢測成功，已切換至MAVLink模式");
    } else {
      LOGI("🎯 未檢測到CUBE/Pixhawk，使用AR1AFC模式");
    }
    
    // 啟用數據流
    system_controller->enableDataFlow(true);
    LOGI("✅ 數據流已啟用");
    
    // 創建命令處理器
    command_handler = new CommandHandler(system_controller, 
                                       &sys_stats.protocol_switches,
                                       &sys_stats.nav_callbacks,
                                       &sys_stats.total_cycles);
    LOGI("✅ 命令處理器已初始化");
    
    LOGI("✅ Phase 1 初始化完成 - 進入運行狀態");
  } else {
    LOGE("❌ SystemController 初始化失敗");
  }
}

void loop() {
  // 🔍 暫時關閉頻率控制，測試真實調用頻率
  
  // 🚨 逐一測試死機原因 - 暫時停用所有可能的死機源
  
  // 測試1: 停用 HAL 數據處理
  // hal::processAllSensorData();
  
  // 🎮 處理Serial命令 (應該是安全的)
  if (command_handler) {
    command_handler->checkAndProcessCommands();
  }
  
  // 測試2: 重新啟用系統週期處理 - 測試是否為死機源
  processOneCycle();
  
  // 測試3: 重新啟用數據流處理 - 測試是否為死機源
  if (system_controller && system_controller->getState() == SystemState::RUNNING) {
    system_controller->tick(); // 觸發ProtocolManagerDualMode數據流
  }
  
  sys_stats.total_cycles++;

  // 協議觸發 + 封包頻率計算
  static uint32_t last_trigger_time = 0;
  static uint32_t protocol_packet_count = 0;
  uint32_t current_time = millis();
  
  if (system_controller && system_controller->getState() == SystemState::RUNNING) {
    if (current_time - last_trigger_time >= 10) {  // 每10ms觸發
      if (system_controller->getProtocolManager()) {
        system_controller->getProtocolManager()->continuousUpdate(GNSSData{}, NavigationState{}, IMUData{});
        protocol_packet_count++;  // 計算實際協議輸出
      }
      last_trigger_time = current_time;
    }
  }
  
  // 每秒顯示當前協議的輸出頻率
  static uint32_t last_heartbeat_time = 0;
  static uint32_t heartbeat_counter = 0;
  static uint32_t last_protocol_count = 0;
  
  if (current_time - last_heartbeat_time >= 1000) {
    heartbeat_counter++;
    uint32_t packets_per_sec = protocol_packet_count - last_protocol_count;
    last_protocol_count = protocol_packet_count;
    
    const char* protocol_name = system_controller ? system_controller->getCurrentProtocolName() : "INIT";
    LOGI("💓 #%lu - %s: %luHz", heartbeat_counter, protocol_name, packets_per_sec);
    last_heartbeat_time = current_time;
  }
  
  // 移除 10 秒等待的 MTI Heading Reset 延遲執行
  
  // 🧪 暫時測試 Serial1 接收（每3秒檢查一次）
  static uint32_t last_serial1_check = 0;
  // 注掉主循環的 Serial1 調試信息 - 已確認命令接收正常工作
  // if (current_time - last_serial1_check > 3000) {
  //   if (hal::Serial1.available() > 0) {
  //     Serial.println("🔍 [主循環] Serial1 收到資料！");
  //     Serial.print("可用字節數: ");
  //     Serial.println(hal::Serial1.available());
  //     
  //     // 讀取並顯示所有資料
  //     while (hal::Serial1.available()) {
  //       char c = hal::Serial1.read();
  //       Serial.print("0x");
  //       Serial.print((uint8_t)c, HEX);
  //       Serial.print(" ");
  //     }
  //     Serial.println();
  //   } else {
  //     Serial.println("🔍 [主循環] Serial1 無資料");
  //   }
  //   last_serial1_check = current_time;
  // }
  
  delay(3);
}

// 處理一個系統週期
void processOneCycle() {
  if (!system_controller) return;
  
  
  hal::processAllSensorData();
  
 
  system_controller->tick(); // 處理完整的數據流
}

// MTI 設備初始化設定
void setupMTIDevice() {
  // 獲取 MTI UART 介面
  UARTInterface* mti_interface = hal::getIMUInterface();
  if (!mti_interface) {
    LOGE("❌ 無法獲取 MTI UART 介面");
    return;
  }
  
  // 創建 MTI 驅動程序並執行完整初始化 (包含 Heading Reset)
  XsensMTIDriver mti_driver(mti_interface);
  mti_driver.initialize();
  
  // 立即執行 Heading Reset，不等待
  LOGI("🔧 執行 MTI Heading Reset...");
  mti_driver.resetHeading();
  
  LOGI("✅ MTI 完整初始化完成");
}

// 移除了延遲執行的 Heading Reset 函數，改為在 setupMTIDevice() 中直接執行

// 報告系統統計
void reportSystemStats() {
  if (system_controller) {
    auto stats = system_controller->getStats();
    LOGI("📊 系統統計:");
    LOGI("    DFI 處理: %lu", stats.dfi_process_count);
    LOGI("    協議發送: %lu", stats.protocol_send_count);
    LOGI("    當前協議: %s", system_controller->getCurrentProtocolName());
    
    // 顯示數據流狀態
    if (sys_stats.nav_callbacks > 0) {
      LOGI("  🟢 數據流狀態: 正常（有導航回調）");
    } else {
      LOGW("  🟡 數據流狀態: 等待數據（無導航回調）");
    }
  }
  
  LOGI("  總循環數: %lu", sys_stats.total_cycles);
  LOGI("  協議切換: %lu", sys_stats.protocol_switches);
  LOGI("===============================");
}

