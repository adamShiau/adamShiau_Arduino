#ifndef GNSS_AHRS_FUSION_H
#define GNSS_AHRS_FUSION_H

#include <Arduino.h>
#include <MAVLink.h>
#include "../communication/myMessage.h"

// 輸出配置結構
struct OutputConfig {
    bool enable_gps_raw_int = false;
    bool enable_gps_status = false;
    bool enable_gps_input = false;
    bool enable_odometry = true;
};

// 融合模組狀態
struct FusionStatus {
    bool gnss_valid = false;
    bool mtdata2_valid = false;
    bool fusion_active = false;
    bool mavlink_sent = false;
    unsigned long last_gnss_update = 0;
    unsigned long last_mtdata2_update = 0;
    unsigned long last_fusion_send = 0;
    int gnss_quality = 0;
    int fusion_count = 0;
    int mavlink_send_count = 0;
};

// 衛星資訊結構
struct SatelliteInfo {
    uint8_t prn;        // 衛星 ID
    uint8_t elevation;  // 仰角 (度)
    uint16_t azimuth;   // 方位角 (度) - 使用 uint16_t 支援 0-359
    uint8_t snr;        // SNR (dBHz，空值填0)
    bool used;          // 是否用於定位
};

// 擴展 GNSS 資料結構
struct GNSSExtendedData {
    SatelliteInfo satellites[20];  // 支援最多20顆衛星
    int satellites_in_view = 0;    // GSV中的衛星數量
    int gga_quality = 0;          // GGA quality indicator
    int satellites_used = 0;       // GGA 中用於定位的衛星數
    
    // 雙天線 GNSS 支援 ($PLSHD)
    bool plshd_valid = false;      // $PLSHD 資料有效性
    int ant1_satellites = 0;       // 天線1衛星數
    int ant2_satellites = 0;       // 天線2衛星數
    float baseline_length_m = 0.0; // 基線長度 (公尺)
    float heading_deg = 0.0;       // 雙天線航向角 (度)
    float elevation_deg = 0.0;     // 仰角 (度)
    unsigned long plshd_timestamp = 0; // $PLSHD 時間戳
};

// GNSS 資料結構
struct GNSSData {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double velocity_north = 0.0;
    double velocity_east = 0.0;
    double velocity_down = 0.0;
    double course = 0.0;
    double speed = 0.0;
    int quality = 0;
    int satellites = 0;  // 衛星數量
    bool valid = false;
    unsigned long timestamp = 0;
    float gnss_heading_deg = 0.0;
    float yaw_rad = 0.0;
    float yaw_accuracy = 0.0;
    float antenna_separation_m = 0.0;
    bool heading_valid = false;
};

// MTDATA2 資料結構
struct MTData2 {
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float angular_velocity[3] = {0.0f, 0.0f, 0.0f};
    float acceleration[3] = {0.0f, 0.0f, 0.0f};
    float orientation[3] = {0.0f, 0.0f, 0.0f};
    uint32_t timestamp = 0;
    bool valid = false;
};

// ODOMETRY 數據結構 - 用於脫鉤傳送
struct OdomData {
    // 位置 (來自GNSS)
    double pos_n = 0.0;         // North position (latitude)
    double pos_e = 0.0;         // East position (longitude) 
    double pos_d = 0.0;         // Down position (altitude)
    
    // 速度 (來自GNSS)
    float vel_n = 0.0f;         // North velocity
    float vel_e = 0.0f;         // East velocity
    float vel_d = 0.0f;         // Down velocity
    
    // 姿態 (來自IMU)
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Quaternion [w,x,y,z]
    float angular_vel[3] = {0.0f, 0.0f, 0.0f}; // Angular velocity [roll,pitch,yaw]
    
    // 協方差矩陣
    float pose_covariance[36] = {0};      // 6x6 pose covariance
    float velocity_covariance[36] = {0};  // 6x6 velocity covariance
    
    // 元數據
    uint64_t timestamp_usec = 0;        // 微秒時間戳
    uint8_t frame_id = 0;               // 座標系ID
    uint8_t child_frame_id = 0;         // 子座標系ID
    uint8_t estimator_type = 0;         // 估計器類型
    uint8_t quality = 100;              // 數據品質
    uint8_t reset_counter = 0;          // 重置計數器
    
    bool valid = false;                 // 數據有效性
};

// 傳送統計結構
struct TransmissionStats {
    uint32_t tx_ok_count = 0;          // 成功傳送計數
    uint32_t tx_drop_count = 0;        // 丟棄計數
    unsigned long last_stats_time = 0;  // 上次統計時間
    float success_rate = 100.0f;       // 成功率百分比
};

// 融合模組類別
class GNSSAHRSFusion {
private:
    GNSSData gnss_data;
    MTData2 mtdata2_data;
    FusionStatus status;
    bool debug_enabled = false;
    bool time_check_enabled = false;
    
    // 脫鉤傳送相關
    OdomData latest_odom_data;          // 最新融合數據
    TransmissionStats tx_stats;         // 傳送統計
    
    // 內部函數
    void debugPrint(const String& message);
    void updateFusionStatus();
    bool isDataReady();
    void adaptiveFusionCovariance(mavlink_odometry_t &odom);
    float calculateAverageSNR();
    void parseGSVSentence(const String& nmea_sentence);
    
public:
    GNSSAHRSFusion();
    
    // 設定函數
    void setDebugMode(bool enabled);
    void setTimeCheckMode(bool enabled);
    
    // 資料輸入函數
    void updateGNSSData(const String& nmea_sentence);
    void updateMTData2(const my_data_4f& quat, const my_data_3f& omg, 
                      const my_data_3f& acc, const my_data_3f& ori, uint32_t timestamp);
    
    // 主要融合函數
    bool processFusion();
    
    // MAVLink 輸出函數
    bool sendMAVLinkOdometry(HardwareSerial& serial_port, uint64_t timestamp_usec);
    bool smartSendOdometry(HardwareSerial& serial_port, uint64_t timestamp_usec);
    void sendMAVLink_GPS_RAW_INT(HardwareSerial& serial_port, uint64_t timestamp_usec);
    void sendMAVLink_GPS_STATUS(HardwareSerial& serial_port);
    void sendMAVLink_GPS_INPUT(HardwareSerial& serial_port, uint64_t timestamp_usec);
    
    // 脫鉤機制函數
    void updateOdomData();              // 更新融合數據到latest_odom_data
    const OdomData& getLatestOdom();    // 獲取最新可用數據
    void printTransmissionStats();      // 顯示傳送統計
    
    // 狀態查詢函數
    FusionStatus getFusionStatus();
    GNSSData getGNSSData();  // 新增：獲取 GNSS 詳細數據
    void printDetailedStatus();
    
    // 測試函數
    void runDiagnostics();
};

// 函數宣告
uint8_t convertFixType(int gga_quality);

// 外部變數宣告
extern OutputConfig fusionConfig;
extern GNSSExtendedData gnss_extended_data;
extern unsigned long last_gps_raw_time;
extern unsigned long last_gps_status_time;
extern const unsigned long GPS_RAW_INTERVAL;
extern const unsigned long GPS_STATUS_INTERVAL;

#endif // GNSS_AHRS_FUSION_H