#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <Arduino.h>

// GPS 定位狀態
enum GpsFixType {
    GPS_NO_FIX = 0,
    GPS_2D_FIX = 2,
    GPS_3D_FIX = 3,
    GPS_DGPS = 4,
    GPS_RTK_FLOAT = 5,
    GPS_RTK_FIXED = 6
};

// 衛星系統類型
enum SatSystemType {
    SAT_GPS = 0,
    SAT_GLONASS = 1,
    SAT_GALILEO = 2,
    SAT_SYSTEM_COUNT = 3
};

// 單一衛星系統資料
struct SatelliteSystem {
    uint8_t sat_count;          // 該星系可見衛星數
    uint8_t cno_values[12];     // 最多12顆衛星的CNO值 (dB-Hz)
    uint8_t prn_numbers[12];    // 對應的衛星PRN號碼
    uint8_t valid_sats;         // 有效的衛星數量 (CNO > 0)

    void reset() {
        sat_count = 0;
        valid_sats = 0;
        for (int i = 0; i < 12; i++) {
            cno_values[i] = 0;
            prn_numbers[i] = 0;
        }
    }
};

// GNSS 資料結構
struct GpsData {
    // 位置資料
    double latitude;            // 緯度 (度)
    double longitude;           // 經度 (度)
    float altitude;             // 海拔 (米)

    // 定位品質
    uint8_t fix_type;           // 定位狀態
    uint8_t satellites_used;    // 參與定位的衛星數
    uint8_t satellites_visible; // 可見衛星總數
    float hdop;                 // 水平精度因子

    // 運動資料
    float speed;                // 地面速度 (m/s)
    float heading;              // 航向角 (度)

    // 海拔濾波資料
    float altitude_history[10]; // 最近10個海拔值
    int altitude_count;         // 已收集的數量 (0-10)
    int altitude_index;         // 循環索引
    float altitude_mean;        // 平均值
    float altitude_std_dev;     // 標準差
    bool altitude_stable;       // 海拔是否穩定

    volatile uint32_t gga_epoch;   // 每成功解析到一筆GGA就+1

    // 四個衛星系統的 CNO 資料
    SatelliteSystem sat_systems[SAT_SYSTEM_COUNT];

    // 時間資料 (高精度)
    uint8_t hour, minute, second;
    uint16_t millisecond;           // 毫秒 (0-999)
    uint8_t day, month;
    uint16_t year;

    // 頻率計算變量
    uint32_t gga_count;             // GGA 封包計數
    uint32_t gga_start_time;        // 開始統計時間 (ms)
    float gga_frequency;            // 計算出的 GGA 頻率 (Hz)

    // 狀態標記
    bool valid_position;
    bool valid_time;
    uint32_t last_update;

    void reset() {
        latitude = longitude = 0.0;
        altitude = 0.0;
        fix_type = GPS_NO_FIX;
        satellites_used = satellites_visible = 0;
        hdop = 99.99;
        speed = heading = 0.0;

        // 初始化海拔濾波變量
        altitude_count = 0;
        altitude_index = 0;
        altitude_mean = 0.0;
        altitude_std_dev = 0.0;
        altitude_stable = false;
        for (int i = 0; i < 10; i++) {
            altitude_history[i] = 0.0;
        }

        gga_epoch = 0;

        // 初始化頻率計算變量
        gga_count = 0;
        gga_start_time = millis();
        gga_frequency = 0.0;

        hour = minute = second = 0;
        millisecond = 0;
        day = month = 0;
        year = 0;
        valid_position = valid_time = false;
        last_update = 0;

        for (int i = 0; i < SAT_SYSTEM_COUNT; i++) {
            sat_systems[i].reset();
        }
    }
};

class NmeaParser {
public:
    NmeaParser();

    // 主要解析函數
    bool parseLine(const char* nmea_line);

    // 12/20 新增：以 GGA 為 epoch 的解析版本
    bool parseGGA_epoch(const char* line);  // 以GGA時間變化推進epoch

    // 資料存取
    GpsData* getData() { return &gps_data; }
    bool hasValidFix() { return gps_data.fix_type >= GPS_2D_FIX; }
    bool hasRtkFix() { return gps_data.fix_type >= GPS_RTK_FLOAT; }


    // 工具函數
    void reset();
    bool isDataFresh(uint32_t timeout_ms = 5000);

private:
    GpsData gps_data;

    // NMEA 句子解析器
    bool parseGGA(const char* line);    // 位置和定位品質
    bool parseRMC(const char* line);    // 時間、速度和航向
    bool parseHDT(const char* line);    // 真航向
    bool parseGSA(const char* line);    // DOP 和使用的衛星
    bool parseGSV(const char* line);    // 衛星可見度和 CNO

    // 海拔濾波函數
    bool updateAltitudeFilter(float raw_altitude);

    // GGA 頻率計算
    void updateGGAFrequency();
    void calculateAltitudeStats();

    // 輔助函數
    int splitFields(const char* line, char fields[][20], int max_fields);
    double parseCoordinate(const char* coord, const char* direction);
    float parseFloat(const char* str);
    int parseInt(const char* str);
    bool parseTime(const char* time_str);
    bool parseDate(const char* date_str);
    int identifyTalkerID(const char* line);     // 識別星系類型
    bool validateChecksum(const char* line);    // 驗證校驗和
};

#endif // NMEA_PARSER_H