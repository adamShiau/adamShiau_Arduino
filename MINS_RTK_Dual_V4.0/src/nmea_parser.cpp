#include "nmea_parser.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

NmeaParser::NmeaParser() {
    reset();
}

void NmeaParser::reset() {
    gps_data.reset();
}

bool NmeaParser::parseLine(const char* nmea_line) {
    if (!nmea_line || strlen(nmea_line) < 6) {
        return false;
    }

    // 只處理標準 NMEA 格式（以 $ 開頭）
    if (nmea_line[0] != '$') {
        return false;
    }

    // 檢查是否為 PAIR 命令回覆
    if (strncmp(nmea_line + 1, "PAIR", 4) == 0) {
        return false;  // 這是命令回覆，不是 NMEA 資料，忽略
    }


    // 判斷句子類型並解析
    const char* msg_type = nmea_line + 3; // 跳過 $xx 部分

    if (strncmp(msg_type, "GGA", 3) == 0) {
        // return parseGGA(nmea_line);
        return parseGGA_epoch(nmea_line);
    }
    // 不再使用RMC句子，改為完全依賴GGA
    // else if (strncmp(msg_type, "RMC", 3) == 0) {
    //     return parseRMC(nmea_line);
    // }
    else if (strncmp(msg_type, "HDT", 3) == 0) {
        return parseHDT(nmea_line);
    }
    else if (strncmp(msg_type, "GSA", 3) == 0) {
        return parseGSA(nmea_line);
    }
    else if (strncmp(msg_type, "GSV", 3) == 0) {
        return parseGSV(nmea_line);
    }

    return false; // 未知句子類型
}

bool NmeaParser::parseGGA_epoch(const char* line) {
    // $GNGGA,time,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt,alt_unit,height,height_unit,dgps,dgps_id*checksum

    // ✅ 保留：更新 GGA 頻率統計
    updateGGAFrequency();

    char fields[15][20];
    int field_count = splitFields(line, fields, 15);
    if (field_count < 10) {
        return false;
    }

    // --- 先記住舊時間（用來判斷「時間是否真的更新」）---
    uint8_t prev_h = gps_data.hour;
    uint8_t prev_m = gps_data.minute;
    uint8_t prev_s = gps_data.second;
    uint16_t prev_ms = gps_data.millisecond;

    bool time_ok = false;

    // 解析UTC時間 (GGA句子的第1個欄位)
    if (strlen(fields[1]) > 0) {
        time_ok = parseTime(fields[1]);  // 你的 parseTime 會在沒有 .sss 時把 millisecond 設 0
        if (time_ok) {
            gps_data.valid_time = true;

            // 你原本的日期處理保留
            gps_data.day = 16;
            gps_data.month = 12;
            gps_data.year = 2025;

            // ⭐ 只有「時間真的變了」才推進 epoch
            if (gps_data.hour != prev_h ||
                gps_data.minute != prev_m ||
                gps_data.second != prev_s ||
                gps_data.millisecond != prev_ms) {
                gps_data.gga_epoch++;
            }
        }
    }

    // 解析緯度
    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
        gps_data.latitude = parseCoordinate(fields[2], fields[3]);
        gps_data.valid_position = true;
    }

    // 解析經度
    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
        gps_data.longitude = parseCoordinate(fields[4], fields[5]);
        gps_data.valid_position = true;
    }

    // 解析定位品質
    if (strlen(fields[6]) > 0) {
        int quality = parseInt(fields[6]);
        switch (quality) {
            case 0: gps_data.fix_type = GPS_NO_FIX; break;
            case 1: gps_data.fix_type = GPS_3D_FIX; break;
            case 2: gps_data.fix_type = GPS_DGPS; break;
            case 4: gps_data.fix_type = GPS_RTK_FIXED; break;
            case 5: gps_data.fix_type = GPS_RTK_FLOAT; break;
            default: gps_data.fix_type = GPS_3D_FIX; break;
        }
    }

    // 解析衛星數
    if (strlen(fields[7]) > 0) {
        gps_data.satellites_used = parseInt(fields[7]);
    }

    // 解析 HDOP
    if (strlen(fields[8]) > 0) {
        gps_data.hdop = parseFloat(fields[8]);
    }

    // 解析實際海拔
    if (strlen(fields[9]) > 0) {
        float msl_altitude = parseFloat(fields[9]);  // MSL高度
        float geoid_separation = 0.0f;

        // 取得橢球面分離度修正值
        if (strlen(fields[11]) > 0) {
            geoid_separation = parseFloat(fields[11]);
        }

        float raw_altitude = msl_altitude + geoid_separation;

        // 使用海拔濾波器處理
        if (!updateAltitudeFilter(raw_altitude)) {
            gps_data.altitude = raw_altitude;
        }
    }

    gps_data.last_update = millis();
    return true;
}


bool NmeaParser::parseGGA(const char* line) {
    // $GNGGA,time,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt,alt_unit,height,height_unit,dgps,dgps_id*checksum

    // 更新 GGA 頻率統計
    updateGGAFrequency();

    char fields[15][20];
    int field_count = splitFields(line, fields, 15);

    if (field_count < 10) {
        return false;
    }

    // 解析UTC時間 (GGA句子的第1個欄位)
    if (strlen(fields[1]) > 0) {
        parseTime(fields[1]);
        gps_data.valid_time = true;  // GGA包含有效時間

        // 由於GGA沒有日期，使用當前系統日期 (假設系統時間正確)
        // 實際應用中可能需要從其他來源獲得日期
        gps_data.day = 16;     // 預設日期，實際應用中需要修正
        gps_data.month = 12;   // 預設月份
        gps_data.year = 2025;  // 預設年份
    }

    // 解析緯度
    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
        gps_data.latitude = parseCoordinate(fields[2], fields[3]);
        gps_data.valid_position = true;
    }

    // 解析經度
    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
        gps_data.longitude = parseCoordinate(fields[4], fields[5]);
        gps_data.valid_position = true;
    }

    // 解析定位品質
    if (strlen(fields[6]) > 0) {
        int quality = parseInt(fields[6]);
        // Serial.print("🎯 GGA品質欄位: ");
        // Serial.print(quality);
        switch (quality) {
            case 0: gps_data.fix_type = GPS_NO_FIX; /* Serial.println(" → NO_FIX"); */ break;
            case 1: gps_data.fix_type = GPS_3D_FIX; /* Serial.println(" → 3D_FIX"); */ break;
            case 2: gps_data.fix_type = GPS_DGPS; /* Serial.println(" → DGPS"); */ break;
            case 4: gps_data.fix_type = GPS_RTK_FIXED; /* Serial.println(" → RTK_FIXED"); */ break;
            case 5: gps_data.fix_type = GPS_RTK_FLOAT; /* Serial.println(" → RTK_FLOAT"); */ break;
            default: gps_data.fix_type = GPS_3D_FIX; /* Serial.println(" → 3D_FIX (default)"); */ break;
        }
    }

    // 解析衛星數
    if (strlen(fields[7]) > 0) {
        gps_data.satellites_used = parseInt(fields[7]);
    }

    // 解析 HDOP
    if (strlen(fields[8]) > 0) {
        gps_data.hdop = parseFloat(fields[8]);
    }

    // 解析實際海拔
    if (strlen(fields[9]) > 0) {
        float msl_altitude = parseFloat(fields[9]);  // MSL高度
        float geoid_separation = 0.0f;

        // 取得橢球面分離度修正值
        if (strlen(fields[11]) > 0) {
            geoid_separation = parseFloat(fields[11]);
        }

        // 計算原始實際海拔 = MSL高度 + 橢球面分離度
        float raw_altitude = msl_altitude + geoid_separation;

        // 使用海拔濾波器處理 (只有 RTK 品質時才濾波)
        if (!updateAltitudeFilter(raw_altitude)) {
            // 濾波器拒絕或不適用，使用原始值
            gps_data.altitude = raw_altitude;
        }
        // 如果濾波成功，altitude 已在 updateAltitudeFilter 中更新
    }

    // 🔍 GPS Raw Data Debug Output
    Serial.println("===== GPS Data =====");

    // 定位狀態和位置
    Serial.print("Fix: ");
    Serial.print(gps_data.fix_type);
    Serial.print(" | Valid Pos: ");
    Serial.print(gps_data.valid_position ? "YES" : "NO");
    Serial.print(" | Data Age: ");
    Serial.print(millis() - gps_data.last_update);
    Serial.println("ms");

    Serial.print("Lat: ");
    Serial.print(gps_data.latitude, 8);
    Serial.print(" | Lon: ");
    Serial.print(gps_data.longitude, 8);
    Serial.print(" | Alt: ");
    Serial.print(gps_data.altitude);
    Serial.println("m");

    // 衛星和精度
    Serial.print("Sats Used: ");
    Serial.print(gps_data.satellites_used);
    Serial.print(" | Visible: ");
    Serial.print(gps_data.satellites_visible);
    Serial.print(" | HDOP: ");
    Serial.println(gps_data.hdop);

    // 運動資料
    Serial.print("Speed: ");
    Serial.print(gps_data.speed);
    Serial.print("m/s | Heading: ");
    Serial.print(gps_data.heading);
    Serial.println("°");

    // 時間資料
    if(gps_data.valid_time) {
        Serial.print("Time: ");
        if(gps_data.hour < 10) Serial.print("0");
        Serial.print(gps_data.hour);
        Serial.print(":");
        if(gps_data.minute < 10) Serial.print("0");
        Serial.print(gps_data.minute);
        Serial.print(":");
        if(gps_data.second < 10) Serial.print("0");
        Serial.print(gps_data.second);
        Serial.print(" | Date: ");
        if(gps_data.day < 10) Serial.print("0");
        Serial.print(gps_data.day);
        Serial.print("/");
        if(gps_data.month < 10) Serial.print("0");
        Serial.print(gps_data.month);
        Serial.print("/");
        Serial.println(gps_data.year);
    } else {
        Serial.println("Time: Invalid");
    }

    // 海拔濾波狀態
    Serial.print("Altitude Stable: ");
    Serial.print(gps_data.altitude_stable ? "YES" : "NO");
    Serial.print(" | Count: ");
    Serial.print(gps_data.altitude_count);
    Serial.print(" | Mean: ");
    Serial.print(gps_data.altitude_mean);
    Serial.print(" | Std Dev: ");
    Serial.println(gps_data.altitude_std_dev);

    // GGA 頻率統計
    Serial.print("GGA Frequency: ");
    Serial.print(gps_data.gga_frequency, 2);
    Serial.print("Hz | GGA Count: ");
    Serial.println(gps_data.gga_count);

    Serial.println("====================");

    gps_data.last_update = millis();
    return true;
}

bool NmeaParser::parseRMC(const char* line) {
    // $GNRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir*checksum

    char fields[12][20];
    int field_count = splitFields(line, fields, 12);

    if (field_count < 10) {
        return false;
    }

    // 檢查數據有效性
    if (strlen(fields[2]) == 0 || fields[2][0] != 'A') {
        return false; // 數據無效
    }

    // 解析時間
    if (strlen(fields[1]) > 0) {
        parseTime(fields[1]);
    }

    // 解析速度（節轉 m/s）
    if (strlen(fields[7]) > 0) {
        float speed_knots = parseFloat(fields[7]);
        gps_data.speed = speed_knots * 0.514444f; // 節轉 m/s
    }

    // 不從 RMC 解析航向，只從 VTG 取得

    // 解析日期
    if (strlen(fields[9]) > 0) {
        parseDate(fields[9]);
        gps_data.valid_time = true;
    }

    gps_data.last_update = millis();
    return true;
}


bool NmeaParser::parseHDT(const char* line) {
    // $GNHDT,heading,T*checksum

    char fields[5][20];
    int field_count = splitFields(line, fields, 5);

    if (field_count < 2) {
        return false;
    }

    // 解析真航向
    if (strlen(fields[1]) > 0) {
        gps_data.heading = parseFloat(fields[1]);
    } else {
        gps_data.heading = 0.0;  // HDT 欄位為空時清除航向
    }

    gps_data.last_update = millis();
    return true;
}

bool NmeaParser::parseGSA(const char* line) {
    // $GNGSA,mode1,mode2,sat1,sat2,...,sat12,pdop,hdop,vdop*checksum

    char fields[18][20];
    int field_count = splitFields(line, fields, 18);

    if (field_count < 17) {
        return false;
    }

    // 解析 fix 模式
    if (strlen(fields[2]) > 0) {
        int mode = parseInt(fields[2]);
        switch (mode) {
            case 1: gps_data.fix_type = GPS_NO_FIX; break;
            case 2: gps_data.fix_type = GPS_2D_FIX; break;
            case 3: gps_data.fix_type = GPS_3D_FIX; break;
        }
    }

    // 計算使用的衛星數量
    int used_sats = 0;
    for (int i = 3; i <= 14; i++) {
        if (strlen(fields[i]) > 0 && parseInt(fields[i]) > 0) {
            used_sats++;
        }
    }
    gps_data.satellites_used = used_sats;

    // 解析 HDOP
    if (strlen(fields[16]) > 0) {
        gps_data.hdop = parseFloat(fields[16]);
    }

    gps_data.last_update = millis();
    return true;
}

bool NmeaParser::parseGSV(const char* line) {
    // $GPGSV,total_msg,msg_num,total_sats,sat1_prn,sat1_elev,sat1_azim,sat1_cno,...*checksum

    char fields[20][20];
    int field_count = splitFields(line, fields, 20);

    if (field_count < 8) {
        return false;
    }

    // 識別衛星系統
    int sys_id = identifyTalkerID(line);
    if (sys_id < 0 || sys_id >= SAT_SYSTEM_COUNT) {
        return false;
    }

    SatelliteSystem* sat_sys = &gps_data.sat_systems[sys_id];

    // 解析總衛星數（只在第一條訊息中）
    int msg_num = parseInt(fields[2]);
    if (msg_num == 1 && strlen(fields[3]) > 0) {
        sat_sys->sat_count = parseInt(fields[3]);
        sat_sys->valid_sats = 0; // 重置計數
    }

    // 解析衛星資料（每條訊息最多4顆衛星）
    for (int sat = 0; sat < 4 && sat_sys->valid_sats < 12; sat++) {
        int prn_idx = 4 + sat * 4;     // PRN
        int cno_idx = 4 + sat * 4 + 3; // CNO

        if (prn_idx >= field_count || cno_idx >= field_count) {
            break;
        }

        if (strlen(fields[prn_idx]) > 0) {
            int prn = parseInt(fields[prn_idx]);
            int cno = 0;

            if (strlen(fields[cno_idx]) > 0) {
                cno = parseInt(fields[cno_idx]);
            }

            if (prn > 0) {
                int idx = sat_sys->valid_sats;
                sat_sys->prn_numbers[idx] = prn;
                sat_sys->cno_values[idx] = cno;  // 允許 CNO = 0，但統計時會排除
                sat_sys->valid_sats++;
            }
        }
    }

    // 更新總可見衛星數
    gps_data.satellites_visible = 0;
    for (int i = 0; i < SAT_SYSTEM_COUNT; i++) {
        gps_data.satellites_visible += gps_data.sat_systems[i].sat_count;
    }

    gps_data.last_update = millis();
    return true;
}


bool NmeaParser::isDataFresh(uint32_t timeout_ms) {
    return (millis() - gps_data.last_update) <= timeout_ms;
}

// 輔助函數實作

int NmeaParser::splitFields(const char* line, char fields[][20], int max_fields) {
    int field_count = 0;
    int field_pos = 0;
    int line_pos = 0;

    while (line[line_pos] && field_count < max_fields) {
        if (line[line_pos] == ',' || line[line_pos] == '*') {
            fields[field_count][field_pos] = '\0';
            field_count++;
            field_pos = 0;
        } else {
            if (field_pos < 19) { // 避免溢位
                fields[field_count][field_pos] = line[line_pos];
                field_pos++;
            }
        }
        line_pos++;
    }

    // 處理最後一個欄位
    if (field_count < max_fields) {
        fields[field_count][field_pos] = '\0';
        field_count++;
    }

    return field_count;
}

double NmeaParser::parseCoordinate(const char* coord, const char* direction) {
    if (!coord || !direction || strlen(coord) < 4) {
        return 0.0;
    }

    double raw_coord = atof(coord);
    int degrees = (int)(raw_coord / 100);
    double minutes = raw_coord - (degrees * 100);

    double result = degrees + (minutes / 60.0);

    // 處理南緯和西經
    if (direction[0] == 'S' || direction[0] == 'W') {
        result = -result;
    }

    return result;
}

float NmeaParser::parseFloat(const char* str) {
    return (str && strlen(str) > 0) ? atof(str) : 0.0f;
}

int NmeaParser::parseInt(const char* str) {
    return (str && strlen(str) > 0) ? atoi(str) : 0;
}

bool NmeaParser::parseTime(const char* time_str) {
    if (!time_str || strlen(time_str) < 6) {
        return false;
    }

    // 解析 HHMMSS 或 HHMMSS.sss 格式
    char hour_str[3] = {time_str[0], time_str[1], '\0'};
    char min_str[3] = {time_str[2], time_str[3], '\0'};
    char sec_str[3] = {time_str[4], time_str[5], '\0'};

    gps_data.hour = atoi(hour_str);
    gps_data.minute = atoi(min_str);
    gps_data.second = atoi(sec_str);

    // 解析小數秒部分 (如果存在)
    gps_data.millisecond = 0;
    if (strlen(time_str) > 6 && time_str[6] == '.') {
        char ms_str[4] = {0};  // 初始化為0

        // 提取小數點後最多3位數字
        int ms_len = 0;
        for (int i = 7; i < strlen(time_str) && ms_len < 3 && isdigit(time_str[i]); i++) {
            ms_str[ms_len++] = time_str[i];
        }

        if (ms_len > 0) {
            int ms_value = atoi(ms_str);
            // 根據位數調整為毫秒 (padding zeros if needed)
            if (ms_len == 1) ms_value *= 100;      // .1 -> 100ms
            else if (ms_len == 2) ms_value *= 10;  // .12 -> 120ms
            // ms_len == 3: 已經是毫秒格式

            gps_data.millisecond = ms_value;
        }
    }

    return true;
}

bool NmeaParser::parseDate(const char* date_str) {
    if (!date_str || strlen(date_str) < 6) {
        return false;
    }

    // DDMMYY 格式
    char day_str[3] = {date_str[0], date_str[1], '\0'};
    char mon_str[3] = {date_str[2], date_str[3], '\0'};
    char year_str[3] = {date_str[4], date_str[5], '\0'};

    // 檢查字符是否為數字
    for (int i = 0; i < 6; i++) {
        if (date_str[i] < '0' || date_str[i] > '9') {
            return false;
        }
    }

    int day = atoi(day_str);
    int month = atoi(mon_str);
    int year = atoi(year_str);

    // 驗證範圍
    if (day < 1 || day > 31 || month < 1 || month > 12 || year < 0 || year > 99) {
        return false;
    }

    gps_data.day = (uint8_t)day;
    gps_data.month = (uint8_t)month;
    gps_data.year = (uint16_t)(2000 + year); // 確保類型轉換

    return true;
}

int NmeaParser::identifyTalkerID(const char* line) {
    if (!line || strlen(line) < 3) {
        return -1;
    }

    // 檢查 talker ID ($xxyyy 中的 xx 部分)
    if (strncmp(line + 1, "GP", 2) == 0) return SAT_GPS;      // GPS
    if (strncmp(line + 1, "GL", 2) == 0) return SAT_GLONASS;  // GLONASS
    if (strncmp(line + 1, "GA", 2) == 0) return SAT_GALILEO;  // Galileo
    if (strncmp(line + 1, "GN", 2) == 0) return SAT_GPS;      // Multi-GNSS，歸類為 GPS
    // 排除中國北斗系統
    // if (strncmp(line + 1, "GB", 2) == 0) return SAT_BEIDOU;  // BeiDou (中國)

    return -1; // 未知系統或已排除的系統
}

bool NmeaParser::validateChecksum(const char* line) {
    // 簡化實作：假設校驗和正確
    // 實際應用中可以實作完整的 NMEA 校驗和驗證
    return true;
}

// 海拔濾波函數實作
bool NmeaParser::updateAltitudeFilter(float raw_altitude) {
    // Serial.print("🔍 海拔濾波檢查 - Fix Type: ");
    // Serial.print(gps_data.fix_type);
    // Serial.print(" (需要≥");
    // Serial.print(GPS_3D_FIX);
    // Serial.print(") 海拔: ");
    // Serial.println(raw_altitude);

    // 只有在 3D_FIX 以上時才進行濾波
    if (gps_data.fix_type < GPS_3D_FIX) {
        // Serial.println("❌ GPS模式不足，跳過濾波");
        return false;
    }

    // Serial.println("✅ 進入RTK濾波模式");

    // 如果還沒有收集到10個值，直接加入並使用原始值
    if (gps_data.altitude_count < 10) {
        gps_data.altitude_history[gps_data.altitude_index] = raw_altitude;
        gps_data.altitude_index = (gps_data.altitude_index + 1) % 10;
        gps_data.altitude_count++;

        gps_data.altitude = raw_altitude;
        gps_data.altitude_stable = false;
        Serial.print("🔄 收集海拔樣本 ");
        Serial.print(gps_data.altitude_count);
        Serial.print("/10: ");
        Serial.println(raw_altitude);
        return false;
    }

    // 計算統計值（基於現有的10個值）
    calculateAltitudeStats();

    // 調試：顯示歷史記錄
    // Serial.print("📈 歷史記錄[");
    // for (int i = 0; i < 10; i++) {
    //     Serial.print(gps_data.altitude_history[i], 1);
    //     if (i < 9) Serial.print(",");
    // }
    // Serial.print("] 索引:");
    // Serial.print(gps_data.altitude_index);
    // Serial.print(" 數量:");
    // Serial.println(gps_data.altitude_count);

    // 檢查新值是否在4個標準差內（最小門檻30公尺）
    float deviation = abs(raw_altitude - gps_data.altitude_mean);
    float threshold = 4.0 * gps_data.altitude_std_dev;  // 4倍標準差
    if (threshold < 30.0) {
        threshold = 30.0;  // 最小門檻30公尺
    }
    // Serial.print("📊 海拔濾波 - 原值:");
    // Serial.print(raw_altitude);
    // Serial.print(" 平均:");
    // Serial.print(gps_data.altitude_mean);
    // Serial.print(" 標準差:");
    // Serial.print(gps_data.altitude_std_dev);
    // Serial.print(" 偏差:");
    // Serial.print(deviation);
    // Serial.print(" 門檻(4σ/min30):");
    // Serial.print(threshold);

    if (deviation <= threshold) {
        // 新值符合標準，加入歷史記錄並更新海拔
        gps_data.altitude_history[gps_data.altitude_index] = raw_altitude;
        gps_data.altitude_index = (gps_data.altitude_index + 1) % 10;

        gps_data.altitude = raw_altitude;
        gps_data.altitude_stable = true;
        // Serial.println(" ✅通過");
        return true;
    } else {
        // 新值超出標準差，不加入歷史記錄，標記為不穩定
        gps_data.altitude_stable = false;
        // Serial.println(" ❌拒絕");
        return false;
    }
}

// GGA 頻率計算函數
void NmeaParser::updateGGAFrequency() {
    gps_data.gga_count++;

    uint32_t current_time = millis();
    uint32_t elapsed_time = current_time - gps_data.gga_start_time;

    // 每5秒更新一次頻率計算 (避免頻繁計算)
    if (elapsed_time >= 5000) {
        gps_data.gga_frequency = (float)gps_data.gga_count * 1000.0 / elapsed_time;

        // 重置計數器以便下次統計
        gps_data.gga_count = 0;
        gps_data.gga_start_time = current_time;
    }
}

void NmeaParser::calculateAltitudeStats() {
    // 計算平均值
    float sum = 0.0;
    for (int i = 0; i < 10; i++) {
        sum += gps_data.altitude_history[i];
    }
    gps_data.altitude_mean = sum / 10.0;

    // 計算標準差
    float variance_sum = 0.0;
    for (int i = 0; i < 10; i++) {
        float diff = gps_data.altitude_history[i] - gps_data.altitude_mean;
        variance_sum += diff * diff;
    }

    // 使用樣本標準差公式 (n-1)
    float variance = variance_sum / 9.0;
    gps_data.altitude_std_dev = sqrt(variance);
}