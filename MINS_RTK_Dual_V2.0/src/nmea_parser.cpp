#include "nmea_parser.h"
#include <string.h>
#include <stdlib.h>

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
        return parseGGA(nmea_line);
    }
    else if (strncmp(msg_type, "RMC", 3) == 0) {
        return parseRMC(nmea_line);
    }
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

bool NmeaParser::parseGGA(const char* line) {
    // $GNGGA,time,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt,alt_unit,height,height_unit,dgps,dgps_id*checksum

    char fields[15][20];
    int field_count = splitFields(line, fields, 15);

    if (field_count < 10) {
        return false;
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

    // 解析海拔
    if (strlen(fields[9]) > 0) {
        gps_data.altitude = parseFloat(fields[9]);
    }

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

    gps_data.day = atoi(day_str);
    gps_data.month = atoi(mon_str);
    gps_data.year = 2000 + atoi(year_str); // 假設 21 世紀

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