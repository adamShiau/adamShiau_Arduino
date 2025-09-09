#pragma once

/**
 * NMEA Data Parser
 * 
 * 職責：
 * - 解析 NMEA sentence 為 GNSSData 結構
 * - 支援 GGA、RMC、VTG、GSA、GSV 等標準句子
 * - 支援 PLSHD 自定義句子（航向數據）
 * - 處理數據驗證和單位轉換
 * 
 * 設計原則：
 * - Header-only 實作，提高性能
 * - 無狀態設計，線程安全
 * - 完整的錯誤檢查和數據驗證
 * - 支援即時解析
 */

#include "../data/data_types.h"
#include "../util/log.h"
#include "parser_utils.h"
#include <cstring>
#include <cstdlib>
#include <cmath>

namespace NmeaParser {
    
    // 前置聲明
    inline bool parseGGA(const char* sentence, GNSSData& gnss_data);
    inline bool parseRMC(const char* sentence, GNSSData& gnss_data);
    inline bool parseVTG(const char* sentence, GNSSData& gnss_data);
    inline bool parseGSA(const char* sentence, GNSSData& gnss_data);
    inline bool parsePLSHD(const char* sentence, GNSSData& gnss_data);
    
    // NMEA 句子類型
    enum class NmeaSentenceType {
        GGA,        // Global Positioning System Fix Data
        RMC,        // Recommended Minimum Course
        VTG,        // Track Made Good and Ground Speed
        GSA,        // GPS DOP and Active Satellites
        GSV,        // GPS Satellites in View
        PLSHD,      // Proprietary: Heading Data
        UNKNOWN
    };
    
    /**
     * @brief 確定 NMEA 句子類型
     * @param sentence NMEA 句子字串
     * @return 句子類型
     */
    inline NmeaSentenceType getSentenceType(const char* sentence) {
        if (!sentence || strlen(sentence) < 6) return NmeaSentenceType::UNKNOWN;
        
        // 跳過 $ 符號
        const char* type_start = sentence + 1;
        
        // 跳過 talker ID（前兩個字符）
        if (strlen(type_start) >= 5) {
            type_start += 2;
        }
        
        if (strncmp(type_start, "GGA", 3) == 0) return NmeaSentenceType::GGA;
        if (strncmp(type_start, "RMC", 3) == 0) return NmeaSentenceType::RMC;
        if (strncmp(type_start, "VTG", 3) == 0) return NmeaSentenceType::VTG;
        if (strncmp(type_start, "GSA", 3) == 0) return NmeaSentenceType::GSA;
        if (strncmp(type_start, "GSV", 3) == 0) return NmeaSentenceType::GSV;
        if (strncmp(sentence + 1, "PLSHD", 5) == 0) return NmeaSentenceType::PLSHD;
        
        return NmeaSentenceType::UNKNOWN;
    }
    
    /**
     * @brief 解析 NMEA sentence 為 GNSSData
     * @param sentence_data NMEA sentence 數據（ASCII 字串）
     * @param sentence_length sentence 長度
     * @param gnss_data 輸出的 GNSS 數據結構
     * @return true 解析成功
     */
    inline bool parseNmeaSentence(const uint8_t* sentence_data, size_t sentence_length, GNSSData& gnss_data) {
        if (!sentence_data || sentence_length < 6) {
            LOG_WARN("NMEA_PARSER", "Sentence 數據無效或太短");
            return false;
        }
        
        // 轉換為字串（確保 null-terminated）
        char sentence[512];
        size_t copy_length = (sentence_length < sizeof(sentence) - 1) ? sentence_length : sizeof(sentence) - 1;
        memcpy(sentence, sentence_data, copy_length);
        sentence[copy_length] = '\0';
        
        // 儲存原始 NMEA 句子
        if (copy_length < sizeof(gnss_data.nmea_sentence)) {
            memcpy(gnss_data.nmea_sentence, sentence, copy_length + 1);
            gnss_data.nmea_length = copy_length;
        }
        
        // 檢查句子格式
        if (sentence[0] != '$') {
            LOG_WARN("NMEA_PARSER", "無效的 NMEA 起始字符");
            return false;
        }
        
        // 確定句子類型
        NmeaSentenceType type = getSentenceType(sentence);
        
        // 🔧 重要修復：初始化所有關鍵字段，防止 fix_type=255 等異常值
        gnss_data.timestamp_us = micros();
        gnss_data.schema = DATA_SCHEMA_VERSION;
        gnss_data.fix_type = GNSS_FIX_NONE;  // 預設為無定位
        gnss_data.flags = 0;                 // 清除所有標誌
        gnss_data.heading_source = 0;        // 初始化航向源為未知
        gnss_data.baseline_length = 0.0f;    // 初始化基線長度
        
        // 根據句子類型解析 - 添加優先級處理
        switch (type) {
            case NmeaSentenceType::PLSHD:
                // 🔧 最高優先級：雙天線GNSS航向數據
                if (parsePLSHD(sentence, gnss_data)) {
                    gnss_data.heading_source = 1; // 標記為雙天線源
                    return true;
                }
                return false;
            case NmeaSentenceType::GGA:
                return parseGGA(sentence, gnss_data);
            case NmeaSentenceType::RMC:
                // 🔧 解析RMC但避免覆蓋更高優先級的航向數據
                {
                    bool had_high_priority_heading = (gnss_data.flags & GNSS_HEADING_VALID) && 
                                                   (gnss_data.heading_source == 1);
                    float saved_heading = gnss_data.gnss_heading;
                    float saved_accuracy = gnss_data.heading_accuracy;
                    
                    bool result = parseRMC(sentence, gnss_data);
                    
                    // 如果之前有更高優先級的航向數據，恢復它
                    if (had_high_priority_heading && result) {
                        gnss_data.gnss_heading = saved_heading;
                        gnss_data.heading_accuracy = saved_accuracy;
                        gnss_data.heading_source = 1;
                        gnss_data.flags |= GNSS_HEADING_VALID;
                    }
                    
                    return result;
                }
            case NmeaSentenceType::VTG:
                // 🔇 雙天線系統優化：VTG 與 RMC 功能重複，暫時註解
                // {
                //     bool had_dual_antenna = (gnss_data.flags & GNSS_HEADING_VALID) && 
                //                           (gnss_data.heading_source == 1);
                //     if (!had_dual_antenna) {
                //         if (parseVTG(sentence, gnss_data)) {
                //             gnss_data.heading_source = 2; // 標記為VTG源
                //             return true;
                //         }
                //     }
                //     return parseVTG(sentence, gnss_data); // 仍解析速度數據
                // }
                return false;  // 跳過 VTG 處理
            case NmeaSentenceType::GSA:
                // 🔇 雙天線系統優化：GSA 衛星狀態對導航輸出不關鍵，暫時註解
                // return parseGSA(sentence, gnss_data);
                return false;  // 跳過 GSA 處理
            default:
                // 未知句子類型，不是錯誤，只是跳過
                return false;
        }
    }
    
    
    /**
     * @brief 解析 GGA 句子（定位數據）
     */
    inline bool parseGGA(const char* sentence, GNSSData& gnss_data) {
        // $GPGGA,time,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt,alt_unit,height,height_unit,dgps,dgps_id*checksum
        
        const char* tokens[15];
        int token_count = tokenize(sentence, ',', tokens, 15);
        
        if (token_count < 10) {
            return false;
        }
        
        // 解析緯度 (DDMM.MMMM)
        if (strlen(tokens[2]) > 0 && strlen(tokens[3]) > 0) {
            if (parseLatitude(tokens[2], tokens[3][0], gnss_data.latitude)) {
                gnss_data.flags |= GNSS_POSITION_VALID;
            }
        }
        
        // 解析經度 (DDDMM.MMMM)
        if (strlen(tokens[4]) > 0 && strlen(tokens[5]) > 0) {
            if (parseLongitude(tokens[4], tokens[5][0], gnss_data.longitude)) {
                gnss_data.flags |= GNSS_POSITION_VALID;
            }
        }
        
        // 解析定位品質
        if (strlen(tokens[6]) > 0) {
            int quality = atoi(tokens[6]);
            switch (quality) {
                case 0: gnss_data.fix_type = GNSS_FIX_NONE; break;
                case 1: gnss_data.fix_type = GNSS_FIX_3D; break;
                case 2: gnss_data.fix_type = GNSS_FIX_DGPS; break;
                case 4: gnss_data.fix_type = GNSS_FIX_RTK_FIXED; break;
                case 5: gnss_data.fix_type = GNSS_FIX_RTK_FLOAT; break;
                default: gnss_data.fix_type = GNSS_FIX_3D; break;
            }
            gnss_data.flags |= GNSS_FIX_VALID;
        }
        
        // 解析衛星數
        if (strlen(tokens[7]) > 0) {
            gnss_data.satellites_used = atoi(tokens[7]);
        }
        
        // 解析 HDOP
        if (strlen(tokens[8]) > 0) {
            gnss_data.hdop = atof(tokens[8]);
            gnss_data.flags |= GNSS_ACCURACY_VALID;
        }
        
        // 解析海拔
        if (strlen(tokens[9]) > 0) {
            gnss_data.altitude_msl = atof(tokens[9]);
        }
        
        return true;
    }
    
    /**
     * @brief 解析 RMC 句子（速度和航向）
     */
    inline bool parseRMC(const char* sentence, GNSSData& gnss_data) {
        // $GPRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir*checksum
        
        const char* tokens[12];
        int token_count = tokenize(sentence, ',', tokens, 12);
        
        if (token_count < 10) {
            return false;
        }
        
        // 檢查數據有效性
        if (strlen(tokens[2]) == 0 || tokens[2][0] != 'A') {
            return false; // 數據無效
        }
        
        // 解析速度（節轉 m/s）
        if (strlen(tokens[7]) > 0) {
            float speed_knots = atof(tokens[7]);
            gnss_data.ground_speed = speed_knots * 0.514444f; // 節轉 m/s
            gnss_data.flags |= GNSS_VELOCITY_VALID;
        }
        
        // 解析航向（度轉弧度）- 只用於 course_over_ground，不設定為 gnss_heading
        if (strlen(tokens[8]) > 0) {
            float course_deg = atof(tokens[8]);
            gnss_data.course_over_ground = course_deg * M_PI / 180.0f;
            // 🔧 禁用：RMC 不再設定 gnss_heading，只有 PLSHD 可以設定 heading
            // gnss_data.gnss_heading = gnss_data.course_over_ground;
            // gnss_data.flags |= GNSS_HEADING_VALID;
            // if (gnss_data.heading_source == 0) {
            //     gnss_data.heading_source = 3; // RMC源
            // }
        }
        
        return true;
    }
    
    /**
     * @brief 解析 VTG 句子（地面速度和航跡）
     */
    inline bool parseVTG(const char* sentence, GNSSData& gnss_data) {
        // $GPVTG,course1,T,course2,M,speed1,N,speed2,K,mode*checksum
        
        const char* tokens[10];
        int token_count = tokenize(sentence, ',', tokens, 10);
        
        if (token_count < 8) {
            return false;
        }
        
        // 解析真航跡角 - 只用於 course_over_ground，不設定為 gnss_heading
        if (strlen(tokens[1]) > 0) {
            float course_deg = atof(tokens[1]);
            gnss_data.course_over_ground = course_deg * M_PI / 180.0f;
            // 🔧 禁用：VTG 不再設定 gnss_heading，只有 PLSHD 可以設定 heading
            // gnss_data.gnss_heading = gnss_data.course_over_ground;
            // gnss_data.flags |= GNSS_HEADING_VALID;
            // if (gnss_data.heading_source == 0) {
            //     gnss_data.heading_source = 2; // VTG源
            // }
        }
        
        // 解析地面速度（km/h 轉 m/s）
        if (strlen(tokens[7]) > 0) {
            float speed_kmh = atof(tokens[7]);
            gnss_data.ground_speed = speed_kmh / 3.6f; // km/h 轉 m/s
            gnss_data.flags |= GNSS_VELOCITY_VALID;
        }
        
        return true;
    }
    
    /**
     * @brief 解析 GSA 句子（DOP 和衛星）
     */
    inline bool parseGSA(const char* sentence, GNSSData& gnss_data) {
        // $GNGSA,mode1,mode2,sat1,sat2,...,sat12,pdop,hdop,vdop,system_id*checksum
        
        const char* tokens[18];
        int token_count = tokenize(sentence, ',', tokens, 18);
        
        if (token_count < 17) {
            return false;
        }
        
        // 🔧 修正：解析 fix_type (mode2 欄位)
        if (strlen(tokens[2]) > 0) {
            int mode2 = atoi(tokens[2]);
            switch (mode2) {
                case 1: gnss_data.fix_type = GNSS_FIX_NONE; break;  // 無定位
                case 2: gnss_data.fix_type = GNSS_FIX_2D; break;    // 2D 定位
                case 3: gnss_data.fix_type = GNSS_FIX_3D; break;    // 3D 定位
                default: gnss_data.fix_type = GNSS_FIX_NONE; break;
            }
            gnss_data.flags |= GNSS_FIX_VALID;
        }
        
        // 解析活躍衛星數量 - 改進計算邏輯
        int active_sats = 0;
        for (int i = 3; i <= 14; i++) {  // tokens[3] to tokens[14] 是衛星PRN
            if (strlen(tokens[i]) > 0) {
                int prn = atoi(tokens[i]);
                if (prn > 0) {  // 只計算有效的PRN號碼
                    active_sats++;
                }
            }
        }
        // 🔧 修正：GSA中的衛星數應該是used，不是visible
        if (active_sats > 0) {
            gnss_data.satellites_used = active_sats;
        }
        
        // 解析 DOP 值
        if (strlen(tokens[15]) > 0) {
            gnss_data.pdop = atof(tokens[15]);
        }
        if (strlen(tokens[16]) > 0) {
            gnss_data.hdop = atof(tokens[16]);
        }
        if (strlen(tokens[17]) > 0) {
            gnss_data.vdop = atof(tokens[17]);
            gnss_data.flags |= GNSS_ACCURACY_VALID;
        }
        
        return true;
    }
    
    /**
     * @brief 解析 PLSHD 句子（航向數據）
     * 格式：$PLSHD,valid_flag,sat_ant1,sat_ant2,length,heading,elevation*checksum
     */
    inline bool parsePLSHD(const char* sentence, GNSSData& gnss_data) {
        // $PLSHD,1,27,35,1.547,40.056,-16.737*checksum
        
        
        const char* tokens[7];
        int token_count = tokenize(sentence, ',', tokens, 7);
        
        if (token_count < 6) {
            return false;
        }
        
        // 檢查有效性標誌 (token[1])
        if (strlen(tokens[1]) > 0) {
            int valid_flag = atoi(tokens[1]);
            // 0: invalid, 1: valid GNSS, 9: valid MEMS
            if (valid_flag != 1 && valid_flag != 9) {
                return false; // 數據無效
            }
        }
        
        // 解析天線1衛星數 (token[2])
        if (strlen(tokens[2]) > 0) {
            int sat_ant1 = atoi(tokens[2]);
            if (sat_ant1 > 0) {
                gnss_data.satellites_used = sat_ant1;
            }
        }
        
        // 解析天線間距離 (token[4]) - 用於驗證
        if (strlen(tokens[4]) > 0) {
            float antenna_length = atof(tokens[4]);
            // 記錄天線間距離，可用於精度評估
            if (antenna_length > 0.1f && antenna_length < 10.0f) {
                // 合理的天線間距離範圍
                gnss_data.baseline_length = antenna_length;
            }
        }
        
        // 解析航向 (token[5]) - 主要數據
        if (strlen(tokens[5]) > 0) {
            float heading_deg = atof(tokens[5]);
            // 🔧 PLSHD航向範圍：0-360度
            if (isfinite(heading_deg) && heading_deg >= 0.0f && heading_deg < 360.0f) {
                gnss_data.gnss_heading = heading_deg * M_PI / 180.0f;
                gnss_data.flags |= GNSS_HEADING_VALID;
                
                // 根據天線間距離估算精度
                if (gnss_data.baseline_length > 0) {
                    // 根據LOCOSYS手冊的精度公式估算
                    if (gnss_data.baseline_length >= 2.0f) {
                        gnss_data.heading_accuracy = 0.1f * M_PI / 180.0f;  // 0.1°
                    } else if (gnss_data.baseline_length >= 1.0f) {
                        gnss_data.heading_accuracy = 0.2f * M_PI / 180.0f;  // 0.2°
                    } else if (gnss_data.baseline_length >= 0.5f) {
                        gnss_data.heading_accuracy = 0.4f * M_PI / 180.0f;  // 0.4°
                    } else {
                        gnss_data.heading_accuracy = 2.0f * M_PI / 180.0f;  // 2.0°
                    }
                }
            }
        }
        
        return true;
    }
    
    
    /**
     * @brief 檢查 NMEA 數據的基本有效性
     */
    inline bool isValidNmeaData(const GNSSData& gnss_data) {
        // 🔧 修正：有 3D fix 就視為有效，或需要位置/速度/航向數據
        if (gnss_data.fix_type == GNSS_FIX_3D || gnss_data.fix_type == GNSS_FIX_2D || 
            gnss_data.fix_type == GNSS_FIX_DGPS || gnss_data.fix_type == GNSS_FIX_RTK_FIXED || 
            gnss_data.fix_type == GNSS_FIX_RTK_FLOAT) {
            return true;  // 有效定位就接受
        }
        
        // 備用：至少需要位置、速度或航向數據
        return (gnss_data.flags & (GNSS_POSITION_VALID | GNSS_VELOCITY_VALID | GNSS_HEADING_VALID)) != 0;
    }
}