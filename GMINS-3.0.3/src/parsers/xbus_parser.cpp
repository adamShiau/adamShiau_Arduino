#include "xbus_parser.h"
#include "../util/math_utils.h"

namespace XbusParser {

    bool parseXbusFrame(const uint8_t* frame_data, size_t frame_length, IMUData& imu_data) {
        // LOG_DEBUG("XBUS_PARSER", "parseXbusFrame called");
        
        if (!frame_data || frame_length < 5) { // 最小: FA FF MID LEN CHK
            // LOG_ERROR("XBUS_PARSER", "frame too short");
            return false;
        }
        
        // 初始化 IMU 數據結構
        memset(&imu_data, 0, sizeof(imu_data));
        imu_data.schema = DATA_SCHEMA_VERSION;
        // 時間戳將在 parseDataPayload 中設置，如果沒有 SAMPLE_TIME_FINE 則使用 micros()
        imu_data.timestamp_us = 0;
        
        // 解析 frame 結構 (支援擴展長度)
        const uint8_t* p = frame_data;
        if (p[0] != 0xFA || p[1] != 0xFF) {
            // LOG_WARN("XBUS_PARSER", "無效的 preamble/bus_id: 0x%02X 0x%02X", p[0], p[1]);
            return false;
        }
        
        uint8_t message_id = p[2];
        uint8_t length_field = p[3];
        size_t header_size = 4;
        size_t data_length = 0;
        
        // 處理擴展長度 (LEN=0xFF)
        if (length_field != 0xFF) {
            data_length = length_field;
        } else {
            if (frame_length < 7) { // FA FF MID FF Hi Lo CHK
                LOG_WARN("XBUS_PARSER", "擴展長度 frame 太短");
                return false;
            }
            data_length = (size_t(p[4]) << 8) | p[5];
            header_size = 6;
        }
        
        // 驗證總長度
        size_t expected_length = header_size + data_length + 1; // +1 for checksum
        if (frame_length != expected_length) {
            LOG_ERROR("XBUS_PARSER", "LENGTH mismatch - expected=%u, actual=%u", 
                     (unsigned)expected_length, (unsigned)frame_length);
            return false;
        }
        
        XbusMessageType msg_type = static_cast<XbusMessageType>(message_id);
        if (msg_type != XbusMessageType::MT_DATA2 && msg_type != XbusMessageType::MT_DATA) {
            // LOG_ERROR("XBUS_PARSER", "❌ MSG_TYPE: 0x%02X", message_id);
            return false;
        }
        
        // ✅ 已確定正確的 XBUS 校驗和算法：總和為0驗證法
        uint8_t sum = 0;
        // 從 Bus ID (frame_data[1]) 開始計算到 checksum (包含 checksum)，總和低8位應為0
        for (size_t i = 1; i <= header_size + data_length; i++) {
            sum += frame_data[i];
        }
        
        if (sum != 0) {
            // LOG_ERROR("XBUS_PARSER", "CHECKSUM mismatch - sum=0x%02X (should be 0)", sum);
            return false;
        } else {
            // LOG_DEBUG("XBUS_PARSER", "✅ CHECKSUM OK: sum=0x%02X", sum);
        }
        
        // 解析數據部分
        const uint8_t* data_start = frame_data + header_size;
        bool payload_result = XbusParser::parseDataPayload(data_start, data_length, imu_data);
        
        // 調試：顯示解析結果
        if (!payload_result) {
            // LOG_ERROR("XBUS_PARSER", "PAYLOAD parsing failed");
        } else {
            bool is_valid = (imu_data.flags & (IMU_ACCEL_VALID | IMU_QUATERNION_VALID)) != 0;
            // LOG_DEBUG("XBUS_PARSER", "PAYLOAD parsed: flags=0x%04X, valid=%s", 
            //          imu_data.flags, is_valid ? "YES" : "NO");
        }
        return payload_result;
    }
    
    bool parseDataPayload(const uint8_t* data, size_t length, IMUData& imu_data) {
        size_t offset = 0;
        bool has_data = false;
        
        // 統計計數器 (用於調試)
        static uint32_t parse_count = 0;
        static uint32_t unknown_xdi_count = 0;
        static uint32_t bad_len_count = 0;
        parse_count++;
        
        LOG_DEBUG("XBUS_PARSER", "🔍 開始解析 MTData2 payload: total_length=%zu", length);
        
        while (offset + 3 <= length) { // MTData2: XDI(2B) + LEN(1B) 最小 3 bytes
            // 讀取 XDI（Data Identifier）和長度
            uint16_t xdi = (data[offset] << 8) | data[offset + 1];
            uint16_t block_len = data[offset + 2];  // 長度字段只有 1 字節
            offset += 3;
            
            // LOG_DEBUG("XBUS_PARSER", "🔍 XDI block: 0x%04X, len=%u, offset=%zu, remaining=%zu", 
            //          xdi, block_len, offset-3, length - (offset-3));
            
            // 檢查數據邊界
            if (offset + block_len > length) {
                LOG_WARN("XBUS_PARSER", "❌ XDI 0x%04X block 超出邊界: len=%u, remain=%zu", 
                         xdi, block_len, length - offset);
                break;
            }
            
            // 根據 XDI 解析數據
            XbusDataID xdi_enum = static_cast<XbusDataID>(xdi);
            
            // 特別關注 ROTATION_MATRIX
            if (xdi == 0x2020) {
                // LOG_DEBUG("XBUS_PARSER", "🎯 發現 ROTATION_MATRIX XDI: 0x%04X, len=%u", xdi, block_len);
            }
            
            // 檢查是否為已知 XDI
            bool is_known_xdi = (xdi_enum == XbusDataID::PACKET_COUNTER ||
                                xdi_enum == XbusDataID::SAMPLE_TIME_FINE ||
                                xdi_enum == XbusDataID::TEMPERATURE ||
                                xdi_enum == XbusDataID::ACCELERATION ||
                                xdi_enum == XbusDataID::ACCELERATION_NED ||
                                xdi_enum == XbusDataID::RATE_OF_TURN ||
                                xdi_enum == XbusDataID::RATE_OF_TURN_NED ||
                                xdi_enum == XbusDataID::MAGNETIC_FIELD ||
                                xdi_enum == XbusDataID::MAGNETIC_FIELD_NED ||
                                xdi_enum == XbusDataID::QUATERNION ||
                                xdi_enum == XbusDataID::QUATERNION_NED ||
                                xdi_enum == XbusDataID::ROTATION_MATRIX ||
                                xdi_enum == XbusDataID::EULER_ANGLES ||
                                xdi_enum == XbusDataID::STATUS_BYTE);
            
            // LOG_DEBUG("XBUS_PARSER", "XDI 0x%04X: known=%s, 即將進入 switch", xdi, is_known_xdi ? "YES" : "NO");
            
            // 特別調試 ROTATION_MATRIX 處理
            if (xdi == 0x2020) {
                // LOG_DEBUG("XBUS_PARSER", "🎯 ROTATION_MATRIX 檢測: xdi=0x%04X, xdi_enum=%d, ROTATION_MATRIX=%d", 
                //          xdi, (int)xdi_enum, (int)XbusDataID::ROTATION_MATRIX);
                // LOG_DEBUG("XBUS_PARSER", "🎯 比較結果: (xdi_enum == XbusDataID::ROTATION_MATRIX) = %s", 
                //          (xdi_enum == XbusDataID::ROTATION_MATRIX) ? "TRUE" : "FALSE");
            }
            
            switch (xdi_enum) {
                case XbusDataID::PACKET_COUNTER:
                    if (block_len == 2) {
                        imu_data.packet_counter = (data[offset] << 8) | data[offset + 1];
                        has_data = true;
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "PACKET_COUNTER 長度錯誤: expected=2, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::SAMPLE_TIME_FINE:
                    if (block_len == 4) {
                        uint32_t sample_time = parseUint32(data + offset);
                        // SAMPLE_TIME_FINE 通常是 10kHz 計數器，轉換為微秒
                        imu_data.timestamp_us = sample_time * 100; // 10kHz = 100us per tick
                        has_data = true;
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "SAMPLE_TIME_FINE 長度錯誤: expected=4, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::TEMPERATURE:
                    if (block_len == 4) {
                        imu_data.temperature = parseFloat32(data + offset);
                        imu_data.flags |= IMU_TEMPERATURE_VALID;
                        has_data = true;
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "TEMPERATURE 長度錯誤: expected=4, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::ACCELERATION:
                case XbusDataID::ACCELERATION_NED:
                    if (block_len == 12) {
                        imu_data.accel_x = parseFloat32(data + offset);
                        imu_data.accel_y = parseFloat32(data + offset + 4);
                        imu_data.accel_z = parseFloat32(data + offset + 8);
                        imu_data.flags |= IMU_ACCEL_VALID;
                        has_data = true;
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "ACCELERATION 長度錯誤: expected=12, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::RATE_OF_TURN:
                case XbusDataID::RATE_OF_TURN_NED:
                    if (block_len == 12) {
                        imu_data.gyro_x = parseFloat32(data + offset);
                        imu_data.gyro_y = parseFloat32(data + offset + 4);
                        imu_data.gyro_z = parseFloat32(data + offset + 8);
                        imu_data.flags |= IMU_GYRO_VALID;
                        has_data = true;
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "RATE_OF_TURN 長度錯誤: expected=12, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::MAGNETIC_FIELD:
                case XbusDataID::MAGNETIC_FIELD_NED:
                    if (block_len == 12) {
                        imu_data.mag_x = parseFloat32(data + offset);
                        imu_data.mag_y = parseFloat32(data + offset + 4);
                        imu_data.mag_z = parseFloat32(data + offset + 8);
                        imu_data.flags |= IMU_MAG_VALID;
                        has_data = true;
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "MAGNETIC_FIELD 長度錯誤: expected=12, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::QUATERNION:
                case XbusDataID::QUATERNION_NED:
                    if (block_len == 16) {
                        imu_data.quat_w = parseFloat32(data + offset);
                        imu_data.quat_x = parseFloat32(data + offset + 4);
                        imu_data.quat_y = parseFloat32(data + offset + 8);
                        imu_data.quat_z = parseFloat32(data + offset + 12);
                        
                        // 四元數正規化和 NaN 檢查
                        float norm = sqrt(imu_data.quat_w*imu_data.quat_w + imu_data.quat_x*imu_data.quat_x + 
                                         imu_data.quat_y*imu_data.quat_y + imu_data.quat_z*imu_data.quat_z);
                        if (norm > 0.1f && isfinite(norm)) {
                            imu_data.quat_w /= norm;
                            imu_data.quat_x /= norm;
                            imu_data.quat_y /= norm;
                            imu_data.quat_z /= norm;
                            imu_data.flags |= IMU_QUATERNION_VALID;
                            has_data = true;
                        } else {
                            LOG_DEBUG("XBUS_PARSER", "四元數無效或非有限值");
                        }
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "QUATERNION 長度錯誤: expected=16, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::ROTATION_MATRIX:
                    // LOG_DEBUG("XBUS_PARSER", "🎯🎯🎯 進入 ROTATION_MATRIX case: block_len=%u", block_len);
                    if (block_len == 36) {
                        // ROTATION_MATRIX: 3x3 旋轉矩陣 (9個 float32)
                        float r11 = parseFloat32(data + offset);      // [0][0]
                        float r12 = parseFloat32(data + offset + 4);  // [0][1] 
                        float r13 = parseFloat32(data + offset + 8);  // [0][2]
                        float r21 = parseFloat32(data + offset + 12); // [1][0]
                        float r22 = parseFloat32(data + offset + 16); // [1][1]
                        float r23 = parseFloat32(data + offset + 20); // [1][2]
                        float r31 = parseFloat32(data + offset + 24); // [2][0]
                        float r32 = parseFloat32(data + offset + 28); // [2][1]
                        float r33 = parseFloat32(data + offset + 32); // [2][2]
                        
                        // LOG_DEBUG("XBUS_PARSER", "🔢 旋轉矩陣前三元素: R11=%.6f R12=%.6f R13=%.6f", r11, r12, r13);
                        
                        // 檢查矩陣數值有效性
                        if (isfinite(r11) && isfinite(r12) && isfinite(r13) && 
                            isfinite(r21) && isfinite(r22) && isfinite(r23) &&
                            isfinite(r31) && isfinite(r32) && isfinite(r33)) {
                            
                            // 從旋轉矩陣計算歐拉角 (ZYX convention) - 🔧 修正yaw計算公式
                            float roll, pitch, yaw;
                            
                            // 計算 pitch (俯仰角) - 限制在 [-π/2, π/2]
                            float sin_pitch = -r31;
                            if (sin_pitch > 1.0f) sin_pitch = 1.0f;
                            if (sin_pitch < -1.0f) sin_pitch = -1.0f;
                            pitch = asin(sin_pitch);
                            
                            // 計算 roll (滾轉角) 和 yaw (偏航角)
                            float cos_pitch = cos(pitch);
                            if (cos_pitch > 1e-6) { // 避免萬向鎖
                                // 🔧 修正：正確的ZYX歐拉角計算公式
                                roll = atan2(r32, r33);
                                yaw = atan2(r21, r11);
                            } else {
                                // 萬向鎖情況下的處理
                                roll = atan2(-r23, r22);
                                yaw = 0.0f;
                            }
                            
                            // 🔧 新增：標準化角度到[-π, π]範圍
                            roll = mu::WrapAngleRad(roll);
                            pitch = mu::WrapAngleRad(pitch);  
                            yaw = mu::WrapAngleRad(yaw);
                            
                            // LOG_DEBUG("XBUS_PARSER", "🔢 計算出歐拉角: R=%.6f P=%.6f Y=%.6f", roll, pitch, yaw);
                            
                            // 設置歐拉角數據
                            imu_data.euler_roll = roll;
                            imu_data.euler_pitch = pitch;
                            imu_data.euler_yaw = yaw;
                            imu_data.flags |= IMU_EULER_VALID;
                            has_data = true;
                            
                            // LOG_DEBUG("XBUS_PARSER", "✅ ROTATION_MATRIX 轉歐拉角成功: flags=0x%04X", imu_data.flags);
                        } else {
                            LOG_WARN("XBUS_PARSER", "❌ 旋轉矩陣包含非有限值");
                        }
                    } else {
                        LOG_WARN("XBUS_PARSER", "❌ ROTATION_MATRIX 長度錯誤: expected=36, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::EULER_ANGLES:
                case XbusDataID::EULER_ANGLES_NED:
                    // LOG_DEBUG("XBUS_PARSER", "🎯🎯🎯 進入 EULER_ANGLES case: block_len=%u", block_len);
                    if (block_len == 12) {
                        // EULER_ANGLES: Roll, Pitch, Yaw (float32, rad) - 支援ENU和NED格式
                        float roll = parseFloat32(data + offset);
                        float pitch = parseFloat32(data + offset + 4);
                        float yaw = parseFloat32(data + offset + 8);
                        
                        // LOG_DEBUG("XBUS_PARSER", "🔢 Float32 歐拉角: R=%.6f P=%.6f Y=%.6f", roll, pitch, yaw);
                        
                        if (isfinite(roll) && isfinite(pitch) && isfinite(yaw)) {
                            imu_data.euler_roll = roll;
                            imu_data.euler_pitch = pitch;
                            imu_data.euler_yaw = yaw;
                            imu_data.flags |= IMU_EULER_VALID;
                            has_data = true;
                            // LOG_DEBUG("XBUS_PARSER", "✅ EULER_ANGLES (float32) 設置成功");
                        } else {
                            LOG_WARN("XBUS_PARSER", "❌ Float32 歐拉角包含非有限值");
                        }
                    } else if (block_len == 24) {
                        // EULER_ANGLES: Roll, Pitch, Yaw (float64, rad) - MTI 常用格式
                        double roll = parseFloat64(data + offset);
                        double pitch = parseFloat64(data + offset + 8);
                        double yaw = parseFloat64(data + offset + 16);
                        
                        // LOG_DEBUG("XBUS_PARSER", "🔢 Float64 歐拉角: R=%.6f P=%.6f Y=%.6f", roll, pitch, yaw);
                        
                        if (isfinite(roll) && isfinite(pitch) && isfinite(yaw)) {
                            // 轉換為 float 並設置歐拉角數據
                            imu_data.euler_roll = (float)roll;
                            imu_data.euler_pitch = (float)pitch;
                            imu_data.euler_yaw = (float)yaw;
                            
                            // 設置歐拉角有效標誌
                            imu_data.flags |= IMU_EULER_VALID;
                            has_data = true;
                            
                            // LOG_DEBUG("XBUS_PARSER", "✅ EULER_ANGLES (float64) 設置成功: flags=0x%04X", imu_data.flags);
                        } else {
                            LOG_WARN("XBUS_PARSER", "❌ Float64 歐拉角包含非有限值");
                        }
                    } else {
                        LOG_WARN("XBUS_PARSER", "❌ EULER_ANGLES 長度錯誤: expected=12 or 24, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                case XbusDataID::STATUS_BYTE:
                    if (block_len == 1) {
                        uint8_t status = data[offset];
                        // 根據 Xsens 文檔正確解析狀態位元
                        if ((status & 0x01) == 0) { // bit 0: orientation valid
                            imu_data.flags |= IMU_CALIBRATED;
                        }
                        has_data = true;
                    } else {
                        LOG_DEBUG("XBUS_PARSER", "STATUS_BYTE 長度錯誤: expected=1, actual=%u", block_len);
                        bad_len_count++;
                    }
                    break;
                    
                default:
                    // 未知 XDI：使用長度跳過整個 block (不會失去同步)
                    unknown_xdi_count++;
                    if (xdi == 0x2020) {
                        LOG_ERROR("XBUS_PARSER", "❌❌❌ ROTATION_MATRIX (0x2020) 進入 default case! xdi_enum=%d", (int)xdi_enum);
                    }
                    LOG_DEBUG("XBUS_PARSER", "❓ Unknown XDI: 0x%04X, len=%u, skipping...", xdi, block_len);
                    break;
            }
            
            // 統一跳過當前 block
            offset += block_len;
        }
        
        LOG_DEBUG("XBUS_PARSER", "🔍 Payload parse complete: has_data=%s, processed_blocks=%zu", 
                 has_data ? "true" : "false", (offset > 0) ? 1 : 0);
        
        if (has_data) {
            // 如果沒有設備時間戳，使用系統時間
            if (imu_data.timestamp_us == 0) {
                imu_data.timestamp_us = micros();
            }
            
            imu_data.flags |= IMU_DATA_FRESH;
            // 計算數據品質（簡單實作）
            imu_data.data_quality = XbusParser::calculateDataQuality(imu_data);
            LOG_DEBUG("XBUS_PARSER", "✅ IMU data ready: flags=0x%04X, quality=%u", 
                     imu_data.flags, imu_data.data_quality);
        } else {
            LOG_DEBUG("XBUS_PARSER", "❌ No valid IMU data extracted");
        }
        
        return has_data;
    }

} // namespace XbusParser