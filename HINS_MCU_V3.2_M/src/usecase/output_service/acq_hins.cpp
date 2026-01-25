#include "../../common.h"
// #include "output_mode_config.h"
#include "../../myUART.h"
#include <string.h> // memset, memcpy
#include "../../app/app_state.h"
#include "../../utils/crc32.h"

#include "ahrs_attitude_lib.h"
#include "ahrs_transform_lib.h"
#include "../../drivers/link/hins_link.h"
#include "../../utils/endian.h"
// #include "../../MadgwickAHRS_IMU.h"



#ifndef INT_SYNC
  #define INT_SYNC 1
#endif

#ifndef EXT_SYNC
  #define EXT_SYNC 2
#endif

#ifndef STOP_RUN
  #define STOP_RUN 4
#endif

// ---- HINS streaming control (raw MIP frames, include checksum) ------------
static const uint8_t HINS_SET_TO_IDLE[] = { 0x75,0x65,0x01,0x02,0x02,0x02,0xE1,0xC7 };  // Stop stream
static const uint8_t HINS_RESUME[]      = { 0x75,0x65,0x01,0x02,0x02,0x06,0xE5,0xCB };  // resume
static const uint8_t HINS_STREAM_ON[]      = { 0x75,0x65,0x0C,0x05,0x05,0x11,0x01,0x82,0x01,0x85,0x1C };  // Data Stream Control_ON
static const uint8_t HINS_STREAM_OFF[]      = { 0x75,0x65,0x0C,0x05,0x05,0x11,0x01,0x82,0x00,0x84,0x1B  };  // Data Stream Control_OFF
// ---- HINS streaming packet signature --------------------------------------
static const uint8_t HINS_STREAM_HDR[] = { 0x75,0x65,0x82,0x13,0x13,0x49 };
static const uint16_t HINS_FIELD_DATA_LEN = 17;

static float g_heading_offset = 0.0f; // 存儲 IMU 與 GNSS DA 的偏差


// INT_SYNC / EXT_SYNC / STOP_RUN are defined in common.h

#define DATA_DELAY_CNT 5

// 讀取來自 FPGA payload 長度（13 個 float × 4 bytes），不含 header 與 CRC
#define SENSOR_PAYLOAD_LEN 52
// 姿態 3 floats = 12 bytes
#define ATT_PAYLOAD_LEN    12
// 合併後總長度
#define TOTAL_PAYLOAD_LEN  (44 + ATT_PAYLOAD_LEN)


static my_sensor_t sensor_raw = {}, sensor_cali = {};

static ahrs_att_stage_ctx_t g_att_ctx;
static uint8_t g_att_ctx_inited = 0;

static uint8_t  data_cnt = 0;
static uint32_t try_cnt = 0;
static my_att_t my_att, my_GYRO_cali, my_ACCL_cali;


static void ahrs_reset_runtime_state(void)
{
    sensor_raw = {};
    sensor_cali = {};

    data_cnt = 0;
    try_cnt  = 0;

    // reset extracted attitude-stage states
    if (!g_att_ctx_inited) {
        ahrs_att_stage_init(&g_att_ctx, 100.0f);
        g_att_ctx_inited = 1;
    } else {
        ahrs_att_stage_reset(&g_att_ctx);
    }
}


static void ahrs_start_stream(cmd_ctrl_t* rx)
{
    DEBUG_PRINT("acq_ahrs start\n");
    // init extracted attitude-stage context (one-time)
    if (!g_att_ctx_inited) {
        ahrs_att_stage_init(&g_att_ctx, 100.0f);
        g_att_ctx_inited = 1;
    }
    // ahrs_attitude.resetAttitude(true);     // 清 quaternion + 角度連續化狀態
    // ahrs_att_stage_init(&g_att_ctx, 100.0f); // 或至少 reset g_att_ctx 的 MA/LP

    ahrs_attitude.captureYawZeroLocal(); // reset yaw0 (relative zero)
    rx->run = 1;

    // Start streaming from FPGA
    sendCmd(g_cmd_port_fpga, HDR_ABBA, TRL_5556, 2, 2, 2);
    delay(10);

    // Start streaming from HINS (raw MIP)
    g_heading_offset = 0.0f; // 每次啟動時重置 Offset，重新抓取 GNSS 基準
    hins_send_mip_raw(g_cmd_port_hins, HINS_STREAM_ON);
    delay(2);
    // hins_send_mip_raw(g_cmd_port_hins, HINS_RESUME);
    // delay(2);

    // one-shot init for this run (moved into attitude lib)
    ahrs_att_stage_prepare_run(&g_att_ctx);
    reset_FPGA_timer();

    
}

static void ahrs_stop_stream(cmd_ctrl_t* rx)
{
    DEBUG_PRINT("acq_ahrs select stop\n");
    ahrs_attitude.resetAttitude(true); // reset attitude and yaw0
    rx->run = 0;

    sendCmd(g_cmd_port_fpga, HDR_ABBA, TRL_5556, 2, 4, 2);
    delay(10);

    hins_send_mip_raw(g_cmd_port_hins, HINS_STREAM_OFF);
    delay(2);

    // stop should fully clear internal states
    ahrs_reset_runtime_state();
}

static void ahrs_handle_setup(cmd_ctrl_t* rx)
{
    if (rx->select_fn != SEL_HINS) return;
    rx->select_fn = SEL_IDLE; // consume command
    DEBUG_PRINT("-> select acq_ahrs mode\n");

    if (rx->value == INT_SYNC || rx->value == EXT_SYNC) {
        ahrs_start_stream(rx);
    } else if (rx->value == STOP_RUN) {
        ahrs_stop_stream(rx);
    }
}

// ---- run sub-stages --------------------------------------------------------
static bool ahrs_stage_read_stream(uint8_t** pkt_out)
{
    *pkt_out = readDataStream(HDR_ABBA, 2, TRL_5556, 2, SENSOR_PAYLOAD_LEN, &try_cnt);
    return (*pkt_out != nullptr);
}

static bool ahrs_stage_update_raw(const uint8_t* pkt)
{
    return (update_raw_data((uint8_t*)pkt, &sensor_raw) == 0);
}

static void ahrs_stage_calibrate(fog_parameter_t* fog_parameter)
{
    sensor_data_cali(&sensor_raw, &sensor_cali, fog_parameter);

    // cache to my_att_t for downstream blocks (keep current behavior)
    my_GYRO_cali.float_val[0] = sensor_cali.fog.fogx.step.float_val;
    my_GYRO_cali.float_val[1] = sensor_cali.fog.fogy.step.float_val;
    my_GYRO_cali.float_val[2] = sensor_cali.fog.fogz.step.float_val;
    my_ACCL_cali.float_val[0] = sensor_cali.adxl357.ax.float_val;
    my_ACCL_cali.float_val[1] = sensor_cali.adxl357.ay.float_val;
    my_ACCL_cali.float_val[2] = sensor_cali.adxl357.az.float_val;
}


static bool hins_stage_update_raw(Stream& port, hins_dual_data_t* hins) {
    static const uint8_t HINS_COMPOSITE_HDR[] = {0x75, 0x65, 0x82, 0x21};
    
    // 呼叫非阻塞 Parser
    uint8_t* payload = hins_parse_stream_bytewise(port, HINS_COMPOSITE_HDR, 4, 33);

    if (payload != nullptr) {
        // 解析 0xD3 (TOW)
        hins->gps_tow = be_f64(&payload[2]); 

        // 解析 0x49 (GNSS Data)
        const uint8_t* d49 = &payload[16]; 
        hins->heading_da  = be_f32(&d49[4]);
        hins->heading_unc = be_f32(&d49[8]);
        hins->fix_type    = d49[12];
        hins->status_flag = be_u16(&d49[13]);
        hins->valid_flag_da = be_u16(&d49[15]);

        // 使用 Serial.print 進行詳細 Debug
        static uint32_t last_print = 0;
        if (millis() - last_print > 50) { 
            last_print = millis();
            Serial.print("[HINS_PARSE] TOW: "); Serial.print(hins->gps_tow, 3);
            Serial.print(" | HDG: "); Serial.print(hins->heading_da * 57.29578f, 2); // 1 弧度約等於 57.29578 度
            Serial.print(" | FIX: "); Serial.print(hins->fix_type);
            Serial.print(" | STATUS: 0x"); Serial.print(hins->status_flag, HEX);
            Serial.print(" | VALID: 0x"); Serial.println(hins->valid_flag_da, HEX);
        }
        return true;
    }
    return false;
}

// static bool hins_stage_update_raw(Stream& port, hins_dual_data_t* hins) {

//     // if (port.available()) {
//     //     serial_printf("%02X ", port.read());
//     // }

//     static const uint8_t HINS_COMPOSITE_HDR[] = {0x75, 0x65, 0x82, 0x21};
//     uint8_t payload[33];

//     // 讀取複合包 (0xD3 + 0x49)
//     if (hins_read_stream_payload(port, HINS_COMPOSITE_HDR, 4, 33, payload, 33, 5)) {
//         // --- 解析 0xFF, 0xD3 (GPS Timestamp) ---
//         // Header(2 bytes: 0E D3) + Data(12 bytes)
//         // TOW 在 Data 段的前 8 bytes (payload[2]~[9])
//         hins->gps_tow = be_f64(&payload[2]); 

//         // --- 解析 0x82, 0x49 (GNSS Dual Antenna Status) ---
//         // 0xD3 佔 14 bytes (2 header + 12 data)，因此 0x49 Header 在 payload[14..15]
//         const uint8_t* d49 = &payload[16]; 
//         hins->heading_da  = be_f32(&d49[4]); // 跳過 Field TOW
//         hins->heading_unc = be_f32(&d49[8]);
//         hins->fix_type    = d49[12];
//         hins->status_flag = be_u16(&d49[13]);
//         hins->valid_flag_da = be_u16(&d49[15]);

//         // 3. Debug 輸出
//         static uint32_t last_print = 0;
//         if (millis() - last_print > 500) { 
//             last_print = millis();
            
//             // 如果你的 serial_printf 支援 %f
//             // serial_printf("[HINS] TOW:%.3f, HDG:%.4f, FIX:%u, VALID:0x%04X\n", 
//             //               (double)hins->gps_tow, (float)hins->heading_da, 
//             //               hins->fix_type, hins->valid_flag_da);

//             // 如果上面印不出來或顯示 "?"，請改用下面這段原生語法：
//             // /*
//             Serial.print("[HINS] TOW: "); Serial.print(hins->gps_tow, 3);
//             Serial.print(" HDG: "); Serial.print(hins->heading_da, 4);
//             Serial.print(" FIX: "); Serial.println(hins->fix_type);
//             // */
//         }

//         return true;
//     }
    
//     return false;
// }


static void hins_stage_logic_control(Stream& port, const hins_dual_data_t* hins, float imu_heading) {
    // 判斷 GNSS 品質 (Fix Type 1 or 2 且 Valid Bit 0 為 1) [cite: 22]
    if (hins->fix_type >= 1 && (hins->valid_flag_da & 0x0001)) {
        // 狀態 A：校正狀態
        float instant_offset = hins->heading_da - imu_heading;
        
        // 角度環繞校正 (-PI ~ PI)
        while (instant_offset >  PI) instant_offset -= 2.0f * PI;
        while (instant_offset < -PI) instant_offset += 2.0f * PI;

        // 更新偏移量 (Alpha=0.02 低通濾波)
        g_heading_offset = (g_heading_offset * 0.98f) + (instant_offset * 0.02f);
    } 
    else {
        // 狀態 B：保持狀態 (GNSS Lost/None) [cite: 22]
        true_heading_t th;
        memset(&th, 0, sizeof(th)); // 先清空，確保未定義位元組為 0

        // 1. 明確設定時間基準與保留位
        th.ts.timebase = 0x01; // INTERNAL_REFERENCE 
        th.ts.reserved = 0x01; // 依照手冊規範

        // 2. 轉換 TOW 到奈秒 
        th.ts.nanosecs = (uint64_t)(hins->gps_tow * 1e9);
        
        // 3. 設定其餘 Heading 數據
        th.Frame_id = 0x01;    // External estimate
        th.Heading.float_val = imu_heading + g_heading_offset;
        th.Uncertainty.float_val = 0.05f; // 初始漂移預估量
        th.valid_flag = 0x0001; // Bit 0: Valid

        // 4. 呼叫底層發送
        hins_true_heading_standard(port, &th);
    }
}

// Attitude block (candidate to move into dedicated lib later)
static void ahrs_stage_attitude_update(fog_parameter_t* fog_parameter)
{
    my_att_t my_GYRO_case;
    my_att_t my_ACCL_case;

    my_GYRO_case.float_val[0] = sensor_cali.fog.fogx.step.float_val;
    my_GYRO_case.float_val[1] = sensor_cali.fog.fogy.step.float_val;
    my_GYRO_case.float_val[2] = sensor_cali.fog.fogz.step.float_val;
    my_ACCL_case.float_val[0] = sensor_cali.adxl357.ax.float_val;
    my_ACCL_case.float_val[1] = sensor_cali.adxl357.ay.float_val;
    my_ACCL_case.float_val[2] = sensor_cali.adxl357.az.float_val;

    g_att_ctx.out_th = fog_parameter->paramZ[13].data.float_val;
    g_att_ctx.out_th_en = fog_parameter->paramZ[14].data.int_val;

    // Extracted into lib: keep callsite stable.
    ahrs_att_stage_update(&g_att_ctx, &my_GYRO_case, &my_ACCL_case, &my_att);
}

// Frame-transform block (candidate to move into dedicated lib later)
static void ahrs_stage_frame_transform_to_case(void)
{
    // Extracted into lib: keep callsite stable.W
    ahrs_transform_sensorVecToCase(&my_GYRO_cali, &my_ACCL_cali, &sensor_cali);

}

static void ahrs_stage_output_send_if_ready(void)
{
    uint8_t out[TOTAL_PAYLOAD_LEN];

    pack_sensor_payload_from_cali(&sensor_cali, out);
    memcpy(out + 44, my_att.bin_val, ATT_PAYLOAD_LEN);
    // Serial.print(my_att.float_val[0]);
    // Serial.print(" ");
    // Serial.print(my_att.float_val[1]);
    // Serial.print(" ");
    // Serial.println(my_att.float_val[2]);


    uint8_t crc[4];
    gen_crc32(KVH_HEADER, out, TOTAL_PAYLOAD_LEN, crc);

    if (data_cnt >= DATA_DELAY_CNT) {
        g_cmd_port_output.write(KVH_HEADER, sizeof(KVH_HEADER));
        g_cmd_port_output.write(out, TOTAL_PAYLOAD_LEN);
        g_cmd_port_output.write(crc, 4);
    }
}

static void ahrs_run_tick(cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    if (rx->run != 1) return;

    uint8_t* pkt = nullptr;
    if (!ahrs_stage_read_stream(&pkt)) return;
    if (data_cnt < DATA_DELAY_CNT) data_cnt++;

    if (!ahrs_stage_update_raw(pkt)) return;

    // ---- HINS 資料流更新與邏輯控制 ----
    if (hins_stage_update_raw(g_cmd_port_hins, &sensor_raw.hins)) 
    {
        // hins_stage_logic_control(g_cmd_port_hins, &sensor_raw.hins, my_att.float_val[2]);
    }
    // ----------------------------------------

    ahrs_stage_calibrate(fog_parameter);
    ahrs_stage_frame_transform_to_case();
    ahrs_stage_attitude_update(fog_parameter);
    ahrs_stage_output_send_if_ready();
}

void acq_hins (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    // 1) handle setup commands (start/stop)
    ahrs_handle_setup(rx);

    // 2) run pipeline tick (only when rx->run == 1)
    ahrs_run_tick(rx, fog_parameter);
}


