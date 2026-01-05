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
// ---- HINS streaming packet signature --------------------------------------
static const uint8_t HINS_STREAM_HDR[] = { 0x75,0x65,0x82,0x13,0x13,0x49 };
static const uint16_t HINS_FIELD_DATA_LEN = 17;


// INT_SYNC / EXT_SYNC / STOP_RUN are defined in common.h

#define DATA_DELAY_CNT 5

// 讀取來自 FPGA payload 長度（11 個 float × 4 bytes），不含 header 與 CRC
#define SENSOR_PAYLOAD_LEN 44
// 姿態 3 floats = 12 bytes
#define ATT_PAYLOAD_LEN    12
// 合併後總長度
#define TOTAL_PAYLOAD_LEN  (SENSOR_PAYLOAD_LEN + ATT_PAYLOAD_LEN)


static my_sensor_t sensor_raw = {}, sensor_cali = {};

static ahrs_att_stage_ctx_t g_att_ctx;
static uint8_t g_att_ctx_inited = 0;

static uint8_t  data_cnt = 0;
static uint32_t try_cnt = 0;
static my_att_t my_att, my_GYRO_cali, my_ACCL_cali;
// ============================================================================
// AHRS mode refactor helpers
//   Goal: split "setup" (start/stop/one-shot init) and "run" (stream->raw->cali
//         ->attitude->frame-transform->output) into separate functions.
//   Note: The "attitude" and "frame-transform" blocks are additionally factored
//         into standalone static helpers so they can be moved into a dedicated
//         library later without changing the call sites.

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
    ahrs_attitude.captureYawZeroLocalCase(); // reset yaw0 (relative zero)
    rx->run = 1;

    // Start streaming from FPGA
    sendCmd(g_cmd_port_fpga, HDR_ABBA, TRL_5556, 2, 2, 2);
    delay(10);

    // Start streaming from HINS (raw MIP)
    hins_send_mip_raw(g_cmd_port_hins, HINS_SET_TO_IDLE);
    delay(2);
    hins_send_mip_raw(g_cmd_port_hins, HINS_RESUME);
    delay(2);

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

    hins_send_mip_raw(g_cmd_port_hins, HINS_SET_TO_IDLE);
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

static bool hins_stage_update_raw(const uint8_t* d, uint16_t len)
{
    if (!d || len < 17) return false;

    const float tow_s      = be_f32(&d[0]);
    const float heading_rad= be_f32(&d[4]);
    const float unc_rad    = be_f32(&d[8]);
    const uint8_t fix_type = d[12];
    const uint16_t status  = be_u16(&d[13]);
    const uint16_t valid   = be_u16(&d[15]);

    sensor_raw.hins.tow.float_val         = tow_s;
    sensor_raw.hins.heading.float_val     = heading_rad;
    sensor_raw.hins.heading_unc.float_val = unc_rad;
    sensor_raw.hins.fix_type              = fix_type;
    sensor_raw.hins.status_flag           = status;
    sensor_raw.hins.valid_flag            = valid;

    return true;
}

// Attitude block (candidate to move into dedicated lib later)
static void ahrs_stage_attitude_update(void)
{
    // Extracted into lib: keep callsite stable.
    ahrs_att_stage_update(&g_att_ctx, &my_GYRO_cali, &my_ACCL_cali, &my_att);
}

// Frame-transform block (candidate to move into dedicated lib later)
static void ahrs_stage_frame_transform_to_case(void)
{
    // Extracted into lib: keep callsite stable.
    ahrs_transform_sensorVecToCase(&my_GYRO_cali, &my_ACCL_cali, &sensor_cali);
}

static void ahrs_stage_output_send_if_ready(void)
{
    uint8_t out[TOTAL_PAYLOAD_LEN];
    pack_sensor_payload_from_cali(&sensor_cali, out);
    memcpy(out + SENSOR_PAYLOAD_LEN, my_att.bin_val, ATT_PAYLOAD_LEN);

    uint8_t crc[4];
    gen_crc32(KVH_HEADER, out, TOTAL_PAYLOAD_LEN, crc);

    if (data_cnt >= DATA_DELAY_CNT) {
        Serial2.write(KVH_HEADER, sizeof(KVH_HEADER));
        Serial2.write(out, TOTAL_PAYLOAD_LEN);
        Serial2.write(crc, 4);
    }
}

static void ahrs_run_tick(cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    if (rx->run != 1) return;

    uint8_t* pkt = nullptr;
    if (!ahrs_stage_read_stream(&pkt)) return;
    if (data_cnt < DATA_DELAY_CNT) data_cnt++;

    if (!ahrs_stage_update_raw(pkt)) return;

    // ---- [NEW] Read HINS stream (0x82,0x49) and update sensor_raw.hins ----
    {
        uint8_t hins_payload[HINS_FIELD_DATA_LEN];

        const bool hins_ok = hins_read_stream_payload(
            g_cmd_port_hins,
            HINS_STREAM_HDR, (uint16_t)sizeof(HINS_STREAM_HDR),
            (uint16_t)HINS_FIELD_DATA_LEN,
            hins_payload, (uint16_t)sizeof(hins_payload),
            2  // timeout_ms (best-effort)
        );

        if (hins_ok) {
            (void)hins_stage_update_raw(hins_payload, (uint16_t)sizeof(hins_payload));

            static uint32_t last_ms = 0;
            uint32_t now = millis();
            if (now - last_ms >= 200) { // 5Hz print
                last_ms = now;
                const float rad = sensor_raw.hins.heading.float_val;
                const float deg = rad * 57.2957795f;
                const float unc = sensor_raw.hins.heading_unc.float_val * 57.2957795f;
                // Serial.print(deg);
                // Serial.print(",");
                // Serial.println(unc);
            }
        }
    }
    // ----------------------------------------------------------------------

    ahrs_stage_calibrate(fog_parameter);
    ahrs_stage_attitude_update();
    ahrs_stage_frame_transform_to_case();
    // ---- [TEST] True Heading feedback to HINS via transact (10Hz) ----
    {
        static uint32_t t0_ms = 0;
        static uint32_t last_tx_ms = 0;
        const uint32_t now = millis();
        if (t0_ms == 0) t0_ms = now;

        if (now - last_tx_ms >= 100) { // 10Hz
            last_tx_ms = now;

            const uint8_t  timebase   = 0x01;        // INTERNAL_REFERENCE
            const uint32_t ns32       = (uint32_t)((now - t0_ms) * 1000000UL); // ms->ns (mod 2^32)
            const uint8_t  frame_id   = 0x01;        // external estimate
            const float    heading    = 0.0f;        // fixed test
            const float    unc        = 0.0349066f;  // ~2 deg
            const uint16_t valid      = 0x0001;

            uint8_t ack_code = 0xFF, ack_echo = 0xFF;
            Status st = hins_true_heading_transact_u64ns(
                g_cmd_port_hins,
                20,
                &sensor_raw.true_heading,
                &ack_code,
                &ack_echo
            );

            static uint32_t last_print = 0;
            if (now - last_print >= 500) {
                last_print = now;
                Serial.print("[HINS_TX] st=");
                Serial.print((int)st);

                Serial.print(" ack=0x");
                if (ack_code < 0x10) Serial.print('0');
                Serial.print(ack_code, HEX);

                Serial.print(" echo=0x");
                if (ack_echo < 0x10) Serial.print('0');
                Serial.print(ack_echo, HEX);

                Serial.print(" ns=");
                Serial.print((unsigned long)ns32);

                Serial.print(" heading=");
                Serial.print(heading, 6);   // 6 digits after decimal

                Serial.print(" unc=");
                Serial.print(unc, 6);

                Serial.println();

            }
        }
    }

    ahrs_stage_output_send_if_ready();
}

void acq_hins (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    // 1) handle setup commands (start/stop)
    ahrs_handle_setup(rx);

    // 2) run pipeline tick (only when rx->run == 1)
    ahrs_run_tick(rx, fog_parameter);
}


