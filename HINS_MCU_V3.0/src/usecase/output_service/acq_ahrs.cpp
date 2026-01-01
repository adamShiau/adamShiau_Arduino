#include "../../common.h"
// #include "output_mode_config.h"
#include "../../myUART.h"
#include <string.h> // memset, memcpy
#include "../../app/app_state.h"
#include "../../utils/crc32.h"

#include "ahrs_attitude_lib.h"
#include "ahrs_transform_lib.h"

#ifndef INT_SYNC
  #define INT_SYNC 1
#endif

#ifndef EXT_SYNC
  #define EXT_SYNC 2
#endif

#ifndef STOP_RUN
  #define STOP_RUN 4
#endif


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
    sendCmd(Serial1, HDR_ABBA, TRL_5556, 2, 2, 2);
    delay(10);

    // one-shot init for this run (moved into attitude lib)
    ahrs_att_stage_prepare_run(&g_att_ctx);
    reset_FPGA_timer();

    // Keep historical behavior: do NOT clear all runtime state on start.
    // If you want "clean start" semantics, call ahrs_reset_runtime_state() here.
}

static void ahrs_stop_stream(cmd_ctrl_t* rx)
{
    DEBUG_PRINT("acq_ahrs select stop\n");
    ahrs_attitude.resetAttitude(true); // reset attitude and yaw0
    rx->run = 0;

    sendCmd(Serial1, HDR_ABBA, TRL_5556, 2, 4, 2);

    // stop should fully clear internal states
    ahrs_reset_runtime_state();
}

static void ahrs_handle_setup(cmd_ctrl_t* rx)
{
    if (rx->select_fn != SEL_AHRS) return;
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

    ahrs_stage_calibrate(fog_parameter);
    ahrs_stage_attitude_update();
    ahrs_stage_frame_transform_to_case();
    ahrs_stage_output_send_if_ready();
}

void acq_ahrs (cmd_ctrl_t* rx, fog_parameter_t* fog_parameter)
{
    // 1) handle setup commands (start/stop)
    ahrs_handle_setup(rx);

    // 2) run pipeline tick (only when rx->run == 1)
    ahrs_run_tick(rx, fog_parameter);
}
