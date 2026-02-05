/*
 * common.cpp
 *
 * Arduino implementation of helpers defined in common.h
 * - get_uart_cmd(): interprets a parsed packet (data pointer) into cmd_ctrl_t.
 * - cmd_mux(): selects the MUX group based on cmd range.
 *
 * Assumptions:
 * - 'data' points to a buffer where:
 *     data[0] = condition (RX_CONDITION_*)
 *     data[1].. = payload (layout depends on condition)
 * - Layouts (must match your readDataDynamic() sizes):
 *   condition 1 (ABBA_5556): payload = [cmd][v3][v2][v1][v0][ch]
 *   condition 2 (CDDC_5758): payload = [cmd][SN[0..11]]
 *   condition 3 (EFFE_5354): payload = [cmd][v3][v2][v1][v0][ch]
 */

#include "common.h"
#include <ctype.h>
#include <string.h>
#include "app/app_state.h"
// #include "domain/protocol/ack_codec_v1.h"
// #include "utils/version_info.h"

// #include "utils/serial_printf.h"

static const float SF_ACCL_2G	= 0.061e-3F;
static const float SF_ACCL_4G	=0.122e-3F;
static const float SF_ACCL_8G	=0.244e-3F;
static const float SF_ACCL_16G	=0.488e-3F;

static const float SF_GYRO_125DPS	=4.37e-3F;
static const float SF_GYRO_250DPS	=8.75e-3F;
static const float SF_GYRO_500DPS	=17.5e-3F;
static const float SF_GYRO_1000DPS	=35e-3F;
static const float SF_GYRO_2000DPS	=70e-3F;
static const float SF_GYRO_4000DPS	=140e-3F;
static const float SF_TEMP =0.00390625F;

static const float COE_TIMER = 0.0001;

static const float COE_TEMP_AD590 = 0.00007165585;
static const float VIN_MON_COFF = 0.0000023505;
static const float ADC3p3_COEE = 0.00000039339; // 3.3/2^23 = 3.93390656e-7

const uint8_t HDR_ABBA[2] = {0xAB, 0xBA};
const uint8_t HDR_CDDC[2] = {0xCD, 0xDC};
const uint8_t HDR_EFFE[2] = {0xEF, 0xFE};
const uint8_t TRL_5556[2] = {0x55, 0x56};
const uint8_t TRL_5758[2] = {0x57, 0x58};
const uint8_t TRL_5354[2] = {0x53, 0x54};
const uint8_t KVH_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};

// #define CASE_MEMS

// -----------------------------------------------------------------------------
// Forward declare Serial1 (constructed in myUART.cpp)
// class Uart;
// extern Uart Serial1;
// -----------------------------------------------------------------------------

// first order temperature compensation, one T
static inline float sf_temp_comp_1st(float temp, float slope, float offset) {
  return slope * temp + offset;
}

// first order temperature compensation, three T
static inline float bias_temp_comp_1st_3t(float temp,
                                          float T1, float T2,
                                          float s1, float o1,
                                          float s2, float o2,
                                          float s3, float o3) {
  float slope, offset;
  if (temp < T1)      { slope = s1; offset = o1; }
  else if (temp < T2) { slope = s2; offset = o2; }
  else                { slope = s3; offset = o3; }
  return slope * temp + offset;
}

/* Helper: assemble big-endian 4 bytes into signed 32-bit */
static inline int32_t be_bytes_to_i32(uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0)
{
  uint32_t u =
      ((uint32_t)b3 << 24) |
      ((uint32_t)b2 << 16) |
      ((uint32_t)b1 <<  8) |
      ((uint32_t)b0 <<  0);
  return (int32_t)u;  // interpret as signed if needed
}

void dumpPkt(uint8_t *pkt, int len) {
    if (!pkt) {
        DEBUG_PRINT("pkt = NULL\n");
        return;
    }

    DEBUG_PRINT("Payload len = %d\n", len);

    for (int i = 0; i < len; i++) {
        if (i % 4 == 0) {
            DEBUG_PRINT("[%d] ", i/4);
        }
        DEBUG_PRINT("%02X ", pkt[i]);
        if (i % 4 == 3) {
            DEBUG_PRINT("\n");
        }
    }
    DEBUG_PRINT("\n");
}

/**
 * @brief Update my_sensor_t from a 44-byte payload in the fixed order.
 *
 * pkt  : pointer to 44-byte payload returned by readDataBytewise()
 * out  : target structure to fill
 * return: 0 on success, -1 on invalid args
 */
int update_raw_data(const uint8_t* pkt, my_sensor_t* out)
{
    if (!pkt || !out) return -1;

    int idx = 0;

    // fog.*.step
    memcpy(out->fog.fogz.step.bin_val,  &pkt[idx], 4); idx += 4;
    memcpy(out->m_gyro.gz.bin_val,      &pkt[idx], 4); idx += 4;
    memcpy(out->fog.fogy.step.bin_val,  &pkt[idx], 4); idx += 4;
    memcpy(out->fog.fogx.step.bin_val,  &pkt[idx], 4); idx += 4;

    // adxl357 {ax, ay, az}
    memcpy(out->adxl357.ax.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->adxl357.ay.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->adxl357.az.bin_val,     &pkt[idx], 4); idx += 4;

    // temp {x, y, z}
    memcpy(out->temp.tempz.bin_val,     &pkt[idx], 4); idx += 4;
    // memcpy(out->temp.tempy.bin_val,     &pkt[idx], 4); idx += 4;
    // memcpy(out->temp.tempx.bin_val,     &pkt[idx], 4); idx += 4;

    // house keeping
    memcpy(out->hk.Vin_mon.bin_val,         &pkt[idx], 4); idx += 4;
    memcpy(out->hk.Tact_mon.bin_val,        &pkt[idx], 4); idx += 4;
    memcpy(out->hk.pump_pd_mon.bin_val,     &pkt[idx], 4); idx += 4;

    // adxl357.temp
    memcpy(out->adxl357.temp.bin_val,   &pkt[idx], 4); idx += 4;

    // time
    memcpy(out->time.time.bin_val,      &pkt[idx], 4); idx += 4;

    return 0;
}



/* -------------------------------------------------------------------------- */
/* Parse a UART command buffer into cmd_ctrl_t                                 */
/* -------------------------------------------------------------------------- */
void get_uart_cmd(uint8_t* data, cmd_ctrl_t* rx)
{
  if (!data || !rx) return;

  rx->condition = (rx_condition_t)data[0];

  if (rx->condition == RX_CONDITION_ABBA_5556) {
    // payload: [cmd][v3][v2][v1][v0][ch]  (6 bytes total)
    rx->complete = 1;
    rx->cmd      = data[1];
    rx->value    = be_bytes_to_i32(data[2], data[3], data[4], data[5]);
    rx->ch       = data[6];

    DEBUG_PRINT("condition: %d, ", RX_CONDITION_ABBA_5556);
    DEBUG_PRINT("cmd: %x, value: %d, ch: %d\n", rx->cmd, rx->value, rx->ch);

  }
  else if (rx->condition == RX_CONDITION_CDDC_5758) {
    // payload: [cmd][SN[0..11]] (12 bytes SN, not including NUL)
    rx->complete = 1;
    rx->cmd      = data[1];

    for (int i = 0; i < 12; i++) {
      rx->SN[i] = data[i + 2];
    }
    rx->SN[12] = '\0';

  }
  else if (rx->condition == RX_CONDITION_EFFE_5354) {
    // payload: [cmd][v3][v2][v1][v0][ch]
    rx->complete = 1;
    rx->cmd      = data[1];
    rx->value    = be_bytes_to_i32(data[2], data[3], data[4], data[5]);
    rx->ch       = data[6];

    DEBUG_PRINT("condition: %d, ", RX_CONDITION_EFFE_5354);
    DEBUG_PRINT("cmd: %x, value: %d, ch: %d\n", rx->cmd, rx->value, rx->ch);
  }
  else {
    // Unknown condition; leave rx->complete unchanged (typically 0)
  }
}

/* -------------------------------------------------------------------------- */
/* Choose MUX group based on cmd (same logic as your Nios version)             */
/* -------------------------------------------------------------------------- */
void cmd_mux(cmd_ctrl_t* rx)
{
  if (!rx) return;

  if (rx->complete == 1) {
    rx->complete = 0;  // consume the command

    if (rx->cmd > 7)  {rx->mux = (uint8_t)MUX_PARAMETER; DEBUG_PRINT("cmd_mux: MUX_PARAMETER\n");}
    else              {rx->mux = (uint8_t)MUX_OUTPUT; DEBUG_PRINT("cmd_mux: MUX_OUTPUT\n");}
  }
}

/**
 * @brief Update parameter container with rx->value at given index.
 *
 * Writes rx->value into fog_inst according to rx->ch:
 *   ch==1 -> paramX[idx].data.int_val
 *   ch==2 -> paramY[idx].data.int_val
 *   ch==3 -> paramZ[idx].data.int_val
 *   ch==4 -> misalignment[idx].data.int_val
 * Also sets .type = TYPE_INT for that slot.
 *
 * @param rx       Parsed command (must not be NULL)
 * @param fog_inst Target container (must not be NULL)
 * @param idx      Parameter index
 *                 - 0..PAR_LEN-1 for ch=1..3
 *                 - 0..MIS_LEN-1 for ch=4
 */
void update_parameter_container(const cmd_ctrl_t* rx, fog_parameter_t* fog_inst, uint8_t idx)
{
  if (!rx || !fog_inst) return;

  switch (rx->ch) {
    case 3: // paramX
      if (idx < PAR_LEN) {
        fog_inst->paramX[idx].data.int_val = rx->value;
        DEBUG_PRINT("paramX[%d] = %d\n", idx, fog_inst->paramX[idx].data.int_val);
      }
      break;

    case 2: // paramY
      if (idx < PAR_LEN) {
        fog_inst->paramY[idx].data.int_val = rx->value;
        DEBUG_PRINT("paramY[%d] = %d\n", idx, fog_inst->paramY[idx].data.int_val);
      }
      break;

    case 1: // paramZ
      if (idx < PAR_LEN) {
        fog_inst->paramZ[idx].data.int_val = rx->value;
        DEBUG_PRINT("paramZ[%d] = %d\n", idx, fog_inst->paramZ[idx].data.int_val);
      }
      break;

    case 4: // misalignment
      if (idx < MIS_LEN) {
        fog_inst->misalignment[idx].data.int_val = rx->value;
        // DEBUG_PRINT("misalignment[%d] = %d\n", idx, fog_inst->misalignment[idx].data.int_val);
      }
      break;

    case 6: // configuration
      if (idx < CFG_LEN) {
        fog_inst->config[idx].data.int_val = rx->value;
        // DEBUG_PRINT("misalignment[%d] = %d\n", idx, fog_inst->misalignment[idx].data.int_val);
      }
      break;

    default:
      // unknown channel -> ignore
      break;
  }
}


void reset_FPGA_timer(void)
{
	DEBUG_PRINT("reset_FPGA_timer\n");
	sendCmd(g_cmd_port_fpga, HDR_ABBA, TRL_5556, CMD_HW_TIMER_RST, 1, 1);
	delay(10); 
	sendCmd(g_cmd_port_fpga, HDR_ABBA, TRL_5556, CMD_HW_TIMER_RST, 0, 1);
}

// void set_data_rate(uint32_t rate)
// {
// 	sendCmd(g_cmd_port_fpga, HDR_ABBA, TRL_5556, CMD_SYNC_CNT, rate, 1);
// }

#ifndef FOG_JSON_TIMEOUT_MS
#define FOG_JSON_TIMEOUT_MS 1500  // 等待 JSON 完整到達的逾時（毫秒）
#endif

#ifndef FOG_JSON_BUF_SIZE
#define FOG_JSON_BUF_SIZE 1200    // JSON 暫存緩衝大小（依實際內容調整）
#endif

// Convert int32_t to mem_unit_t as an integer payload
static inline mem_unit_t mem_from_i32(int32_t v) {
  mem_unit_t m;
  m.type = TYPE_INT;
  m.data.int_val = v;
  return m;
}

/**
 * @brief 從串流 (Stream) 讀取一個 JSON 物件字串
 *
 * 此函數會嘗試從指定的 Stream 讀取資料，直到遇到完整的 JSON 物件
 * （由 '{' 開始並以對應的 '}' 結束）。讀取到的 JSON 內容會存入
 * 輸出緩衝區 out。
 *
 * @param s          資料來源，需為 Arduino Stream 物件 (ex: Serial, WiFiClient)
 * @param out        輸出緩衝區，用來存放讀取到的 JSON 字串
 * @param out_cap    輸出緩衝區容量（避免溢位）
 * @param timeout_ms 讀取的逾時時間（毫秒）
 *
 * @return 實際讀取到的字元數；若逾時或發生錯誤，回傳 0
 */
size_t read_json_object(Stream& s, char* out, size_t out_cap, uint32_t timeout_ms)
{
  if (!out || out_cap < 3) return 0;

  const uint32_t t0 = millis();
  bool   started = false;
  int    depth   = 0;
  size_t i       = 0;

  // Read until timeout or full JSON object captured
  while ((millis() - t0) < timeout_ms) {

    // No data yet: yield to background tasks and try again
    if (!s.available()) { yield(); continue; }

    int c = s.read();
    if (c < 0) continue;  // defensive guard

    if (!started) {
      // Wait for the first '{' to start capturing
      if (c == '{') {
        started = true;
        depth   = 1;
        if (i < out_cap - 1) out[i++] = '{';
      }
      // ignore bytes before '{'
    } else {
      // Track brace depth to detect the matching closing '}'
      if (c == '{')        depth++;
      else if (c == '}')   depth--;

      // Append current char if buffer has room (keep one for NUL)
      if (i < out_cap - 1) out[i++] = (char)c;

      // When depth returns to 0, full JSON object is done
      if (depth == 0) break;
    }
  }

  if (!started || depth != 0) return 0;  // timeout or incomplete
  out[i] = '\0';
  return i + 1;                           // include terminating NUL
}


// Context used by the callback to know where to store values
typedef struct {
  fog_parameter_t* fog;
  uint8_t ch;        // 1 -> paramX[], 2 -> paramY[], 3 -> paramZ[]
} fog_cb_ctx_t;

// Store one key/value into the proper channel array
static void fog_store_cb(int key, int32_t val, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog) return;
  if (key < 0 || key >= PAR_LEN) return;  // arrays are 0..PAR_LEN-1

  switch (C->ch) {
    case 3: C->fog->paramX[key].data.int_val = val; break;
    case 2: C->fog->paramY[key].data.int_val = val; break;
    case 1: C->fog->paramZ[key].data.int_val = val; break;
    default: break;
  }
}

static void imu_cali_store_cb(int key, int32_t val, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog) return;
  if (key < 0 || key >= MIS_LEN) return;  // arrays are 0..MIS_LEN-1

  C->fog->misalignment[key].data.int_val = val;
}

// ---- SN 的 callback：把 payload 字串寫入 fog->sn（長度保護 + NUL 結尾）----
static void sn_store_cb(const char* s, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog || !s) return;

  // 你的 fog_parameter_t: uint8_t sn[13];（可日後擴大）
  // 這裡以 sizeof(fog->sn) 為準，避免 Magic Number
  size_t cap = sizeof(C->fog->sn);
  size_t n = strlen(s);
  if (n >= cap) n = cap - 1;              // 保留一格給 NUL
  memcpy(C->fog->sn, s, n);
  C->fog->sn[n] = '\0';                    // 以 C-string NUL 結尾
}

void pack_sensor_payload_from_cali(const my_sensor_t* cali, uint8_t* out)
{
  if (!cali || !out) return;
  int idx = 0;

  memcpy(&out[idx], cali->fog.fogx.step.bin_val, 4); idx += 4;
  memcpy(&out[idx], cali->fog.fogy.step.bin_val, 4); idx += 4;
  memcpy(&out[idx], cali->fog.fogz.step.bin_val, 4); idx += 4;

  memcpy(&out[idx], cali->adxl357.ax.bin_val, 4);    idx += 4;
  memcpy(&out[idx], cali->adxl357.ay.bin_val, 4);    idx += 4;
  memcpy(&out[idx], cali->adxl357.az.bin_val, 4);    idx += 4;

  memcpy(&out[idx], cali->temp.tempx.bin_val, 4);    idx += 4;
  memcpy(&out[idx], cali->temp.tempy.bin_val, 4);    idx += 4;
  memcpy(&out[idx], cali->temp.tempz.bin_val, 4);    idx += 4;

  memcpy(&out[idx], cali->adxl357.temp.bin_val, 4);  idx += 4;

  memcpy(&out[idx], cali->time.time.bin_val, 4);     idx += 4;
}

void sensor_data_cali(const my_sensor_t* raw, my_sensor_t* cali, fog_parameter_t* fog_parameter)
{
  if (!raw || !cali || !fog_parameter) return;

  // === Copy raw data to cali structure ===
  // float tx   = ((float)raw->temp.tempx.int_val) * COE_TEMP_AD590 - 273.15;
  // float ty   = ((float)raw->temp.tempy.int_val) * COE_TEMP_AD590 - 273.15; 
  float tz   = ((float)raw->temp.tempz.int_val) * COE_TEMP_AD590 - 273.15; 
  float vin_mon = ((float)raw->hk.Vin_mon.int_val) * VIN_MON_COFF;
  float tacc = ((float)raw->adxl357.temp.int_val) * SF_TEMP + 25.0;

  //------ debug ADC voltage------------------

  // float temp_Volt = ((float)raw->temp.tempz.int_val) * ADC3p3_COEE;
  // float vin_Volt = ((float)raw->hk.Vin_mon.int_val) * ADC3p3_COEE;
  // float Tact_Volt = ((float)raw->hk.Tact_mon.int_val) * ADC3p3_COEE;
  // float pumpPd_Volt = ((float)raw->hk.pump_pd_mon.int_val) * ADC3p3_COEE;
  
  // Serial.print("PD_T:"); Serial.print(temp_Volt, 3);
  // Serial.print("   |Vin:"); Serial.print(vin_Volt, 3);
  // Serial.print("   |Tact:"); Serial.print(Tact_Volt, 3);
  // Serial.print("   |Pump:"); Serial.print(pumpPd_Volt, 3);
  // Serial.println();
  //-----------------------------------------------

  // === Gyro scale factor（一次線性）===
  float sf_x_gyro = SF_GYRO_1000DPS;
  float sf_y_gyro = SF_GYRO_1000DPS;

  #ifdef CASE_MEMS
    float sf_z_gyro = SF_GYRO_1000DPS;
  #else
    float sf_z_gyro = sf_temp_comp_1st(tz,
    fog_parameter->paramZ[17].data.float_val,
    fog_parameter->paramZ[18].data.float_val);
  #endif

  // float sf_x_gyro = sf_temp_comp_1st(tx,
  //     fog_parameter->paramX[17].data.float_val,
  //     fog_parameter->paramX[18].data.float_val);

  // float sf_y_gyro = sf_temp_comp_1st(ty,
  //     fog_parameter->paramY[17].data.float_val,
  //     fog_parameter->paramY[18].data.float_val);

    // Serial.println("\nsf_gyro: slope, offset:");
    // Serial.print(fog_parameter->paramX[17].data.float_val,4); Serial.print(","); 
    // Serial.print(fog_parameter->paramX[18].data.float_val,4); Serial.println();
    // Serial.print(fog_parameter->paramY[17].data.float_val,4); Serial.print(","); 
    // Serial.print(fog_parameter->paramY[18].data.float_val,4); Serial.println();
    // Serial.print(fog_parameter->paramZ[17].data.float_val,4); Serial.print(","); 
    // Serial.print(fog_parameter->paramZ[18].data.float_val,4); Serial.println();
  

  // === Accel scale factor（一次線性；用 adxl 溫度）===
  float sf_x_acc = SF_ACCL_16G;
  float sf_y_acc = SF_ACCL_16G;
  float sf_z_acc = SF_ACCL_16G;
  // float sf_x_acc = sf_temp_comp_1st(tacc,
  //     fog_parameter->paramX[31].data.float_val,
  //     fog_parameter->paramX[32].data.float_val);

  // float sf_y_acc = sf_temp_comp_1st(tacc,
  //     fog_parameter->paramY[31].data.float_val,
  //     fog_parameter->paramY[32].data.float_val);

  // float  = sf_temp_comp_1st(tacc,
  //     fog_parameter->paramZ[31].data.float_val,
  //     fog_parameter->paramZ[32].data.float_val);
      // Serial.println("\nsf_accl: slope, offset:");
      // Serial.print(fog_parameter->paramX[31].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramX[32].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramY[31].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramY[32].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramZ[31].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramZ[32].data.float_val); Serial.println();

  // === Gyro bias（三區段一次線性）===
  float bx_gyro = 0;
  float by_gyro = 0;

  #ifdef CASE_MEMS
    float bz_gyro = 0;
  #else
    float bz_gyro = bias_temp_comp_1st_3t(
      tz, fog_parameter->paramZ[23].data.float_val, fog_parameter->paramZ[24].data.float_val,
      fog_parameter->paramZ[25].data.float_val, fog_parameter->paramZ[26].data.float_val,
      fog_parameter->paramZ[27].data.float_val, fog_parameter->paramZ[28].data.float_val,
      fog_parameter->paramZ[29].data.float_val, fog_parameter->paramZ[30].data.float_val
    );
  #endif
  // float bx_gyro = bias_temp_comp_1st_3t(
  //     tx,
  //     fog_parameter->paramX[23].data.float_val,  // T1
  //     fog_parameter->paramX[24].data.float_val,  // T2
  //     fog_parameter->paramX[25].data.float_val,  // s1
  //     fog_parameter->paramX[26].data.float_val,  // o1
  //     fog_parameter->paramX[27].data.float_val,  // s2
  //     fog_parameter->paramX[28].data.float_val,  // o2
  //     fog_parameter->paramX[29].data.float_val,  // s3
  //     fog_parameter->paramX[30].data.float_val   // o3
  // );

  // float by_gyro = bias_temp_comp_1st_3t(
  //     ty, fog_parameter->paramY[23].data.float_val, fog_parameter->paramY[24].data.float_val,
  //     fog_parameter->paramY[25].data.float_val, fog_parameter->paramY[26].data.float_val,
  //     fog_parameter->paramY[27].data.float_val, fog_parameter->paramY[28].data.float_val,
  //     fog_parameter->paramY[29].data.float_val, fog_parameter->paramY[30].data.float_val
  // );

  // Serial.println("\nbs_gyro: T1, T2, s1, o1, s2, o2, s3, o3:");
  // Serial.print(fog_parameter->paramX[23].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramX[24].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramX[25].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramX[26].data.float_val); Serial.print(",");
  // Serial.print(fog_parameter->paramX[27].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramX[28].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramX[29].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramX[30].data.float_val); Serial.println();
  // Serial.print(fog_parameter->paramY[23].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramY[24].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramY[25].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramY[26].data.float_val); Serial.print(",");
  // Serial.print(fog_parameter->paramY[27].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramY[28].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramY[29].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramY[30].data.float_val); Serial.println();
  // Serial.print(fog_parameter->paramZ[23].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramZ[24].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramZ[25].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramZ[26].data.float_val); Serial.print(",");
  // Serial.print(fog_parameter->paramZ[27].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramZ[28].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramZ[29].data.float_val); Serial.print(","); 
  // Serial.print(fog_parameter->paramZ[30].data.float_val); Serial.println();

  // === Accel bias（一次線性；用 adxl 溫度）===
  float bx_acc = 0;
  float by_acc = 0;
  float bz_acc = 0;
  // float bx_acc = sf_temp_comp_1st(tacc,
  //     fog_parameter->paramX[33].data.float_val,
  //     fog_parameter->paramX[34].data.float_val);

  // float by_acc = sf_temp_comp_1st(tacc,
  //     fog_parameter->paramY[33].data.float_val,
  //     fog_parameter->paramY[34].data.float_val);

  // float  = sf_temp_comp_1st(tacc,
  //     fog_parameter->paramZ[33].data.float_val,
  //     fog_parameter->paramZ[34].data.float_val);
      // Serial.println("\nbs_accl: s, o:");
      // Serial.print(fog_parameter->paramX[33].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramX[34].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramY[33].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramY[34].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramZ[33].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramZ[34].data.float_val); Serial.println();

  // === Gyro 溫補（scale factor & bias）===
  float gx_comp = ((float)raw->fog.fogx.step.int_val) * sf_x_gyro - bx_gyro;
  float gy_comp = ((float)raw->fog.fogy.step.int_val) * sf_y_gyro - by_gyro;
  
  #ifdef CASE_MEMS
    float gz_comp = ((float)raw->m_gyro.gz.int_val) * sf_z_gyro - bz_gyro;
  #else
    float gz_comp = ((float)raw->fog.fogz.step.int_val) * sf_z_gyro - bz_gyro;
  #endif

  // Serial.print(raw->fog.fogx.step.float_val); Serial.print(","); 
  // Serial.print(raw->fog.fogy.step.float_val); Serial.print(","); 
  // Serial.print(raw->fog.fogz.step.float_val); Serial.println();Serial.println();

  // === Accel 溫補（scale factor & bias）===
  float ax_comp = ((float)raw->adxl357.ax.int_val) * sf_x_acc - bx_acc;
  float ay_comp = ((float)raw->adxl357.ay.int_val) * sf_y_acc - by_acc;
  float az_comp = ((float)raw->adxl357.az.int_val) * sf_z_acc - bz_acc;

  // === Gyro misalignment（misalignment[12..23]）===
  float cgx = fog_parameter->misalignment[12].data.float_val;
  float cgy = fog_parameter->misalignment[13].data.float_val;
  float cgz = fog_parameter->misalignment[14].data.float_val;
  float g11 = fog_parameter->misalignment[15].data.float_val;
  float g12 = fog_parameter->misalignment[16].data.float_val;
  float g13 = fog_parameter->misalignment[17].data.float_val;
  float g21 = fog_parameter->misalignment[18].data.float_val;
  float g22 = fog_parameter->misalignment[19].data.float_val;
  float g23 = fog_parameter->misalignment[20].data.float_val;
  float g31 = fog_parameter->misalignment[21].data.float_val;
  float g32 = fog_parameter->misalignment[22].data.float_val;
  float g33 = fog_parameter->misalignment[23].data.float_val;
  // Serial.println("\nMIS_gyro:");
  // Serial.print(g11); Serial.print(","); Serial.print(g12); Serial.print(","); Serial.println(g13);
  // Serial.print(g21); Serial.print(","); Serial.print(g22); Serial.print(","); Serial.println(g23);
  // Serial.print(g31); Serial.print(","); Serial.print(g32); Serial.print(","); Serial.println(g33);
 

  float gx_cal = g11*gx_comp + g12*gy_comp + g13*gz_comp + cgx;
  float gy_cal = g21*gx_comp + g22*gy_comp + g23*gz_comp + cgy;
  float gz_cal = g31*gx_comp + g32*gy_comp + g33*gz_comp + cgz;

  // === Gyro 訊號飽和 (Saturation)  ===
  float limit_x = fog_parameter->paramX[12].data.float_val;
  float limit_y = fog_parameter->paramY[12].data.float_val;
  float limit_z = fog_parameter->paramZ[12].data.float_val;

  // X 軸飽和
  if (limit_x > 0) { // 確保上限值為正
      if (gx_cal > limit_x) {
          gx_cal = limit_x;
      } else if (gx_cal < -limit_x) {
          gx_cal = -limit_x;
      }
  }

  // Y 軸飽和
  if (limit_y > 0) { // 確保上限值為正
      if (gy_cal > limit_y) {
          gy_cal = limit_y;
      } else if (gy_cal < -limit_y) {
          gy_cal = -limit_y;
      }
  }

  // Z 軸飽和
  if (limit_z > 0) { // 確保上限值為正
      if (gz_cal > limit_z) {
          gz_cal = limit_z;
      } else if (gz_cal < -limit_z) {
          gz_cal = -limit_z;
      }
  }

  // Serial.print(fog_parameter->paramX[12].data.float_val);
  // Serial.print(", ");
  // Serial.print(fog_parameter->paramY[12].data.float_val);
  // Serial.print(", ");
  // Serial.println(fog_parameter->paramZ[12].data.float_val);

  // === Accel misalignment（misalignment[0..11]）===
  float cax = fog_parameter->misalignment[0].data.float_val;
  float cay = fog_parameter->misalignment[1].data.float_val;
  float caz = fog_parameter->misalignment[2].data.float_val;
  float a11 = fog_parameter->misalignment[3].data.float_val;
  float a12 = fog_parameter->misalignment[4].data.float_val;
  float a13 = fog_parameter->misalignment[5].data.float_val;
  float a21 = fog_parameter->misalignment[6].data.float_val;
  float a22 = fog_parameter->misalignment[7].data.float_val;
  float a23 = fog_parameter->misalignment[8].data.float_val;
  float a31 = fog_parameter->misalignment[9].data.float_val;
  float a32 = fog_parameter->misalignment[10].data.float_val;
  float a33 = fog_parameter->misalignment[11].data.float_val;
  // Serial.println("\nMIS_accl:");
  // Serial.print(a11); Serial.print(","); Serial.print(a12); Serial.print(","); Serial.println(a13);
  // Serial.print(a21); Serial.print(","); Serial.print(a22); Serial.print(","); Serial.println(a23);
  // Serial.print(a31); Serial.print(","); Serial.print(a32); Serial.print(","); Serial.println(a33);

  float ax_cal = a11*ax_comp + a12*ay_comp + a13*az_comp + cax;
  float ay_cal = a21*ax_comp + a22*ay_comp + a23*az_comp + cay;
  float az_cal = a31*ax_comp + a32*ay_comp + a33*az_comp + caz;

  // === 輸出到 cali 結構 ===
  cali->fog.fogx.step.float_val = gx_cal;
  cali->fog.fogy.step.float_val = gy_cal;
  cali->fog.fogz.step.float_val = gz_cal;

  cali->adxl357.ax.float_val = ax_cal;
  cali->adxl357.ay.float_val = ay_cal;
  cali->adxl357.az.float_val = az_cal;

  // 溫度 / 時間直接沿用 raw
  cali->temp.tempx.float_val = tz;
  cali->temp.tempy.float_val = tz;
  cali->temp.tempz.float_val = tz;
  cali->adxl357.temp.float_val = tacc;
  // Serial.println(cali->temp.tempx.float_val);
  
  // time
  float time = (float)(raw->time.time.int_val) * COE_TIMER;
  cali->time.time.float_val = time;
}

#include "usecase/parameter_service.h"
#include "app/app_state.h"



