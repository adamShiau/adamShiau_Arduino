#include "dump_service.h"
#include <ctype.h>
#include <string.h>

// #include "../app/app_state.h"
#include "../domain/model/command_id.h"
#include "../drivers/link/nios_link.h"
#include "../domain/protocol/ack_codec_v1.h"
#include "../utils/version_info.h"

// 你 common.h 裡的 header/trailer 全域常數（目前是 extern）
// #include "../common.h"


// These are defined in common.cpp and declared as extern in common.h originally.
// To avoid including common.h (and circular deps), we redeclare externs here.
extern const uint8_t HDR_ABBA[2];
extern const uint8_t TRL_5556[2];

// Ports are defined in app_state.cpp; declare here to avoid including app_state.h
extern Stream& g_cmd_port_fpga;
extern Stream& g_cmd_port_output;

/* ---------- Dump interface ---------- */

// =====================  底層（Low-level）  =====================

/**
 * @brief 計算 CRC16-CCITT 校驗碼 (poly=0x1021, init=0xFFFF)
 */
static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}
static uint16_t crc16_ccitt_buf(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) crc = crc16_ccitt_update(crc, data[i]);
  return crc;
}

static size_t readline_with_timeout(Stream& port, char* line, size_t line_cap, uint32_t timeout_ms)
{
  if (line_cap == 0) return 0;
  size_t n = 0;
  uint32_t t0 = millis();

  while ((millis() - t0) < timeout_ms)
  {
    while (port.available())
    {
      int c = port.read();
      if (c < 0) break;

      if (c == '\n') {
        while (n > 0 && line[n-1] == '\r') n--;
        line[n] = '\0';
        return n;
      }

      if (n + 1 < line_cap) {
        line[n++] = (char)c;
      } else {
        line[line_cap - 1] = '\0';
        return n;
      }
    }
    delay(1);
  }
  return 0;
}

// ---- framing 封包讀取的結果 ----
enum DumpReadResult : uint8_t {
  DUMP_OK = 0,
  DUMP_TIMEOUT,
  DUMP_FORMAT_ERR,
  DUMP_LEN_MISMATCH,
  DUMP_CRC_FAIL,
  DUMP_OVERFLOW
};

static DumpReadResult read_dump_packet(Stream& port,
                                       uint32_t timeout_ms,
                                       uint32_t* out_seq,
                                       uint8_t*  out_ch,
                                       char*     payload_buf,
                                       size_t    payload_cap,
                                       size_t*   out_payload_len)
{
  if (!payload_buf || payload_cap < 2) return DUMP_OVERFLOW;
  if (out_payload_len) *out_payload_len = 0;
  if (out_seq) *out_seq = 0;
  if (out_ch)  *out_ch  = 0;

  char line[1024];
  size_t ln = readline_with_timeout(port, line, sizeof(line), timeout_ms);
  if (ln == 0) return DUMP_TIMEOUT;

  char* p = line;
  while (*p && isspace((unsigned char)*p)) ++p;
  if (*p != '@') return DUMP_FORMAT_ERR;
  ++p;

  char* c1 = strchr(p, ',');      if (!c1) return DUMP_FORMAT_ERR;
  *c1 = '\0';
  char* c2 = strchr(c1 + 1, ','); if (!c2) return DUMP_FORMAT_ERR;
  *c2 = '\0';
  char* c3 = strchr(c2 + 1, ','); if (!c3) return DUMP_FORMAT_ERR;
  *c3 = '\0';

  char* endptr = nullptr;
  uint32_t seq = strtoul(p, &endptr, 10);
  if (endptr == p) return DUMP_FORMAT_ERR;

  unsigned long ch_ul = strtoul(c1 + 1, &endptr, 10);
  if (endptr == (c1 + 1) || ch_ul > 255UL) return DUMP_FORMAT_ERR;
  uint8_t ch = (uint8_t)ch_ul;

  uint32_t len = strtoul(c2 + 1, &endptr, 10);
  if (endptr == (c2 + 1)) return DUMP_FORMAT_ERR;

  char* star = strrchr(c3 + 1, '*');
  if (!star) return DUMP_FORMAT_ERR;
  *star = '\0';

  const char* payload = c3 + 1;
  size_t payload_len  = strlen(payload);

  if (payload_len != len) return DUMP_LEN_MISMATCH;

  if (strlen(star + 1) < 4) return DUMP_FORMAT_ERR;
  char crc_hex[5] = {0};
  crc_hex[0] = star[1];
  crc_hex[1] = star[2];
  crc_hex[2] = star[3];
  crc_hex[3] = star[4];
  uint16_t rx_crc = (uint16_t)strtoul(crc_hex, nullptr, 16);

  uint16_t calc = crc16_ccitt_buf((const uint8_t*)payload, payload_len);
  if (calc != rx_crc) return DUMP_CRC_FAIL;

  if (payload_len + 1 > payload_cap) return DUMP_OVERFLOW;
  memcpy(payload_buf, payload, payload_len);
  payload_buf[payload_len] = '\0';

  if (out_seq) *out_seq = seq;
  if (out_ch)  *out_ch  = ch;
  if (out_payload_len) *out_payload_len = payload_len;

  return DUMP_OK;
}

/**
 * @brief 以 AB BA ... 55 56 框架送指令；val[31:1]=seq, val[0]=NACK flag。
 */
static inline void send_cmd_seq(uint8_t cmd, uint8_t ch, uint32_t seq, bool nack_flag=false) {
  uint32_t val = ((seq & 0x7FFFFFFFu) << 1) | (nack_flag ? 1u : 0u);
  sendCmd(g_cmd_port_fpga, HDR_ABBA, TRL_5556, cmd, (int32_t)val, ch);
}

// =====================  解析（Parse helpers）  =====================

typedef void (*kv_cb_t)(int key, int32_t val, void* ctx);

static void parse_simple_json_ints(const char* js, kv_cb_t cb, void* ctx)
{
  if (!js || !cb) return;

  const char* p = js;
  while (*p) {
    while (*p && *p != '"') p++;
    if (*p != '"') break;
    p++;

    int key = 0;
    while (*p >= '0' && *p <= '9') {
      key = key * 10 + (*p - '0');
      p++;
    }

    while (*p && *p != '"') p++;
    if (*p == '"') p++;

    while (*p && *p != ':') p++;
    if (*p == ':') p++;

    int sign = 1;
    if (*p == '-') { sign = -1; p++; }

    int32_t val = 0;
    while (*p >= '0' && *p <= '9') {
      val = val * 10 + (*p - '0');
      p++;
    }
    val *= sign;

    cb(key, val, ctx);

    while (*p && *p != ',' && *p != '}') p++;
    if (*p == ',') p++;
    else if (*p == '}') break;
  }
}

static void parse_string(const char* payload, void (*cb)(const char* s, void*), void* ctx)
{
  if (!payload || !cb) return;
  cb(payload, ctx);
}

// Context used by the callback to know where to store values
typedef struct {
  fog_parameter_t* fog;
  uint8_t ch;        // 1/2/3 -> param arrays; 4 -> misalignment; 5 -> SN
} fog_cb_ctx_t;

static void fog_store_cb(int key, int32_t val, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog) return;
  if (key < 0 || key >= PAR_LEN) return;

  // NOTE: keep behavior identical to your current common.cpp mapping
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
  if (key < 0 || key >= MIS_LEN) return;

  C->fog->misalignment[key].data.int_val = val;
}

static void config_store_cb(int key, int32_t val, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog) return;
  if (key < 0 || key >= CFG_LEN) return;

  C->fog->config[key].type = TYPE_INT;
  C->fog->config[key].data.int_val = val;
}


static void sn_store_cb(const char* s, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog || !s) return;

  size_t cap = sizeof(C->fog->sn);
  size_t n = strlen(s);
  if (n >= cap) n = cap - 1;
  memcpy(C->fog->sn, s, n);
  C->fog->sn[n] = '\0';
}

// =====================  中層（Mid-level）  =====================

static uint32_t g_seq = 0;
static const uint32_t FOG_TIMEOUT_MS = 1500;
static const size_t   SCRATCH_MAX    = 1024;

static bool recv_and_store(fog_parameter_t* fog,
                           uint8_t expect_ch,
                           uint32_t /*expect_seq*/,
                           uint32_t timeout_ms,
                           char* scratch, size_t scratch_cap)
{
  uint32_t seq=0; uint8_t ch=0; size_t plen=0;
  (void)plen;

  DumpReadResult r = read_dump_packet(g_cmd_port_fpga, timeout_ms, &seq, &ch,
                                      scratch, scratch_cap, &plen);
  if (r != DUMP_OK) return false;

  if (ch != expect_ch) return false;

  if (expect_ch == 1 || expect_ch == 2 || expect_ch == 3) {
    fog_cb_ctx_t ctx{ fog, expect_ch };
    parse_simple_json_ints(scratch, fog_store_cb, &ctx);

  } else if (expect_ch == 4) {
    fog_cb_ctx_t ctx{ fog, 4 };
    parse_simple_json_ints(scratch, imu_cali_store_cb, &ctx);

  } else if (expect_ch == 5) {
    fog_cb_ctx_t ctx{ fog, 5 };
    parse_string(scratch, sn_store_cb, &ctx);

  } else if (expect_ch == 6) {
    fog_cb_ctx_t ctx{ fog, 6 };
    parse_simple_json_ints(scratch, config_store_cb, &ctx);

  } else if (expect_ch == 7) {
    // VERSION: merge MCU_VERSION + FPGA version string
    char merged[SCRATCH_MAX + 64];
    const char* fpga_str = scratch;

    if (fpga_str && fpga_str[0] != '\0') {
      snprintf(merged, sizeof(merged), "%s,%s", MCU_VERSION, fpga_str);
    } else {
      snprintf(merged, sizeof(merged), "%s", MCU_VERSION);
    }

    strncpy(scratch, merged, scratch_cap - 1);
    scratch[scratch_cap - 1] = '\0';
  }

  const uint8_t cmd_id =
      (expect_ch == 4) ? CMD_DUMP_MIS :
      (expect_ch == 5) ? CMD_DUMP_SN  :
      (expect_ch == 6) ? CMD_DUMP_CONFIG :
      (expect_ch == 7) ? CMD_DUMP_VERSION :
                         CMD_DUMP_FOG;

  const uint16_t out_len = (uint16_t)strlen(scratch);
  send_result_v1(g_cmd_port_output, cmd_id, AckStatus::OK,
                 (const uint8_t*)scratch, out_len);

  return true;
}

static bool request_and_update(fog_parameter_t* fog,
                               uint8_t ch,
                               uint32_t timeout_ms,
                               int max_retry)
{
  if (!fog) return false;

  static char scratch[SCRATCH_MAX];

  uint8_t req_cmd =
    (ch == 4) ? CMD_DUMP_MIS :
    (ch == 5) ? CMD_DUMP_SN  :
    (ch == 6) ? CMD_DUMP_CONFIG :
    (ch == 7) ? CMD_DUMP_VERSION :
                CMD_DUMP_FOG;

  for (int attempt = 0; attempt < max_retry; ++attempt) {
    send_cmd_seq(req_cmd, ch, g_seq, false);
    if (recv_and_store(fog, ch, g_seq, timeout_ms, scratch, sizeof(scratch))) {
      g_seq = (g_seq + 1) & 0x7FFFFFFF;
      return true;
    }

    send_cmd_seq(req_cmd, ch, g_seq, true);
    if (recv_and_store(fog, ch, g_seq, timeout_ms, scratch, sizeof(scratch))) {
      g_seq = (g_seq + 1) & 0x7FFFFFFF;
      return true;
    }

    delay(20 + attempt * 10);
  }

  const uint8_t cmd_id =
    (ch == 4) ? CMD_DUMP_MIS :
    (ch == 5) ? CMD_DUMP_SN  :
    (ch == 6) ? CMD_DUMP_CONFIG :
    (ch == 7) ? CMD_DUMP_VERSION :
                CMD_DUMP_FOG;

  send_result_v1(g_cmd_port_output, cmd_id, AckStatus::TIMEOUT, nullptr, 0);
  return false;
}

// =====================  高層（High-level APIs）  =====================

bool dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch) {
  if (!fog_inst) return false;
  if (ch < 1 || ch > 3) return false;
  return request_and_update(fog_inst, ch, FOG_TIMEOUT_MS, 5);
}

bool dump_misalignment_param(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;
  return request_and_update(fog_inst, 4, FOG_TIMEOUT_MS, 5);
}

bool dump_SN(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;
  return request_and_update(fog_inst, 5, FOG_TIMEOUT_MS, 5);
}

bool dump_config(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;
  return request_and_update(fog_inst, 6, FOG_TIMEOUT_MS, 5);
}

bool dump_version(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;
  return request_and_update(fog_inst, 7, FOG_TIMEOUT_MS, 5);
}

bool boot_capture_all(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;

  bool ok = true;
  ok &= dump_fog_param(fog_inst, 1);        delay(10);
  ok &= dump_fog_param(fog_inst, 2);        delay(10);
  ok &= dump_fog_param(fog_inst, 3);        delay(10);
  ok &= dump_misalignment_param(fog_inst);  delay(10);
  ok &= dump_config(fog_inst);              delay(10);
  ok &= dump_SN(fog_inst);                  delay(10);
  return ok;
}