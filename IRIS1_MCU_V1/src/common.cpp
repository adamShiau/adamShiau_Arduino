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
// #include <stdio.h>    // vsnprintf

/* ===================== serial_printf implementation ===================== */

#ifndef SERIAL_PRINTF_STACKBUF
#define SERIAL_PRINTF_STACKBUF 128   // Small messages use stack buffer
#endif

#ifndef SERIAL_PRINTF_ALLOW_HEAP
#define SERIAL_PRINTF_ALLOW_HEAP 1   // Large messages use heap if needed
#endif

/* Default output: USB Serial. You can change with serial_set_stream(). */
static Print* g_serial_out = &Serial;

void serial_set_stream(Print* s)
{
  if (s) g_serial_out = s;
}

int serial_vprintf(const char* fmt, va_list ap)
{
  if (!g_serial_out || !fmt) return 0;

  // First pass: compute required length (copy va_list)
  va_list aq;
  va_copy(aq, ap);
  int needed = vsnprintf(nullptr, 0, fmt, aq);
  va_end(aq);
  if (needed < 0) return needed;

  // Small message: stack buffer
  if (needed < (int)SERIAL_PRINTF_STACKBUF) {
    char buf[SERIAL_PRINTF_STACKBUF];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n > 0) g_serial_out->write((const uint8_t*)buf, (size_t)n);
    return n;
  }

  // Large message: heap (optional), or truncated
#if SERIAL_PRINTF_ALLOW_HEAP
  char* big = (char*)malloc((size_t)needed + 1);
  if (!big) {
    // Fallback: truncated
    char buf[SERIAL_PRINTF_STACKBUF];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n > 0) g_serial_out->write((const uint8_t*)buf, (size_t)min(n, (int)sizeof(buf)));
    return n;
  }
  int n = vsnprintf(big, (size_t)needed + 1, fmt, ap);
  if (n > 0) g_serial_out->write((const uint8_t*)big, (size_t)n);
  free(big);
  return n;
#else
  char buf[SERIAL_PRINTF_STACKBUF];
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);  // truncated if too long
  if (n > 0) g_serial_out->write((const uint8_t*)buf, (size_t)min(n, (int)sizeof(buf)));
  return n;
#endif
}

int serial_printf(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int n = serial_vprintf(fmt, ap);
  va_end(ap);
  return n;
}

/* ===================== End of serial_printf implementation ===================== */

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

size_t sendCmd(Print& port, const uint8_t header[2], const uint8_t trailer[2], uint8_t cmd, 
  int32_t value, uint8_t ch)
{
  size_t n = 0;

  // header (2 bytes)
  n += port.write(header, 2);

  // cmd (1 byte)
  n += port.write(&cmd, 1);

  // value (4 bytes, big-endian)
  uint32_t v = (uint32_t)value;  // cast to avoid sign-propagation on shifts
  uint8_t b[4] = {
    (uint8_t)((v >> 24) & 0xFF),
    (uint8_t)((v >> 16) & 0xFF),
    (uint8_t)((v >>  8) & 0xFF),
    (uint8_t)((v >>  0) & 0xFF)
  };
  n += port.write(b, 4);

  // ch (1 byte)
  n += port.write(&ch, 1);

  // trailer (2 bytes)
  n += port.write(trailer, 2);

  return n;  // expected 10
}

void fog_parameter(cmd_ctrl_t* rx, fog_parameter_t* fog_inst)
{

	if(rx->mux == MUX_PARAMETER){
        DEBUG_PRINT("fog_parameter\n");
        rx->mux = MUX_ESCAPE;

			if(rx->condition == RX_CONDITION_ABBA_5556 || rx->condition == RX_CONDITION_EFFE_5354) {
				switch(rx->cmd ){
					
					case CMD_DUMP_FOG: {
						DEBUG_PRINT("CMD_DUMP_FOG:\n");
						dump_fog_param(fog_inst, rx->ch);
						break;
					} 
					case CMD_DUMP_MIS: {
						DEBUG_PRINT("CMD_DUMP_MIS:\n");
						// dump_misalignment_param(fog_inst);
						break;
					}
					default:{
						DEBUG_PRINT("condition 1 default case\n");
					} 
				}
		
			}
			else if(rx->condition == RX_CONDITION_CDDC_5758) {
				switch(rx->cmd ){
					// case CMD_WRITE_SN: {
					// 	DEBUG_PRINT("CMD_WRITE_SN:\n");
					// 	alt_32 SN1, SN2, SN3;
					// 	SN1 =  rx->SN[0]<<24 | rx->SN[1]<<16 | rx->SN[2]<<8 | rx->SN[3];
					// 	SN2 =  rx->SN[4]<<24 | rx->SN[5]<<16 | rx->SN[6]<<8 | rx->SN[7];
					// 	SN3 =  rx->SN[8]<<24 | rx->SN[9]<<16 | rx->SN[10]<<8 | rx->SN[11];
					// 	PARAMETER_Write_f(MEM_BASE_SN, 0, SN1);
					// 	PARAMETER_Write_f(MEM_BASE_SN, 1, SN2);
					// 	PARAMETER_Write_f(MEM_BASE_SN, 2, SN3);
					// 	for (alt_u8 i = 0; i < 13; i++) {
					// 		fog_inst->sn[i] = rx->SN[i];
					// 	}
					// 	break;
					// }
					// default:{
					// 	DEBUG_PRINT("condition 2 default case\n");
					// } 
				}
			}
			
		}

}

#ifndef FOG_JSON_TIMEOUT_MS
#define FOG_JSON_TIMEOUT_MS 1500  // 等待 JSON 完整到達的逾時（毫秒）
#endif

#ifndef FOG_JSON_BUF_SIZE
#define FOG_JSON_BUF_SIZE 1200    // JSON 暫存緩衝大小（依實際內容調整）
#endif

/* 從 Stream 讀一個 JSON 物件：從第一個 '{' 開始，直到對應的 '}'。 */
static size_t read_json_object(Stream& s, char* out, size_t out_cap, uint32_t timeout_ms)
{
  if (!out || out_cap < 3) return 0;

  uint32_t t0 = millis();
  bool started = false;
  int depth = 0;
  size_t i = 0;

  // Read characters from Stream `s` until a full JSON object is captured
  // (from the first '{' to its matching '}' ) or until `timeout_ms` expires.
  while ((millis() - t0) < timeout_ms) {

    // No data available yet: yield to background tasks and try again.
    if (!s.available()) {
      yield();
      continue;
    }

    // Read one byte; if read failed (shouldn't happen after available()), skip.
    int c = s.read();
    if (c < 0) continue;

    if (!started) {
      
      if (c == '{') {
        started = true;                 // Found the first '{' — begin capturing.
        depth = 1;                      // We are now inside one level of braces.
        if (i < out_cap - 1) out[i++] = '{'; // Store that opening brace.
      }
      // Ignore any bytes that appear before the first '{'.
    }
    else {
      // We are capturing: track brace depth to find the matching closing '}'.
      if (c == '{')       depth++;      // Nested '{' → go one level deeper.
      else if (c == '}')  depth--;      // '}' → close one nesting level.

      // Append the current character (including the final '}') if buffer has room.
      if (i < out_cap - 1) out[i++] = (char)c;

      // When depth returns to 0, we've closed the original '{' → complete object.
      if (depth == 0) break;
    }
  }

  if (!started || depth != 0) return 0; // timeout 或不完整
  out[i] = '\0';
  return i + 1; // 含 NUL
}

/* 解析形如：{"0":123, "1":-456, ...} 的簡單 JSON（key 是字串數字、value 是整數） */
typedef void (*kv_cb_t)(int key, int32_t val, void* ctx);

static void parse_simple_json_ints(const char* js, kv_cb_t cb, void* ctx)
{
  if (!js || !cb) return;

  const char* p = js;
  while (*p) {
    // 找 key 的起始引號
    while (*p && *p != '"') p++;
    if (*p != '"') break;
    p++;

    // 讀 key（數字字串）
    long key = 0;
    bool any_digit = false;
    while (*p && isdigit((unsigned char)*p)) {
      any_digit = true;
      key = key * 10 + (*p - '0');
      p++;
    }
    if (!any_digit) break;

    // 找 key 結束引號
    while (*p && *p != '"') p++;
    if (*p != '"') break;
    p++; // skip closing "

    // 跳過空白與冒號
    while (*p && isspace((unsigned char)*p)) p++;
    if (*p != ':') { while (*p && *p != ':') p++; if (*p != ':') break; }
    p++; // skip ':'
    while (*p && isspace((unsigned char)*p)) p++;

    // 讀 value（有號整數）
    bool neg = false;
    if (*p == '-') { neg = true; p++; }
    long val = 0;
    bool any_val_digit = false;
    while (*p && isdigit((unsigned char)*p)) {
      any_val_digit = true;
      val = val * 10 + (*p - '0');
      p++;
    }
    if (!any_val_digit) break;
    if (neg) val = -val;

    // 回呼
    cb((int)key, (int32_t)val, ctx);

    // 移到下一對鍵值（逗點或結束）
    while (*p && *p != ',' && *p != '}') p++;
    if (*p == ',') { p++; continue; }
    else if (*p == '}') break;
  }
}

/* 寫入 fog_inst->paramX/Y/Z 的 callback */
typedef struct {
  fog_parameter_t* fog;
  uint8_t ch;
} fog_cb_ctx_t;

static void fog_store_cb(int key, int32_t val, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog) return;
  if (key < 0 || key > PAR_LEN) return;

  // 假設 paramX/Y/Z 的元素型別是整數（例如 int32_t 或 mem_unit_t）
  switch (C->ch) {
    case 1: C->fog->paramX[key] = (int32_t)val; break;
    case 2: C->fog->paramY[key] = (int32_t)val; break;
    case 3: C->fog->paramZ[key] = (int32_t)val; break;
    default: break;
  }
}

/* 主流程：送指令 -> 收 JSON -> 解析存入 -> 透過 Serial1 原樣回送 JSON */
void dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch)
{
  if (!fog_inst) return;

  // 1) 送 command
  static const uint8_t HDR[2] = {0xAB, 0xBA};
  static const uint8_t TRL[2] = {0x55, 0x56};
  sendCmd(Serial4, HDR, TRL, CMD_DUMP_FOG, 2, ch);

  // 2) 從 Serial4 收 JSON（完整一個物件）
  char json_buf[FOG_JSON_BUF_SIZE];
  size_t n = read_json_object(Serial4, json_buf, sizeof(json_buf), FOG_JSON_TIMEOUT_MS);
  if (n == 0) {
    serial_printf("dump_fog_param: timeout/bad JSON from Serial4\n");
    return;
  }

  // 3) 解析並存入 fog_inst
  fog_cb_ctx_t ctx = { fog_inst, ch };
  parse_simple_json_ints(json_buf, fog_store_cb, &ctx);

  // 4) 把這段 JSON 透過 Serial1 送出（原樣轉送）
  Serial1.write((const uint8_t*)json_buf, strlen(json_buf));
  Serial1.write('\n'); // 可選：加換行方便 PC 端閱讀
}

