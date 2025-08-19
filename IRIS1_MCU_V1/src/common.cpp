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


// -----------------------------------------------------------------------------
// Forward declare Serial4 (constructed in myUART.cpp)
class Uart;
extern Uart Serial4;
// -----------------------------------------------------------------------------

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

// Convert int32_t to mem_unit_t as an integer payload
static inline mem_unit_t mem_from_i32(int32_t v) {
  mem_unit_t m;
  m.type = TYPE_INT;
  m.data.int_val = v;
  return m;
}

// Read one JSON object from a Stream: from first '{' to its matching '}'.
// Returns total bytes written to 'out' including the final NUL, or 0 on timeout/invalid.
static size_t read_json_object(Stream& s, char* out, size_t out_cap, uint32_t timeout_ms)
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


// Parse simple JSON object like: {"0":169,"1":8192,...}
// Keys are quoted numeric strings; values are signed integers.
// For each pair, call cb(key, value, ctx).
static void parse_simple_json_ints(const char* js, void (*cb)(int /*key*/, int32_t /*val*/, void* /*ctx*/), void* ctx)
{
  if (!js || !cb) return;

  const char* p = js;
  while (*p) {
    // Find starting quote of the next key
    while (*p && *p != '"') p++;
    if (*p != '"') break;  // no more key
    p++;

    // Read numeric key
    long key = 0;
    bool any_digit = false;
    while (*p && isdigit((unsigned char)*p)) {
      any_digit = true;
      key = key * 10 + (*p - '0');
      p++;
    }
    if (!any_digit) break;

    // Skip to closing quote of key
    while (*p && *p != '"') p++;
    if (*p != '"') break;
    p++; // skip closing "

    // Skip spaces and find ':'
    while (*p && isspace((unsigned char)*p)) p++;
    if (*p != ':') { while (*p && *p != ':') p++; if (*p != ':') break; }
    p++; // skip ':'
    while (*p && isspace((unsigned char)*p)) p++;

    // Read signed integer value
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

    // Dispatch
    cb((int)key, (int32_t)val, ctx);

    // Move to next pair (skip until ',' or end '}')
    while (*p && *p != ',' && *p != '}') p++;
    if (*p == ',') { p++; continue; }
    else if (*p == '}') break;
  }
}

// Context used by the callback to know where to store values
typedef struct {
  fog_parameter_t* fog;
  uint8_t ch;        // 1 -> paramX[], 2 -> paramY[], 3 -> paramZ[]
} fog_cb_ctx_t;


// Store one key/value into the proper channel array (as TYPE_INT.int_val)
static void fog_store_cb(int key, int32_t val, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog) return;
  if (key < 0 || key >= PAR_LEN) return;  // arrays are 0..PAR_LEN-1

  mem_unit_t m = mem_from_i32(val);

  switch (C->ch) {
    case 1: C->fog->paramX[key] = m; break;
    case 2: C->fog->paramY[key] = m; break;
    case 3: C->fog->paramZ[key] = m; break;
    default: break;
  }
}


// Main entry: send command, receive JSON, parse into fog_inst, then forward JSON via Serial1
void dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch)
{
  if (!fog_inst) return;

  // 1) Send command to FPGA over Serial4
  static const uint8_t HDR[2] = {0xAB, 0xBA};
  static const uint8_t TRL[2] = {0x55, 0x56};
  sendCmd(Serial4, HDR, TRL, CMD_DUMP_FOG, 2, ch);

  // 2) Receive a full JSON object from Serial4
  char json_buf[FOG_JSON_BUF_SIZE];
  size_t n = read_json_object(Serial4, json_buf, sizeof(json_buf), FOG_JSON_TIMEOUT_MS);
  if (n == 0) {
    // Optional debug
    Serial.println(F("dump_fog_param: timeout or malformed JSON from Serial4"));
    return;
  }

  // 3) Parse and store into fog_inst->paramX/Y/Z by channel
  fog_cb_ctx_t ctx = { fog_inst, ch };
  parse_simple_json_ints(json_buf, fog_store_cb, &ctx);

  // 4) Forward the raw JSON to PC via Serial1 (TX)
  Serial1.write((const uint8_t*)json_buf, strlen(json_buf));
  Serial1.write('\n');  // optional newline for readability
}

