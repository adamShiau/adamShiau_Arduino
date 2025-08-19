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

void dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch) {
    if (!fog_inst || ch < 1 || ch > 3) return; // Ensure the pointer is valid and ch is within range
    
    mem_unit_t* param;
    switch (ch) {
        case 1: param = fog_inst->paramX; break;
        case 2: param = fog_inst->paramY; break;
        case 3: param = fog_inst->paramZ; break;
        default: return; 
    }
    
    char buffer[1024]; // Assume maximum output length
    int offset = 0;
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "{");
    
    for (int i = 0; i < PAR_LEN; i++) {
		offset += snprintf(buffer + offset, sizeof(buffer) - offset, "\"%d\":%ld", i, param[i].data.int_val);
        if (i < PAR_LEN - 1) offset += snprintf(buffer + offset, sizeof(buffer) - offset, ", "); // Add comma if not the last element
    }
    
    snprintf(buffer + offset, sizeof(buffer) - offset, "}\n"); // Close JSON structure
    DEBUG_PRINT("%s", buffer); // Print the formatted JSON string
	// uart_printf("%s", buffer);
	send_json_uart(buffer); // Send the JSON data via UART
}

// void send_json_uart(const char* buffer) {
//     while (*buffer) {
//         // checkByte((alt_u8)*buffer);
// 		checkByte_dbg((alt_u8)*buffer);
//         buffer++;
//     }
// }
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
						dump_misalignment_param(fog_inst);
						break;
					}
					default:{
						DEBUG_PRINT("condition 1 default case\n");
					} 
				}
		
			}
			else if(rx->condition == 2) {
				switch(rx->cmd ){
					case CMD_WRITE_SN: {
						DEBUG_PRINT("CMD_WRITE_SN:\n");
						alt_32 SN1, SN2, SN3;
						SN1 =  rx->SN[0]<<24 | rx->SN[1]<<16 | rx->SN[2]<<8 | rx->SN[3];
						SN2 =  rx->SN[4]<<24 | rx->SN[5]<<16 | rx->SN[6]<<8 | rx->SN[7];
						SN3 =  rx->SN[8]<<24 | rx->SN[9]<<16 | rx->SN[10]<<8 | rx->SN[11];
						PARAMETER_Write_f(MEM_BASE_SN, 0, SN1);
						PARAMETER_Write_f(MEM_BASE_SN, 1, SN2);
						PARAMETER_Write_f(MEM_BASE_SN, 2, SN3);
						for (alt_u8 i = 0; i < 13; i++) {
							fog_inst->sn[i] = rx->SN[i];
						}
						break;
					}
					default:{
						DEBUG_PRINT("condition 2 default case\n");
					} 
				}
			}
			
		}

}

