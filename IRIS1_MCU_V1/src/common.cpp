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

const uint8_t HDR_ABBA[2] = {0xAB, 0xBA};
const uint8_t TRL_5556[2] = {0x55, 0x56};
const uint8_t HDR_OUT[4] = {0xFE, 0x81, 0xFF, 0x55};


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
    memcpy(out->fog.fogx.step.bin_val,  &pkt[idx], 4); idx += 4;
    memcpy(out->fog.fogy.step.bin_val,  &pkt[idx], 4); idx += 4;
    memcpy(out->fog.fogz.step.bin_val,  &pkt[idx], 4); idx += 4;

    // adxl357 {ax, ay, az}
    memcpy(out->adxl357.ax.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->adxl357.ay.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->adxl357.az.bin_val,     &pkt[idx], 4); idx += 4;

    // temp {x, y, z}
    memcpy(out->temp.tempx.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->temp.tempy.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->temp.tempz.bin_val,     &pkt[idx], 4); idx += 4;

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
    case 1: // paramX
      if (idx < PAR_LEN) {
        fog_inst->paramX[idx].data.int_val = rx->value;
      }
      break;

    case 2: // paramY
      if (idx < PAR_LEN) {
        fog_inst->paramY[idx].data.int_val = rx->value;
      }
      break;

    case 3: // paramZ
      if (idx < PAR_LEN) {
        fog_inst->paramZ[idx].data.int_val = rx->value;
      }
      break;

    case 4: // misalignment
      if (idx < MIS_LEN) {
        fog_inst->misalignment[idx].data.int_val = rx->value;
      }
      break;

    default:
      // unknown channel -> ignore
      break;
  }
}

/**
 * @brief 傳送一個帶有標頭、指令、數值與標尾的封包到指定輸出埠
 *
 * 將輸入參數組裝成一個完整的通訊封包，並透過繼承自 Arduino Print
 * 類別的物件（如 Serial, Serial1, EthernetClient 等）送出。
 *
 * 封包格式一般為：
 *   [header(2 bytes)] [cmd(1 byte)] [ch(1 byte)] [value(4 bytes)] [trailer(2 bytes)]
 *
 * @param port    目標輸出埠，需為 Print 介面的物件 (ex: Serial)
 * @param header  封包的標頭 (2 bytes)，用來識別封包開始
 * @param trailer 封包的標尾 (2 bytes)，用來識別封包結束
 * @param cmd     指令代碼 (1 byte)，表明要執行的命令
 * @param value   指令參數或資料 (4 bytes, int32_t)
 * @param ch      通道或子代碼 (1 byte)，用來區分不同通道或資料類型
 *
 * @return 實際送出的位元組數
 */
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
        DEBUG_PRINT("fog_parameter mode\n");
        rx->mux = MUX_ESCAPE;

			if(rx->condition == RX_CONDITION_ABBA_5556 || rx->condition == RX_CONDITION_EFFE_5354) {
		
        switch(rx->cmd ){
					case CMD_MOD_FREQ: {
						DEBUG_PRINT("CMD_MOD_FREQ:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MOD_FREQ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MOD_FREQ - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_MOD_AMP_H: {
						DEBUG_PRINT("CMD_MOD_AMP_H:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MOD_AMP_H, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MOD_AMP_H - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_MOD_AMP_L: {
						DEBUG_PRINT("CMD_MOD_AMP_L:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MOD_AMP_L, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MOD_AMP_L - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_POLARITY: {
						DEBUG_PRINT("CMD_POLARITY:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_POLARITY, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_POLARITY - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_WAIT_CNT: {
						DEBUG_PRINT("CMD_WAIT_CNT:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_WAIT_CNT, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_WAIT_CNT - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_ERR_AVG: {
						DEBUG_PRINT("CMD_ERR_AVG:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_ERR_AVG, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_ERR_AVG - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_GAIN1: {
						DEBUG_PRINT("CMD_GAIN1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_GAIN1, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_GAIN1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_CONST_STEP: {
						DEBUG_PRINT("CMD_CONST_STEP:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_CONST_STEP, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_CONST_STEP - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_FB_ON: {
						DEBUG_PRINT("CMD_FB_ON:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_FB_ON, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_FB_ON - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_GAIN2: {
						DEBUG_PRINT("CMD_GAIN2:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_GAIN2, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_GAIN2 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_ERR_OFFSET: {
						DEBUG_PRINT("CMD_ERR_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_ERR_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_ERR_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					} 
					case CMD_DAC_GAIN: {
						DEBUG_PRINT("CMD_DAC_GAIN:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_DAC_GAIN, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_DAC_GAIN - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_CUT_OFF: {
						DEBUG_PRINT("CMD_CUT_OFF:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_CUT_OFF, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_CUT_OFF - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_COMP_T1: {
						DEBUG_PRINT("CMD_SF_COMP_T1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_COMP_T1, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_COMP_T1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_COMP_T2: {
						DEBUG_PRINT("CMD_SF_COMP_T2:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_COMP_T2, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_COMP_T2 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_1_SLOPE: {
						DEBUG_PRINT("CMD_SF_1_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_1_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_1_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_1_OFFSET: {
						DEBUG_PRINT("CMD_SF_1_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_1_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_1_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_2_SLOPE: {
						DEBUG_PRINT("CMD_SF_2_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_2_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_2_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_2_OFFSET: {
						DEBUG_PRINT("CMD_SF_2_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_2_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_2_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_3_SLOPE: {
						DEBUG_PRINT("CMD_SF_3_SLOPE:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_3_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_3_SLOPE - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_SF_3_OFFSET: {
						DEBUG_PRINT("CMD_SF_3_OFFSET:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_3_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_3_OFFSET - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_BIAS_COMP_T1: {
						DEBUG_PRINT("CMD_BIAS_COMP_T1:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_COMP_T1, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_COMP_T1 - CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
							
						}			
						break;
					}
					case CMD_BIAS_COMP_T2: {
						DEBUG_PRINT("CMD_BIAS_COMP_T2:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_COMP_T2, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_COMP_T2 - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_1_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_1_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_1_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_1_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_1_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_1_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_1_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_1_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_2_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_2_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_2_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_2_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_2_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_2_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_2_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_2_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_3_SLOPE: {
						DEBUG_PRINT("CMD_BIAS_3_SLOPE:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_3_SLOPE, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_3_SLOPE - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_3_OFFSET: {
						DEBUG_PRINT("CMD_BIAS_3_OFFSET:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_3_OFFSET, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_3_OFFSET - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_SF_SLOPE_XLM: {
						DEBUG_PRINT("CMD_SF_SLOPE_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_SLOPE_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_SLOPE_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_SF_OFFSET_XLM: {
						DEBUG_PRINT("CMD_SF_OFFSET_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SF_OFFSET_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_SF_OFFSET_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_SLOPE_XLM: {
						DEBUG_PRINT("CMD_BIAS_SLOPE_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_SLOPE_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_SLOPE_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					case CMD_BIAS_OFFSET_XLM: {
						DEBUG_PRINT("CMD_BIAS_OFFSET_XLM:\n");
            if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_BIAS_OFFSET_XLM, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_BIAS_OFFSET_XLM - CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
            }
            else if(rx->condition == RX_CONDITION_EFFE_5354) {

            }			
						break;
					}
					/***------------- mis-alignment command, accl */
					case CMD_MIS_AX: {
						DEBUG_PRINT("CMD_MIS_AX:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_AX, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AX - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_AY: {
						DEBUG_PRINT("CMD_MIS_AY:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_AY, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AY - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_AZ: {
						DEBUG_PRINT("CMD_MIS_AZ:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_AZ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_AZ - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A11: {
						DEBUG_PRINT("CMD_MIS_A11:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A11, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A11 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A12: {
						DEBUG_PRINT("CMD_MIS_A12:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A12, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A12 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A13: {
						DEBUG_PRINT("CMD_MIS_A13:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A13, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A13 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A21: {
						DEBUG_PRINT("CMD_MIS_A21:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A21, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A21 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A22: {
						DEBUG_PRINT("CMD_MIS_A22:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A22, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A22 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A23: {
						DEBUG_PRINT("CMD_MIS_A23:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A23, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A23 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A31: {
						DEBUG_PRINT("CMD_MIS_A31:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A31, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A31 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A32: {
						DEBUG_PRINT("CMD_MIS_A32:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A32, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A32 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_A33: {
						DEBUG_PRINT("CMD_MIS_A33:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_A33, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_A33 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					/***------------- mis-alignment command, gyro */
					case CMD_MIS_GX: {
						DEBUG_PRINT("CMD_MIS_GX:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_GX, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GX - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_GY: {
						DEBUG_PRINT("CMD_MIS_GY:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_GY, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GY - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_GZ: {
						DEBUG_PRINT("CMD_MIS_GZ:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_GZ, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_GZ - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G11: {
						DEBUG_PRINT("CMD_MIS_G11:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G11, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G11 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G12: {
						DEBUG_PRINT("CMD_MIS_G12:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G12, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G12 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G13: {
						DEBUG_PRINT("CMD_MIS_G13:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G13, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G13 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G21: {
						DEBUG_PRINT("CMD_MIS_G21:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G21, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G21 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G22: {
						DEBUG_PRINT("CMD_MIS_G22:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G22, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G22 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G23: {
						DEBUG_PRINT("CMD_MIS_G23:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G23, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G23 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G31: {
						DEBUG_PRINT("CMD_MIS_G31:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G31, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G31 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G32: {
						DEBUG_PRINT("CMD_MIS_G32:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G32, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G32 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
					case CMD_MIS_G33: {
						DEBUG_PRINT("CMD_MIS_G33:\n");
						if(rx->ch != 4) {DEBUG_PRINT("Ch value must be 4:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_MIS_G33, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_MIS_G33 - MIS_CONTAINER_TO_CMD_OFFSET);
              DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == 3) {
							
						}
						break;
					}
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
					case CMD_DUMP_SN: {
						DEBUG_PRINT("CMD_DUMP_SN:\n");
						// dump_SN(fog_inst);
						break;
					} 
					case CMD_DATA_OUT_START: { // not use now
						DEBUG_PRINT("CMD_DATA_OUT_START:\n");
						// start_flag = rx->value;
						break;
					}
					case CMD_SYNC_CNT: {
						DEBUG_PRINT("CMD_SYNC_CNT:\n");
						// IOWR(VARSET_BASE, var_sync_count, rx->value);
						break;
					} 
					case CMD_HW_TIMER_RST: {
						DEBUG_PRINT("CMD_HW_TIMER_RST:\n");
						// IOWR(VARSET_BASE, var_timer_rst, rx->value);
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


/**
 * @brief 解析簡單的 JSON，將 key 與整數值 (int32_t) 傳給 callback
 *
 * @param js   JSON 字串
 * @param cb   使用者提供的 callback 函數 (型別 kv_cb_t)
 * @param ctx  使用者上下文指標，會原封不動傳回給 callback
 */
void parse_simple_json_ints(const char* js, kv_cb_t cb, void* ctx)
{
  if (!js || !cb) return;

  const char* p = js;
  while (*p) { // 外層迴圈：確保還沒走到字串結尾 '\0'

    // 1. 找下一個 key 的起始引號 '"'
    while (*p && *p != '"') p++;   // 跳過無關字元直到找到引號
    if (*p != '"') break;          // 如果不是 '"'（可能到結尾），就跳出迴圈
    p++;                           // 跳過這個起始引號，使 p 指到 key 的第一個字元

    // 2. 擷取 key 數字（JSON 格式假設 key 是 "0", "1", ...）
    int key = 0; 
    while (*p >= '0' && *p <= '9') {  // 確定當前字元是Ascii數字 '0' ~ '9' 區間內
        key = key * 10 + (*p - '0');  // 轉成整數（例如 '2''3' → 23）
        p++;                          // 移動到下一個字元
    }

    // 3. 找 key 的結尾引號
    while (*p && *p != '"') p++;      // 跳過數字後，直到遇到結束引號
    if (*p == '"') p++;               // 如果確實遇到 '"', 就略過它

    // 4. 找冒號 ':'（key 和 value 的分隔符號）
    while (*p && *p != ':') p++;      // 跳過空白或其他符號
    if (*p == ':') p++;               // 略過 ':'，使 p 指向 value 開頭

    // 5. 擷取 value 整數（允許負號）
    int sign = 1; 
    if (*p == '-') {                  // 如果有負號
        sign = -1;
        p++;
    }

    int32_t val = 0;
    while (*p >= '0' && *p <= '9') {  // 當前字元是數字
        val = val * 10 + (*p - '0');  // 組成整數
        p++;
    }
    val *= sign;                      // 套用正負號

    // 6. 把解析出的 key/value 傳給 callback
    cb(key, val, ctx);

    // 7. 跳過逗號或空白，繼續下一輪
    while (*p && *p != ',' && *p != '}') p++;
    if (*p == ',') p++;               // 如果是逗號，移到下一個項目
    else if (*p == '}') break;        // 如果是物件結尾 '}', 結束迴圈

  }
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
    case 1: C->fog->paramX[key].data.int_val = val; break;
    case 2: C->fog->paramY[key].data.int_val = val; break;
    case 3: C->fog->paramZ[key].data.int_val = val; break;
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



void dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch)
{
  if (!fog_inst) return;

  // 1) Send command to FPGA over Serial4
  // static const uint8_t HDR[2] = {0xAB, 0xBA};
  // static const uint8_t TRL[2] = {0x55, 0x56};
  sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_DUMP_FOG, 2, ch);

  // 2) Receive a full JSON object from Serial4
  char json_buf[FOG_JSON_BUF_SIZE];
  size_t n = read_json_object(Serial4, json_buf, sizeof(json_buf), FOG_JSON_TIMEOUT_MS);
  if (n == 0) {
    // debug
    DEBUG_PRINT("dump_fog_param: timeout or malformed JSON from Serial4");
    return;
  }

  // 3) Parse and store into fog_inst->paramX/Y/Z by channel
  fog_cb_ctx_t ctx = { fog_inst, ch };
  parse_simple_json_ints(json_buf, fog_store_cb, &ctx);

  // 4) Forward the raw JSON to PC via Serial1 (TX)
  Serial1.write((const uint8_t*)json_buf, strlen(json_buf));
  Serial1.write('\n');  // optional newline for readability
}

void dump_misalignment_param(fog_parameter_t* fog_inst)
{
  if (!fog_inst) return;

  // 1) Send command to FPGA over Serial4

  sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_DUMP_MIS, 2, 4);

  // 2) Receive a full JSON object from Serial4
  char json_buf[FOG_JSON_BUF_SIZE];
  size_t n = read_json_object(Serial4, json_buf, sizeof(json_buf), FOG_JSON_TIMEOUT_MS);
  if (n == 0) {
    // debug
    DEBUG_PRINT("dump_imu_mis-alignment: timeout or malformed JSON from Serial4");
    return;
  }

  // 3) Parse and store into fog_inst->paramX/Y/Z by channel
  fog_cb_ctx_t ctx = { fog_inst, 4 };
  parse_simple_json_ints(json_buf, imu_cali_store_cb, &ctx);

  // 4) Forward the raw JSON to PC via Serial1 (TX)
  Serial1.write((const uint8_t*)json_buf, strlen(json_buf));
  Serial1.write('\n');  // optional newline for readability
} 

