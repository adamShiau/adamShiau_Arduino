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
const uint8_t KVH_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};


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
    memcpy(out->fog.fogy.step.bin_val,  &pkt[idx], 4); idx += 4;
    memcpy(out->fog.fogx.step.bin_val,  &pkt[idx], 4); idx += 4;

    // adxl357 {ax, ay, az}
    memcpy(out->adxl357.ax.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->adxl357.ay.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->adxl357.az.bin_val,     &pkt[idx], 4); idx += 4;

    // temp {x, y, z}
    memcpy(out->temp.tempz.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->temp.tempy.bin_val,     &pkt[idx], 4); idx += 4;
    memcpy(out->temp.tempx.bin_val,     &pkt[idx], 4); idx += 4;

    // adxl357.temp
    memcpy(out->adxl357.temp.bin_val,   &pkt[idx], 4); idx += 4;

    // time
    memcpy(out->time.time.bin_val,      &pkt[idx], 4); idx += 4;

    return 0;
}

uint32_t crc_table[256];

void crc32_init_table() {
	for (int i = 0; i < 256; ++i) {
		uint32_t remainder = i << 24;
		for (int bit = 0; bit < 8; ++bit) {
			if (remainder & 0x80000000) {
				remainder = (remainder << 1) ^ POLYNOMIAL_32;
			} else {
				remainder = (remainder << 1);
			}
		}
		crc_table[i] = remainder;
	}
}

/**
 * @brief Generate CRC32 for KVH_HEADER + payload
 * 
 * @param header  pointer to 4-byte KVH header
 * @param payload pointer to 44-byte payload
 * @param payload_len length of payload (should be 44)
 * @param crc_out pointer to 4-byte array for output (big-endian)
 */
void gen_crc32(const uint8_t* header, const uint8_t* payload, size_t payload_len, uint8_t* crc_out)
{
    uint32_t remainder = 0xFFFFFFFF;

    // header (固定 4B)
    for (int i = 0; i < 4; i++) {
        uint8_t index = (remainder >> 24) ^ header[i];
        remainder = (remainder << 8) ^ crc_table[index];
    }

    // payload (TOTAL_PAYLOAD_LEN)
    for (size_t i = 0; i < payload_len; i++) {
        uint8_t index = (remainder >> 24) ^ payload[i];
        remainder = (remainder << 8) ^ crc_table[index];
    }

    // 輸出 big-endian
    crc_out[0] = (remainder >> 24) & 0xFF;
    crc_out[1] = (remainder >> 16) & 0xFF;
    crc_out[2] = (remainder >> 8) & 0xFF;
    crc_out[3] = remainder & 0xFF;
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
        fog_inst->cnofig[idx].data.int_val = rx->value;
        DEBUG_PRINT("cnofig[%d] = %d\n", idx, fog_inst->cnofig[idx].data.int_val);
      }
      break;

    default:
      // unknown channel -> ignore
      break;
  }
}

void readout_parameter_container(const cmd_ctrl_t* rx, fog_parameter_t* fog_inst, uint8_t idx)
{
  if (!rx || !fog_inst) return;

  data_t data;

  switch (rx->ch) {
    case 3: // paramX
      if (idx < PAR_LEN) {
        data.int_val = fog_inst->paramX[idx].data.int_val;
        DEBUG_PRINT("paramX[%d] = %d\n", idx, fog_inst->paramX[idx].data.int_val);
      }
      break;

    case 2: // paramY
      if (idx < PAR_LEN) {
        data.int_val = fog_inst->paramY[idx].data.int_val;
        DEBUG_PRINT("paramY[%d] = %d\n", idx, fog_inst->paramY[idx].data.int_val);
      }
      break;

    case 1: // paramZ
      if (idx < PAR_LEN) {
        data.int_val = fog_inst->paramZ[idx].data.int_val;
        DEBUG_PRINT("paramZ[%d] = %d\n", idx, fog_inst->paramZ[idx].data.int_val);
      }
      break;

    case 4: // misalignment
      if (idx < MIS_LEN) {
        data.int_val = fog_inst->misalignment[idx].data.int_val;
        DEBUG_PRINT("misalignment[%d] = %d\n", idx, fog_inst->misalignment[idx].data.int_val);
      }
      break;

    case 6: // configuration
      if (idx < CFG_LEN) {
        data.int_val = fog_inst->cnofig[idx].data.int_val;
        DEBUG_PRINT("configuration[%d] = %d\n", idx, fog_inst->misalignment[idx].data.int_val);
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
          /***------------- configuration */
          case CMD_CFG_DR: {
						DEBUG_PRINT("CMD_CFG_DR:\n");
						if(rx->ch != 6) {DEBUG_PRINT("Ch value must be 6:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_CFG_DR, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_CFG_DR - CFG_CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
              readout_parameter_container(rx, fog_inst, CMD_CFG_DR - CFG_CONTAINER_TO_CMD_OFFSET);
						}
						break;
					}

          case CMD_CFG_BR: {
						DEBUG_PRINT("CMD_CFG_BR:\n");
						if(rx->ch != 6) {DEBUG_PRINT("Ch value must be 6:\n"); break;}
						if(rx->condition == RX_CONDITION_ABBA_5556) {
              sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_CFG_BR, rx->value, rx->ch);
              update_parameter_container(rx, fog_inst, CMD_CFG_BR - CFG_CONTAINER_TO_CMD_OFFSET);
							DEBUG_PRINT("WRITE: %d\n", rx->value);	
						}
						else if(rx->condition == RX_CONDITION_EFFE_5354) {
              readout_parameter_container(rx, fog_inst, CMD_CFG_BR - CFG_CONTAINER_TO_CMD_OFFSET);
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
						dump_SN(fog_inst);
						break;
					} 
          case CMD_DUMP_CFG: {
						DEBUG_PRINT("CMD_DUMP_CFG:\n");
						dump_cfg_param(fog_inst);
						break;
					} 
					case CMD_DATA_OUT_START: { // not use now
						DEBUG_PRINT("CMD_DATA_OUT_START:\n");
						// start_flag = rx->value;
						break;
					}
					case CMD_SYNC_CNT: {
						DEBUG_PRINT("CMD_SYNC_CNT:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SYNC_CNT, rx->value, rx->ch);
						}			
						break;
					} 
					case CMD_HW_TIMER_RST: {
						DEBUG_PRINT("CMD_HW_TIMER_RST:\n");	
						if(rx->condition == RX_CONDITION_ABBA_5556) {
							sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_HW_TIMER_RST, rx->value, rx->ch);
						}			
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

void reset_FPGA_timer(void)
{
	DEBUG_PRINT("reset_FPGA_timer\n");
	sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_HW_TIMER_RST, 1, 1);
	delay(10); 
	sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_HW_TIMER_RST, 0, 1);
}

void set_data_rate(uint32_t rate)
{
	sendCmd(Serial4, HDR_ABBA, TRL_5556, CMD_SYNC_CNT, rate, 1);
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

// ---- 通用的「字串 payload」解析器：單純把 payload 當成 C-string 丟入 callback ----
void parse_string(const char* payload, void (*cb)(const char* s, void*), void* ctx)
{
  if (!payload || !cb) return;
  cb(payload, ctx);
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

static void cfg_store_cb(int key, int32_t val, void* user)
{
  fog_cb_ctx_t* C = (fog_cb_ctx_t*)user;
  if (!C || !C->fog) return;
  if (key < 0 || key >= CFG_LEN) return;  // arrays are 0..CFG_LEN-1

  C->fog->config[key].data.int_val = val;
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

/***
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
 */
/***
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
 */
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
  float tx   = raw->temp.tempx.float_val;
  float ty   = raw->temp.tempy.float_val;
  float tz   = raw->temp.tempz.float_val;
  float tacc = raw->adxl357.temp.float_val;

  // === Gyro scale factor（一次線性）===
  float sf_x_gyro = sf_temp_comp_1st(tx,
      fog_parameter->paramX[17].data.float_val,
      fog_parameter->paramX[18].data.float_val);

  float sf_y_gyro = sf_temp_comp_1st(ty,
      fog_parameter->paramY[17].data.float_val,
      fog_parameter->paramY[18].data.float_val);

  float sf_z_gyro = sf_temp_comp_1st(tz,
      fog_parameter->paramZ[17].data.float_val,
      fog_parameter->paramZ[18].data.float_val);
      // Serial.println("\nsf_gyro: slope, offset:");
      // Serial.print(fog_parameter->paramX[17].data.float_val,4); Serial.print(","); 
      // Serial.print(fog_parameter->paramX[18].data.float_val,4); Serial.println();
      // Serial.print(fog_parameter->paramY[17].data.float_val,4); Serial.print(","); 
      // Serial.print(fog_parameter->paramY[18].data.float_val,4); Serial.println();
      // Serial.print(fog_parameter->paramZ[17].data.float_val,4); Serial.print(","); 
      // Serial.print(fog_parameter->paramZ[18].data.float_val,4); Serial.println();

  // === Accel scale factor（一次線性；用 adxl 溫度）===
  float sf_x_acc = sf_temp_comp_1st(tacc,
      fog_parameter->paramX[31].data.float_val,
      fog_parameter->paramX[32].data.float_val);

  float sf_y_acc = sf_temp_comp_1st(tacc,
      fog_parameter->paramY[31].data.float_val,
      fog_parameter->paramY[32].data.float_val);

  float sf_z_acc = sf_temp_comp_1st(tacc,
      fog_parameter->paramZ[31].data.float_val,
      fog_parameter->paramZ[32].data.float_val);
      // Serial.println("\nsf_accl: slope, offset:");
      // Serial.print(fog_parameter->paramX[31].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramX[32].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramY[31].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramY[32].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramZ[31].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramZ[32].data.float_val); Serial.println();

  // === Gyro bias（三區段一次線性）===
  float bx_gyro = bias_temp_comp_1st_3t(
      tx,
      fog_parameter->paramX[23].data.float_val,  // T1
      fog_parameter->paramX[24].data.float_val,  // T2
      fog_parameter->paramX[25].data.float_val,  // s1
      fog_parameter->paramX[26].data.float_val,  // o1
      fog_parameter->paramX[27].data.float_val,  // s2
      fog_parameter->paramX[28].data.float_val,  // o2
      fog_parameter->paramX[29].data.float_val,  // s3
      fog_parameter->paramX[30].data.float_val   // o3
  );

  float by_gyro = bias_temp_comp_1st_3t(
      ty, fog_parameter->paramY[23].data.float_val, fog_parameter->paramY[24].data.float_val,
      fog_parameter->paramY[25].data.float_val, fog_parameter->paramY[26].data.float_val,
      fog_parameter->paramY[27].data.float_val, fog_parameter->paramY[28].data.float_val,
      fog_parameter->paramY[29].data.float_val, fog_parameter->paramY[30].data.float_val
  );

  float bz_gyro = bias_temp_comp_1st_3t(
      tz, fog_parameter->paramZ[23].data.float_val, fog_parameter->paramZ[24].data.float_val,
      fog_parameter->paramZ[25].data.float_val, fog_parameter->paramZ[26].data.float_val,
      fog_parameter->paramZ[27].data.float_val, fog_parameter->paramZ[28].data.float_val,
      fog_parameter->paramZ[29].data.float_val, fog_parameter->paramZ[30].data.float_val
  );
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
  float bx_acc = sf_temp_comp_1st(tacc,
      fog_parameter->paramX[33].data.float_val,
      fog_parameter->paramX[34].data.float_val);

  float by_acc = sf_temp_comp_1st(tacc,
      fog_parameter->paramY[33].data.float_val,
      fog_parameter->paramY[34].data.float_val);

  float bz_acc = sf_temp_comp_1st(tacc,
      fog_parameter->paramZ[33].data.float_val,
      fog_parameter->paramZ[34].data.float_val);
      // Serial.println("\nbs_accl: s, o:");
      // Serial.print(fog_parameter->paramX[33].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramX[34].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramY[33].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramY[34].data.float_val); Serial.println();
      // Serial.print(fog_parameter->paramZ[33].data.float_val); Serial.print(","); 
      // Serial.print(fog_parameter->paramZ[34].data.float_val); Serial.println();


  // === Gyro 溫補（scale factor & bias）===
  float gx_comp = raw->fog.fogx.step.float_val * sf_x_gyro - bx_gyro;
  float gy_comp = raw->fog.fogy.step.float_val * sf_y_gyro - by_gyro;
  float gz_comp = raw->fog.fogz.step.float_val * sf_z_gyro - bz_gyro;
  // Serial.print(raw->fog.fogx.step.float_val); Serial.print(","); 
  // Serial.print(raw->fog.fogy.step.float_val); Serial.print(","); 
  // Serial.print(raw->fog.fogz.step.float_val); Serial.println();Serial.println();

  // === Accel 溫補（scale factor & bias）===
  float ax_comp = raw->adxl357.ax.float_val * sf_x_acc - bx_acc;
  float ay_comp = raw->adxl357.ay.float_val * sf_y_acc - by_acc;
  float az_comp = raw->adxl357.az.float_val * sf_z_acc - bz_acc;

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
  cali->temp.tempx.float_val = tx;
  cali->temp.tempy.float_val = ty;
  cali->temp.tempz.float_val = tz;
  cali->adxl357.temp.float_val = tacc;
  cali->time.time.float_val = raw->time.time.float_val;
}

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


/**
 * @brief 從串口讀取一行字串直到遇到 '\n' 或逾時。
 *
 * 這個函式會從指定的串口中讀取資料，直到：
 *  - 讀到換行字元 '\n'（表示一行結束）
 *  - 或者超過設定的逾時時間。
 * 
 * 它會去除行尾的 '\r'，並確保結果字串有 '\0' 結尾。
 * 緩衝區必須至少能存 1 個字元和結尾符。
 *
 * @param port        要讀取的串口物件（例如 Serial、Serial1）。
 * @param line        存放讀到的一行字串的緩衝區。
 * @param line_cap    緩衝區大小（包含結尾 '\0' 的空間）。
 * @param timeout_ms  逾時時間（毫秒）。
 *
 * @return 讀取到的字元數（不包含結尾 '\0'）。若超時或無資料則回傳 0。
 */
static size_t readline_with_timeout(Stream& port, char* line, size_t line_cap, uint32_t timeout_ms)
{
    if (line_cap == 0) return 0;          // 緩衝區大小不夠
    size_t n = 0;                         // 已讀取的字元數
    uint32_t t0 = millis();               // 記錄開始時間

    while ((millis() - t0) < timeout_ms)  // 檢查是否逾時
    {
        while (port.available())          // 串口有資料可讀
        {
            int c = port.read();          // 讀取一個字元
            if (c < 0) break;             // 讀取失敗就跳出

            if (c == '\n') {              // 遇到換行表示一行結束
                while (n > 0 && line[n-1] == '\r') n--;  // 移除尾端 '\r'
                line[n] = '\0';           // 補上字串結尾
                return n;                 // 回傳已讀字數
            }

            if (n + 1 < line_cap) {       // 還有空間可寫
                line[n++] = (char)c;      // 存入字元
            } else {
                line[line_cap - 1] = '\0'; // 滿了就補結尾
                return n;                  // 回傳已讀字數
            }
        }
        delay(1); // 稍微休息，避免忙迴圈
    }

    return 0;  // 逾時
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

/**
 * @brief 從串口讀取並解析一個完整的 dump 封包。
 *
 * 封包格式為：
 * @<seq>,<ch>,<len>,<payload>*<CRC16>\r\n
 *
 * - @ 符號開頭。
 * - 以逗號分隔的三個欄位：序號 (seq)、通道號 (ch)、資料長度 (len)。
 * - 接著一段 payload，可為 JSON 或一般字串。
 * - 以星號 '*' 後接 4 個十六進位字元表示 CRC16 (CCITT)。
 * - 行尾為 "\r\n"。
 *
 * 函式會：
 * - 在 timeout_ms 毫秒內讀取一整行封包。
 * - 解析 seq、ch、payload。
 * - 驗證 payload 的長度與 CRC16。
 * - 若驗證通過，將 payload 複製到呼叫者提供的緩衝區，並補上 '\0' 結尾。
 *
 * @param port            要讀取的串口物件 (例如 Serial1)。
 * @param timeout_ms      逾時時間（毫秒）。
 * @param out_seq         指向變數，用來存放解析出的序號，可為 nullptr。
 * @param out_ch          指向變數，用來存放解析出的通道，可為 nullptr。
 * @param payload_buf     用來存放 payload 的緩衝區。
 * @param payload_cap     緩衝區大小（包含結尾 '\0'），必須足夠容納 payload。
 * @param out_payload_len 指向變數，用來存放 payload 的實際長度（不含 '\0'），可為 nullptr。
 *
 * @return DumpReadResult
 * - DUMP_OK：成功解析封包。
 * - DUMP_TIMEOUT：在 timeout_ms 內未讀到完整封包。
 * - DUMP_FORMAT_ERR：格式錯誤（缺逗號、缺 '*'、非數字等）。
 * - DUMP_LEN_MISMATCH：payload 長度與 len 不符。
 * - DUMP_CRC_FAIL：CRC 驗證失敗。
 * - DUMP_OVERFLOW：payload 超過緩衝區容量。
 *
 * @note
 * - 這個函式不會清空 port buffer，請在呼叫前確保緩衝區中資料是正確的封包。
 * - 會自動去掉結尾的 '\r'。
 * - 適用於各種 dump：FOG JSON、mis-alignment JSON、序號字串等。
 */
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

    // 1) 讀取一整行（直到 '\n' 或 timeout）
    char line[1024];
    size_t ln = readline_with_timeout(port, line, sizeof(line), timeout_ms);
    if (ln == 0) return DUMP_TIMEOUT;

    // 2) 檢查開頭是否為 '@'
    char* p = line;
    while (*p && isspace((unsigned char)*p)) ++p;
    if (*p != '@') return DUMP_FORMAT_ERR;
    ++p;

    // 3) 找出三個逗號，分割 seq、ch、len
    char* c1 = strchr(p, ',');      if (!c1) return DUMP_FORMAT_ERR;
    *c1 = '\0';
    char* c2 = strchr(c1 + 1, ','); if (!c2) return DUMP_FORMAT_ERR;
    *c2 = '\0';
    char* c3 = strchr(c2 + 1, ','); if (!c3) return DUMP_FORMAT_ERR;
    *c3 = '\0';

    // 4) 將字串轉換為數字
    char* endptr = nullptr;
    uint32_t seq = strtoul(p, &endptr, 10);
    if (endptr == p) return DUMP_FORMAT_ERR;
    unsigned long ch_ul = strtoul(c1 + 1, &endptr, 10);
    if (endptr == (c1 + 1) || ch_ul > 255UL) return DUMP_FORMAT_ERR;
    uint8_t ch = (uint8_t)ch_ul;
    uint32_t len = strtoul(c2 + 1, &endptr, 10);
    if (endptr == (c2 + 1)) return DUMP_FORMAT_ERR;

    // 5) 找出 '*'，切出 payload 與 CRC
    char* star = strrchr(c3 + 1, '*');
    if (!star) return DUMP_FORMAT_ERR;
    *star = '\0';
    const char* payload = c3 + 1;
    size_t payload_len  = strlen(payload);

    // 6) 檢查 payload 長度
    if (payload_len != len) return DUMP_LEN_MISMATCH;

    // 7) 讀取 CRC16
    if (strlen(star + 1) < 4) return DUMP_FORMAT_ERR;
    char crc_hex[5] = {0};
    crc_hex[0] = star[1];
    crc_hex[1] = star[2];
    crc_hex[2] = star[3];
    crc_hex[3] = star[4];
    uint16_t rx_crc = (uint16_t)strtoul(crc_hex, nullptr, 16);

    // 8) 計算 CRC16 驗證
    uint16_t calc = crc16_ccitt_buf((const uint8_t*)payload, payload_len);
    if (calc != rx_crc) return DUMP_CRC_FAIL;

    // 9) 複製 payload 到呼叫者緩衝區
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
  sendCmd(Serial4, HDR_ABBA, TRL_5556, cmd, (int32_t)val, ch);
}

// =====================  中層（Mid-level）  =====================

static uint32_t g_seq = 0;               // 成功收到並驗證通過後才 ++
static const uint32_t FOG_TIMEOUT_MS = 500;
static const size_t   SCRATCH_MAX    = 1024;

/**
 * @brief 收一包、驗證、依 ch 分流存放。
 *        ch=1/2/3 → fog_store_cb；ch=4 → imu_cali_store_cb；ch=5 → SN（字串）。
 * @param sn_out 可為 nullptr；若 ch=5 且不為 nullptr，會複製序號字串到此。
 */
static bool recv_and_store(fog_parameter_t* fog,
                           uint8_t expect_ch,
                           uint32_t expect_seq,
                           uint32_t timeout_ms,
                           char* scratch, size_t scratch_cap)
{
  uint32_t seq=0; uint8_t ch=0; size_t plen=0;
  DumpReadResult r = read_dump_packet(Serial4, timeout_ms, &seq, &ch,
                                      scratch, scratch_cap, &plen);
  if (r != DUMP_OK) return false;
  // 若要嚴格比對 echo 的 seq/ch，可加入：
  if (ch != expect_ch /*|| seq != expect_seq*/) {
    // 你也可以選擇接受 ch 不同的封包並丟棄；這裡採嚴格模式
    return false;
  }

  if (expect_ch == 1 || expect_ch == 2 || expect_ch == 3) {
	// FOG X/Y/Z para → JSON：{"0":...}
    fog_cb_ctx_t ctx{ fog, expect_ch };
    parse_simple_json_ints(scratch, fog_store_cb, &ctx);

  } else if (expect_ch == 4) {
	// Mis-alignment → JSON
    fog_cb_ctx_t ctx{ fog, 4 };
    parse_simple_json_ints(scratch, imu_cali_store_cb, &ctx);

  } else if (expect_ch == 5) {
	// SN → 純字串，走 parse_string + sn_store_cb
    fog_cb_ctx_t ctx{ fog, 5 };
    parse_string(scratch, sn_store_cb, &ctx);
  } else if (expect_ch == 6) {
	// config → JSON
    fog_cb_ctx_t ctx{ fog, 6 };
    parse_simple_json_ints(scratch, cfg_store_cb, &ctx);
  }
  
  // RS422 輸出
  Serial1.write((const uint8_t*)scratch, strlen(scratch));
  Serial1.write('\n');
  return true;
}

/**
 * @brief Stop-and-Wait 取資料，支援一次 NACK 快速重送。
 * @param sn_out/sn_cap 僅 ch=5 需要；其他 ch 可傳 nullptr/0。
 */
static bool request_and_update(fog_parameter_t* fog,
                               uint8_t ch,
                               uint32_t timeout_ms,
                               int max_retry)
{
  if (!fog) return false;

  static char scratch[SCRATCH_MAX];

  for (int attempt = 0; attempt < max_retry; ++attempt) {
    // 普通 REQ
    uint8_t cmd;

    switch (ch) {
        case 4:
            cmd = CMD_DUMP_MIS;
            break;
        case 5:
            cmd = CMD_DUMP_SN;
            break;
        case 6:
            cmd = CMD_DUMP_CFG;
            break;
        default:
            cmd = CMD_DUMP_FOG;
            break;
    }

    send_cmd_seq(cmd, ch, g_seq, /*nack*/false);

    if (recv_and_store(fog, ch, g_seq, timeout_ms, scratch, sizeof(scratch))) {
      g_seq = (g_seq + 1) & 0x7FFFFFFF;
      return true;
    }

    // NACK（若 Nios II 尚未支援，等效於再送一次）
    send_cmd_seq(cmd, ch, g_seq, /*nack*/false);

    if (recv_and_store(fog, ch, g_seq, timeout_ms, scratch, sizeof(scratch))) {
      g_seq = (g_seq + 1) & 0x7FFFFFFF;
      return true;
    }

    delay(20 + attempt * 10);
  }
  return false;
}


// =====================  高層（High-level APIs）  =====================

/**
 * @brief 抓某一軸 FOG 參數（X=1/Y=2/Z=3），payload 是 JSON。
 */
bool dump_fog_param(fog_parameter_t* fog_inst, uint8_t ch) {
  if (!fog_inst) return false;
  if (ch < 1 || ch > 3) return false;
  return request_and_update(fog_inst, ch, FOG_TIMEOUT_MS, /*retries*/5);
}

/**
 * @brief 抓 Mis-alignment（ch=4），payload 是 JSON。
 */
bool dump_misalignment_param(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;
  return request_and_update(fog_inst, /*ch*/4, FOG_TIMEOUT_MS, /*retries*/5);
}

/**
 * @brief 抓序號 SN（ch=5），payload 是一般字串。
 */
bool dump_SN(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;
  return request_and_update(fog_inst, /*ch*/5, FOG_TIMEOUT_MS, /*retries*/5);
}

/**
 * @brief 抓 configuration（ch=6），payload 是 JSON。
 */
bool dump_cfg_param(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;
  return request_and_update(fog_inst, /*ch*/6, FOG_TIMEOUT_MS, /*retries*/5);
}

/**
 * @brief 上電後一次抓齊：FOG X/Y/Z、Mis-alignment、SN。
 * @return 全部都成功則回 true；只要有一項失敗就回 false。
 */
bool boot_capture_all(fog_parameter_t* fog_inst) {
  if (!fog_inst) return false;

  bool ok = true;
  ok &= dump_fog_param(fog_inst, 1);   // X
  delay(10);
  ok &= dump_fog_param(fog_inst, 2);   // Y
  delay(10);
  ok &= dump_fog_param(fog_inst, 3);   // Z
  delay(10);
  ok &= dump_misalignment_param(fog_inst); // MIS
  delay(10);
  ok &= dump_cfg_param(fog_inst); // config
  delay(10);
//   ok &= dump_SN(fog_inst);             // SN -> 寫進 fog_inst->sn

  return ok;
}