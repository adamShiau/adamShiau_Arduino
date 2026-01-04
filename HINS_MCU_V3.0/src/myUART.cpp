/*
 * myUART.cpp
 *
 * Implementation for myUART.h
 * - Provides fixed-protocol parser for Serial1 (readDataDynamic).
 * - Provides generic header/trailer parser for Serial4 (readData).
 *
 * Design choices:
 *   - C-style API, but compiled as C++ (Arduino core requirement).
 *   - Static per-port parser contexts; safe for use from setup()/loop().
 *   - Do NOT call these from ISR; let Uart::IrqHandler() fill ring buffers.
 */

#include "myUART.h"

/* ----------------------------------------------------------------------------
 * UART object availability
 * ----------------------------------------------------------------------------
 * Serial1 is provided by the Arduino core for Nano 33 IoT (variant.h/.cpp).
 * DO NOT re-define Serial1 here to avoid multiple-definition conflicts.
 *
 * If your core does NOT construct Serial2/Serial3/Serial4, you may define them
 * here. On Nano 33 IoT, the core typically only constructs Serial1; so it's
 * safe to provide Serial2..Serial4 below.
 */

Uart Serial1(&sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX);
Uart Serial2(&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial3(&sercom1, 13, 8,  SERCOM_RX_PAD_1, UART_TX_PAD_2);
Uart Serial4(&sercom3, 10, 9,  SERCOM_RX_PAD_3, UART_TX_PAD_2);

/* ----------------------------------------------------------------------------
 * Serial1 fixed-protocol parser (3 formats)
 * ----------------------------------------------------------------------------
 * Formats:
 *   #1: Header = AB BA, Trailer = 55 56, DataLen = 6
 *   #2: Header = CD DC, Trailer = 57 58, DataLen = 13
 *   #3: Header = EF FE, Trailer = 53 54, DataLen = 6
 */
#define HEADER1_SIZE   2
#define HEADER2_SIZE   2
#define HEADER3_SIZE   2
#define TRAILER1_SIZE  2
#define TRAILER2_SIZE  2
#define TRAILER3_SIZE  2
#define HEADER4_SIZE   2
#define TRAILER4_SIZE  2
#define DATA1_SIZE     6
#define DATA2_SIZE     13
#define DATA3_SIZE     6
#define MAX_DATA_SIZE  128
#define BUFFER_SIZE    (MAX_DATA_SIZE + 1)  // +1 for condition byte at buffer[0]

static const uint8_t HEADER1[]  = {0xAB, 0xBA};
static const uint8_t TRAILER1[] = {0x55, 0x56};
static const uint8_t HEADER2[]  = {0xCD, 0xDC};
static const uint8_t TRAILER2[] = {0x57, 0x58};
static const uint8_t HEADER3[]  = {0xEF, 0xFE};
static const uint8_t TRAILER3[] = {0x53, 0x54};
static const uint8_t HEADER4[]  = {0xBC, 0xCB};
static const uint8_t TRAILER4[] = {0x51, 0x52};


/* C-style state machine */
typedef enum {
  EXPECTING_HEADER = 0,
  EXPECTING_PAYLOAD,
  EXPECTING_TRAILER
} State;

/* Per-port parser context for Serial1 fixed protocol */
typedef struct {
  uint8_t  buffer[BUFFER_SIZE];     // [0] = condition; payload starts at [1]
  uint8_t  bytes_received;          // counts payload/trailer bytes
  State    state;
  uint8_t  condition;               // 0=unknown, 1..3 as formats above
  uint8_t  data_size_expected;      // 6/13/6
  const uint8_t* expected_trailer;  // points to TRAILERx
  uint8_t  trailer_size;            // 2
  // header match progress (independent counters for each possible header)
  uint8_t  hdr_idx1, hdr_idx2, hdr_idx3, hdr_idx4;
} ParserCtx1;

static ParserCtx1 ctx_ser1;

/* Reset Serial1 parser context */
static void init_ctx(ParserCtx1* C) {
  memset(C, 0, sizeof(*C));
  C->state = EXPECTING_HEADER;
}

/* ----------------------------------------------------------------------------
 * Serial4 generic parser context (header/trailer/length are caller-provided)
 * ----------------------------------------------------------------------------
 */
typedef enum {
  RD_FIND_HEADER = 0,
  RD_READ_PAYLOAD,
  RD_CHECK_TRAILER
} RD_State;

typedef struct {
  RD_State state;
  uint16_t hdr_idx;                    // progress inside header
  uint16_t pay_idx;                    // progress inside payload
  uint16_t trl_idx;                    // progress inside trailer
  uint16_t datalen;                    // expected payload length
  uint8_t  payload[MAX_DATA_SIZE4];    // only the data bytes (returned to caller)
} RD_Ctx4;

static RD_Ctx4 rd4;

/* Reset Serial4 generic parser context */
static inline void rd4_reset(void) {
  rd4.state   = RD_FIND_HEADER;
  rd4.hdr_idx = 0;
  rd4.pay_idx = 0;
  rd4.trl_idx = 0;
  rd4.datalen = 0;
}

/* ----------------------------------------------------------------------------
 * Public: initialization
 * ----------------------------------------------------------------------------
 */
void myUART_init(void)
{
  // USB CDC debug
  Serial.begin(230400);

  // UARTs
  Serial1.begin(115200); // to HINS FPGA
  Serial2.begin(230400); // Output UART connector J1
  Serial3.begin(115200); // to CV7
  Serial4.begin(115200); // to GNSS recv

  // Pin multiplexing (board-specific; matches your existing setup)
  pinPeripheral(24, PIO_SERCOM);      // SERCOM2: TX=24, RX=25
  pinPeripheral(25, PIO_SERCOM);

  pinPeripheral(8,  PIO_SERCOM);      // SERCOM1: TX=8 , RX=13
  pinPeripheral(13, PIO_SERCOM);

  pinPeripheral(10, PIO_SERCOM_ALT);  // SERCOM3: TX=9 , RX=10 (ALT)
  pinPeripheral(9,  PIO_SERCOM_ALT);

  // (Optional) adjust IRQ priorities: lower value = higher priority
  // NVIC_SetPriority(SERCOM5_IRQn, 2); // Serial1
  // NVIC_SetPriority(SERCOM3_IRQn, 3); // Serial4
  // NVIC_SetPriority(SERCOM2_IRQn, 3); // Serial2
  // NVIC_SetPriority(SERCOM1_IRQn, 3); // Serial3

  // Reset parser states
  init_ctx(&ctx_ser1);
  rd4_reset();

  // Drain any stale bytes
  // while (Serial1.available()) (void)Serial1.read();
  // while (Serial4.available()) (void)Serial4.read();
}

/* ----------------------------------------------------------------------------
 * Interrupt handlers: delegate to Arduino Uart::IrqHandler()
 * ----------------------------------------------------------------------------
 */
void SERCOM2_Handler(void) { Serial2.IrqHandler(); }
void SERCOM1_Handler(void) { Serial3.IrqHandler(); }
void SERCOM3_Handler(void) { Serial4.IrqHandler(); }
void SERCOM5_Handler(void) { Serial1.IrqHandler(); }

/* ----------------------------------------------------------------------------
 * Serial1 fixed-protocol parser (PC commands)
 * ----------------------------------------------------------------------------
 * Returns:
 *   - NULL if incomplete.
 *   - Pointer to ctx_ser1.buffer on success:
 *       buffer[0]   = condition (1..3)
 *       buffer[1..] = payload    (length depends on condition)
 */
uint8_t* readDataDynamic(uint32_t* try_cnt)
{
  // Legacy behavior: use Serial2 as the command input port.
  // Prefer calling readDataDynamic(Stream& port, ...) in new code.
  return readDataDynamic(Serial2, try_cnt);
}

uint8_t* readDataDynamic(Stream& port, uint32_t* try_cnt)
{
  if (port.available() == 0) return NULL;

  int di = port.read();
  if (di == -1) return NULL;           // no data; defensive guard
  uint8_t data = (uint8_t)di;

  ParserCtx1* C = &ctx_ser1;

  switch (C->state) {
    case EXPECTING_HEADER:
      // Try 3 headers independently (minimal-change logic)
      if (C->hdr_idx1 < HEADER1_SIZE && data == HEADER1[C->hdr_idx1]) {
        C->hdr_idx1++;
        if (C->hdr_idx1 == HEADER1_SIZE) {
          C->condition          = 1;
          C->data_size_expected = DATA1_SIZE;
          C->expected_trailer   = TRAILER1;
          C->trailer_size       = TRAILER1_SIZE;
          C->state              = EXPECTING_PAYLOAD;
          C->bytes_received     = 0;
          C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
        }
      } else if (C->hdr_idx2 < HEADER2_SIZE && data == HEADER2[C->hdr_idx2]) {
        C->hdr_idx2++;
        if (C->hdr_idx2 == HEADER2_SIZE) {
          C->condition          = 2;
          C->data_size_expected = DATA2_SIZE;
          C->expected_trailer   = TRAILER2;
          C->trailer_size       = TRAILER2_SIZE;
          C->state              = EXPECTING_PAYLOAD;
          C->bytes_received     = 0;
          C->hdr_idx1 = C->hdr_idx3 = C->hdr_idx4 = 0;
        }
      } else if (C->hdr_idx3 < HEADER3_SIZE && data == HEADER3[C->hdr_idx3]) {
        C->hdr_idx3++;
        if (C->hdr_idx3 == HEADER3_SIZE) {
          C->condition          = 3;
          C->data_size_expected = DATA3_SIZE;
          C->expected_trailer   = TRAILER3;
          C->trailer_size       = TRAILER3_SIZE;
          C->state              = EXPECTING_PAYLOAD;
          C->bytes_received     = 0;
          C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx4 = 0;
        }
      } else if (C->hdr_idx4 < HEADER4_SIZE && data == HEADER4[C->hdr_idx4]) {
        C->hdr_idx4++;
        if (C->hdr_idx4 == HEADER4_SIZE) {
          C->condition          = 4;

          // 方案 4B：先讀 [cmd][len] 兩個 byte，讀到 len 後再決定還要讀多少 params
          C->data_size_expected = 2;                 // cmd + len
          C->expected_trailer   = TRAILER4;
          C->trailer_size       = TRAILER4_SIZE;

          C->state              = EXPECTING_PAYLOAD;
          C->bytes_received     = 0;

          // reset other header match progress
          C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
        }
      }
      else {
        // mismatch; reset header progress
        C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
        if (try_cnt) (*try_cnt)++;
      }
      break;

    case EXPECTING_PAYLOAD:

      // ---- special handling for SN write (condition 2, cmd=0x6E) ----
      // Packet format (condition 2):
      //   HEADER2 (2B) + payload[13B] + TRAILER2(2B)
      //   payload[0] = cmd (0x6E), payload[1..12] = SN ASCII
      //
      // Allow GUI to send fewer than 12 SN chars:
      //   after receiving cmd(0x6E), if we see TRAILER2 starting early,
      //   pad the remaining payload bytes with ASCII space (0x20) and
      //   continue trailer matching.
      if (C->condition == 2 &&
          C->bytes_received >= 1 &&              // cmd already received
          C->buffer[1] == 0x6E &&                // CMD_WRITE_SN
          C->expected_trailer &&
          data == C->expected_trailer[0] &&      // early trailer byte 0
          C->bytes_received < C->data_size_expected)
      {
        // pad remaining payload with spaces
        for (uint8_t i = C->bytes_received; i < C->data_size_expected; i++) {
          C->buffer[i + 1] = 0x20;               // ASCII space
        }

        // we've already matched trailer[0] with 'data'
        C->state          = EXPECTING_TRAILER;
        C->bytes_received = 1;                   // next byte should match trailer[1]
        break;
      }

      // ---- normal payload collecting ----
      if (C->bytes_received < MAX_DATA_SIZE) {
        C->buffer[C->bytes_received + 1] = data;  // payload begins at buffer[1]
        C->bytes_received++;

        // condition 4 (HINS var payload):
        // payload layout in buffer[1..] = [cmd][len][params...]
        // when we have received cmd+len (2 bytes), we know how many params to read.
        if (C->condition == 4 && C->bytes_received == 2) {
          uint8_t len = C->buffer[2];              // buffer[1]=cmd, buffer[2]=len
          uint16_t total = (uint16_t)2 + len;      // cmd + len + params(len bytes)

          if (total > MAX_DATA_SIZE) {
            // overflow → drop packet and resync
            C->state          = EXPECTING_HEADER;
            C->bytes_received = 0;
            C->condition      = 0;
            C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
            if (try_cnt) (*try_cnt)++;
            break;
          }

          C->data_size_expected = (uint8_t)total;
        }

      } else {
        // overflow: drop packet
        C->state          = EXPECTING_HEADER;
        C->bytes_received = 0;
        C->condition      = 0;
        C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
        if (try_cnt) (*try_cnt)++;
        break;
      }

      if (C->bytes_received >= C->data_size_expected) {
        C->state          = EXPECTING_TRAILER;
        C->bytes_received = 0;
      }
      break;

    case EXPECTING_TRAILER:
      if (!C->expected_trailer || C->bytes_received >= C->trailer_size) {
        // abnormal; reset
        C->state     = EXPECTING_HEADER;
        C->condition = 0;
        C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
        if (try_cnt) (*try_cnt)++;
        break;
      }

      if (data != C->expected_trailer[C->bytes_received]) {
        // trailer mismatch; drop and resync
        C->state          = EXPECTING_HEADER;
        C->bytes_received = 0;
        C->condition      = 0;
        C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
        if (try_cnt) (*try_cnt)++;
      } else {
        C->bytes_received++;
        if (C->bytes_received >= C->trailer_size) {
          // complete packet
          C->buffer[0] = C->condition;
          C->state          = EXPECTING_HEADER;
          C->bytes_received = 0;
          C->hdr_idx1 = C->hdr_idx2 = C->hdr_idx3 = C->hdr_idx4 = 0;
          if (try_cnt) *try_cnt = 0;
          return C->buffer;
        }
      }
      break;
  }

  return NULL;  // not complete yet
}


/* ----------------------------------------------------------------------------
 * Serial4 generic parser (FPGA data)
 * ----------------------------------------------------------------------------
 * Returns:
 *   - NULL if incomplete or mismatch.
 *   - Pointer to rd4.payload on success (length == datalen).
 */
uint8_t* readDataStream(const uint8_t* header, uint8_t header_len,
                  const uint8_t* trailer, uint8_t trailer_len,
                  uint16_t datalen, uint32_t* try_cnt)
{
  // Parameter validation
  if (!header || header_len == 0 || datalen == 0 || datalen > MAX_DATA_SIZE4) {
    if (try_cnt) (*try_cnt)++;
    rd4_reset();
    return NULL;
  }

  while (Serial1.available() > 0) {
    int di = Serial1.read();
    if (di == -1) break;                 // defensive
    uint8_t b = (uint8_t)di;

    switch (rd4.state) {
      case RD_FIND_HEADER:
        // Continuous header match (byte-by-byte)
        if (b == header[rd4.hdr_idx]) {
          rd4.hdr_idx++;
          if (rd4.hdr_idx >= header_len) {
            rd4.state   = RD_READ_PAYLOAD;
            rd4.pay_idx = 0;
            rd4.datalen = datalen;
          }
        } else {
          // Simple partial-match fallback (KMP-like, very simplified)
          rd4.hdr_idx = (b == header[0]) ? 1 : 0;
          if (try_cnt) (*try_cnt)++;
        }
        break;

      case RD_READ_PAYLOAD:
        rd4.payload[rd4.pay_idx++] = b;
        if (rd4.pay_idx >= rd4.datalen) {
          if (trailer_len == 0) {
            // No trailer: success
            uint8_t* ret = rd4.payload;
            rd4_reset();
            if (try_cnt) *try_cnt = 0;
            return ret;
          } else {
            rd4.state   = RD_CHECK_TRAILER;
            rd4.trl_idx = 0;
          }
        }
        break;

      case RD_CHECK_TRAILER:
        if (!trailer || rd4.trl_idx >= trailer_len) {
          if (try_cnt) (*try_cnt)++;
          rd4_reset();
          break;
        }
        if (b != trailer[rd4.trl_idx++]) {
          // Trailer mismatch: drop and resync
          if (try_cnt) (*try_cnt)++;
          rd4_reset();
        } else if (rd4.trl_idx >= trailer_len) {
          // Complete packet
          uint8_t* ret = rd4.payload;
          rd4_reset();
          if (try_cnt) *try_cnt = 0;
          return ret;
        }
        break;
    } // switch
  } // while

  return NULL;  // not complete yet
}

uint8_t* readDataBytewise(const uint8_t* header, uint8_t header_len,
                  const uint8_t* trailer, uint8_t trailer_len,
                  uint16_t datalen, uint32_t* try_cnt)
{
  // 參數檢查
  if (!header || header_len == 0 || datalen == 0 || datalen > MAX_DATA_SIZE4) {
    if (try_cnt) (*try_cnt)++;
    rd4_reset();
    return NULL;
  }

  // 如果沒有資料可讀，直接回 NULL
  if (Serial1.available() == 0) {
    return NULL;
  }

  int di = Serial1.read();
  if (di == -1) {
    return NULL;  // 防禦性檢查
  }
  uint8_t b = (uint8_t)di;

  switch (rd4.state) {
    case RD_FIND_HEADER:
      // 逐 byte 比對 header
      if (b == header[rd4.hdr_idx]) {
        rd4.hdr_idx++;
        if (rd4.hdr_idx >= header_len) {
          rd4.state   = RD_READ_PAYLOAD;
          rd4.pay_idx = 0;
          rd4.datalen = datalen;
        }
      } else {
        // fallback: 如果匹配失敗，看看要不要回到第一個 header byte
        rd4.hdr_idx = (b == header[0]) ? 1 : 0;
        if (try_cnt) (*try_cnt)++;
      }
      break;

    case RD_READ_PAYLOAD:
      rd4.payload[rd4.pay_idx++] = b;
      if (rd4.pay_idx >= rd4.datalen) {
        if (trailer_len == 0) {
          // 無 trailer，直接完成
          uint8_t* ret = rd4.payload;
          rd4_reset();
          if (try_cnt) *try_cnt = 0;
          return ret;
        } else {
          rd4.state   = RD_CHECK_TRAILER;
          rd4.trl_idx = 0;
        }
      }
      break;

    case RD_CHECK_TRAILER:
      if (!trailer || rd4.trl_idx >= trailer_len) {
        if (try_cnt) (*try_cnt)++;
        rd4_reset();
        break;
      }
      if (b != trailer[rd4.trl_idx++]) {
        // trailer mismatch → reset
        if (try_cnt) (*try_cnt)++;
        rd4_reset();
      } else if (rd4.trl_idx >= trailer_len) {
        // trailer 完成
        uint8_t* ret = rd4.payload;
        rd4_reset();
        if (try_cnt) *try_cnt = 0;
        return ret;
      }
      break;
  }

  return NULL;  // 預設情況：還沒湊齊一包
}

