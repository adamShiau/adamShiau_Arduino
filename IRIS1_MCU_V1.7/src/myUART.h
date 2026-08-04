#ifndef MYUART_H
#define MYUART_H
/*
 * myUART.h
 *
 * Minimal-change UART helpers for Arduino SAMD21 (e.g., Nano 33 IoT).
 * - Keeps a C-style API and your original "readDataDynamic()" usage.
 * - Serial3: fixed protocol with 3 headers/trailers (PC / RS422 cmd); use readDataDynamic().
 *   [2026-08 HW change] This was originally Serial1. Moved to Serial3 because of a
 *   hardware issue on the Serial1 (SERCOM5) line; both cmd RX and sensor-data TX now
 *   go through Serial3. The function name readDataDynamic() is kept unchanged so
 *   callers (IRIS1_MCU_V1.7.ino) don't need to change.
 * - Serial4: generic parser: caller provides header/trailer/payload length; use readData().
 *
 * IMPORTANT:
 *   - Implementation uses Arduino C++ types (Uart), so compile the implementation as .cpp.
 *   - Parsers return pointers to STATIC internal buffers (not re-entrant; don't call from ISRs).
 *   - DO NOT declare C++ objects (like 'Uart Serial1') inside extern "C".
 */

#include <Arduino.h>
#include "wiring_private.h"   // for sercomX symbols and pinPeripheral()

extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */
/**
 * @brief Initialize UARTs and pin multiplexing; reset parser states; drain RX.
 *
 * - Serial  (USB CDC): 230400
 * - Serial1 (SERCOM5): 230400 (PC)  -- provided by the Arduino core
 *   [2026-08 HW change] No longer used for cmd RX or sensor-data TX due to a
 *   hardware issue on this line; kept initialized but otherwise idle.
 * - Serial2 (SERCOM2): 115200
 * - Serial3 (SERCOM1): 230400 (PC / RS422 cmd -- moved from Serial1)
 * - Serial4 (SERCOM3): 115200 (FPGA)
 *
 * NOTE: Pin mappings are board-specific and match the user's previous setup:
 *   SERCOM2: TX=24, RX=25
 *   SERCOM1: TX=8 , RX=13
 *   SERCOM3: TX=9 , RX=10 (ALT function)
 */
void myUART_init(void);

/* -------------------------------------------------------------------------- */
/* Interrupt handlers (delegate to Arduino Uart::IrqHandler())                */
/* -------------------------------------------------------------------------- */
void SERCOM2_Handler(void);
void SERCOM1_Handler(void);
void SERCOM3_Handler(void);
void SERCOM5_Handler(void);

/* -------------------------------------------------------------------------- */
/* Serial3 (PC commands, RS422): fixed 3-format protocol                      */
/* [2026-08 HW change] Moved from Serial1 to Serial3 (see myUART.cpp)         */
/* -------------------------------------------------------------------------- */
/**
 * @brief Parse one byte (if available) from Serial3 using a fixed protocol:
 *        - Header/Trailer/Length sets:
 *            1) Header AB BA, Trailer 55 56, DataLen=6
 *            2) Header CD DC, Trailer 57 58, DataLen=13
 *            3) Header EF FE, Trailer 53 54, DataLen=6
 *        - On success, returns a pointer to a static buffer:
 *            buffer[0] = condition (1, 2, or 3)
 *            buffer[1..] = payload (length per condition)
 *        - On incomplete/no match, returns NULL.
 *
 * @param try_cnt Optional mismatch counter; increments on header/trailer fails.
 *                Set to NULL if you don't need it. On success, it is reset to 0.
 * @return uint8_t* Pointer to static buffer, or NULL if not complete yet.
 *
 * USAGE:
 *   uint8_t* buf = readDataDynamic(&try1);
 *   get_uart_cmd(buf, &my_cmd);   // your existing function
 */
uint8_t* readDataDynamic(uint32_t* try_cnt);

/* -------------------------------------------------------------------------- */
/* Serial4 (FPGA data): generic header/trailer + fixed data length            */
/* -------------------------------------------------------------------------- */
/**
 * @brief Parse from Serial4 using caller-specified header/trailer and payload length.
 *
 * Behavior:
 *   1) Looks for an exact header byte-by-byte (continuous match).
 *   2) When header is matched, reads exactly `datalen` bytes into an internal payload buffer.
 *   3) If trailer_len > 0, verifies exact trailer match; if trailer_len == 0, trailer check is skipped.
 *   4) Returns pointer to payload (length == datalen) only if header/payload/(trailer) all match.
 *   5) Otherwise returns NULL (keep calling until it completes).
 *
 * Notes:
 *   - Uses a STATIC internal buffer; valid until the next call completes.
 *   - NOT re-entrant; call from main loop, not from ISR.
 *   - `datalen` must be 1..MAX_DATA_SIZE4.
 *
 * @param header      Pointer to header bytes (e.g., {0xAB,0xBA})
 * @param header_len  Header length in bytes (>=1)
 * @param trailer     Pointer to trailer bytes (may be NULL if trailer_len==0)
 * @param trailer_len Trailer length in bytes (may be 0 for "no trailer")
 * @param datalen     Payload length in bytes (not including header/trailer)
 * @param try_cnt     Optional mismatch counter; increments on partial mismatches.
 *                    Set to NULL if you don't need it. On success, it is reset to 0.
 * @return uint8_t*   Pointer to static payload buffer of size `datalen`, or NULL if not complete yet.
 */

 
#define MAX_DATA_SIZE4  256
uint8_t* readDataStream(const uint8_t* header, uint8_t header_len,
                  const uint8_t* trailer, uint8_t trailer_len,
                  uint16_t datalen, uint32_t* try_cnt);
uint8_t* readDataBytewise(const uint8_t* header, uint8_t header_len,
                  const uint8_t* trailer, uint8_t trailer_len,
                  uint16_t datalen, uint32_t* try_cnt);

#ifdef __cplusplus
} // extern "C"
#endif

/* NOTE:
 * Do NOT redeclare 'extern Uart SerialX' here; the Arduino core already declares
 * them in variant.h with C++ linkage. Redeclaring in C linkage causes errors.
 */

#endif /* MYUART_H */
