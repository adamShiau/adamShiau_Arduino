#include "myUART.h"
#include <wiring_private.h>

// --- command buffers ---
uint8_t  myCmd_header[] = {0xAB, 0xBA};
uint8_t  myCmd_trailer[] = {0x55, 0x56};
uint16_t myCmd_try_cnt;
const uint8_t myCmd_sizeofheader = sizeof(myCmd_header);
const uint8_t myCmd_sizeoftrailer = sizeof(myCmd_trailer);

uint8_t  header[]  = {0xAB, 0xBA};
uint8_t  trailer[] = {0x55};
uint16_t try_cnt;
const uint8_t sizeofheader  = sizeof(header);
const uint8_t sizeoftrailer = sizeof(trailer);

byte uart_cmd, fog_ch;
int  uart_value;
volatile bool cmd_complete = false;
volatile bool fog_woke_flag = false;

// ---- 若你真的不使用 core 內建 Serial1，且已移除 variant 的定義，才在這裡自建 ----
Uart Serial1(&sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX);
Uart Serial2(&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial3(&sercom1, 13, 8,  SERCOM_RX_PAD_1, UART_TX_PAD_2);
Uart Serial4(&sercom3, 10, 9,  SERCOM_RX_PAD_3, UART_TX_PAD_2);

// ---- 上層物件 ----
uartRT myCmd(Serial1, 6);
PIG sp13(Serial2, 16);
PIG sp14(Serial3, 16);
PIG sp9 (Serial4, 16);

// ---- ISRs ----
void SERCOM2_Handler() { Serial2.IrqHandler(); }
void SERCOM1_Handler() { Serial3.IrqHandler(); }
void SERCOM3_Handler() { Serial4.IrqHandler(); }

void SERCOM5_Handler() {
  Serial1.IrqHandler();
  if (uint8_t* cmd = myCmd.readData(myCmd_header, myCmd_sizeofheader,
                                    &myCmd_try_cnt, myCmd_trailer, myCmd_sizeoftrailer)) {
    uart_cmd   = cmd[0];
    uart_value = (cmd[1]<<24) | (cmd[2]<<16) | (cmd[3]<<8) | cmd[4];
    fog_ch     = cmd[5];
    cmd_complete = true;
    fog_woke_flag = true;

    Serial.print("cmd, value, ch: ");
    Serial.print(uart_cmd); Serial.print(", ");
    Serial.print(uart_value); Serial.print(", ");
    Serial.println(fog_ch);
  }
}

// ---- API ----
void myUART_init(void) {
  Serial.begin(230400);   // debug
  Serial1.begin(230400);  // to PC
  Serial2.begin(230400);  // MINS
  Serial3.begin(115200);
  Serial4.begin(115200);

  pinPeripheral(24, PIO_SERCOM);
  pinPeripheral(25, PIO_SERCOM);
  pinPeripheral(8,  PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(10, PIO_SERCOM_ALT);
  pinPeripheral(9,  PIO_SERCOM_ALT);
}

void msg_out(char *msg) {
  Serial.println(msg);
  Serial1.println(msg);
}
