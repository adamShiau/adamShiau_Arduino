#pragma once
#include <Arduino.h>
#include "uartRT.h"
#include "pig_v2.h"

// ---- command buffers & flags (在 .cpp 定義) ----
extern uint8_t  myCmd_header[];
extern uint8_t  myCmd_trailer[];
extern uint16_t myCmd_try_cnt;
extern const uint8_t myCmd_sizeofheader;
extern const uint8_t myCmd_sizeoftrailer;

extern uint8_t  header[];
extern uint8_t  trailer[];
extern uint16_t try_cnt;
extern const uint8_t sizeofheader;
extern const uint8_t sizeoftrailer;

extern byte uart_cmd, fog_ch;
extern int  uart_value;
extern volatile bool cmd_complete;   // ISR 會改寫 → volatile
extern volatile bool fog_woke_flag;

// ---- UART 實例（若你真的自建 Serial1~4）----
extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;

// ---- 高層通道物件 ----
extern uartRT myCmd;
extern PIG sp13;   // SP13 on Serial2
extern PIG sp14;   // SP14 on Serial3
extern PIG sp9;    // SP9  on Serial4

// ---- API ----
void myUART_init(void);
void msg_out(char *msg);

// ---- SERCOM ISRs（只宣告）----
void SERCOM1_Handler();
void SERCOM2_Handler();
void SERCOM3_Handler();
void SERCOM5_Handler();
