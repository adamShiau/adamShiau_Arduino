#pragma once
#include <Arduino.h>
#include "uartRT.h"

// ---- Command Processing Functions ----
void processCommand1(int value, byte channel);
void processCommand2(int value, byte channel);
void processCommand3(int value, byte channel);
void processCommand4(int value, byte channel);
void processCommand5(int value, byte channel);
void processCommand6(int value, byte channel);
void processCommand7(int value, byte channel);
void processCommand8(int value, byte channel);
void processCommand9(int value, byte channel);
void processCommand10(int value, byte channel);

// ---- 數據輸出控制結構 ----
struct DataOutput {
    bool enabled;
    uint32_t interval_ms;
    uint32_t last_send_time;
    uint8_t packet_type;
    uint8_t channel;  // 關聯的通道
    uint32_t last_sent_gga_epoch;   // 避免重送同一筆GGA時間
    uint32_t last_sent_hdt_epoch;   // 避免重送同一筆HDT heading

};

// ---- API ----
void command_init(void);
void processCommands(void);
void processDataOutputs(void);