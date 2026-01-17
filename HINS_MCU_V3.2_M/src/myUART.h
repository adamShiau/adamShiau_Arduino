#ifndef MYUART_H
#define MYUART_H


#include <Arduino.h>
#include "wiring_private.h"   // for sercomX symbols and pinPeripheral()

extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;


void myUART_init(void);

/* -------------------------------------------------------------------------- */
/* Interrupt handlers (delegate to Arduino Uart::IrqHandler())                */
/* -------------------------------------------------------------------------- */
void SERCOM2_Handler(void);
void SERCOM1_Handler(void);
void SERCOM3_Handler(void);
void SERCOM5_Handler(void);


uint8_t* readDataDynamic(uint32_t* try_cnt);

uint8_t* readDataDynamic(Stream* port, uint32_t* try_cnt);

 
#define MAX_DATA_SIZE4  256
uint8_t* readDataStream(const uint8_t* header, uint8_t header_len,
                  const uint8_t* trailer, uint8_t trailer_len,
                  uint16_t datalen, uint32_t* try_cnt);
uint8_t* readDataBytewise(const uint8_t* header, uint8_t header_len,
                  const uint8_t* trailer, uint8_t trailer_len,
                  uint16_t datalen, uint32_t* try_cnt);

#endif /* MYUART_H */
