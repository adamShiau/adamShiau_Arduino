#ifndef MYUART_H
#define MYUART_H
/** define all of the UART resource used in GP1Z
 * 11/23/2023
*/
/***
SERCOM0: I2C     (PA08, PA09) [sda, scl]
SERCOM1: serial3 (PA17, PA18) [rx, tx]
SERCOM2: serial2 (PA15, PA14) [rx, tx]
SERCOM3: serial4 (PA21, PA20) [rx, tx]
SERCOM4: SPI     (PB10, PB11, PA12, PA13) [ss, miso, mosi, sck]
SERCOM5: serial1 (PB23, PB22) [rx, tx]
  
***/
#include "uartRT.h"
// cmd read from GUI
uint8_t myCmd_header[] = {0xAB, 0xBA};
uint8_t myCmd_trailer[] = {0x55, 0x56};
uint16_t myCmd_try_cnt;
const uint8_t myCmd_sizeofheader = sizeof(myCmd_header);
const uint8_t myCmd_sizeoftrailer = sizeof(myCmd_trailer);

uartRT myCmd(Serial1, 6);

uint8_t header[] = {0xAB, 0xBA};
uint8_t trailer[] = {0x55};
uint16_t try_cnt;
const uint8_t sizeofheader = sizeof(header);
const uint8_t sizeoftrailer = sizeof(trailer);

Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial3 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
Uart Serial4 (&sercom3, 10, 9, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler() {Serial2.IrqHandler();}
void SERCOM1_Handler() {Serial3.IrqHandler();}
void SERCOM3_Handler() {Serial4.IrqHandler();}

#include "pig_v2.h"
PIG sp13(Serial2); //SP13
PIG sp14(Serial3, 14); //SP14
PIG sp9(Serial4); //SP14

uartRT SP13_Read(Serial2, 14);
uartRT SP14_Read(Serial3, 14);
uartRT SP9_Read(Serial4, 14);

void myUART_init(void)
{
    Serial.begin(230400); //debug
    Serial1.begin(230400); //to PC
    Serial2.begin(115200); //fog
    Serial3.begin(115200);
    Serial4.begin(115200);

    pinPeripheral(24, PIO_SERCOM);
    pinPeripheral(25, PIO_SERCOM);

    pinPeripheral(8, PIO_SERCOM);
    pinPeripheral(13, PIO_SERCOM);
    
    pinPeripheral(10, PIO_SERCOM_ALT);
    pinPeripheral(9, PIO_SERCOM_ALT);
}

#endif