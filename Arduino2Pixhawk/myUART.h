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
#include "wiring_private.h"

const uint8_t output_header[4] = {0xFE, 0x81, 0xFF, 0x55};

/** Move Serial1 definition from variant.cpp to here*/
Uart Serial1( &sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
Uart Serial2 (&sercom2, 25, 24, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial3 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
Uart Serial4 (&sercom3, 10, 9, SERCOM_RX_PAD_3, UART_TX_PAD_2);

void SERCOM1_Handler() {Serial3.IrqHandler();}
void SERCOM5_Handler() {Serial1.IrqHandler();}

void myUART_init(void)
{
    Serial.begin(115200);  // debug
    Serial1.begin(230400); // output (PX4)
    Serial2.begin(115200); // Xsens
    Serial3.begin(115200); // NMEA_OUT_Serial
    Serial4.begin(115200); // NMEA_IN_Serial


    pinPeripheral(24, PIO_SERCOM);
    pinPeripheral(25, PIO_SERCOM);

    pinPeripheral(8, PIO_SERCOM);
    pinPeripheral(13, PIO_SERCOM);
    
    pinPeripheral(10, PIO_SERCOM_ALT);
    pinPeripheral(9, PIO_SERCOM_ALT);
}

#endif