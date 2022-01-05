#include <Arduino.h>
#include "wiring_private.h"


#define DATA_OUT_START_ADDR 99

#define STOP 			0
#define INT_SYNC_MODE 	1
#define EXT_SYNC_MODE 	2

Uart mySerial13 (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);
Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM1_Handler()
{
    mySerial13.IrqHandler();
}
void SERCOM0_Handler()
{
    mySerial5.IrqHandler();
}

void setup() {
	
	// Reassign pins 13 and 8 to SERCOM (not alt this time)
	pinPeripheral(13, PIO_SERCOM);
	pinPeripheral(8, PIO_SERCOM);
	
	Serial.begin(115200);
	mySerial13.begin(115200);//rx:D13, tx:D8, PIG
	// mySerial5.begin(115200);
	delay(1000);
	sendCmd(DATA_OUT_START_ADDR, INT_SYNC_MODE);
	
}


void loop() {
	delay(10000);
	sendCmd(DATA_OUT_START_ADDR, STOP);
	
	
}

void sendCmd(byte addr, int value)
{
	mySerial13.write(addr);
	mySerial13.write(value>>24 & 0xFF);
	mySerial13.write(value>>16 & 0xFF);
	mySerial13.write(value>>8 & 0xFF);
	mySerial13.write(value & 0xFF);
	delay(1);
}

void send32BitCmd(int value)
{
	mySerial13.write(value>>24 & 0xFF);
	// delay(10);
	mySerial13.write(value>>16 & 0xFF);
	// delay(10);
	mySerial13.write(value>>8 & 0xFF);
	// delay(10);
	mySerial13.write(value & 0xFF);
	// delay(10);
	
}