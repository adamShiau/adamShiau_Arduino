#ifndef SRS200_H
#define SRS200_H

#include <Arduino.h>
#include "wiring_private.h"

class SRS200
{
	public:
		SRS200(void);
		void init(void);
		void readData(unsigned char [], unsigned char []);
		unsigned char* checkHeader(unsigned char []);
		
	private:		
		const unsigned char HEADER[2] = {0xC0, 0xC0};
	
};

Uart mySerial5 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM0_Handler() 
{
	mySerial5.IrqHandler();
}

#endif